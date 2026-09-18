"""Separate pose initialization and common small_gicp acceptance in IMU coordinates."""
import numpy as np
from collections import OrderedDict
import hashlib
from scipy.spatial import cKDTree
from .geometry import pose, transform


class FeatureCache:
    """Bounded worker-local cache of derived geometry; never another store's data."""
    def __init__(self,mebibytes=256):
        self.limit=int(mebibytes*1024**2);self.bytes=0;self.entries=OrderedDict();self.hits=0
        if self.limit<0:raise ValueError('Negative feature cache budget')
    def get(self,kind,xyz,voxel,prepare):
        xyz=np.ascontiguousarray(xyz[:,:3],dtype=np.float64)
        key=(kind,xyz.shape,float(voxel),hashlib.sha256(xyz.tobytes()).digest())
        if key in self.entries:
            self.hits+=1;self.entries.move_to_end(key);return self.entries[key][0]
        result,size=prepare(xyz)
        if size<=self.limit:
            while self.entries and self.bytes+size>self.limit:
                _,(_,old_size)=self.entries.popitem(last=False);self.bytes-=old_size
            self.entries[key]=(result,size);self.bytes+=size
        return result


def bounded_cloud(xyz,voxel,max_points=20000,max_range=80.):
    """Deterministic voxel centroids; keep the frozen full cloud unchanged."""
    import small_gicp
    xyz=np.asarray(xyz[:,:3],dtype=np.float64)
    good=np.isfinite(xyz).all(axis=1)
    if max_range is not None:good&=np.linalg.norm(xyz,axis=1)<=max_range
    xyz=xyz[good]
    if voxel<=0 or max_points<30:raise ValueError('Invalid registration point budget')
    resolution=float(voxel)
    if not len(xyz):return xyz,resolution
    for _ in range(32):
        points=small_gicp.voxelgrid_sampling(xyz,resolution,num_threads=1).points()[:,:3].copy()
        if len(points)<=max_points:return points,resolution
        resolution*=1.25
    raise ValueError('Unable to bound registration cloud')


def refine(target, source, initial, cfg, cache=None):
    import small_gicp
    if initial is None:
        return dict(accepted=False,reason='no_pose_initialization')
    initial=pose(initial)
    def prepare(x):
        points,resolution=bounded_cloud(x,cfg['voxel_m'],cfg.get('max_points',20000),cfg.get('max_range_m',80.))
        # Sampling happened above. Build covariances on exactly that cloud, once.
        cloud=small_gicp.PointCloud(points)
        if len(points)<cfg['min_inliers']:return (cloud,None,resolution),len(points)*256+4096
        tree=small_gicp.KdTree(cloud,num_threads=1)
        small_gicp.estimate_normals_covariances(cloud,tree,num_neighbors=20,num_threads=1)
        return (cloud,tree,resolution),len(points)*256+4096
    if cache:
        kind=('gicp-bounded',cfg.get('max_points',20000),cfg.get('max_range_m',80.),cfg['min_inliers'])
        t,tree,tv=cache.get(kind,target,cfg['voxel_m'],prepare)
        s,_,sv=cache.get(kind,source,cfg['voxel_m'],prepare)
    else:(t,tree,tv),_=prepare(target);(s,_,sv),_=prepare(source)
    target=t.points()[:,:3];source=s.points()[:,:3]
    sizes=dict(target_points=len(target),source_points=len(source),target_voxel_m=tv,source_voxel_m=sv)
    if min(len(target),len(source)) < cfg['min_inliers']:
        return dict(accepted=False,reason='insufficient_points',**sizes)
    assessment_tree=cKDTree(target)
    initial_distance=assessment_tree.query(transform(initial,source),workers=1)[0]
    initial_overlap=float((initial_distance<cfg['correspondence_m']).mean())
    if initial_overlap<cfg.get('min_initial_overlap',.1):
        return dict(accepted=False,reason='initial_low_overlap',initial_overlap=initial_overlap,**sizes)
    alignment=dict(registration_type='GICP',max_correspondence_distance=cfg['correspondence_m'],
                   num_threads=1,max_iterations=cfg.get('max_iterations',40))
    result=small_gicp.align(t,s,tree,initial,**alignment)
    T=pose(result.T_target_source)
    aligned=transform(T,source)
    # Covariance preparation already computed the plane normals needed here.
    target_normals=t.normals()[:,:3]
    distance,idx=assessment_tree.query(aligned,workers=1)
    reverse=cKDTree(aligned).query(target,workers=1)[0]
    mask=distance<cfg['inlier_m']
    count=int(mask.sum())
    overlap=min(float(mask.mean()),float((reverse<cfg['inlier_m']).mean()))
    rmse=float(np.sqrt(np.mean(distance[mask]**2))) if count else None
    # Independently estimate point-to-plane observability. Point-to-point Hessians can
    # misleadingly constrain tangential motion on a single plane through sample IDs.
    normals=target_normals[idx[mask]]
    # Right perturbation of T: derivative R[-skew(source), I], in source tangent frame.
    ns=normals@T[:3,:3]
    J=np.column_stack([np.cross(source[mask],ns),ns])
    H=J.T@J/max(count,1)
    eig=np.linalg.eigvalsh(H)
    condition=float(eig[-1]/max(eig[0],1e-15))
    reason='accepted'
    if not result.converged: reason='not_converged'
    if count<cfg['min_inliers']: reason='insufficient_inliers'
    elif overlap<cfg['min_overlap']: reason='low_overlap'
    elif rmse is None or rmse>cfg['max_rmse_m']: reason='high_rmse'
    elif eig[0]<cfg['min_observability'] or condition>cfg['max_condition']: reason='unobservable'
    # Uncertainty is a documented conservative model, not a calibrated covariance claim.
    noise=max(rmse or cfg['translation_sigma_floor_m'],cfg['translation_sigma_floor_m'])
    information=H/(noise**2)
    covariance=np.linalg.inv(information+np.eye(6)*1e-12)
    covariance+=np.diag([np.deg2rad(cfg['rotation_sigma_floor_deg'])**2]*3+
                        [cfg['translation_sigma_floor_m']**2]*3)
    information=np.linalg.inv(covariance)
    return dict(accepted=reason=='accepted',reason=reason,T_i_j=T.tolist(),information=information.tolist(),
        initial_overlap=initial_overlap,**sizes,
        converged=bool(result.converged),inliers=count,overlap=overlap,rmse_m=rmse,
        observability_eigenvalues=eig.tolist(),condition=condition,iterations=int(result.iterations),
        native_hessian=np.asarray(result.H).tolist(),native_error=float(result.error),
        uncertainty='right tangent point-to-plane Hessian averaged per inlier, RMSE with conservative additive covariance floors')
