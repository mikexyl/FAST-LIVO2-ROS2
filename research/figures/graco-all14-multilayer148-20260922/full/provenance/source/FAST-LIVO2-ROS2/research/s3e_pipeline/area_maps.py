"""Accumulated-map area snapshots: geometric membership, never scan windows."""
import numpy as np


def id_ranges(ids):
    ranges=[]
    for value in ids:
        value=int(value)
        if ranges and ranges[-1][1]==value:ranges[-1][1]=value+1
        else:ranges.append([value,value+1])
    return ranges


def shared_ids(a,b):
    """Intersect sorted half-open persistent point-ID ranges without expansion."""
    i=j=0
    while i<len(a) and j<len(b):
        if max(a[i][0],b[j][0])<min(a[i][1],b[j][1]):return True
        if a[i][1]<=b[j][1]:i+=1
        else:j+=1
    return False


def area_mask(points,up,radius,tolerance=0.):
    points=np.asarray(points,dtype=float);up=np.asarray(up,dtype=float)
    horizontal=points-np.outer(points@up,up)
    return np.linalg.norm(horizontal,axis=1)<=radius+tolerance


def validate_area(row,data):
    if row.get('schema_version')!=5 or row.get('strategy')!='area' or row.get('age_limit_s','missing') is not None:
        raise ValueError('Invalid accumulated area-map schema')
    T=np.array(row['T_world_imu']).reshape(4,4);up=np.asarray(row['area_up_world'])
    radius=row['area_radius_m']
    if (not np.isfinite(T).all() or not np.isfinite(up).all() or not np.isclose(np.linalg.norm(up),1) or
            not np.isfinite(radius) or radius<=0 or row['area_shape']!='horizontal_disk_unbounded_height' or
            not np.allclose(row['area_center_world'],T[:3,3]) or row['frame']!='snapshot_anchor_imu'):
        raise ValueError('Invalid area-map region or anchor')
    gravity=np.asarray(row['gravity_world_m_s2'])
    if not np.allclose(up,-gravity/np.linalg.norm(gravity)):
        raise ValueError('Area gravity mismatch')
    ids=data['point_ids'];scans=data['point_scan_ids'];eids=data['ellipsoid_point_ids'];points=data['points'];e=data['ellipsoids']
    if any(v.dtype.kind not in 'iu' or v.ndim!=1 for v in (ids,scans,eids)):
        raise ValueError('Invalid area-map provenance arrays')
    if (points.shape!=(row['geometry_count'],3) or e.shape!=(row['ellipsoid_count'],15) or
            len(ids)!=len(points) or len(scans)!=len(ids) or len(eids)!=len(e) or
            not all(np.isfinite(v).all() for v in (points,e)) or
            np.any(np.diff(ids.astype(np.int64))<=0) or np.any(ids<0) or np.any(ids>=row['archive_point_count']) or
            np.any(np.diff(eids.astype(np.int64))<=0) or not np.isin(eids,ids).all() or
            np.any(scans<0) or np.any(scans>row['anchor_scan_id']) or
            sorted(set(scans.tolist()))!=row['member_scan_ids'] or id_ranges(ids)!=row['geometry_id_ranges']):
        raise ValueError('Invalid area-map geometry membership')
    tolerance=32*np.finfo(np.float32).eps*max(1.,np.max(np.abs(T[:3,3])),radius)
    if not area_mask(points,T[:3,:3].T@up,radius,tolerance).all():
        raise ValueError('Point outside selected area')
    if len(eids) and not np.array_equal(points[np.searchsorted(ids,eids)],e[:,:3]):
        raise ValueError('Ellipsoid/evidence membership mismatch')
    if (row['available_ns']<max(row['stamp_ns'],row['anchor_sensor_ns'],row['last_member_ns']) or
            row['last_member_ns']>row['anchor_sensor_ns'] or not row['complete'] or
            row['retrievable']!=bool(len(eids))):
        raise ValueError('Noncausal or incomplete area snapshot')
    return points.astype(float)@T[:3,:3].T+T[:3,3]


class AreaHistoryAudit:
    def __init__(self):
        self.world=np.empty((0,3));self.scans=np.empty(0,dtype=np.int64)
        self.last_anchor=-1;self.last_archive_count=0
    def add(self,row,data):
        world=validate_area(row,data)
        if row['anchor_sensor_ns']<=self.last_anchor or row['archive_point_count']<self.last_archive_count:
            raise ValueError('Nonchronological accumulated map')
        count=row['archive_point_count'];ids=data['point_ids'];center=np.asarray(row['area_center_world']);up=np.asarray(row['area_up_world'])
        if count>len(self.scans):
            extra=max(count,2*len(self.scans))-len(self.scans)
            self.world=np.concatenate([self.world,np.zeros((extra,3))]);self.scans=np.r_[self.scans,np.full(extra,-1)]
        known_ids=np.flatnonzero(self.scans>=0)
        inside=area_mask(self.world[known_ids]-center,up,row['area_radius_m'],-1e-3)
        if not np.isin(known_ids[inside],ids,assume_unique=True).all():
            raise ValueError('Previously available in-area point was lost')
        shared=self.scans[ids]>=0
        if (not np.array_equal(self.scans[ids[shared]],data['point_scan_ids'][shared]) or
                not np.allclose(self.world[ids[shared]],world[shared],atol=1e-3,rtol=1e-6)):
            raise ValueError('Persistent map point identity changed')
        self.world[ids]=world;self.scans[ids]=data['point_scan_ids']
        self.last_anchor=row['anchor_sensor_ns'];self.last_archive_count=row['archive_point_count']


def render_area_surface(centers,axes,basis,ground,radius):
    """Enclose every selected ellipsoid, then clip solely in horizontal XY.

    CUDA's finite voxel-key domain is an implementation constraint, not a range
    filter. Fall back to the equivalent CPU sampler outside that domain.
    """
    from .ellipsoid_cuda import SurfaceSampler
    from .ellipsoid_bev import surface_points
    bound=float(np.max(np.linalg.norm(centers,axis=1)+np.max(axes,axis=1),initial=0))+.5
    if 8*int(np.ceil(bound/.25))**3>=0xffffffff:
        surface,sampling=surface_points(centers,axes,basis,max_range=None)
        sampling=dict(sampling,device='CPU',range_filter=None)
    else:
        sampler=SurfaceSampler(max_range=bound)
        try:surface,sampling=sampler.render(centers,axes,basis)
        finally:sampler.close()
        sampling=dict(sampling,enclosing_sampler_radius_m=bound,range_filter=None)
    level=surface@ground[:3,:3].T+ground[:3,3]
    keep=np.linalg.norm(level[:,:2],axis=1)<=radius
    return surface[keep],dict(sampling,area_surface_points=int(keep.sum()),area_radius_m=radius)
