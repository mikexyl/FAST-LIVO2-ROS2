"""Audit cached gravity projections; display a saved planar hypothesis only."""
from pathlib import Path
import hashlib,json,shutil,zlib
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.gravity_bev import gravity_ground
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

B=Path(__file__).resolve().parent;W=B/'source/output'
O=B.parents[1]/'FAST-LIVO2-ROS2/research/figures/graco-ground-aerial-local-20260921/viewpoint-diagnostic/gravity-check'
O.mkdir(exist_ok=False)
gallery=json.loads((W/'bev-gallery/manifest.json').read_text())
def features(r,k):
    d=json.loads(zlib.decompress((W/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib').read_bytes()))
    return {k:unpack_array(v) for k,v in d['mapclosures'].items()}
audit=[]
for r in ('ground06','aerial06'):
    keys=[json.loads(l) for l in (W/f'prepared-{r}/store/keyframes.jsonl').read_text().splitlines()]
    reference=np.loadtxt(W/f'reference/{r}_gt.txt');t0=reference[0,0]
    interpolate=Slerp(reference[:,0]-t0,Rotation.from_quat(reference[:,4:8]))
    for row in keys:
        k=row['keyframe_id'];G=features(r,k)['ground'];g=np.asarray(row['gravity_imu_m_s2'])
        R=np.array(row['T_world_body'])[:3,:3]
        assert np.allclose(G,gravity_ground(g),atol=1e-12)
        assert np.allclose(R@g,row['gravity_world_m_s2'],atol=1e-7)
        assert np.allclose(G[:3,:3]@G[:3,:3].T,np.eye(3),atol=1e-12)
        assert np.isclose(np.linalg.det(G[:3,:3]),1.)
        projected=G[:3,:3]@g
        refR=interpolate([row['stamp_ns']/1e9-t0]).as_matrix()[0]
        refg=refR.T@np.array([0.,0.,-1.])
        angle=float(np.degrees(np.arccos(np.clip((g/np.linalg.norm(g))@refg,-1,1))))
        audit.append(dict(robot=r,key=k,gravity_imu_m_s2=g.tolist(),projected_gravity_m_s2=projected.tolist(),
            cached_projection_matches_native_gravity=True,evaluation_only_tilt_error_vs_INS_deg=angle))

q,c=27,17
with np.load(O.parent/f'aerial06-{q}-ground06-{c}-native.npz') as data:T=data['T_i_j']
L=features('aerial06',q)['ground']@T@np.linalg.inv(features('ground06',c)['ground'])
planar=np.eye(3);planar[:2,:2]=L[:2,:2];planar[:2,2]=L[:2,3]
assert np.allclose(planar[:2,:2].T@planar[:2,:2],np.eye(2),atol=1e-8)
fig,axes=plt.subplots(1,2,figsize=(12,7),layout='constrained',sharex=True,sharey=True)
corners=[]
for ax,r,k,registration in [(axes[0],'ground06',c,planar),(axes[1],'aerial06',q,np.eye(3))]:
    m=next(m for m in gallery['maps'] if m['robot']==r and m['key']==k)
    im=plt.imread(W/'bev-gallery'/m['image']);h,w=im.shape[:2]
    lx,ly=m['lower_bound'];res=gallery['density_map_resolution_m']
    # Native image column is level Y, row is level X; plot physical XY axes.
    pixel_to_level=np.array([[0.,res,res*(lx+.5)],[res,0.,res*(ly+.5)],[0.,0.,1.]])
    affine=registration@pixel_to_level
    ax.imshow(im,cmap='gray',vmin=0,vmax=1,interpolation='nearest',
        transform=Affine2D(affine)+ax.transData,origin='upper')
    corner=np.array([[-.5,-.5,1.],[w-.5,-.5,1.],[-.5,h-.5,1.],[w-.5,h-.5,1.]])@affine.T
    corners.append(corner[:,:2]);ax.set_facecolor('black');ax.set_aspect('equal')
    angle=next(a['evaluation_only_tilt_error_vs_INS_deg'] for a in audit if a['robot']==r and a['key']==k)
    ax.set_title(f'{r} / {k}\nGravity direction error versus INS: {angle:.2f}°')
    ax.set_xlabel('Common horizontal X [m]')
corners=np.concatenate(corners);lo=corners.min(axis=0)-2;hi=corners.max(axis=0)+2
for ax in axes:ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1])
axes[0].set_ylabel('Common horizontal Y [m]')
fig.suptitle('Both gravity-leveled BEVs, displayed with a common heading\nSaved RANSAC yaw + XY translation only; no GT alignment, no accepted 3D loop',fontsize=14)
fig.savefig(O/'same-heading-bevs.png',dpi=170);plt.close(fig)
summary=dict(rows=audit,ground_truth_used_for_images=False,ground_truth_used_only_to_evaluate_gravity=True,
    relative_planar_yaw_deg=float(np.degrees(np.arctan2(L[1,0],L[0,0]))),
    all_cached_projections_verified=True,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
shutil.copy2(Path(__file__),O/'inspect_gravity.py')
print(json.dumps(dict(relative_planar_yaw_deg=summary['relative_planar_yaw_deg'],verified=len(audit),
    selected=[a for a in audit if (a['robot'],a['key']) in [('ground06',17),('aerial06',27)]]),indent=2))
