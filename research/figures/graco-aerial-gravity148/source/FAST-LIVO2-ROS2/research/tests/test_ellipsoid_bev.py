"""Check geometry and frame semantics independently of BEV visual appearance."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.ellipsoid_bev import unit_surface, surface_points, validate_ellipsoids, decode_ellipsoids


def test_native_empty_snapshot_and_padded_cloud_field_decoding():
    from types import SimpleNamespace as NS
    names=['x','y','z','a','b','c',*[f'v{r}{c}' for r in range(3) for c in range(3)],'map_id','primitive']
    fields=[NS(name=n,offset=4*i,count=1,datatype=7 if i<15 else 6) for i,n in enumerate(names)]
    msg=NS(fields=fields,width=0,height=1,is_bigendian=False,row_step=0,point_step=68,data=b'')
    result=decode_ellipsoids(msg)
    assert result['centers'].shape==(0,3) and result['basis'].shape==(0,3,3)
    # Non-packed rows and big endian fields exercise coordinate export decoding.
    msg.width=1;msg.height=2;msg.row_step=80;msg.is_bigendian=True;msg.data=bytearray(160)
    for row in range(2):
        values=np.array([row,2,3,1,.2,.01,*np.eye(3).ravel()],dtype='>f4')
        msg.data[row*80:row*80+60]=values.tobytes()
        msg.data[row*80+60:row*80+68]=np.array([row+100,85],dtype='>u4').tobytes()
    result=decode_ellipsoids(msg)
    assert np.allclose(result['centers'],[[0,2,3],[1,2,3]])
    assert np.allclose(result['axes'],[[1,.2,.01]]*2)
    assert result['map_id'].tolist()==[100,101] and result['primitive'].tolist()==[85,85]


def test_surface_samples_lie_on_ellipsoid_and_are_sign_symmetric():
    u=unit_surface(128)
    assert np.allclose(np.linalg.norm(u,axis=1),1)
    assert np.allclose(u.mean(axis=0),0)
    center=np.array([4.,-2.,1.]);axes=np.array([1.,.2,.002])
    R=Rotation.from_euler('zyx',[.7,.2,-.4]).as_matrix()
    points=(u*axes)@R.T+center
    assert np.allclose(np.sum(((points-center)@R/axes)**2,axis=1),1)
    a={tuple(np.round(x,10)) for x in u}
    assert a=={tuple(np.round(x,10)) for x in u*[1,-1,1]}


def test_render_uses_axes_orientation_and_preserves_thin_planes():
    center=np.array([[0.,0.,0.]]);axes=np.array([[1.,.2,.0001]])
    basis=np.eye(3)[None]
    p,stats=surface_points(center,axes,basis,spacing=.1,voxel=.02,max_range=None)
    assert stats['surface_samples']>100 and len(p)>30
    assert np.ptp(p[:,0])>1.9 and np.ptp(p[:,1])>.38 and np.ptp(p[:,2])<=.0002
    basis=Rotation.from_euler('z',90,degrees=True).as_matrix()[None]
    p,_=surface_points(center,axes,basis,spacing=.1,voxel=.02,max_range=None)
    assert np.ptp(p[:,1])>1.9 and np.ptp(p[:,0])<.41


def test_invalid_axes_rejected_but_improper_eigenvector_basis_supported():
    c=np.zeros((1,3));a=np.ones((1,3));basis=np.diag([-1.,1.,1.])[None]
    validate_ellipsoids(c,a,basis)
    with pytest.raises(ValueError):validate_ellipsoids(c,a*0,basis)
    with pytest.raises(ValueError):validate_ellipsoids(c,a,basis*2)
    with pytest.raises(ValueError):surface_points(c,a,basis,spacing=0)


def test_native_density_orb_and_pose_survive_ellipsoid_surface_input():
    import s3e_mapclosures_native as native
    import s3e_mapclosures_inspection as inspection
    from s3e_pipeline.mapclosures_inspection import check_features
    # Deterministic structures with distinct densities, not a single degenerate
    # plane. The rigid translation is an exact integer number of BEV cells.
    rng=np.random.default_rng(43)
    centers=np.concatenate([rng.normal([x,y,1.],[1,1,2],(150,3))
        for x,y in rng.uniform(-40,40,(70,2))])
    axes=np.tile([.5,.2,.05],(len(centers),1));basis=np.tile(np.eye(3),(len(centers),1,1))
    points,_=surface_points(centers,axes,basis)
    engine=native.MapClosures(.5,.05,50)
    candidate=engine.describe(points);query=engine.describe(points+[3.,-2.,0.])
    debug=inspection.Inspector(.5,.05,50).density(points,candidate['ground'])
    check_features(debug,candidate)
    result=engine.pair(query,candidate)
    assert len(candidate['xy'])>20 and result['valid_pose'] and result['inliers']>10
    assert np.allclose(result['T_i_j'][:3,3],[3,-2,0],atol=.05)
    reverse=engine.pair(candidate,query)
    assert np.allclose(result['T_i_j']@reverse['T_i_j'],np.eye(4),atol=.05)


def test_bounded_basis_roundoff_preserves_reflections_and_rejects_deformation():
    from s3e_pipeline.ellipsoid_bev import normalize_exported_basis
    exact=Rotation.from_euler('xyz',[.3,-.4,.7]).as_matrix()
    exact[:,0]*=-1
    exported=exact.copy();exported[0,1]+=2.7e-5
    repaired,stats=normalize_exported_basis(exported[None])
    assert np.linalg.det(repaired[0])<0
    assert np.allclose(repaired[0].T@repaired[0],np.eye(3),atol=1e-12)
    assert stats['max_gram_error']>0 and stats['max_basis_change']<3e-5
    assert np.max(np.abs(repaired[0]-exact))<3e-5
    with pytest.raises(ValueError):normalize_exported_basis((exact*1.01)[None])
    with pytest.raises(ValueError):normalize_exported_basis(np.full((1,3,3),np.nan))
