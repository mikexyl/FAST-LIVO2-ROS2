import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.gravity_bev import gravity_ground, submap_gravity


@pytest.mark.parametrize('gravity', [[0,0,-9.81], [0,0,9.81], [9.81,0,0], [0,7,7], [.3,7,-7]])
def test_gravity_is_vertical_and_vertical_walls_collapse(gravity):
    ground = gravity_ground(gravity)
    R = ground[:3,:3]
    up = -np.asarray(gravity)/np.linalg.norm(gravity)
    np.testing.assert_allclose(R@up, [0,0,1], atol=1e-14)
    np.testing.assert_allclose(R.T@R, np.eye(3), atol=1e-14)
    assert np.isclose(np.linalg.det(R), 1)
    column = np.array([3.,5.,-4.]) + np.arange(50)[:,None]*up
    xy = (column@R.T)[:,:2]
    np.testing.assert_allclose(xy, np.tile(xy[0],(50,1)), atol=1e-13)


@pytest.mark.parametrize('gravity', [[0,0,0], [np.nan,0,1], [0,np.inf,1], [1,2]])
def test_invalid_gravity_never_falls_back_to_identity(gravity):
    with pytest.raises(ValueError): gravity_ground(gravity)


def test_reconstruction_is_explicit_causal_and_bound_to_payload():
    row=dict(robot_id='aerial05',submap_id=8,stamp_ns=100,begin_ns=80,sha256='abc')
    sidecar=dict(robot_id='aerial05',ground_truth_used=False,available_ns=10,
                 method='startup_accelerometer_reconstruction',submaps={
                     '8':dict(stamp_ns=100,payload_sha256='abc',gravity_imu_m_s2=[0,7,7])})
    with pytest.raises(ValueError,match='no anchor gravity'):submap_gravity(row)
    ground, provenance=submap_gravity(row,sidecar)
    assert provenance['reconstructed'] and not np.array_equal(ground,np.eye(4))
    for field, value in [('stamp_ns',101),('sha256','other'),('begin_ns',0),('robot_id','aerial06')]:
        with pytest.raises(ValueError):submap_gravity(dict(row,**{field:value}),sidecar)
    native=dict(row,gravity_imu_m_s2=[0,0,-9.81],gravity_world_m_s2=[0,0,-9.81],
                T_world_imu=np.eye(4).tolist(),gravity_source='ellipselio_filter_at_anchor')
    assert not submap_gravity(native)[1]['reconstructed']
    with pytest.raises(ValueError):submap_gravity(native,sidecar)


def test_native_explicit_projection_equals_leveled_cloud():
    import s3e_mapclosures_native as native
    rng=np.random.default_rng(26)
    centers=rng.uniform(-45,45,(300,3));centers[:,2]=0
    cloud=np.vstack([p+rng.normal(0,.4,(rng.integers(30,200),3)) for p in centers])
    tilted=cloud@Rotation.from_euler('xyz',[2.4,.1,.3]).as_matrix().T
    ground=gravity_ground(Rotation.from_euler('xyz',[2.4,.1,.3]).apply([0,0,-9.81]))
    before=tilted.copy()
    engine=native.MapClosures(.5,.05,50)
    actual=engine.describe(tilted,ground=ground)
    expected=engine.describe(tilted@ground[:3,:3].T,ground=np.eye(4))
    assert len(actual['xy'])>10
    np.testing.assert_array_equal(actual['ground'],ground)
    np.testing.assert_array_equal(actual['xy'],expected['xy'])
    np.testing.assert_array_equal(actual['bits'],expected['bits'])
    np.testing.assert_array_equal(tilted,before)
    with pytest.raises(ValueError):engine.describe(tilted,ground=np.zeros((4,4)))


def test_native_loop_and_reversed_loop_return_original_imu_frames():
    import s3e_mapclosures_native as native
    rng=np.random.default_rng(31)
    reference=dict(xy=rng.uniform(-80,80,(60,2)),bits=rng.integers(0,256,(60,32),dtype=np.uint8),
                   ground=gravity_ground([.1,6,7]))
    query=copy.deepcopy(reference);query['ground']=gravity_ground([.5,-7,6])
    planar=np.eye(4);planar[:3,:3]=Rotation.from_euler('z',.6).as_matrix();planar[:2,3]=[3,-7]
    query['xy']=reference['xy']@planar[:2,:2].T+planar[:2,3]/.5
    expected=np.linalg.inv(query['ground'])@planar@reference['ground']
    engine=native.MapClosures(.5,.05,50)
    forward=engine.pair(query,reference);reverse=engine.pair(reference,query)
    assert forward['inliers']==reverse['inliers']==60
    np.testing.assert_allclose(forward['T_i_j'],expected,atol=1e-5)
    np.testing.assert_allclose(reverse['T_i_j'],np.linalg.inv(expected),atol=1e-5)


def test_preparation_changes_projection_without_rotating_evidence(tmp_path, monkeypatch):
    import hashlib
    import json
    import zlib
    from s3e_pipeline import ellipsoid_cuda
    from s3e_pipeline.backends import unpack_array
    from s3e_pipeline.recent_submaps import prepare
    class Sampler:
        def render(self, centers, axes, basis):return centers, dict(surface_samples=len(centers))
        def close(self):pass
    monkeypatch.setattr(ellipsoid_cuda,'SurfaceSampler',Sampler)
    rng=np.random.default_rng(17)
    points=rng.uniform(-10,10,(1000,3)).astype(np.float32)
    ellipsoids=np.column_stack([points,np.ones((1000,3)),np.tile(np.eye(3).ravel(),(1000,1))])
    source=tmp_path/'source';source.mkdir()
    payload=source/'submap-000000.npz'
    np.savez_compressed(payload,points=points,ellipsoids=ellipsoids)
    row=dict(robot_id='aerial05',submap_id=0,complete=True,retrievable=True,member_scan_ids=[0,1],
             begin_ns=1,end_ns=10,last_member_ns=9,stamp_ns=9,available_ns=10,
             T_world_imu=np.eye(4).tolist(),payload=payload.name,sha256=hashlib.sha256(payload.read_bytes()).hexdigest(),
             gravity_imu_m_s2=[0,7,7],gravity_source='ellipselio_filter_at_anchor')
    (source/'index.jsonl').write_text(json.dumps(row)+'\n')
    (source/'manifest.json').write_text(json.dumps(dict(complete=True,
        index_sha256=hashlib.sha256((source/'index.jsonl').read_bytes()).hexdigest())))
    before={p.name:p.read_bytes() for p in source.iterdir()}
    cfg=dict(backend=dict(mapclosures=dict(density_map_resolution=.5,density_threshold=.05,hamming_distance_threshold=50)))
    prepare(source,tmp_path/'gravity',cfg)
    cfg['backend']['mapclosures']['projection_alignment']='local_ground'
    prepare(source,tmp_path/'legacy',cfg)
    def packet(name):return json.loads(zlib.decompress((tmp_path/name/'ellipsoid/000000.json.zlib').read_bytes()))
    leveled=packet('gravity');legacy=packet('legacy')
    np.testing.assert_allclose(unpack_array(leveled['mapclosures']['ground']),gravity_ground([0,7,7]))
    for field in ('submap_id','member_scan_ids','payload_sha256'):assert leveled[field]==legacy[field]
    assert (tmp_path/'gravity/store/000000.npz').read_bytes()==(tmp_path/'legacy/store/000000.npz').read_bytes()
    assert before=={p.name:p.read_bytes() for p in source.iterdir()}
