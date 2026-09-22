import copy
import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.backends import pack_array
from s3e_pipeline.multilayer_mapclosures import NAMES, VERSION, MultilayerMatcher, decode_layers, describe_layers
import s3e_mapclosures_native as native

OPTIONS = dict(density_map_resolution=.5, density_threshold=.05, hamming_distance_threshold=50,
               max_hypotheses_per_query=20, inliers_threshold=5)

def packet(seed=4):
    rng=np.random.default_rng(seed)
    return dict(ground=np.eye(4),xy=rng.uniform(-80,80,(12,2)),
                bits=rng.integers(0,256,(12,32),dtype=np.uint8))

def descriptor(packets):
    encode=lambda p:{k:pack_array(v) for k,v in p.items()}
    return dict(mapclosures=encode(packets['near']),mapclosures_multilayer=dict(version=VERSION,
        terrain=dict(available=True),layers={n:encode(packets[n]) for n in NAMES}))

def test_native_raw_matches_are_eligible_and_in_map_coordinates():
    p=packet();q=packet();q['xy']+=np.array([26,-14])
    e=native.MapClosures(.5,.05,50);e.add(7,p);e.add(9,p)
    raw=e.query_correspondences(q,[7]);assert len(raw)==1 and raw[0]['keyframe_id']==7
    np.testing.assert_allclose(raw[0]['query_xy']-raw[0]['candidate_xy'],np.tile([26,-14],(12,1)),atol=1e-5)
    assert len(raw[0]['hamming'])==12 and np.all(raw[0]['hamming']==0)
    assert e.query_correspondences(q,[])==[]
    with pytest.raises(ValueError):e.query_correspondences(q,[8])

def test_joint_index_pose_direction_gravity_and_serialization():
    p={n:packet(i+5) for i,n in enumerate(NAMES)};q=copy.deepcopy(p)
    R=Rotation.from_euler('z',.4).as_matrix()[:2,:2]
    Gq=np.eye(4);Gq[:3,:3]=Rotation.from_euler('xy',[.2,-.3]).as_matrix()
    for n in NAMES:q[n]['xy']=p[n]['xy']@R.T+[26,-14];q[n]['ground']=Gq
    dp,dq=descriptor(p),descriptor(q)
    dq=json.loads(json.dumps(dq))
    m=MultilayerMatcher(OPTIONS);m.add(4,dp)
    h=m.query(dq,[4])[0]
    T=np.eye(4);T[:2,:2]=R;T[:2,3]=[13,-7]
    assert h['passes_2d_gate'] and h['inliers']==60 and len(h['layer_support'])==5
    np.testing.assert_allclose(h['T_i_j'],np.linalg.inv(Gq)@T,atol=1e-5)
    np.testing.assert_allclose(m.pair(dp,dq)['T_i_j'],np.linalg.inv(h['T_i_j']),atol=1e-5)
    assert m.query(dq,[])==[]

def test_full_height_control_cannot_rescue_empty_layers():
    p={n:packet(i) for i,n in enumerate(NAMES)};d=descriptor(p)
    for n in NAMES:d['mapclosures_multilayer']['layers'][n]= {k:pack_array(v) for k,v in dict(
        ground=np.eye(4),xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8)).items()}
    m=MultilayerMatcher(OPTIONS);m.add(0,d)
    assert m.query(d,[0])==[] and not m.pair(d,d)['valid_pose']

def test_failed_terrain_records_empty_layers():
    e=native.MapClosures(.5,.05,50)
    result=describe_layers(e,np.zeros((10,3)),np.zeros((10,3)),np.eye(4))
    assert not result['terrain']['available'] and all(v==0 for v in result['feature_counts'].values())

def test_mixed_gravity_and_unavailable_terrain_rejected():
    d=descriptor({n:packet(i) for i,n in enumerate(NAMES)})
    d['mapclosures_multilayer']['layers']['low']['ground']=pack_array(np.eye(4)*2)
    with pytest.raises(ValueError,match='gravity'):decode_layers(d)
    d=descriptor({n:packet(i) for i,n in enumerate(NAMES)})
    d['mapclosures_multilayer']['terrain']['available']=False
    with pytest.raises(ValueError,match='Unavailable'):decode_layers(d)
