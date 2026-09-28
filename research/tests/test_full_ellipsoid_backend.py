import copy
import hashlib
import json
from pathlib import Path
import sys
import zlib
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import read_json
from s3e_pipeline.ellipsoid_backend import prepare_row,prepared_evidence
from s3e_pipeline.distributed import select_branches
from s3e_pipeline.mixed_pgo import settings
from test_ellipsoid_raster import primitive


def test_primitive_only_preparation_and_terrain_failure(tmp_path,monkeypatch):
    monkeypatch.setattr('s3e_pipeline.gravity_bev.submap_gravity',lambda *a:(np.eye(4),
        dict(projection='orthographic gravity-horizontal',source='test IMU')))
    e=primitive();path=tmp_path/'native.npz';np.savez_compressed(path,ellipsoids=e)
    row=dict(strategy='area',payload=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        robot_id='A',available_ns=100,stamp_ns=90,last_member_ns=90,begin_ns=0,
        area_radius_m=2,T_world_imu=np.eye(4).tolist(),submap_id=0,member_scan_ids=[0])
    cfg=dict(backend=dict(ellipsoid_only=True,registration=dict(method='ellipsoid'),mapclosures=dict(
        renderer='geometric_coverage',projection_alignment='gravity',density_map_resolution=.25,
        density_threshold=.05,hamming_distance_threshold=50,multilayer=dict(enabled=True))))
    store=tmp_path/'store';desc=tmp_path/'ellipsoid';store.mkdir();desc.mkdir()
    item,timing=prepare_row(tmp_path,row,0,store,desc,cfg)
    assert timing['point_arrays_loaded'] is False
    assert not timing['terrain']['available']
    with np.load(store/'000000.npz') as data:
        assert data.files==['ellipsoids']
        assert np.array_equal(prepared_evidence(data,item,{}),e)
    packet=json.loads(zlib.decompress((desc/'000000.json.zlib').read_bytes()))
    assert not packet['mapclosures_multilayer']['terrain']['available']
    with np.load(tmp_path/'rendering/000000.npz') as images:
        assert images['fullheight'].any()


def test_shared_channel_budget_and_dedup():
    def candidate(k,scores):
        return ('B',dict(keyframe_id=k,sources=['mapclosures'],branch_scores=dict(mapclosures=max(scores.values())),channel_scores=scores))
    candidates=[candidate(0,dict(fullheight=120)),candidate(1,dict(layers=40)),candidate(2,dict(fullheight=80))]
    selected,_=select_branches(candidates,dict(mapclosures=2),{},100,0)
    assert {x['keyframe_id'] for _,x in selected}=={0,1}
    candidates[0][1]['channel_scores']['layers']=150
    selected,_=select_branches(candidates,dict(mapclosures=2),{},100,0)
    assert {x['keyframe_id'] for _,x in selected}=={0,2}
    assert len(selected)==2


def test_raster_keypoint_coordinates_and_pose():
    import s3e_mapclosures_native as native
    engine=native.MapClosures(.5,.05,50)
    rng=np.random.default_rng(19);image=rng.integers(0,256,(256,256),dtype=np.uint8)
    q=engine.describe_image(image,np.array([-128,-128]),.5,np.eye(4))
    c=engine.describe_image(image,np.array([-118,-108]),.5,np.eye(4))
    assert len(q['xy'])>20
    assert np.allclose(c['xy']-q['xy'],[10,20])
    h=engine.pair(q,c)
    assert h['valid_pose'] and h['inliers']>20
    assert np.allclose(h['T_i_j'][:3,3],[-5,-10,0],atol=1e-5)


def ellipsoid_network(tmp_path):
    from test_registration_exchange import network
    workers,queue,pcm,edges=network(tmp_path)
    for r,w in workers.items():
        e=np.concatenate([primitive((i*.5,0,0)) for i in range(150)])
        digest=hashlib.sha256(e.astype('<f8').tobytes()).hexdigest()
        row=w.rows[0];row.update(ellipsoid_evidence_policy=dict(voxel_m=.4,max_count=50000),ellipsoid_evidence_sha256=digest)
        np.savez_compressed(w.store/'000000.npz',ellipsoids=e)
        w.cfg=settings(dict(enabled=True,factor='ellipsoid'));w.ellipsoid=True
        from s3e_pipeline.artifacts import digest as hash_config
        w.fingerprint=hash_config(w.cfg)
    for local_edges in edges.values():
        for pair,edge in local_edges.items():
            edge['diagnostics']=dict(ellipsoid_endpoints=[dict(robot=r,key=k,stamp_ns=10+k,cloud_frame=r+'/imu',
                payload_sha256=workers[r].rows[0]['ellipsoid_evidence_sha256']) for r,k in pair])
    return workers,queue,pcm,edges


def test_ellipsoid_exchange_keeps_verified_subset(tmp_path):
    from test_registration_exchange import drain
    workers,queue,pcm,edges=ellipsoid_network(tmp_path)
    sent=drain(workers,queue,pcm,edges)
    for w in workers.values():
        manifest=read_json(w.output/'manifest.json');assert manifest['schema_version']==2
        for cloud in manifest['clouds']:
            assert cloud['geometry_type']=='native_ellipsoids_v1'
            a=np.fromfile(w.output/cloud['file'],dtype='<f8').reshape(-1,15)
            assert len(a)==150
    assert all(e['body']['payload']['schema_version']==2 for e in sent if e['kind']=='registration_response')


@pytest.mark.parametrize('fault',['stamp','frame','axes','type','evidence_hash','changed_duplicate'])
def test_ellipsoid_endpoint_faults_fail_closed(tmp_path,fault):
    from s3e_pipeline.backends import pack_array,unpack_array
    workers,queue,pcm,edges=ellipsoid_network(tmp_path)
    for robot,w in workers.items():w.advance(pcm,edges[robot])
    receiver=workers['Alpha'];payload=copy.deepcopy(workers['Bob'].payload(0))
    event=dict(src='Bob',dst='Alpha',kind='registration_response',body=dict(
        session_id=receiver.session,configuration=receiver.fingerprint,payload=payload))
    if fault=='changed_duplicate':
        receiver.receive(copy.deepcopy(event));receiver.advance(pcm,edges['Alpha'])
        # Immutable duplicates include preprocessing metadata, even if geometry
        # and its verified endpoint digest are otherwise unchanged.
        payload['effective_voxel_m']=.5
    elif fault=='stamp':payload['stamp_ns']+=1
    elif fault=='frame':payload['cloud_frame']='Bob/odom'
    elif fault=='type':payload['geometry_type']='point_cloud'
    elif fault=='evidence_hash':payload['evidence_sha256']='0'*64
    else:
        e=unpack_array(payload['cloud']);e[0,3]=-1
        payload['cloud']=pack_array(e)
        payload['payload_sha256']=payload['evidence_sha256']=hashlib.sha256(e.tobytes()).hexdigest()
    with pytest.raises(ValueError):
        receiver.receive(event);receiver.advance(pcm,edges['Alpha'])
    assert not receiver.published


def test_ellipsoid_duplicate_requests_are_idempotent(tmp_path):
    from test_registration_exchange import drain
    workers,queue,pcm,edges=ellipsoid_network(tmp_path)
    for robot,w in workers.items():w.advance(pcm,edges[robot])
    request=copy.deepcopy(next(e for e in queue if e['kind']=='registration_request'))
    queue.append(request)
    sent=drain(workers,queue,pcm,edges)
    responses=[e['body']['payload'] for e in sent if e['kind']=='registration_response' and
               e['src']==request['dst'] and e['dst']==request['src']]
    assert len(responses)==2 and responses[0]==responses[1]
    assert all(w.published for w in workers.values())


def test_native_center_height_initialization_uses_no_cloud_preprocessor(monkeypatch):
    from s3e_pipeline.vertical_initialization import initialize_vertical
    monkeypatch.setattr('s3e_pipeline.vertical_initialization.bounded_cloud',lambda *a,**k:(_ for _ in ()).throw(AssertionError('Point cloud preprocessor used')))
    xyz=np.array(np.meshgrid(np.arange(10),np.arange(10),[0.,2.,5.])).reshape(3,-1).T
    source=xyz-[0,0,8.]
    T,info=initialize_vertical(xyz,source,np.eye(4),np.eye(4),np.eye(4),primitive_centers=True)
    assert T[2,3]==pytest.approx(8.) and info['geometry']=='native ellipsoid centers'
