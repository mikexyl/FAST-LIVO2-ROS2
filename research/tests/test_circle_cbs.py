"""Spatial IDs, immutable point transport and real growing distributed graphs."""
import copy
import os
from pathlib import Path
import sys
import time
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import file_hash, read_json, write_jsonl
from s3e_pipeline.cbs_bridge import local_graph
from s3e_pipeline.circle_cbs import solution_graph
from s3e_pipeline.circle_submaps import PREPROCESSING
from s3e_pipeline.geometry import inv
from s3e_pipeline.persistent_cbs import PersistentSession, common_reference_ready
from s3e_pipeline.registration_exchange import RegistrationExchange
from s3e_pipeline.spatial_cbs_contract import endpoint
from test_registration_exchange import network, drain


def spatialize(row, path, voxel=.4):
    with np.load(path) as f: cloud=f['cloud'].copy()
    np.savez_compressed(path,cloud=cloud,point_ids=np.arange(len(cloud)))
    T=np.asarray(row.get('T_world_body',np.eye(4)))
    row.update(schema_version=6,strategy='fixed_spatial',placement_mode='circle',
        T_world_body=T.tolist(),T_world_imu=T.ravel().tolist(),T_horizontal_map=np.eye(4).tolist(),
        frame='creation_anchor_imu',available_ns=row['stamp_ns']+10**10,seal_ns=row['stamp_ns']+10**10,
        last_member_ns=row['stamp_ns']+10**10,complete=True,footprint_radius_m=200.,footprint_center_xy=[0.,0.],
        geometry_id_ranges=[[0,len(cloud)]],geometry_count=len(cloud),geometry_preprocessing=PREPROCESSING,
        gravity_world_m_s2=[0.,0.,-9.81],gravity_imu_m_s2=(T[:3,:3].T@[0.,0.,-9.81]).tolist(),
        gravity_source='synthetic fixed gravity',sha256=file_hash(path),cbs_evidence_voxel_m=voxel)
    row.pop('submap_end_ns',None)


def spatial_network(tmp_path):
    workers,queue,pcm,edges=network(tmp_path,['Bob','Carol','Alpha'])
    for r,w in workers.items():
        w.cfg['max_range_m']=None
        # Construct new workers so the configuration fingerprint is consistent.
        spatialize(w.rows[0],w.store/'000000.npz')
        workers[r]=RegistrationExchange(r,w.robots,w.rows,w.store,w.store/'spatial','s/1',w.cfg,queue.append,{})
    for incident in edges.values():
        for pair in incident:
            if pair[0][1]!=0: continue
            incident[pair]=dict(i=list(pair[0]),j=list(pair[1]),diagnostics=dict(
                spatial_endpoints=[endpoint(workers[r].rows[k]) for r,k in pair]))
    return workers,queue,pcm,edges


def test_spatial_protocol_membership_and_cache(tmp_path,monkeypatch):
    workers,queue,pcm,edges=spatial_network(tmp_path)
    sent=drain(workers,queue,pcm,edges)
    assert sum(e['kind']=='registration_response' for e in sent)==2
    for w in workers.values():
        for c in read_json(w.output/'manifest.json')['clouds']:
            assert c['schema_version']==3 and c['submap_endpoint']['available_ns']>c['stamp_ns']
    second={r:RegistrationExchange(r,w.robots,w.rows,w.store,w.store/'second','s/2',w.cfg,queue.append,w.cache)
            for r,w in workers.items()}
    monkeypatch.setattr(np,'load',lambda *a,**k:pytest.fail('Immutable evidence reloaded'))
    assert not any(e['kind']=='registration_response' for e in drain(second,queue,pcm,edges))


@pytest.mark.parametrize('fault',['hash','membership','frame','expected','crop','cache'])
def test_spatial_fail_closed(tmp_path,fault):
    workers,queue,pcm,edges=spatial_network(tmp_path)
    w=workers['Bob'];row=w.rows[0]
    if fault=='hash': row['sha256']='0'*64
    elif fault=='membership': row['geometry_id_ranges']=[[1,row['geometry_count']+1]]
    elif fault=='frame': row['frame']='world'
    elif fault=='crop': w.cfg['max_range_m']=80
    elif fault=='expected':
        for edge in edges['Bob'].values():
            for meta in edge.get('diagnostics',{}).get('spatial_endpoints',[]):
                if meta['robot']=='Bob': meta['available_ns']+=1
    else:
        drain(workers,queue,pcm,edges)
        row['footprint_center_xy']=[1.,0.]
        workers={r:RegistrationExchange(r,v.robots,v.rows,v.store,v.store/'next','s/2',v.cfg,queue.append,v.cache)
                 for r,v in workers.items()}
    with pytest.raises(ValueError): drain(workers,queue,pcm,edges)


def test_duplicate_creation_times_are_distinct_keys(tmp_path):
    workers,_,_,_=spatial_network(tmp_path);row=workers['Bob'].rows[0]
    rows=[row,dict(row,keyframe_id=1)]
    nodes,edges=local_graph(rows,'Bob',dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15))
    assert len(nodes)==2 and nodes[0]['stamp_ns']==nodes[1]['stamp_ns']
    np.testing.assert_allclose(edges[0]['T_i_j'],np.eye(4))
    snap=dict(poses=nodes,constraints=[],summary=dict(robots={'Bob':dict(component='Bob')}))
    assert len(solution_graph(snap,{'Bob':rows})['poses'])==2
    snap['poses']=nodes[:1]
    with pytest.raises(ValueError): solution_graph(snap,{'Bob':rows})


def test_reference_barrier_follows_retained_components_and_native_order():
    robots=['z','b','a','island']
    snap=dict(constraints=[dict(i=['z',0],j=['b',0]),dict(i=['b',0],j=['a',0])],
        summary=dict(robots={r:dict(component='z' if r!='island' else 'island') for r in robots}))
    assert common_reference_ready(snap,robots)
    snap['summary']['robots']['a']['component']='b'
    assert not common_reference_ready(snap,robots)
    snap['constraints']=snap['constraints'][:1]
    snap['summary']['robots']['a']['component']='a'
    assert common_reference_ready(snap,robots)


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1',reason='Requires isolated persistent GPU CBS overlay')
def test_native_growing_spatial_graph(tmp_path):
    from test_persistent_cbs import fixture
    cfg,stores,rows,loops,truth=fixture(tmp_path,True)
    cfg['dpgo']['registration_factors']['max_range_m']=None
    cfg['dpgo']['ros_domain_id']=194
    for r,keys in rows.items():
        # Two footprints created in one map update share an IMU anchor.
        keys[1]['stamp_ns']=keys[0]['stamp_ns'];keys[1]['T_world_body']=copy.deepcopy(keys[0]['T_world_body'])
        truth[r,1]=truth[r,0]
        with np.load(stores[r]/'000000.npz') as f: np.savez_compressed(stores[r]/'000001.npz',cloud=f['cloud'])
        for row in keys: spatialize(row,stores[r]/f"{row['keyframe_id']:06d}.npz",.01)
        write_jsonl(stores[r]/'keyframes.jsonl',keys)
    for e in loops:
        e['diagnostics']=dict(spatial_endpoints=[endpoint(rows[r][k]) for r,k in (e['i'],e['j'])])
    session=PersistentSession(cfg,Path(__file__).resolve().parents[3],tmp_path/'session',stores)
    try:
        start=time.monotonic()
        while not session.ready():
            assert time.monotonic()-start<30;time.sleep(.1)
        pids=[p.pid for _,p in session.processes]
        for count,selected,final in [(2,[e for e in loops if e['i'][1]==0],False),(6,loops,False),(6,loops,True)]:
            session.submit({r:keys[:count] for r,keys in rows.items()},selected,final)
            snap=None;start=time.monotonic()
            while time.monotonic()-start<2 or snap is None or (final and not snap['summary']['finished']):
                snap=session.snapshot();time.sleep(.1);assert time.monotonic()-start<65
            assert len(snap['poses'])==count*3 and [p.pid for _,p in session.processes]==pids
        solution_graph(snap,rows)
        for row in snap['poses']:
            T=inv(truth['Alpha',0])@truth[row['robot_id'],row['keyframe_id']]
            np.testing.assert_allclose(row['T_world_body'],T,atol=.01)
        for r in rows:
            assert not read_json(session.output/r/'registration/3/transport.json')['requested']
            assert snap['summary']['robots'][r]['registration']['success']
    finally: session.close()


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1',reason='Requires isolated persistent CBS overlay')
@pytest.mark.parametrize('gpu',[False,True])
def test_native_persistent_five_robot_chain(tmp_path,gpu):
    from test_persistent_cbs import fixture
    robots=['Alpha','Bob','Carol','Delta','Echo']
    cfg,stores,rows,loops,truth=fixture(tmp_path,gpu,robots,list(zip(robots,robots[1:])))
    cfg['dpgo'].update(ros_domain_id=195,persistent_settle_iterations=80)
    if gpu:
        cfg['dpgo']['registration_factors']['max_range_m']=None
        for r,keys in rows.items():
            for row in keys: spatialize(row,stores[r]/f"{row['keyframe_id']:06d}.npz",.01)
            write_jsonl(stores[r]/'keyframes.jsonl',keys)
        for edge in loops:
            edge['diagnostics']=dict(spatial_endpoints=[endpoint(rows[r][k]) for r,k in (edge['i'],edge['j'])])
    session=PersistentSession(cfg,Path(__file__).resolve().parents[3],tmp_path/'session',stores)
    try:
        deadline=time.monotonic()+30
        while not session.ready():
            assert time.monotonic()<deadline;time.sleep(.1)
        pids=[p.pid for _,p in session.processes]
        # Start as isolated robots, then admit the sparse chain. No restart or
        # direct Alpha-to-Echo measurement may supply the missing root frame.
        for selected,final in [([],False),(loops,False),(loops,True)]:
            session.submit(rows,selected,final)
            start=time.monotonic();snap=None
            while snap is None or (final and not snap['summary']['finished']) or time.monotonic()-start<2:
                snap=session.snapshot();time.sleep(.1)
                assert time.monotonic()-start<90
            assert [p.pid for _,p in session.processes]==pids
        assert common_reference_ready(snap,robots)
        assert snap['summary']['loops']==12
        for row in snap['poses']:
            expected=inv(truth['Alpha',0])@truth[row['robot_id'],row['keyframe_id']]
            np.testing.assert_allclose(row['T_world_body'],expected,atol=.01)
        if gpu:
            for r in robots:
                assert snap['summary']['robots'][r]['registration']['success']
                assert not read_json(session.output/r/'registration/3/transport.json')['requested']
    finally: session.close()
