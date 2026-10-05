import copy
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.circle_cover import CircleCoverConfig
from s3e_pipeline.circle_online import GrowingCircles, PublicationConfig, maturation_decision
from s3e_pipeline.circle_submaps import validate_row, validate_geometry, id_ranges
from s3e_pipeline.distributed import Worker
from s3e_pipeline.spatial_cbs_contract import endpoint


def observation(step, world, translation=(0.,0.,0.)):
    world=np.asarray(world,float);T=np.eye(4);T[:3,3]=translation
    ids=np.arange(len(world),dtype=np.int32);stamp=(100+step)*10**9
    row=dict(schema_version=5,strategy='area',robot_id='A',submap_id=step,
        stamp_ns=stamp,available_ns=stamp,anchor_sensor_ns=stamp,last_member_ns=stamp,
        begin_ns=100*10**9,complete=True,retrievable=False,geometry_count=len(world),
        ellipsoid_count=0,archive_point_count=len(world),geometry_id_ranges=id_ranges(ids),
        member_scan_ids=[0] if len(world) else [],anchor_scan_id=10+step,
        age_limit_s=None,area_shape='horizontal_disk_unbounded_height',area_radius_m=200.,
        area_center_world=list(translation),area_up_world=[0.,0.,1.],
        gravity_world_m_s2=[0.,0.,-9.81],T_world_imu=T.ravel().tolist(),frame='snapshot_anchor_imu')
    data=dict(points=(world-T[:3,3]).astype(np.float32),point_ids=ids,
        point_scan_ids=np.zeros(len(world),np.int32),ellipsoid_point_ids=np.empty(0,np.int32),
        ellipsoids=np.empty((0,15),np.float32))
    return row,data


def test_creation_geometry_cannot_use_future_observations(tmp_path):
    growing=GrowingCircles(CircleCoverConfig(radius_m=10.,coverage_target=.99))
    first=np.array([[x,y,30.] for x in range(4) for y in range(4)])
    a,data=observation(0,first);created,history=growing.add(a,data,'a'*64,10)
    assert len(created)==1 and history['diagnostics']['coverage']>=.99
    old_row,old_cloud,old_ids=copy.deepcopy(created[0]);old_centers=growing.planner.circles
    # New observations include one inside the old footprint and a remote island.
    later=np.vstack([first,[2.,2.,100.],[80.,0.,0.],[81.,0.,0.]])
    b,data=observation(1,later,translation=(5.,0.,0.));new,event=growing.add(b,data,'b'*64,20)
    assert new and growing.planner.circles[:len(old_centers)]==old_centers
    np.testing.assert_array_equal(created[0][1],old_cloud)
    np.testing.assert_array_equal(created[0][2],old_ids)
    assert created[0][0]==old_row and len(old_ids)==len(first)
    assert all(r['last_member_ns']==b['stamp_ns'] for r,_,_ in new)
    for row,cloud,ids in [*created,*new]:validate_geometry(row,cloud,ids)
    growing.finish(tmp_path)
    with np.load(tmp_path/'union.npz') as union:assert len(union['world'])==len(later)
    # Full union is retained for reconstruction, without changing old evidence.
    assert len(created[0][1])==len(first)


def test_revisit_adds_no_circle_and_clock_domains_remain_distinct():
    growing=GrowingCircles(CircleCoverConfig(radius_m=10.))
    world=np.array([[x,0.,0.] for x in range(5)])
    row,data=observation(0,world);created,_=growing.add(row,data,'a'*64,400)
    item=created[0][0]
    item.update(availability_clock='unix_wall_descriptor_publication',available_ns=500,
        snapshot_received_wall_ns=400,cbs_evidence_voxel_m=.4,sha256='b'*64)
    validate_row(item); assert endpoint(item)['available_ns']==500
    later,data=observation(1,world);new,_=growing.add(later,data,'c'*64,600)
    assert new==[] and len(growing.ids)==len(world)
    for fault in ('early_wall','early_sensor','changed_seal','policy'):
        bad=copy.deepcopy(item)
        if fault=='early_wall':bad['available_ns']=399
        if fault=='early_sensor':bad['native_available_ns']=bad['stamp_ns']-1
        if fault=='changed_seal':bad['seal_ns']+=1
        if fault=='policy':bad['geometry_publication_policy']='future'
        with pytest.raises(ValueError):validate_row(bad)


def test_native_future_ids_rejected_before_placement():
    growing=GrowingCircles(CircleCoverConfig())
    row,data=observation(0,[[1.,0.,0.]])
    row['archive_point_count']=0
    with pytest.raises(ValueError):growing.add(row,data,'a'*64,10)
    assert growing.planner is None


def test_verified_spatial_endpoint_provenance_survives_reversal():
    worker=Worker.__new__(Worker);worker.robot='Z';worker.queue_size=0;worker.backend_name='mapclosures'
    meta=[dict(robot='Z',key=1,source_sha256='z'),dict(robot='A',key=2,source_sha256='a')]
    event=dict(src='A',_verification_id=0,spatial_endpoints=meta,
        body=dict(query_id=1,candidate_id=2,score=10,query_stamp_ns=100))
    reply=dict(result=dict(accepted=True,T_i_j=np.eye(4).tolist(),information=np.eye(6).tolist()),seconds=.1,cache_hit=False)
    result=worker.finish_verification(event,200,reply,0.)
    edge=result['outgoing'][0]['body']
    assert edge['i']==['A',2] and edge['j']==['Z',1]
    assert edge['diagnostics']['spatial_endpoints']==meta


def test_maturation_keeps_center_anchor_and_accumulates_until_seal(tmp_path):
    cfg=PublicationConfig(mode='mature_then_freeze',minimum_age_s=2.,stability_window_s=2.,maximum_wait_s=8.)
    growing=GrowingCircles(CircleCoverConfig(radius_m=10.),cfg)
    first=np.array([[x,y,0.] for x in range(4) for y in range(4)])
    row,data=observation(0,first);created,_=growing.add(row,data,'a'*64,10)
    assert not created and not growing.release(lambda r,c:(True,{}))
    old=growing.planner.circles;anchor=row['T_world_imu']
    later=np.vstack([first,[1.,1.,8.]])
    for step in (1,2,3):
        native,data=observation(step,later,translation=(step,0.,0.))
        growing.add(native,data,str(step)*64,10+step)
        published=growing.release(lambda r,c:(True,dict(features=10)))
        if step<3:assert not published  # Point growth has not settled for two seconds.
    assert len(published)==1 and growing.planner.circles==old
    item,cloud,ids=published[0]
    assert item['T_world_imu']==anchor and item['stamp_ns']==row['stamp_ns']
    assert item['created_update']==0 and item['sealed_update']==3 and len(ids)==17
    assert item['maturation']['reason']=='stable' and item['seal_ns']==native['available_ns']
    validate_geometry(item,cloud,ids)
    frozen=cloud.copy()
    native,data=observation(4,np.vstack([later,[2.,2.,9.]]))
    growing.add(native,data,'f'*64,20);assert growing.release(lambda r,c:(True,{}))==[]
    np.testing.assert_array_equal(published[0][1],frozen)
    growing.finish(tmp_path)


def test_quality_gate_deadline_and_fifo_identity():
    cfg=PublicationConfig(mode='mature_then_freeze',minimum_age_s=1.,stability_window_s=1.,maximum_wait_s=3.)
    growing=GrowingCircles(CircleCoverConfig(radius_m=10.),cfg)
    points=np.array([[1.,0.,0.],[80.,0.,0.]])
    for step in range(4):
        row,data=observation(step,points);growing.add(row,data,str(step)*64,step)
        published=growing.release(lambda r,c:(r['keyframe_id']!=0,dict(features=0)))
        if step<3:assert not published  # Later footprints cannot reorder permanent IDs.
    assert [r['keyframe_id'] for r,_,_ in published]==[0,1]
    assert published[0][0]['maturation']['reason']=='deadline'
    assert not published[0][0]['maturation']['quality_ready']
    for row,cloud,ids in published:validate_geometry(row,cloud,ids)


def test_shutdown_tails_are_explicit_not_claimed_mature():
    growing=GrowingCircles(CircleCoverConfig(radius_m=10.),PublicationConfig(mode='mature_then_freeze'))
    row,data=observation(0,[[1.,0.,0.]])
    growing.add(row,data,'a'*64,100)
    published=growing.release(lambda r,c:(False,{}),final=True)
    assert published[0][0]['maturation']['reason']=='end_of_stream'
    assert published[0][0]['maturation']['age_s']==0 and not growing.pending
    assert growing.release(lambda r,c:(False,{}),final=True)==[]


def test_stability_checks_points_and_spatial_growth():
    cfg=PublicationConfig(mode='mature_then_freeze')
    h=[dict(native_available_ns=0,points=1000,cells=10),dict(native_available_ns=30*10**9,points=1010,cells=15)]
    assert maturation_decision(h,0,30*10**9,cfg)['reason']=='growing'
    h[-1]['cells']=10;h[-1]['points']=1500
    assert maturation_decision(h,0,30*10**9,cfg)['reason']=='growing'
    h[-1]['points']=1010
    assert maturation_decision(h,0,30*10**9,cfg)['reason']=='stable'
    with pytest.raises(ValueError):PublicationConfig(maximum_wait_s=10.)


def test_mature_endpoint_cannot_claim_seal_before_creation():
    growing=GrowingCircles(CircleCoverConfig(),PublicationConfig(mode='mature_then_freeze'))
    row,data=observation(0,[[1.,0.,0.]])
    growing.add(row,data,'a'*64,100)
    item=growing.release(lambda r,c:(True,{}),final=True)[0][0]
    item.update(availability_clock='unix_wall_descriptor_publication',available_ns=101)
    validate_row(item)
    for field,value in [('sealed_update',-1),('native_available_ns',item['stamp_ns']-1)]:
        changed=copy.deepcopy(item);changed[field]=value
        with pytest.raises(ValueError):validate_row(changed)
