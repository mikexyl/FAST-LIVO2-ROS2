"""Causal accumulated geometry: all ages/heights, exact spatial membership."""
import copy
import io
import json
import struct
import numpy as np
import pytest
from s3e_pipeline.area_maps import validate_area,AreaHistoryAudit,id_ranges,shared_ids,render_area_surface
from s3e_pipeline.submap_writer import consume
from s3e_pipeline.distributed import eligible_observation


def fixture():
    points=np.array([[0,0,-40],[4,0,-120],[10,0,500]],dtype='<f4')
    ids=np.array([0,8,12],dtype='<i4');scans=np.array([0,5,10],dtype='<i4')
    e=np.column_stack([points,np.full((3,3),.2),np.tile(np.eye(3).reshape(1,9),(3,1))]).astype('<f4')
    T=np.eye(4);T[:3,3]=[0,0,40]
    row=dict(schema_version=5,strategy='area',robot_id='A',submap_id=0,keyframe_id=0,
        geometry_count=3,ellipsoid_count=3,archive_point_count=13,point_membership='all in area',
        member_scan_ids=[0,5,10],geometry_id_ranges=id_ranges(ids),anchor_scan_id=100,
        anchor_sensor_ns=101*10**9,stamp_ns=101*10**9,available_ns=101*10**9,
        begin_ns=10**9,last_member_ns=11*10**9,end_ns=101*10**9+1,complete=True,retrievable=True,
        age_limit_s=None,area_shape='horizontal_disk_unbounded_height',area_center_world=[0,0,40],
        area_up_world=[0,0,1],area_radius_m=10.,frame='snapshot_anchor_imu',T_world_imu=T.tolist(),
        gravity_world_m_s2=[0,0,-9.81],gravity_imu_m_s2=[0,0,-9.81],gravity_source='ellipselio_filter_at_anchor')
    return row,dict(points=points,ellipsoids=e,point_ids=ids,point_scan_ids=scans,ellipsoid_point_ids=ids.copy())


def packet(row,data):
    header=json.dumps(row).encode()
    return struct.pack('<I',len(header))+header+b''.join(data[k].tobytes() for k in
        ['points','ellipsoids','point_ids','point_scan_ids','ellipsoid_point_ids'])


def test_all_ages_heights_and_immutable_native_payload(tmp_path):
    row,data=fixture();world=validate_area(row,data)
    assert world[1,2]==-80 and world[2,2]==540
    consume(io.BytesIO(packet(row,data)+b'\0'*4),tmp_path/'area')
    with np.load(tmp_path/'area/submap-000000.npz') as saved:
        for key in data:np.testing.assert_array_equal(saved[key],data[key])
    with pytest.raises(FileExistsError):consume(io.BytesIO(b'\0'*4),tmp_path/'area')


@pytest.mark.parametrize('kind',['outside','future','wrong_ids','wrong_ellipsoid','age_limit','early_available'])
def test_rejects_invalid_area_membership(kind):
    row,data=fixture()
    if kind=='outside':data['points'][1,0]=10.1
    elif kind=='future':data['point_scan_ids'][0]=101
    elif kind=='wrong_ids':row['geometry_id_ranges']=[[0,13]]
    elif kind=='wrong_ellipsoid':data['ellipsoids'][0,1]=1
    elif kind=='age_limit':row['age_limit_s']=10
    else:row['available_ns']-=1
    with pytest.raises(ValueError):validate_area(row,data)


def test_history_detects_dropped_old_points_on_return():
    row,data=fixture();audit=AreaHistoryAudit();audit.add(row,data)
    # Empty selection while drone is far away is permitted; archive is unchanged.
    away=copy.deepcopy(row);away.update(submap_id=1,keyframe_id=1,anchor_sensor_ns=102*10**9,
        stamp_ns=102*10**9,available_ns=102*10**9,geometry_count=0,ellipsoid_count=0,
        member_scan_ids=[],geometry_id_ranges=[],retrievable=False,area_center_world=[100,0,40])
    away['T_world_imu'][0][3]=100
    empty={k:v[:0].copy() for k,v in data.items()};audit.add(away,empty)
    back=copy.deepcopy(row);back.update(submap_id=2,keyframe_id=2,anchor_sensor_ns=103*10**9,
        stamp_ns=103*10**9,available_ns=103*10**9)
    missing={k:v[1:].copy() for k,v in data.items()}
    bad=copy.deepcopy(back);bad.update(geometry_count=2,ellipsoid_count=2,
        member_scan_ids=[5,10],geometry_id_ranges=id_ranges(missing['point_ids']))
    with pytest.raises(ValueError,match='point was lost'):audit.add(bad,missing)
    audit.add(back,data)


def test_same_robot_reused_map_points_do_not_make_loops():
    row,_=fixture();q=dict(stamp_ns=200*10**9,anchor_ns=200*10**9,strategy='area',
        geometry_id_ranges=[[8,10]],begin_ns=0,end_ns=201*10**9)
    assert not eligible_observation(row,q,True,30*10**9)
    assert eligible_observation(row,q,False,30*10**9)
    q['geometry_id_ranges']=[[20,30]]
    assert eligible_observation(row,q,True,30*10**9) # Same time range is not point overlap.
    assert shared_ids([[0,5],[8,10]],[[4,8]])
    assert not shared_ids([[0,5],[8,10]],[[5,8]])


def test_unbounded_height_surface_clips_only_horizontal_region():
    # Far above/below drone: CPU fallback avoids CUDA's finite voxel-key domain.
    centers=np.array([[0,0,-300],[9.95,0,300.]])
    axes=np.full((2,3),.2);basis=np.tile(np.eye(3),(2,1,1))
    points,meta=render_area_surface(centers,axes,basis,np.eye(4),10.)
    assert len(points)>0 and np.max(np.linalg.norm(points[:,:2],axis=1))<=10
    assert np.any(points[:,2]<-290) and np.any(points[:,2]>290)
    assert meta['range_filter'] is None and meta['device']=='CPU'


def test_prepare_preserves_every_selected_map_point(tmp_path,monkeypatch):
    import s3e_pipeline.area_maps as area
    import s3e_pipeline.recent_submaps as prepare
    import yaml
    from pathlib import Path
    row,data=fixture();source=tmp_path/'source';consume(io.BytesIO(packet(row,data)+b'\0'*4),source)
    monkeypatch.setattr(area,'render_area_surface',lambda *args:(np.tile(data['points'],(20,1)),{}))
    cfg=yaml.safe_load((Path(__file__).parents[2]/'scripts/recent_submaps/area_rollout.yaml').read_text())
    out=tmp_path/'prepared';prepare.prepare(source,out,cfg)
    with np.load(out/'store/000000.npz') as prepared:np.testing.assert_array_equal(prepared['cloud'],data['points'])
    bad=copy.deepcopy(cfg);bad['backend']['evidence']['max_range_m']=80
    with pytest.raises(ValueError,match='3D range'):prepare.prepare(source,tmp_path/'bad',bad)


def test_worker_payload_and_cbs_exchange_keep_far_height_points(tmp_path):
    from types import SimpleNamespace
    from s3e_pipeline.distributed import Worker
    from s3e_pipeline.registration_exchange import RegistrationExchange
    from s3e_pipeline.backends import unpack_array
    rng=np.random.default_rng(1);points=rng.uniform(-5,5,(1000,3));points[:,2]-=120
    row=dict(robot_id='A',keyframe_id=0,strategy='area',stamp_ns=1,submap_end_ns=1,
        cloud_frame='A/imu',body_frame='A/imu',geometry_preprocessing='causal accumulated area map in snapshot IMU frame')
    worker=object.__new__(Worker);worker.seen={0};worker.store=SimpleNamespace(payload=lambda key:(row,points,b''))
    worker.backend_config=dict(evidence=dict(sampling='fixed',voxel_m=.4,max_range_m=None),
        registration=dict(sampling='fixed',max_range_m=None));worker.descriptor=lambda key:{}
    payload=worker.payload(0)
    assert len(payload['cloud'])>100 and payload['cloud'][:,2].max()<-110
    np.savez_compressed(tmp_path/'000000.npz',cloud=points)
    exchange=RegistrationExchange('A',['A'],[row],tmp_path,tmp_path/'exchange','session',
        dict(enabled=True,max_range_m=None),lambda _:None)
    cloud=unpack_array(exchange.payload(0)['cloud'])
    assert len(cloud)>100 and cloud[:,2].max()<-110
