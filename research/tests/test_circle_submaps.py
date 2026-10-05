"""Sealing, independent footprints, stable membership and transport contracts."""
from pathlib import Path
import copy
import sys
import zlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import canonical, file_hash, read_jsonl, write_jsonl
from s3e_pipeline.circle_cover import CircleCoverConfig
from s3e_pipeline.circle_submaps import build, id_ranges, validate_row, validate_geometry
from s3e_pipeline.central_circle import RobotClient, communication_summary
from s3e_pipeline.distributed import eligible_observation
from s3e_pipeline.backends import unpack_payload


def observations(root):
    root.mkdir()
    # Geometry sits far from the trajectory and spans many heights.
    xyz = np.array([[120+x, 60+y, z] for x in range(10) for y in range(10) for z in (-5,20,50)],float)
    rows=[]
    for k,n in enumerate((150,300)):
        T=np.eye(4); T[:3,:3]=Rotation.from_euler('xyz',[.2,-.1,.3]).as_matrix(); T[:3,3]=[k,2*k,20]
        local=(xyz[:n]-T[:3,3])@T[:3,:3]
        np.savez_compressed(root/f'{k:06d}.npz',cloud=local.astype(np.float32))
        rows.append(dict(robot_id='r',keyframe_id=k,stamp_ns=(k+1)*10**9,available_ns=(k+1)*10**9,
            last_member_ns=(k+1)*10**9,begin_ns=0,strategy='area',frame='snapshot_anchor_imu',
            world_frame='r/odom',body_frame='r/imu',cloud_frame='r/imu',complete=True,
            gravity_world_m_s2=[0,0,-9.81],T_world_body=T.tolist(),geometry_count=n,
            geometry_id_ranges=[[0,n]],archive_point_count=300,area_radius_m=80))
    write_jsonl(root/'keyframes.jsonl',rows)
    return rows,xyz


def test_off_trajectory_membership_anchor_and_seal(tmp_path):
    rows,world=observations(tmp_path/'input')
    out=tmp_path/'circle'; result=build(tmp_path/'input',out,CircleCoverConfig(radius_m=40,coverage_target=.99),'circle')
    assert result['actual_point_coverage']==1 and result['metrics']['coverage']>=.99
    keys=read_jsonl(out/'store/keyframes.jsonl')
    assert keys[0]['footprint_center_xy'][0]>100
    for row in keys:
        assert row['available_ns']==rows[-1]['available_ns']
        assert row['stamp_ns']==rows[row['created_update']]['stamp_ns']
        with np.load(out/'store'/f"{row['keyframe_id']:06d}.npz") as f:
            T=np.asarray(row['T_world_body']); recovered=f['cloud']@T[:3,:3].T+T[:3,3]
            np.testing.assert_allclose(recovered,world[f['point_ids']],atol=3e-5)
    trajectory=build(tmp_path/'input',tmp_path/'trajectory',CircleCoverConfig(radius_m=40,coverage_target=.99),'trajectory')
    assert trajectory['actual_point_coverage']==0
    assert trajectory['source_hash']==result['source_hash']
    assert trajectory['footprints']==len(rows)


def test_membership_ranges_and_validation(tmp_path):
    assert id_ranges([1,2,7,9,10])==[[1,3],[7,8],[9,11]]
    with pytest.raises(ValueError):id_ranges([1,1])
    observations(tmp_path/'input');build(tmp_path/'input',tmp_path/'out',CircleCoverConfig(),'circle')
    row=read_jsonl(tmp_path/'out/store/keyframes.jsonl')[0]
    for change in ({'available_ns':0},{'cloud_frame':'wrong'},{'geometry_count':1},{'T_world_body':np.zeros((4,4)).tolist()}):
        with pytest.raises(ValueError):validate_row(dict(row,**change))
    with np.load(tmp_path/'out/store/000000.npz') as f:
        cloud=f['cloud'].copy();ids=f['point_ids'].copy()
    validate_geometry(row,cloud,ids)
    cloud[0]+=1000
    with pytest.raises(ValueError):validate_geometry(row,cloud,ids)


def test_shared_geometry_never_supplies_same_robot_loop():
    row=dict(stamp_ns=0,available_ns=100,strategy='fixed_spatial',geometry_id_ranges=[[1,10]])
    query=dict(stamp_ns=100,anchor_ns=100*10**9,strategy='fixed_spatial',geometry_id_ranges=[[9,12]])
    assert not eligible_observation(row,query,True,30*10**9)
    assert eligible_observation(row,query,False,30*10**9)
    query['geometry_id_ranges']=[[10,12]]
    assert eligible_observation(row,query,True,30*10**9)


def test_actual_transport_bytes_cacheable_geometry_and_shutdown(tmp_path):
    pytest.importorskip('small_gicp')
    observations(tmp_path/'input');root=tmp_path/'out';build(tmp_path/'input',root,CircleCoverConfig(),'circle')
    (root/'descriptors').mkdir()
    row=read_jsonl(root/'store/keyframes.jsonl')[0]
    (root/'descriptors/000000.json.zlib').write_bytes(zlib.compress(canonical({'payload_sha256':row['sha256']})))
    cfg={'evidence':{'voxel_m':.4}}; ledger=[];client=RobotClient(root,'r',cfg,ledger)
    before=file_hash(root/'store/000000.npz')
    try:
        assert client.call('manifest')['rows'][0]==row
        one=client.call('geometry',key=0);two=client.call('geometry',key=0)
        assert one==two and len(unpack_payload(one)['cloud'])>0
        assert file_hash(root/'store/000000.npz')==before
    finally:client.close()
    assert not client.process.is_alive()
    result=communication_summary(ledger)
    assert result['total_bytes']==sum(r['payload_bytes']+4 for r in ledger)
    assert result['by_kind']['geometry']>result['by_kind']['manifest']
