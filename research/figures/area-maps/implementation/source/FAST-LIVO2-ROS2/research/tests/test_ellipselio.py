"""Image-free geometry and explicit frontend/descriptor identity."""
from pathlib import Path
import sys
import numpy as np
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline import data
from s3e_pipeline.frontends import frontend_name
from s3e_pipeline.backends import create,pack_array


def test_image_free_keyframes_transform_lidar_to_body_and_preserve_causality(tmp_path,monkeypatch):
    rows=[]
    for i in range(4):
        T=np.eye(4);T[0,3]=i
        row=dict(robot_id='Alpha',frame_id=i,stamp_ns=10**18+i*10**9,
            T_world_body=T.tolist(),body_frame='Alpha/imu',cloud_frame='Alpha/lidar',
            image_available=False,calibration=dict(R_body_lidar=np.eye(3).ravel().tolist(),t_body_lidar=[.2,0,0]))
        rows.append((row,np.array([[3.,4.,5.,10.]],dtype=np.float32),b''))
    monkeypatch.setattr(data,'frames',lambda _:iter(rows))
    out=tmp_path/'keys'
    cfg=dict(translation_m=.5,rotation_deg=10,max_interval_s=2,submap_s=1,voxel_m=.1)
    result=data.keyframes(None,out,cfg,None)
    assert len(result)==4 and not list(out.glob('*.png'))
    assert all(r['camera'] is None and r['submap_end_ns']==r['stamp_ns'] for r in result)
    assert result[-1]['submap_start_ns']==rows[2][0]['stamp_ns']
    row,cloud,image=data.LocalStore(out,'Alpha').payload(3)
    assert image==b'' and sorted(cloud[:,0])==pytest.approx([2.2,3.2])
    with np.load(out/'000003.npz') as saved:assert saved['scan'][0,:3]==pytest.approx([3.2,4,5])


def test_frontend_namespace_and_visual_descriptor_rejection():
    assert frontend_name({})=='livo'
    assert frontend_name(dict(odometry=dict(frontend='ellipselio')))=='ellipselio'
    with pytest.raises(ValueError):frontend_name(dict(odometry=dict(frontend='unknown')))
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    assert 'visual' not in cfg and cfg['loops']['branch_verification_limits']=={'mapclosures':1}
    backend=create(cfg['backend'])
    with pytest.raises(ValueError,match='Visual descriptors'):
        backend.decode(dict(visual=pack_array(np.array([1.])),mapclosures={}))


def test_evo_accepts_flat_native_pose_export_without_changing_alignment():
    from s3e_pipeline.evo_evaluation import evaluate
    rows=[];xyz=np.array([[0,0,0],[1,0,0],[1,2,0],[0,2,1]],dtype=float)
    stamps=10**18+np.arange(4)*10**9
    for stamp,p in zip(stamps,xyz):
        T=np.eye(4);T[:3,3]=p+[10,20,30]
        rows.append(dict(robot_id='Alpha',component='Alpha',stamp_ns=int(stamp),T_world_body=T.ravel().tolist()))
    result=evaluate(dict(poses=rows),dict(Alpha=(stamps,xyz)),{})
    assert result['Alpha']['samples']==4 and result['Alpha']['rmse_m']<1e-10
    assert result['Alpha']['scale']==1.
