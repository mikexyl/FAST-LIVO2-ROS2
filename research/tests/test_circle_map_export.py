from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import read_json, read_jsonl, write_json, write_jsonl
from s3e_pipeline.circle_map_export import export, DTYPE


def inputs(tmp_path, disconnected=False):
    run=tmp_path/'run';(run/'central').mkdir(parents=True)
    cfg=dict(robots=['a','b'],pgo=dict(alignment_translation_m=2.,alignment_rotation_deg=15.))
    poses=[]
    for robot in cfg['robots']:
        (run/robot/'store').mkdir(parents=True)
        T=np.eye(4); shifted=T.copy();shifted[0,3]=10 if robot=='b' else 0
        row=dict(keyframe_id=0,robot_id=robot,T_world_body=T.tolist(),T_horizontal_map=T.tolist(),
            footprint_center_xy=[0.,0.],footprint_radius_m=1.,gravity_imu_m_s2=[0.,0.,-9.81])
        write_jsonl(run/robot/'store/keyframes.jsonl',[row])
        write_json(run/robot/'placement.json',dict(points=2))
        np.savez_compressed(run/robot/'union.npz',world=[[0.,0.,0.],[2.,0.,3.]],point_ids=[12,14])
        poses.append(dict(robot_id=robot,keyframe_id=0,T_world_body=shifted.tolist()))
    write_json(run/'central/graph.json',dict(poses=poses,components={'a':'a','b':'b' if disconnected else 'a'},
        published_loop_residuals=[]))
    return run,cfg


def test_ply_keeps_every_point_once_and_flags_uncovered(tmp_path):
    run,cfg=inputs(tmp_path); result=export(run,cfg,tmp_path/'out')
    assert not result['ground_truth_used'] and len(result['files'])==1
    path=Path(result['files'][0]['path']);raw=path.read_bytes();offset=raw.index(b'end_header\n')+len(b'end_header\n')
    points=np.frombuffer(raw[offset:],dtype=DTYPE)
    assert len(points)==4 and points['robot_id'].tolist()==[0,0,1,1]
    assert points['uncovered'].tolist()==[0,1,0,1]
    np.testing.assert_allclose(points['x'],[0,2,10,12])
    assert result['files'][0]['points']==4


def test_disconnected_components_are_separate_files(tmp_path):
    run,cfg=inputs(tmp_path,True);result=export(run,cfg,tmp_path/'out')
    assert len(result['files'])==2 and all(len(f['robots'])==1 for f in result['files'])


def test_one_native_gravity_rotation_preserves_joint_geometry(tmp_path):
    run,cfg=inputs(tmp_path)
    path=run/'a/store/keyframes.jsonl';rows=read_jsonl(path)
    rows[0]['gravity_imu_m_s2']=[-9.81,0.,0.];write_jsonl(path,rows)
    result=export(run,cfg,tmp_path/'out');data=Path(result['files'][0]['path']).read_bytes()
    offset=data.index(b'end_header\n')+len(b'end_header\n');points=np.frombuffer(data[offset:],dtype=DTYPE)
    # Native +X is up: both robots' corrected X coordinates become height.
    np.testing.assert_allclose(points['z'],[0,2,10,12])
    np.testing.assert_allclose(points['y'],[0,3,0,3])
    assert not result['ground_truth_used']


def test_inconsistent_published_transforms_refuse_joint_export(tmp_path):
    run,cfg=inputs(tmp_path);graph=read_json(run/'central/graph.json')
    graph['published_loop_residuals']=[dict(translation_m=200.,rotation_deg=0.)]
    write_json(run/'central/graph.json',graph)
    with pytest.raises(ValueError,match='inconsistent'): export(run,cfg,tmp_path/'out')
    assert not list((tmp_path/'out').glob('*.ply'))


def test_detection_only_cannot_call_centralized_optimizer(tmp_path,monkeypatch):
    from s3e_pipeline import central_circle as module
    class Service:
        def __init__(self,*args): pass
        def call(self,op):
            assert op=='manifest'
            return dict(rows=[dict(robot_id='a',keyframe_id=0,retrievable=False)],descriptors={})
        def close(self): pass
    monkeypatch.setattr(module,'RobotClient',Service)
    monkeypatch.setattr(module,'create',lambda *args:Service())
    monkeypatch.setattr(module,'optimize',lambda *args:pytest.fail('Centralized initialization forbidden'))
    monkeypatch.setattr(module,'refine_graph',lambda *args:pytest.fail('Centralized refinement forbidden'))
    result=module.centralized({'a':tmp_path},{'backend':{}},tmp_path,tmp_path/'detection',solve_graph=False)
    assert result['proposed_loops']==0 and (tmp_path/'detection/constraints.jsonl').exists()
