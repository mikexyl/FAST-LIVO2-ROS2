from pathlib import Path
import multiprocessing as mp
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import *
from s3e_pipeline.geometry import *
from s3e_pipeline.registration import refine
from s3e_pipeline.pgo import optimize
from s3e_pipeline.isolation import restrict_reads
from s3e_pipeline.evaluation import gt_position

import yaml
CFG=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/square1-mapclosures.yaml').read_text())


def T(x=0,y=0,z=0,yaw=0):
    out=np.eye(4);out[:3,3]=[x,y,z];out[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();return out

def test_stage_cache_atomic_and_tamper(tmp_path):
    with pytest.raises(RuntimeError):
        with stage_output(tmp_path,'pgo',{}, {},'code') as (work,_):
            write_json(work/'data.json',{'x':1});raise RuntimeError()
    assert not list(tmp_path.rglob('COMPLETE.json'))
    with stage_output(tmp_path,'pgo',{}, {},'code') as (work,cached):
        assert not cached;write_json(work/'data.json',{'x':2})
    final=stage_path(tmp_path,'pgo',{}, {},'code');before=(final/'data.json').stat().st_mtime_ns
    with stage_output(tmp_path,'pgo',{}, {},'code',True) as (work,cached):
        assert cached and work==final
    assert before==(final/'data.json').stat().st_mtime_ns
    write_json(final/'data.json',{'x':3})
    with pytest.raises(ValueError):validate_stage(final)

def test_source_snapshot_survives_edits_and_detects_tampering(tmp_path):
    from s3e_pipeline.snapshot import freeze
    source=tmp_path/'source';source.mkdir();(source/'worker.py').write_text('version = 1\n')
    first=freeze(source,tmp_path/'snapshots')
    assert freeze(source,tmp_path/'snapshots')==first
    (source/'worker.py').write_text('version = 2\n')
    second=freeze(source,tmp_path/'snapshots')
    assert first!=second and (first/'worker.py').read_text()=='version = 1\n'
    (second/'worker.py').write_text('modified\n')
    with pytest.raises(ValueError):freeze(source,tmp_path/'snapshots')

def test_endpoint_direction_and_information():
    a=T(3,4,2,.4);b=T(-2,1,3,-.2)
    edge=constraint(['A',0],['B',0],inv(a)@b,np.diag([2,3,4,5,6,7]))
    reversed=reverse_constraint(edge)
    assert np.allclose(a@pose(edge['T_i_j']),b)
    assert np.allclose(pose(reversed['T_i_j']),inv(b)@a)
    assert np.allclose(reverse_constraint(reversed)['information'],edge['information'])
    with pytest.raises(ValueError):constraint(['A',0],['A',0],a,np.eye(6))
    with pytest.raises(ValueError):constraint(['A',0],['B',0],a,np.zeros((6,6)))

def scene():
    rng=np.random.default_rng(12)
    a=rng.uniform(-5,5,(1000,3));a[:,0]=0
    b=rng.uniform(-5,5,(1000,3));b[:,1]=0
    c=rng.uniform(-5,5,(1000,3));c[:,2]=0
    return np.vstack([a,b,c])

def test_known_transform_and_low_overlap():
    target=scene();truth=T(.4,-.3,.2,.13);source=transform(inv(truth),target)
    r=refine(target,source,T(.35,-.25,.17,.12),CFG['backend']['registration'])
    assert r['accepted'],r
    assert np.linalg.norm(pose(r['T_i_j'])[:3,3]-truth[:3,3])<.08
    assert Rotation.from_matrix(pose(r['T_i_j'])[:3,:3].T@truth[:3,:3]).magnitude()<.02
    r=refine(target,source+100,np.eye(4),CFG['backend']['registration'])
    assert not r['accepted']

def test_plane_degeneracy():
    rng=np.random.default_rng(1);p=rng.uniform(-5,5,(2000,3));p[:,2]=0
    r=refine(p,p,np.eye(4),CFG['backend']['registration'])
    assert not r['accepted'] and r['reason']=='unobservable'

def test_registration_cloud_budget_and_initial_rejection():
    from s3e_pipeline.registration import bounded_cloud
    rng=np.random.default_rng(8);cloud=rng.uniform(-40,40,(20000,3))
    cloud=np.vstack([cloud,[np.nan,0,0],[1e6,1e6,1e6]])
    points,voxel=bounded_cloud(cloud,.4,1000,80)
    assert len(points)<=1000 and voxel>=.4 and np.isfinite(points).all()
    assert np.array_equal(points,bounded_cloud(cloud,.4,1000,80)[0])
    result=refine(scene(),scene()+[50,0,0],np.eye(4),CFG['backend']['registration'])
    assert not result['accepted'] and result['reason']=='initial_low_overlap'

def graph_fixture(false_loop=False):
    rows=[];edges=[];align=T(11,-4,1,.8)
    for robot in ['Alpha','Bob','Carol']:
        for k in range(6):
            world=T(k,k%2,0,.05*k)
            local=inv(align)@world if robot=='Bob' else world
            rows.append(dict(robot_id=robot,keyframe_id=k,stamp_ns=(k+1)*10**9,T_world_body=local.tolist()))
    for k in [0,2,4]:
        edges.append(constraint(['Alpha',k],['Bob',k],np.eye(4),np.eye(6)*10,accepted=True,kind='loop'))
    if false_loop:edges.append(constraint(['Alpha',5],['Bob',5],T(50,30,20,1.3),np.eye(6)*10,accepted=True,kind='loop'))
    return rows,edges

def test_unknown_alignment_disconnected_and_false_loop():
    rows,edges=graph_fixture(True)
    r=optimize(rows,edges,CFG['pgo'])
    assert r['components']=={'Alpha':'Alpha','Bob':'Alpha','Carol':'Carol'}
    assert sum(f['kind']=='anchor' for f in r['factors'])==2
    poses={(p['robot_id'],p['keyframe_id']):pose(p['T_world_body']) for p in r['poses']}
    assert np.linalg.norm(poses['Alpha',2][:3,3]-poses['Bob',2][:3,3])<.01
    false=[f for f in r['factors'] if f['kind']=='loop' and f['i'][1]==5][0]
    assert false['robust_weight']<.01


def test_anchors_rebuilt_when_components_merge():
    rows,edges=graph_fixture()
    disconnected=optimize(rows,[],CFG['pgo'])
    assert sum(f['kind']=='anchor' for f in disconnected['factors'])==3
    merged=optimize(rows,edges+edges[:1],CFG['pgo'])
    assert sum(f['kind']=='anchor' for f in merged['factors'])==2
    assert any(r['reason']=='duplicate_constraint' for r in merged['rejected'])

def test_anchors_rebuilt_after_gnc_rejects_every_bridge():
    rows=[dict(robot_id=r,keyframe_id=k,stamp_ns=k+1,T_world_body=T(k).tolist())
        for r in ['Alpha','Bob'] for k in range(3)]
    edges=[constraint(['Alpha',0],['Bob',0],T(),np.eye(6)*1e6,accepted=True,kind='loop'),
           constraint(['Alpha',2],['Bob',2],T(2),np.eye(6)*1e6,accepted=True,kind='loop')]
    cfg=dict(CFG['pgo'],odometry_rotation_sigma_deg=1e-6,odometry_translation_sigma_m=1e-6,alignment_translation_m=3)
    result=optimize(rows,edges,cfg)
    assert result['initial_components']['Alpha']==result['initial_components']['Bob']
    assert result['components']=={'Alpha':'Alpha','Bob':'Bob'}
    assert sum(f['kind']=='anchor' for f in result['factors'])==2
    loops=[f for f in result['factors'] if f['kind']=='loop']
    assert all(f['robust_weight']<.5 and f['solver_weight']==0 for f in loops)
    assert all(not f['residual_has_shared_gauge'] for f in loops)
    assert all('T_gnc_body' in row for row in result['poses'])

def isolation_child(conn,own,remote):
    try:
        restrict_reads(['/usr','/lib','/lib64','/etc','/dev','/proc',sys.prefix,own])
        Path(own,'own.txt').read_text()
        try:Path(remote,'secret.txt').read_text()
        except PermissionError:
            import subprocess
            child_result=subprocess.check_output([sys.executable,'-c',
                'import pathlib,sys; pathlib.Path(sys.argv[1],"own.txt").read_text()\n'
                'try: pathlib.Path(sys.argv[2],"secret.txt").read_text()\n'
                'except PermissionError: print("denied")\n'
                'else: print("leaked")',own,remote],text=True).strip()
            conn.send(child_result)
        else:conn.send('leaked')
    except Exception as exc:conn.send(repr(exc))

def test_kernel_worker_isolation(tmp_path):
    own=tmp_path/'Alpha';remote=tmp_path/'Bob';own.mkdir();remote.mkdir()
    (own/'own.txt').write_text('ok');(remote/'secret.txt').write_text('no')
    parent,child=mp.get_context('spawn').Pipe();proc=mp.get_context('spawn').Process(target=isolation_child,args=(child,str(own),str(remote)))
    proc.start();child.close();assert parent.poll(10);assert parent.recv()=='denied';proc.join(10);assert proc.exitcode==0

def test_shared_rigid_alignment_no_scale():
    a=scene()[::100];truth=T(12,14,-1,.7);b=transform(truth,a)
    assert np.allclose(rigid_align(a,b),truth)
    fit=rigid_align(a,2*b);assert abs(np.linalg.det(fit[:3,:3])-1)<1e-6
    track=(np.array([10**18,10**18+10**9]),np.array([[0.,0,0],[1,0,0]]))
    assert np.allclose(gt_position(track,10**18+500000000,2*10**9),[.5,0,0])
    assert gt_position(track,10**18-1,2*10**9) is None

def test_calibration_body_camera_change_of_basis():
    B=T(.2,-.1,.4,.2);C=T(-.4,.1,.2,-.3);world=T(10,3,2,.7)
    lidar=np.array([[3.,4.,5.],[-1.,2.,8.]])
    body=transform(B,lidar);camera=transform(C,lidar)
    assert np.allclose(transform(C@inv(B),body),camera)
    assert np.allclose(transform(world,body),transform(world@B,lidar))
    changed=transform(inv(T(.7,-.3,.2,.12)),body)
    assert np.allclose(transform(C@inv(B)@T(.7,-.3,.2,.12),changed),camera)
