import json
import numpy as np
import pytest
from s3e_pipeline.online_io import JsonlTail, atomic_json
from s3e_pipeline.data import LocalStore
from s3e_pipeline.online_epochs import prefix, Cadence, freeze_epoch
from s3e_pipeline.online_display import corrected, corrections


def test_partial_publication_and_truncation(tmp_path):
    path=tmp_path/'rows'; tail=JsonlTail(path)
    assert tail.read()==[]
    path.write_bytes(b'{"key":1}')
    assert tail.read()==[]
    with path.open('ab') as f: f.write(b'\n{"key":2}\n{"key":')
    assert tail.read()==[{'key':1},{'key':2}]
    with pytest.raises(ValueError,match='Incomplete'): tail.finish()
    path.write_bytes(b'')
    with pytest.raises(ValueError,match='truncated'): tail.read()


def test_cbs_cadence_independent_of_solver_completion():
    c=Cadence(10.,100.)
    assert c.take(100.)['missed_deadlines']==0
    assert c.take(109.9) is None
    assert c.take(110.)['missed_deadlines']==0
    assert c.take(120.)['missed_deadlines']==0  # prior solve can still be running
    assert c.take(131.)['deadline_lateness_s']==1.
    assert c.next==140.  # late poll does not shift subsequent deadlines
    assert c.take(165.)['missed_deadlines']==2
    assert c.next==170.  # no fabricated snapshots for missed deadlines
    assert c.take(166.,final=True) is not None  # final tail is flushed immediately


@pytest.mark.parametrize('interval',[0.,-1.,float('nan'),float('inf')])
def test_invalid_cbs_interval(interval):
    with pytest.raises(ValueError): Cadence(interval,0.)


def test_queued_epoch_is_immutable_and_has_only_captured_members(tmp_path):
    source=tmp_path/'prepared-A/store';source.mkdir(parents=True)
    (source/'000000.npz').write_bytes(b'immutable geometry')
    root=tmp_path/'epochs';root.mkdir()
    rows={'A':[dict(keyframe_id=0,available_ns=10)]}
    epoch=freeze_epoch(tmp_path,root,1,rows,[], 'identity',20,dict(target_interval_s=10.))
    rows['A'].append(dict(keyframe_id=1,available_ns=30))
    (source/'000001.npz').write_bytes(b'future')
    assert len((epoch/'A/store/keyframes.jsonl').read_text().splitlines())==1
    assert not (epoch/'A/store/000001.npz').exists()
    assert (epoch/'A/store/000000.npz').stat().st_ino==(source/'000000.npz').stat().st_ino


def test_stream_store_validates_identity_and_availability(tmp_path):
    path=tmp_path/'keyframes.jsonl'; path.write_text('')
    store=LocalStore(tmp_path,'A',streaming=True)
    with path.open('a') as f: f.write(json.dumps(dict(robot_id='A',keyframe_id=0,stamp_ns=999,available_ns=100))+'\n')
    assert len(store.refresh())==1 and store.row(0)['stamp_ns']==999
    with path.open('a') as f: f.write(json.dumps(dict(robot_id='A',keyframe_id=1,stamp_ns=1000,available_ns=99))+'\n')
    with pytest.raises(ValueError,match='availability'): store.refresh()


def test_epoch_excludes_future_rows_and_unreceived_loops(tmp_path):
    edge=dict(i=['A',0],j=['B',0],kind='loop')
    for r in ['A','B']:
        root=tmp_path/f'prepared-{r}/store'; root.mkdir(parents=True)
        (root/'keyframes.jsonl').write_text(''.join(json.dumps(dict(robot_id=r,keyframe_id=i,available_ns=10+i*100))+'\n' for i in range(2)))
        live=tmp_path/f'live/{r}'; live.mkdir(parents=True)
        atomic_json(live/'constraints.json',dict(available_wall_ns=20 if r=='A' else 50,constraints=[edge]))
    rows,edges=prefix(tmp_path,['A','B'],30)
    assert len(rows['A'])==1 and edges==[]
    rows,edges=prefix(tmp_path,['A','B'],60)
    assert len(rows['B'])==1 and edges==[edge]


def test_online_corrections_use_only_published_anchors():
    native=np.eye(4); native[0,3]=5
    optimized=native.copy(); optimized[1,3]=3
    rows=[dict(keyframe_id=0,stamp_ns=100,T_world_body=native.tolist()),
          dict(keyframe_id=1,stamp_ns=200,T_world_body=np.eye(4).tolist())]
    anchors,delta=corrections(rows,{0:optimized})
    points=np.array([[5,0,0],[6,0,0]])
    np.testing.assert_allclose(corrected(points,np.array([100,300]),anchors,delta),[[5,3,0],[6,3,0]])
    assert anchors.tolist()==[100] # Unoptimized/new maps use the latest published correction.


def test_epoch_retains_agreed_loop_during_reciprocal_replacement(tmp_path):
    old=dict(i=['A',0],j=['B',0],kind='loop',version=1)
    new=dict(old,version=2)
    for r in ['A','B']:
        root=tmp_path/f'prepared-{r}/store'; root.mkdir(parents=True)
        (root/'keyframes.jsonl').write_text(json.dumps(dict(robot_id=r,keyframe_id=0,available_ns=10))+'\n')
        live=tmp_path/f'live/{r}'; live.mkdir(parents=True)
        atomic_json(live/'constraints.json',dict(available_wall_ns=100,constraints=[new if r=='A' else old]))
    # A slow snapshot read crossed an owner publication. No cutoff is taken
    # until all reads finish, and asymmetric versions cannot erase the old loop.
    _,edges=prefix(tmp_path,['A','B'],previous=[old]); assert edges==[old]
    atomic_json(tmp_path/'live/B/constraints.json',dict(available_wall_ns=110,constraints=[new]))
    _,edges=prefix(tmp_path,['A','B'],previous=[old]); assert edges==[new]
