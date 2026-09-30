"""Growing-prefix CBS+ with real asynchronous DDS and optional live GPU factors."""
import os
import signal
from pathlib import Path
import time
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.artifacts import read_json, write_jsonl
from s3e_pipeline.geometry import constraint, inv, transform
from s3e_pipeline.persistent_cbs import PersistentSession, parameters, save_snapshot


def test_persistent_parameter_contract():
    cfg={'dpgo':{'pcm':{'enabled':True}}}
    p=parameters(cfg)
    assert p['loop_rate']==1. and p['belief_stage_switch_strategy']=='random'
    assert p['persistent'] and not p['synchronize_optimization_rounds']
    for override in ({'persistent':False},{'enable_soft_reset':True},{'registration_enabled':False}):
        cfg['dpgo']['cbs_parameters']=override
        with pytest.raises(ValueError): parameters(cfg)


def fixture(root, gpu):
    from test_pipeline import scene
    robots=['Alpha','Bob','Carol']; rows={}; stores={}; truth={}; loops=[]
    for index,r in enumerate(robots):
        stores[r]=root/r/'store'; stores[r].mkdir(parents=True); rows[r]=[]
        gauge=np.eye(4); gauge[:3,:3]=Rotation.from_euler('xyz',[.2*index,-.1*index,.7*index]).as_matrix()
        gauge[:3,3]=[7*index,-12*index,3*index]
        for k in range(6):
            T=np.eye(4); T[:3,:3]=Rotation.from_euler('z',.04*k).as_matrix(); T[:3,3]=[k,2*index+np.sin(k),.2*index]
            truth[r,k]=T
            rows[r].append(dict(robot_id=r,keyframe_id=k,stamp_ns=1700000000000000000+k*10**9,
                T_world_body=(inv(gauge)@T).tolist(),cloud_frame=r+'/imu',body_frame=r+'/imu',
                submap_end_ns=1700000000000000000+k*10**9,
                geometry_preprocessing='causal trailing submap in keyframe IMU frame'))
            if gpu: np.savez_compressed(stores[r]/f'{k:06d}.npz',cloud=transform(inv(T),scene()))
        write_jsonl(stores[r]/'keyframes.jsonl',rows[r])
    for a,b in [('Alpha','Bob'),('Bob','Carol'),('Alpha','Carol')]:
        for k in (0,3,5):
            loops.append(constraint([a,k],[b,k],inv(truth[a,k])@truth[b,k],np.eye(6)*100,kind='loop',accepted=True))
    cfg=dict(robots=robots,backend={'registration':{'method':'point_gicp'}},
        pgo=dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15),
        dpgo=dict(pcm=dict(enabled=True,minimum_clique_size=1,probability=.99,timeout_s=20.),
            persistent_loop_rate_hz=5.,persistent_settle_iterations=40,timeout_s=60.,ros_domain_id=169,
            registration_factors=dict(enabled=gpu,factor='vgicp_gpu' if gpu else 'gicp',
                                      num_threads=1,voxel_m=.01,max_points=4000)))
    return cfg,stores,rows,loops,truth


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1',reason='Requires isolated persistent CBS overlay')
@pytest.mark.parametrize('gpu',[False,True])
def test_growing_native_session(tmp_path,gpu):
    cfg,stores,rows,loops,truth=fixture(tmp_path,gpu)
    source=Path(__file__).resolve().parents[3]
    session=PersistentSession(cfg,source,tmp_path/'session',stores)
    histories=[]; snapshots=[]
    try:
        deadline=time.monotonic()+30
        while not session.ready():
            assert time.monotonic()<deadline
            time.sleep(.1)
        pids=[p.pid for _,p in session.processes]
        # Add odometry first; connect robots later; grow the graph; then repeat
        # unchanged evidence to prove bootstrap histories/uploads are retained.
        for count,selected,final in [(2,[],False),(2,[e for e in loops if e['i'][1]==0],False),
                                     (4,[e for e in loops if e['i'][1]<4],False),
                                     (6,loops,False),(6,loops,True)]:
            session.submit({r:rows[r][:count] for r in cfg['robots']},selected,final)
            snap=None
            # Let independent stages operate between arrivals, not only at EOF.
            start=time.monotonic()
            while time.monotonic()-start<2 or snap is None or (final and not snap['summary']['finished']):
                snap=session.snapshot(); time.sleep(.1)
                assert time.monotonic()-start<65
            histories.append(snap['summary']['iterations']); snapshots.append(snap)
            assert [p.pid for _,p in session.processes]==pids
            assert len(snap['poses'])==3*count
            if not gpu and session.revision == 3:
                # A suspended peer cannot answer services. Its neighbors must
                # keep optimizing with retained beliefs instead of blocking.
                peer=next(p for name,p in session.processes if name=='Carol-cbs')
                before=snap['summary']['iterations']['Alpha']
                os.killpg(peer.pid,signal.SIGSTOP)
                try:
                    stop=time.monotonic()+1.5
                    while time.monotonic()<stop:
                        delayed=session.snapshot(); time.sleep(.1)
                    assert delayed['summary']['iterations']['Alpha']>=before+2
                finally: os.killpg(peer.pid,signal.SIGCONT)
        save_snapshot(snap,tmp_path/'final')
        assert all(histories[i+1][r]>histories[i][r] for r in cfg['robots'] for i in range(4))
        assert snap['summary']['loops']==9 and snap['summary']['finished']
        for row in snap['poses']:
            expected=inv(truth['Alpha',0])@truth[row['robot_id'],row['keyframe_id']]
            error=inv(expected)@np.asarray(row['T_world_body'])
            assert np.linalg.norm(error[:3,3])<.01
            assert Rotation.from_matrix(error[:3,:3]).magnitude()<.005
        if gpu:
            before=snapshots[-2]['summary']; after=snapshots[-1]['summary']
            for r in cfg['robots']:
                a=before['robots'][r]['registration']; b=after['robots'][r]['registration']
                assert a['cached_endpoints']==b['cached_endpoints']
                assert a['preparation_s']<=b['preparation_s']
                assert a['registration_factor_count']==b['registration_factor_count']
                transport=read_json(session.output/r/'registration/5/transport.json')
                assert not transport['requested']
                assert all(pair['final_quality_reason']=='accepted' for pair in b['pairs'])
    finally: session.close()


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1',reason='Requires isolated persistent CBS overlay')
def test_pcm_revocation_stops_instead_of_retaining_stale_evidence(tmp_path):
    cfg,stores,rows,loops,truth=fixture(tmp_path,False)
    session=PersistentSession(cfg,Path(__file__).resolve().parents[3],tmp_path/'session',stores)
    bad=next(e.copy() for e in loops if e['i']==['Alpha',0] and e['j']==['Bob',0])
    T=np.asarray(bad['T_i_j']).copy(); T[0,3]+=30; bad['T_i_j']=T.tolist()
    try:
        deadline=time.monotonic()+30
        while not session.ready():
            assert time.monotonic()<deadline; time.sleep(.1)
        session.submit({r:rows[r][:2] for r in cfg['robots']},[bad])
        while session.snapshot() is None: time.sleep(.1)
        good=[e for e in loops if e['i'][0]=='Alpha' and e['j'][0]=='Bob' and e['i'][1]>0]
        session.submit(rows,[bad,*good])
        with pytest.raises(RuntimeError,match='cbs exited'):
            while True: session.snapshot(); time.sleep(.1)
        logs='\n'.join(p.read_text() for p in session.output.glob('*-cbs.log'))
        assert 'cannot retract an admitted factor' in logs
    finally: session.close()


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1',reason='Requires isolated persistent CBS overlay')
def test_online_capture_adapter_keeps_one_session(tmp_path):
    import threading
    from s3e_pipeline.online_epochs import run
    from s3e_pipeline.online_io import atomic_json
    cfg,stores,rows,loops,truth=fixture(tmp_path/'input',True)
    cfg['dpgo'].update(execution='persistent',update_interval_s=.2)
    work=tmp_path/'work'; work.mkdir(); (work/'live').mkdir()
    for r in cfg['robots']:
        (work/f'prepared-{r}').symlink_to(stores[r].parent,target_is_directory=True)
        (work/'live'/r).mkdir()
        for row in rows[r]: row['available_ns']=time.time_ns()
        write_jsonl(stores[r]/'keyframes.jsonl',rows[r][:2])
        atomic_json(work/'live'/r/'constraints.json',dict(constraints=[],available_wall_ns=time.time_ns()))
    errors=[]
    def feed():
        try:
            for count in (4,6):
                time.sleep(2.)
                selected=[e for e in loops if e['i'][1]<count]
                for r in cfg['robots']:
                    temp=stores[r]/'next.jsonl'; write_jsonl(temp,rows[r][:count]); temp.replace(stores[r]/'keyframes.jsonl')
                    incident=[e for e in selected if r in (e['i'][0],e['j'][0])]
                    atomic_json(work/'live'/r/'constraints.json',dict(constraints=incident,available_wall_ns=time.time_ns()))
            time.sleep(2.)
            for r in cfg['robots']: (work/'live'/r/'DONE').touch()
        except BaseException as error: errors.append(error)
    thread=threading.Thread(target=feed); thread.start()
    try: run(work,Path(__file__).resolve().parents[3],cfg)
    finally: thread.join()
    assert not errors
    done=read_json(work/'epochs/DONE'); assert done['mode']=='persistent' and done['revisions']>=3
    latest=read_json(work/'epochs/latest.json'); summary=read_json(Path(latest['path'])/'summary.json')
    assert summary['finished'] and summary['keyframes']==18 and summary['loops']==9
    assert summary['shared_reference']
    sessions=list((work/'epochs').rglob('session.json')); assert len(sessions)==1
    assert len(read_json(sessions[0])['processes'])==6
