"""One fresh local ground/aerial pair; references become available after CBS."""
import json,os,shutil,sys,time,traceback,zlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import yaml
B=Path(__file__).resolve().parent;HOST=B.parents[1];ROOT=B/'source';W=ROOT/'output'
S=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(S),str(ROOT/'FAST-LIVO2-ROS2/research')]
import upstream_area_batch as u
u.ROOT=ROOT
from upstream_area_batch import read,save,sha,quality,call
from s3e_pipeline.graco import mapping_config
PY=ROOT/'.ros2/research-venv/bin/python';RR=ROOT/'.ros2/rerun-venv/bin'
LIB=ROOT/'.ros2/graco-ground-aerial-local-20260921/install/ellipselio/lib/libellipselio_mapping.so'
LAUNCHER=ROOT/'.ros2/graco-ground-aerial-local-20260921/launcher/ellipselio_mapping_mt'
ROBOTS=['ground06','aerial06']
def rows(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]
def sources():
    files=[LIB,LAUNCHER,B/'run.py',B/'env.sh',ROOT/'ellipselio/CMakeLists.txt',
           ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',
           ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node',
           ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so']
    for d in [ROOT/'ellipselio/src',ROOT/'ellipselio/include',ROOT/'ellipselio/msg',S,
              ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline',B/'native']:
        files += [p for p in d.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.h','.hpp','.yaml','.msg','.sh','.so')]
    return {str(p):sha(p) for p in sorted(set(files))}
def backend():
    from s3e_pipeline.dpgo import run
    (W/'dpgo').mkdir(exist_ok=False)
    run(yaml.safe_load((W/'config.yaml').read_text()),read(W/'artifacts.json'),ROOT,W/'dpgo')
if len(sys.argv)>1 and sys.argv[1]=='backend':backend();sys.exit()
assert (B/'BUILD_COMPLETE').exists() and (B/'staging.json').exists()
W.mkdir(exist_ok=False);started=time.monotonic();state=dict(host=os.uname().nodename,robots=ROBOTS,workstation148_used=False)
def mark(phase,**kw):
    state.update(phase=phase,updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),**kw)
    save(W/'status.json',state);print(phase,kw,flush=True)
try:
    mark('preflight')
    import s3e_mapclosures_native as native
    import s3e_mapclosures_inspection as inspection
    from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
    sampler=SurfaceSampler()
    try:sampler.render(np.array([[0.,0.,5.]]),np.array([[.2,.2,.2]]),np.eye(3)[None])
    finally:sampler.close()
    template=HOST/'FAST-LIVO2-ROS2/research/figures/graco-aerial-singleton148-20260921/full/config.yaml'
    cfg=yaml.safe_load(template.read_text())
    cfg.update(robots=ROBOTS,dataset=str(W/'reference'),output_root=str(W),
        experiment_name='GRACO ground-06 + aerial-06-20m: local updated EllipseLIO / accumulated-area BEV / PCM / CBS')
    cfg['evaluation']['expected_connected_robots']=ROBOTS
    cfg['dpgo']['ros_domain_id']=180
    cfg['backend']['read_paths']=[str(B/'native')]
    assert native.upstream_commit==inspection.upstream_commit==cfg['backend']['mapclosures']['upstream_commit']
    (W/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    save(W/'selection.json',read(B/'selection.json'))
    changed={k for k in cfg if cfg[k]!=yaml.safe_load(template.read_text())[k]}
    assert changed <= {'robots','dataset','output_root','experiment_name','evaluation','dpgo','backend'}
    old=yaml.safe_load(template.read_text())
    for k in ('registration','evidence','mapclosures'):assert cfg['backend'][k]==old['backend'][k]
    for k in ('pcm','registration_factors'):assert cfg['dpgo'][k]==old['dpgo'][k]
    save(W/'configuration-comparison.json',dict(geometric_thresholds_unchanged=True,pcm_unchanged=True,registration_factors_unchanged=True))
    (W/'configs').mkdir();(W/'frontends').mkdir()
    trials={r:str(W/'frontends'/r) for r in ROBOTS};save(W/'trials.json',trials)
    calibrations={'ground06':Path('/data/graco/ground-calibration'),
                  'aerial06':Path('/data/graco/aerial-calibration-20251121T084428Z-1-001/aerial-calibration')}
    calhash={};inputs={}
    for r in ROBOTS:
        mapping=mapping_config(ROOT,calibrations[r],r,cfg['keyframes'])
        p=mapping['/**']['ros__parameters'];p.pop('research',None)
        p['mapping']['submaps']=dict(enabled=False)
        p['mapping']['area_maps']=dict(yaml.safe_load((S/'area_maps.yaml').read_text()),odometry=False)
        p['publish']=dict(map=True,scan=True,markers=True,odometry=True,analytics=True,tf=True)
        (W/'configs'/f'{r}.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
        calhash[r]={str(path):sha(path) for path in calibrations[r].glob('*.yaml') if path.name!='groundtruth.yaml'}
        bag=B/'inputs'/r;marker=read(bag/'COMPLETE.json')
        assert all(sha(bag/n)==v for n,v in marker['files'].items())
        meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        inputs[r]=dict(path=str(bag),sha256=marker['files'],sensor_duration_s=meta['duration']['nanoseconds']/1e9)
    save(W/'calibration-hashes.json',calhash);save(W/'inputs.json',inputs)
    frozen=sources();save(W/'source-hashes.json',frozen)
    mark('frontends',replay_rate=1.0,concurrent_robots=2)
    def frontend(item):
        i,r=item;env=dict(os.environ,ROS_DOMAIN_ID=str(182+2*i));trial=Path(trials[r])
        call(['/usr/bin/python3',S/'run_trial.py',r,'--robot',r,'--area-maps','--persistent-odometry',
              '--bag',inputs[r]['path'],'--mapping-config',W/'configs'/f'{r}.yaml','--output-root',W/'frontends',
              '--mapper-executable',LAUNCHER,'--mapping-library',LIB],W/f'{r}-frontend.log',inputs[r]['sensor_duration_s']+300,env)
        call([PY,S/'validate.py',trial],W/f'{r}-validation.log',600,env)
        call(['/usr/bin/python3',S/'sensor_coverage.py',trial],W/f'{r}-coverage.log',120,env)
        q=quality(trial);save(W/f'{r}-quality.json',q)
        assert q['passed'],q
        assert read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
        call([RR/'rerun','rrd','verify',trial/'recording/live.rrd'],W/f'{r}-recording-verification.log',300,env)
        print(r,'frontend verified',q,flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(frontend,enumerate(ROBOTS)))
    save(W/'frontend-gate.json',dict(passed=True,quality={r:read(W/f'{r}-quality.json') for r in ROBOTS}))
    assert sources()==frozen
    artifacts={};mark('descriptors')
    for r in ROBOTS:
        output=W/f'prepared-{r}'
        call([PY,'-m','s3e_pipeline.recent_submaps','--source',Path(trials[r])/'frontend/area_maps',
              '--output',output,'--config',W/'config.yaml'],W/f'{r}-descriptors.log',1800)
        artifacts[f'keyframes.ellipselio.{r}']=str(output)
        artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(output/'ellipsoid')
    save(W/'artifacts.json',artifacts);mark('distributed_pcm_cbs')
    call([PY,B/'run.py','backend'],W/'backend.log',cfg['dpgo']['timeout_s']+120)
    mark('evaluation',backend_complete=True)
    # Original supplied IMU-frame positions are read only after optimization.
    reference=W/'reference';reference.mkdir()
    truth={'ground06':Path('/data/graco/ground-06.txt'),'aerial06':Path('/data/graco/aerial-06-20m.txt')}
    reference_hashes={}
    for r,path in truth.items():
        shutil.copy2(path,reference/f'{r}_gt.txt')
        reference_hashes[r]=dict(source=str(path),sha256=sha(path),evaluation_only=True,backend_completed_before_reference_access=True)
    save(reference/'source-hashes.json',reference_hashes)
    call([PY,S/'rollout_report.py','--work',W,'--trials',W/'trials.json'],W/'evaluation.log',1800)
    call([RR/'python',S/'rollout_rerun.py',W],W/'rerun.log',600)
    call([RR/'rerun','rrd','verify',W/'report/result.rrd'],W/'recording-verification.log',300)
    mark('auditing');keys={};memberships=0;causal=0
    for r in ROBOTS:
        area={x['submap_id']:x for x in rows(Path(trials[r])/'frontend/area_maps/index.jsonl') if x['complete'] and x['retrievable']}
        prepared=rows(W/f'prepared-{r}/store/keyframes.jsonl');assert len(area)==len(prepared)
        for key in prepared:
            native_row=area[key['submap_id']]
            assert all(key[k]==v for k,v in native_row.items() if k!='keyframe_id')
            packet=json.loads(zlib.decompress((W/f"prepared-{r}/ellipsoid/{key['keyframe_id']:06d}.json.zlib").read_bytes()))
            assert packet['member_scan_ids']==key['member_scan_ids'] and packet['payload_sha256']==key['sha256']
            keys[r,key['keyframe_id']]=key;memberships+=1
    for r in ROBOTS:
        for e in rows(W/f'dpgo/{r}/events.jsonl'):
            if e['type']!='ranked':continue
            assert e['query_stamp_ns']==keys[tuple(e['query'])]['available_ns']<=e['delivery_ns']
            assert all(keys[e['candidate_robot'],x['keyframe_id']]['available_ns']<=e['delivery_ns'] for x in e['candidates'])
            causal+=1
    graph=rows(W/'dpgo/poses.jsonl');assert len(graph)==len(keys)
    for n in graph:assert n['stamp_ns']==keys[n['robot_id'],n['keyframe_id']]['stamp_ns'] and np.isfinite(n['T_world_body']).all()
    assert sources()==frozen
    for r in ROBOTS:assert all(sha(Path(inputs[r]['path'])/p)==h for p,h in inputs[r]['sha256'].items())
    save(W/'retention-audit.json',dict(frozen_sources_verified=True,input_bags_unchanged=True,
        descriptor_evidence_memberships=memberships,causal_ranked_events=causal,graph_anchors_verified=True,
        derived_rerun_verified=True,live_rerun_verified=True,derived_rerun_sha256=sha(W/'report/result.rrd'),
        bulk_geometry_retained=True,ground_truth_estimation=False,selection=read(B/'selection.json')))
    result=read(W/'report/report.json')
    mark('complete',wall_s=time.monotonic()-started,connectivity=result['connectivity'],loops=result['runtime']['loops'],
         pcm_rejected=result['pcm']['excluded_loops'],raw_ate={r:v['rmse_m'] for r,v in result['raw'].items()},
         cbs_individual_ate={r:v['rmse_m'] for r,v in result['cbs_individual'].items()},
         cbs_shared_ate={r:v['rmse_m'] for r,v in result['cbs'].items()})
except BaseException as error:
    mark('failed',failed_stage=state.get('phase'),error=repr(error),traceback=traceback.format_exc());raise
