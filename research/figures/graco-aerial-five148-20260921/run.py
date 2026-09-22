"""Fresh A04 capture plus reused A05–08; five isolated retrieval/PCM/CBS workers."""
import argparse,importlib.util,json,os,shutil,sys,time,traceback,zlib
from pathlib import Path
import numpy as np
import yaml
ROOT=Path('/workspace');BASE=ROOT/'.ros2/graco-aerial-five148-20260921';WORK=BASE/'full'
CAPTURE=ROOT/'.ros2/upstream-area-graco-aerial148-20260921'
PREV=ROOT/'.ros2/graco-aerial-height148-20260921/full'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(SCRIPTS),str(ROOT/'FAST-LIVO2-ROS2/research')]
spec=importlib.util.spec_from_file_location('original',CAPTURE/'run.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
u=original.u
from upstream_area_batch import read,save,sha,call,quality,PYTHON,RERUN
ROBOTS=['aerial05','aerial06','aerial07','aerial08','aerial04']
original.FLIGHTS={r:r for r in ROBOTS}
def sources():
    d=u.sources()
    for f in [BASE/'run.py',BASE/'env.sh']:d[str(f.relative_to(ROOT))]=sha(f)
    return d
original.sources=sources
def rows(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]
def mark(phase,**kw):
    state.update(phase=phase,updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),**kw)
    save(WORK/'status.json',state);print(phase,kw,flush=True)
def frozen_payloads():
    return {str(p.relative_to(ROOT)):sha(p) for r in ROBOTS[:-1]
            for p in sorted((CAPTURE/f'full/prepared-{r}').rglob('*')) if p.is_file()}

def prepare():
    assert read(BASE/'staging-status.json')['phase']=='complete'
    WORK.mkdir(exist_ok=False);(WORK/'configs').mkdir();(WORK/'frontends').mkdir()
    cfg=yaml.safe_load((PREV/'config.yaml').read_text())
    cfg.update(robots=ROBOTS,dataset=str(WORK/'reference'),output_root=str(WORK),
               experiment_name='GRACO aerial 04–08: five robots, persistent EllipseLIO and area MapClosures with runtime height initialization')
    cfg['dpgo']['ros_domain_id']=226
    cfg['evaluation']['expected_connected_robots']=ROBOTS
    (WORK/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    inputs=read(PREV/'inputs.json');trials=read(PREV/'trials.json')
    for r in ROBOTS[:-1]:
        (WORK/f'prepared-{r}').symlink_to(CAPTURE/f'full/prepared-{r}',target_is_directory=True)
        shutil.copy2(PREV/'configs'/f'{r}.yaml',WORK/'configs'/f'{r}.yaml')
        for suffix in ['quality.json','recording-verification.log','coverage.log','validation.log']:
            shutil.copy2(PREV/f'{r}-{suffix}',WORK/f'{r}-{suffix}')
    bag=BASE/'inputs/aerial04'
    marker=read(bag/'COMPLETE.json')
    assert all(sha(bag/n)==h for n,h in marker['files'].items())
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    inputs['aerial04']=dict(path=str(bag),sha256={p.name:sha(p) for p in bag.iterdir() if p.is_file()},
        sensor_duration_s=meta['duration']['nanoseconds']/1e9,conversion_audit=read(BASE/'aerial04-conversion-audit.json'))
    trials['aerial04']=str(WORK/'frontends/aerial04')
    mapping=yaml.safe_load((PREV/'configs/aerial05.yaml').read_text())
    mapping['/**']['ros__parameters']['mapping']['namespace']='aerial04'
    (WORK/'configs/aerial04.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
    save(WORK/'inputs.json',inputs);save(WORK/'trials.json',trials)
    shutil.copy2(PREV/'calibration-hashes.json',WORK/'calibration-hashes.json')
    frozen=sources();save(WORK/'source-hashes.json',frozen);save(BASE/'source-hashes.json',frozen)
    save(WORK/'reused-payload-hashes.json',frozen_payloads())
    old_outputs={str(p.relative_to(ROOT)):sha(p) for d in [PREV,CAPTURE/'full'] for p in
        [d/'config.yaml',d/'dpgo/poses.jsonl',d/'dpgo/constraints.jsonl',d/'report/report.json']}
    save(WORK/'original-output-hashes.json',old_outputs)
    return cfg,inputs,trials

def frontend_backend():
    cfg,inputs,trials=prepare();mark('frontend_aerial04',reused_frontends=ROBOTS[:-1],fresh_frontends=['aerial04'])
    trial=Path(trials['aerial04']);env=dict(os.environ,ROS_DOMAIN_ID='228')
    call(['/usr/bin/python3',SCRIPTS/'run_trial.py','aerial04','--robot','aerial04','--area-maps','--persistent-odometry',
          '--bag',inputs['aerial04']['path'],'--mapping-config',WORK/'configs/aerial04.yaml','--output-root',WORK/'frontends',
          '--mapper-executable',u.LAUNCHER,'--mapping-library',u.LIBRARY],WORK/'aerial04-frontend.log',inputs['aerial04']['sensor_duration_s']+300,env)
    mark('validating_aerial04')
    call([PYTHON,SCRIPTS/'validate.py',trial],WORK/'aerial04-validation.log',600,env)
    call(['/usr/bin/python3',SCRIPTS/'sensor_coverage.py',trial],WORK/'aerial04-coverage.log',120,env)
    q=quality(trial);save(WORK/'aerial04-quality.json',q)
    assert q['full_completion'] and q['finite'] and q['chronological'] and q['max_speed_m_s']<=20,q
    assert read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
    try:call([RERUN/'rerun','rrd','verify',trial/'recording/live.rrd'],WORK/'aerial04-recording-verification.log',300,env)
    except RuntimeError:
        assert 'TooManyTables' in (WORK/'aerial04-recording-verification.log').read_text()
        call([PYTHON,ROOT/'.ros2/upstream-area-s3e-20260921/partition_recording.py',trial],WORK/'aerial04-recording-partition.log',1800,env)
    qualities={r:read(WORK/f'{r}-quality.json') for r in ROBOTS}
    save(WORK/'frontend-gate.json',dict(passed=all(v['passed'] for v in qualities.values()),quality=qualities,
        diagnostic_backend=not all(v['passed'] for v in qualities.values()),disconnection_is_valid_outcome=True))
    assert sources()==read(WORK/'source-hashes.json')
    mark('descriptors_aerial04',aerial04_quality=q)
    call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',trial/'frontend/area_maps','--output',WORK/'prepared-aerial04',
          '--config',WORK/'config.yaml'],WORK/'aerial04-descriptors.log',1800)
    artifacts={}
    for r in ROBOTS:
        artifacts[f'keyframes.ellipselio.{r}']=str(WORK/f'prepared-{r}')
        artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(WORK/f'prepared-{r}/ellipsoid')
    save(WORK/'artifacts.json',artifacts);mark('distributed_pcm_cbs')
    call([PYTHON,SCRIPTS/'upstream_area_batch.py','backend','--work',WORK],WORK/'backend.log',cfg['dpgo']['timeout_s']+120)
    assert sources()==read(WORK/'source-hashes.json')
    assert frozen_payloads()==read(WORK/'reused-payload-hashes.json')
    mark('awaiting_reference_export',backend_complete=True,ground_truth_used=False)

def evaluate():
    assert state['phase']=='awaiting_reference_export'
    assert sources()==read(WORK/'source-hashes.json')
    reference=WORK/'reference';shutil.copytree(PREV/'reference',reference)
    new=read(BASE/'reference-aerial04/source-hashes.json')
    shutil.copy2(BASE/'reference-aerial04/aerial04_gt.txt',reference/'aerial04_gt.txt')
    provenance=read(reference/'source-hashes.json');provenance.update(new);save(reference/'source-hashes.json',provenance)
    for r,v in provenance.items():assert sha(reference/f'{r}_gt.txt')==v['output_sha256']
    mark('evaluation')
    call([PYTHON,SCRIPTS/'rollout_report.py','--work',WORK,'--trials',WORK/'trials.json'],WORK/'evaluation.log',1800)
    report=read(WORK/'report/report.json');report.update(reused_frontends=ROBOTS[:-1],fresh_frontends=['aerial04'],
        frontend_quality=read(WORK/'frontend-gate.json'),disconnection_is_valid_outcome=True)
    save(WORK/'report/report.json',report)
    call([RERUN/'python',SCRIPTS/'rollout_rerun.py',WORK],WORK/'rerun.log',600)
    call([RERUN/'rerun','rrd','verify',WORK/'report/result.rrd'],WORK/'recording-verification.log',300)
    mark('auditing');original.audit(WORK,read(WORK/'source-hashes.json'))
    assert frozen_payloads()==read(WORK/'reused-payload-hashes.json')
    assert all(sha(ROOT/p)==h for p,h in read(WORK/'original-output-hashes.json').items())
    mark('complete',raw_ate={r:v['rmse_m'] for r,v in report['raw'].items()},
        individual_cbs_ate={r:v['rmse_m'] for r,v in report['cbs_individual'].items()},
        shared_component_ate={r:v['rmse_m'] for r,v in report['cbs'].items()},
        connectivity=report['connectivity'],loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'])

p=argparse.ArgumentParser();p.add_argument('stage',choices=['run','evaluate']);args=p.parse_args()
state=read(WORK/'status.json') if args.stage=='evaluate' else dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
try:
    if args.stage=='run':frontend_backend()
    else:evaluate()
except Exception as e:
    failed_stage=state.get('phase','prepare');mark('failed',failed_stage=failed_stage,error=repr(e),traceback=traceback.format_exc());raise
