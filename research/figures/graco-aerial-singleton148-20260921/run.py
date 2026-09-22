"""Fresh five-robot runtime retrieval/verification/PCM/CBS with singleton acceptance."""
import importlib.util,json,os,shutil,sys,time,traceback
from pathlib import Path
import yaml
ROOT=Path('/workspace');BASE=ROOT/'.ros2/graco-aerial-singleton148-20260921';WORK=BASE/'full'
PREV=ROOT/'.ros2/graco-aerial-five148-20260921/full'
CAPTURE=ROOT/'.ros2/upstream-area-graco-aerial148-20260921'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(SCRIPTS),str(ROOT/'FAST-LIVO2-ROS2/research')]
spec=importlib.util.spec_from_file_location('original',CAPTURE/'run.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
u=original.u
from upstream_area_batch import read,save,sha,call,PYTHON,RERUN
ROBOTS=['aerial05','aerial06','aerial07','aerial08','aerial04']
original.FLIGHTS={r:r for r in ROBOTS}
def sources():
    d=u.sources()
    for p in [BASE/'run.py',BASE/'env.sh']:d[str(p.relative_to(ROOT))]=sha(p)
    return d
original.sources=sources
def rows(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]
def payload_hashes():
    return {str(p.relative_to(ROOT)):sha(p) for r in ROBOTS
            for p in sorted((PREV/f'prepared-{r}').rglob('*')) if p.is_file()}
state=dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),frontend_replayed=False)
def mark(phase,**kw):
    state.update(phase=phase,updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),**kw)
    save(WORK/'status.json',state);print(phase,kw,flush=True)
started=time.monotonic()
try:
    assert read(PREV/'status.json')['phase']=='complete'
    WORK.mkdir(exist_ok=False)
    cfg=yaml.safe_load((PREV/'config.yaml').read_text())
    cfg.update(dataset=str(WORK/'reference'),output_root=str(WORK),
        experiment_name='GRACO aerial 04–08: singleton PCM and two runtime verification candidates')
    cfg['dpgo']['pcm']['minimum_clique_size']=1
    cfg['loops']['branch_verification_limits']['mapclosures']=2
    cfg['dpgo']['ros_domain_id']=230
    (WORK/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    for name in ['trials.json','inputs.json','calibration-hashes.json','frontend-gate.json']:
        shutil.copy2(PREV/name,WORK/name)
    shutil.copytree(PREV/'configs',WORK/'configs')
    artifacts={}
    for r in ROBOTS:
        target=(PREV/f'prepared-{r}').resolve(strict=True)
        (WORK/f'prepared-{r}').symlink_to(os.path.relpath(target,WORK),target_is_directory=True)
        for suffix in ['quality.json','recording-verification.log','coverage.log','validation.log']:
            shutil.copy2(PREV/f'{r}-{suffix}',WORK/f'{r}-{suffix}')
        artifacts[f'keyframes.ellipselio.{r}']=str(WORK/f'prepared-{r}')
        artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(WORK/f'prepared-{r}/ellipsoid')
    save(WORK/'artifacts.json',artifacts)
    frozen=sources();save(WORK/'source-hashes.json',frozen);save(BASE/'source-hashes.json',frozen)
    immutable=payload_hashes();save(WORK/'reused-payload-hashes.json',immutable)
    previous_hashes={str(p.relative_to(ROOT)):sha(p) for p in [PREV/'config.yaml',PREV/'dpgo/poses.jsonl',
        PREV/'dpgo/constraints.jsonl',PREV/'report/report.json']}
    save(WORK/'original-output-hashes.json',previous_hashes)
    mark('distributed_pcm_cbs',pcm_minimum_clique_size=1,verification_candidates_per_query=2)
    call([PYTHON,SCRIPTS/'upstream_area_batch.py','backend','--work',WORK],WORK/'backend.log',cfg['dpgo']['timeout_s']+120)
    mark('evaluation')
    shutil.copytree(PREV/'reference',WORK/'reference')
    for r,v in read(WORK/'reference/source-hashes.json').items():assert sha(WORK/'reference'/f'{r}_gt.txt')==v['output_sha256']
    call([PYTHON,SCRIPTS/'rollout_report.py','--work',WORK,'--trials',WORK/'trials.json'],WORK/'evaluation.log',1800)
    report=read(WORK/'report/report.json');report.update(frontend_replayed=False,reused_frontends=ROBOTS,
        policy_changes=dict(pcm_minimum_clique_size=[2,1],verification_candidates_per_query=[1,2]))
    save(WORK/'report/report.json',report)
    call([RERUN/'python',SCRIPTS/'rollout_rerun.py',WORK],WORK/'rerun.log',600)
    call([RERUN/'rerun','rrd','verify',WORK/'report/result.rrd'],WORK/'recording-verification.log',300)
    mark('auditing');audit=original.audit(WORK,frozen)
    assert payload_hashes()==immutable
    assert all(sha(ROOT/p)==h for p,h in previous_hashes.items())
    previous=read(PREV/'report/report.json')
    for r in ROBOTS:
        assert report['raw'][r]['rmse_m']==previous['raw'][r]['rmse_m']
        assert sha(WORK/'report'/f'{r}-raw.tum')==sha(PREV/'report'/f'{r}-raw.tum')
    verifications=[e for r in ROBOTS for e in rows(WORK/f'dpgo/{r}/events.jsonl') if e['type']=='verification']
    a4=[e for e in verifications if 'aerial04' in (e['query'][0],e['candidate'][0])]
    assert any(e['query']==['aerial08',11] and e['candidate']==['aerial04',22] for e in a4), 'Skipped A04 candidate was not examined by runtime'
    audit.update(previous_results_unchanged=True,raw_trajectories_unchanged=True,
        reused_descriptor_and_geometry_hashes_verified=True,aerial04_runtime_verifications=a4,
        frontend_replayed=False,policy_changes=report['policy_changes'])
    save(WORK/'retention-audit.json',audit)
    mark('complete',wall_s=time.monotonic()-started,connectivity=report['connectivity'],loops=report['runtime']['loops'],
        pcm_rejected=report['pcm']['excluded_loops'],raw_ate={r:v['rmse_m'] for r,v in report['raw'].items()},
        individual_cbs_ate={r:v['rmse_m'] for r,v in report['cbs_individual'].items()},
        shared_component_ate={r:v['rmse_m'] for r,v in report['cbs'].items()})
except Exception as e:
    mark('failed',failed_stage=state.get('phase','preparing'),error=repr(e),traceback=traceback.format_exc());raise
