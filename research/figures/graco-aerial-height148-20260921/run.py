"""Fresh distributed four-robot backend with runtime vertical initialization."""
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time
import traceback
import zlib
import numpy as np
import yaml

ROOT=Path('/workspace');BASE=ROOT/'.ros2/graco-aerial-height148-20260921';WORK=BASE/'full'
OLD=ROOT/'.ros2/upstream-area-graco-aerial148-20260921';PREV=OLD/'full'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
spec=importlib.util.spec_from_file_location('original_graco',OLD/'run.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
from upstream_area_batch import read,save,sha,call,PYTHON,RERUN
from s3e_pipeline.backends import create
from s3e_pipeline.registration import bounded_cloud
from s3e_pipeline.vertical_initialization import DEFAULTS

source_base=original.sources
def sources():
    result=source_base()
    for p in [Path(__file__),BASE/'env.sh',ROOT/'FAST-LIVO2-ROS2/research/tests/test_vertical_initialization.py']:
        result[str(p.relative_to(ROOT))]=sha(p)
    return result
original.sources=sources

def rows(path):return [json.loads(l) for l in Path(path).read_text().splitlines()]
def now():return time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
state=dict(phase='preparing',started_utc=now(),frontend_replayed=False,
           frontend_source=str(PREV),backend_rerun=True,robots={})
def mark(phase,**values):
    state.update(phase=phase,updated_utc=now(),**values);save(WORK/'status.json',state)
    print(phase,values,flush=True)

def payload_hashes():
    return {str(p.relative_to(ROOT)):sha(p) for robot in cfg['robots']
            for p in sorted((PREV/f'prepared-{robot}').rglob('*')) if p.is_file()}

def prepare():
    assert read(PREV/'status.json')['phase']=='complete'
    assert 'failed' not in (BASE/'tests.log').read_text() and 'passed' in (BASE/'tests.log').read_text()
    WORK.mkdir(exist_ok=False)
    cfg=yaml.safe_load((PREV/'config.yaml').read_text())
    cfg['backend']['mapclosures']['vertical_initialization']=dict(enabled=True,**DEFAULTS)
    cfg.update(dataset=str(WORK/'reference'),output_root=str(WORK),
        experiment_name='GRACO aerial 05–08: runtime vertical initialization, accumulated-area MapClosures / PCM / CBS')
    cfg['dpgo']['ros_domain_id']=222
    (WORK/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    for name in ('trials.json','inputs.json','calibration-hashes.json','frontend-gate.json'):
        shutil.copy2(PREV/name,WORK/name)
    shutil.copytree(PREV/'configs',WORK/'configs')
    artifacts={}
    for robot in cfg['robots']:
        (WORK/f'prepared-{robot}').symlink_to(PREV/f'prepared-{robot}',target_is_directory=True)
        for suffix in ('quality.json','recording-verification.log','coverage.log','validation.log'):
            shutil.copy2(PREV/f'{robot}-{suffix}',WORK/f'{robot}-{suffix}')
        artifacts[f'keyframes.ellipselio.{robot}']=str(WORK/f'prepared-{robot}')
        artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(WORK/f'prepared-{robot}/ellipsoid')
        state['robots'][robot]=dict(status='reused_verified_capture',quality=read(PREV/f'{robot}-quality.json'))
    save(WORK/'artifacts.json',artifacts)
    return cfg

def runtime_regression():
    fixture=read(OLD/'a5-height-diagnostic/exact-evidence/summary.json')
    events={ (tuple(e['query']),tuple(e['candidate'])):e
            for f in (PREV/'dpgo').glob('*/events.jsonl') for e in rows(f) if e['type']=='verification'}
    def check(item):
        q,c=item['query'],item['candidate'];event=events[tuple(q),tuple(c)]
        def packet(endpoint):
            r,k=endpoint
            with np.load(PREV/f'prepared-{r}/store/{k:06d}.npz') as data:cloud=data['cloud'].copy()
            n=len(cloud);cloud,res=bounded_cloud(cloud,.4,max_points=None,max_range=None)
            descriptor=json.loads(zlib.decompress((PREV/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib').read_bytes()))
            return dict(cloud=cloud,descriptor=descriptor,evidence_preprocessing=dict(sampling_policy='fixed',
                effective_voxel_m=res,requested_voxel_m=.4,input_points=n,output_points=len(cloud),max_range_m=None))
        backend=create(cfg['backend'])
        try:result=backend.verify(packet(q),packet(c),dict(mapclosures_hypothesis=event['native']['hypothesis'],sources=['mapclosures']))
        finally:backend.close()
        assert result['accepted']==item['result']['accepted']
        assert result['vertical_initialization']['correction_m']==item['selected_dz_m']
        np.testing.assert_allclose(result['initial_T_i_j'],item['estimated_initializer'],atol=1e-8)
        for key in ('overlap','rmse_m'):
            assert np.isclose(result[key],item['result'][key],atol=1e-6,rtol=1e-4),(q,c,key)
        return dict(query=q,candidate=c,accepted=result['accepted'],height=result['vertical_initialization'],
                    overlap=result['overlap'],rmse_m=result['rmse_m'])
    with ThreadPoolExecutor(max_workers=4) as pool:result=list(pool.map(check,fixture['results']))
    save(BASE/'runtime-regression.json',dict(passed=True,pairs=len(result),ground_truth_used=False,results=result))

started=time.monotonic()
try:
    cfg=prepare();frozen=sources();save(BASE/'source-hashes.json',frozen);save(WORK/'source-hashes.json',frozen)
    immutable=payload_hashes();save(WORK/'reused-payload-hashes.json',immutable)
    original_outputs={str(p.relative_to(ROOT)):sha(p) for p in
        (PREV/'config.yaml',PREV/'dpgo/poses.jsonl',PREV/'dpgo/constraints.jsonl',PREV/'report/report.json')}
    save(WORK/'original-output-hashes.json',original_outputs)
    mark('runtime_regression');runtime_regression()
    assert sources()==frozen
    mark('distributed_pcm_cbs')
    call([PYTHON,SCRIPTS/'upstream_area_batch.py','backend','--work',WORK],WORK/'backend.log',cfg['dpgo']['timeout_s']+120)
    mark('evaluation')
    # Reference trajectories are first accessed after the complete backend exits.
    shutil.copytree(PREV/'reference',WORK/'reference')
    for r,v in read(WORK/'reference/source-hashes.json').items():
        assert sha(WORK/'reference'/f'{r}_gt.txt')==v['output_sha256']
    call([PYTHON,SCRIPTS/'rollout_report.py','--work',WORK,'--trials',WORK/'trials.json'],WORK/'evaluation.log',1800)
    report=read(WORK/'report/report.json')
    report.update(frontend_replayed=False,frontend_source=str(PREV),vertical_initialization_enabled=True)
    save(WORK/'report/report.json',report)
    call([RERUN/'python',SCRIPTS/'rollout_rerun.py',WORK],WORK/'rerun.log',300)
    call([RERUN/'rerun','rrd','verify',WORK/'report/result.rrd'],WORK/'recording-verification.log',300)
    mark('auditing')
    audit=original.audit(WORK,frozen)
    assert payload_hashes()==immutable
    assert all(sha(ROOT/p)==value for p,value in original_outputs.items())
    previous=read(PREV/'report/report.json')
    for r in cfg['robots']:
        assert report['raw'][r]['rmse_m']==previous['raw'][r]['rmse_m']
        assert sha(WORK/'report'/f'{r}-raw.tum')==sha(PREV/'report'/f'{r}-raw.tum')
    verifications=[e for r in cfg['robots'] for e in rows(WORK/f'dpgo/{r}/events.jsonl') if e['type']=='verification']
    assert verifications and all('vertical_initialization' in e for e in verifications)
    for e in verifications:
        d=e['vertical_initialization'];assert not d['ground_truth_used']
        assert np.allclose(np.array(e['bev_initial_T_i_j'])[:3,:3],np.array(e['initial_T_i_j'])[:3,:3])
    audit.update(frontend_replayed=False,frontend_source=str(PREV),
        reused_descriptor_and_geometry_hashes_verified=True,previous_results_unchanged=True,raw_trajectories_unchanged=True,
        runtime_vertical_initializations=len(verifications),
        source_integration='Same persistent-map captures and BEVs; runtime geometric vertical initialization added before unchanged GICP/PCM/CBS')
    save(WORK/'retention-audit.json',audit)
    mark('complete',finished_utc=now(),wall_s=time.monotonic()-started,
        raw_ate={r:v['rmse_m'] for r,v in report['raw'].items()},
        individual_cbs_ate={r:v['rmse_m'] for r,v in report['cbs_individual'].items()},
        shared_cbs_ate={r:v['rmse_m'] for r,v in report['cbs'].items()},
        connectivity=report['connectivity'],loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'])
except BaseException as exc:
    save(BASE/'failure.json',dict(stage=state['phase'],error=repr(exc),traceback=traceback.format_exc()))
    if WORK.exists():mark('failed',error=repr(exc))
    raise
