"""Run the requested backend while retaining the failed frontend stability gate."""
import json
import os
from pathlib import Path
import sys
import traceback
import yaml

work=Path(__file__).resolve().parent
root=work.parents[1]
scripts=root/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path.insert(0,str(scripts))
from graco_aerial import call,read,save,sha,now,PYTHON,RERUN

state=read(work/'status.json')
assert state['phase']=='failed' and read(work/'failure.json')['stage']=='frontends'
cfg=yaml.safe_load((work/'config.yaml').read_text());trials=read(work/'trials.json')
qualities={r:read(work/f'{r}-quality.json') for r in trials}
for robot,q in qualities.items():
    assert q['full_completion'] and q['finite'] and q['chronological'] and q['max_speed_m_s']<=20
    assert read(Path(trials[robot])/'sensor-coverage.json')['full_selected_sensor_tail_reached']
    assert read(Path(trials[robot])/'artifact-validation.json')['payload_hashes_verified']
    assert 'verified without error' in (work/f'{robot}-recording-verification.log').read_text()
assert any(not q['passed'] for q in qualities.values())
frozen=read(work/'source-hashes.json')
for path,expected in frozen.items():assert sha(Path(path))==expected,path
progress=dict(started_utc=now(),phase='descriptors',diagnostic_only=True,frontend_quality=qualities,
    reason='Full finite chronological captures, but aerial08 exceeded the 1-second successful-LiDAR-update gap criterion; that failed gate is preserved. Estimator and backend parameters are unchanged.')
def mark(phase,**fields):
    progress.update(phase=phase,**fields);save(work/'diagnostic-backend-status.json',progress)
try:
    mark('descriptors');artifacts={}
    for robot,trial in trials.items():
        output=work/f'prepared-{robot}'
        call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',Path(trial)/'frontend/submaps',
              '--output',output,'--config',work/'config.yaml'],work/f'{robot}-descriptors.log',1800)
        artifacts[f'keyframes.ellipselio.{robot}']=str(output)
        artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(output/'ellipsoid')
    save(work/'artifacts.json',artifacts);mark('distributed_pcm_cbs')
    call([PYTHON,scripts/'graco_aerial.py','backend','--work',work],work/'backend.log',cfg['dpgo']['timeout_s']+120)
    mark('evaluation');(work/'reference').mkdir()
    truth={'aerial05':Path('/data/graco/aerial-05-40m.txt'),'aerial08':Path('/data/graco/aerial-08-25m.txt')}
    for robot,path in truth.items():(work/'reference'/f'{robot}_gt.txt').symlink_to(path)
    save(work/'reference/source-hashes.json',{str(p):sha(p) for p in truth.values()})
    call([PYTHON,scripts/'rollout_report.py','--work',work,'--trials',work/'trials.json'],work/'evaluation.log',1800)
    report=read(work/'report/report.json')
    report.update(diagnostic_only=True,frontend_quality=qualities,diagnostic_reason=progress['reason'])
    save(work/'report/report.json',report)
    call([RERUN/'python',scripts/'rollout_rerun.py',work],work/'rerun.log',300)
    call([RERUN/'rerun','rrd','verify',work/'report/result.rrd'],work/'recording-verification.log',300)
    for path,expected in frozen.items():assert sha(Path(path))==expected,path
    mark('complete',finished_utc=now(),raw_ate={r:m['rmse_m'] for r,m in report['raw'].items()},
         shared_cbs_ate={r:m['rmse_m'] for r,m in report['cbs'].items()},connectivity=report['connectivity'],
         loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'],frozen_sources_verified=True)
except Exception as error:
    save(work/'diagnostic-backend-failure.json',dict(stage=progress['phase'],error=repr(error),traceback=traceback.format_exc()))
    mark('failed',finished_utc=now(),error=repr(error));raise
