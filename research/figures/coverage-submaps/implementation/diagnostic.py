import json,sys,time,statistics
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[2];B=Path(__file__).resolve().parent;W=B/'smoke';S=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path.insert(0,str(S))
from graco_aerial import call,save,read,sha
started=time.monotonic();state=dict(phase='preparation',frontend_gate_passed=False,ground_truth_used=False)
save(W/'diagnostic-status.json',state)
try:
 cfg=yaml.safe_load((W/'config.yaml').read_text());trials=read(W/'trials.json');artifacts={};summary={}
 for robot,trial in trials.items():
  trial=Path(trial)
  rows=[json.loads(l) for l in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
  stamps=np.array([x['sensor_stamp_ns'] for x in rows]);poses=np.array([x['pose'] for x in rows]);success=stamps[[x['lidar_updated'] for x in rows]]
  gap=float(np.max(np.diff(np.r_[stamps[0],success,stamps[-1]]))/1e9)
  speed=float(np.max(np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1)/(np.diff([x['stamp_ns'] for x in rows])/1e9)))
  finite=bool(np.isfinite(poses).all() and np.all(np.diff(stamps)>0))
  quality=dict(native_poses=len(rows),finite_chronological=finite,max_successful_update_gap_s=gap,max_speed_m_s=speed,smoke_passed=finite and gap<1 and speed<20,full_sequence=False,ground_truth_used=False)
  save(W/f'{robot}-quality.json',quality)
  maps=read(trial/'artifact-validation.json')
  index=[json.loads(l) for l in (trial/'frontend/submaps/index.jsonl').read_text().splitlines()];completed=[r for r in index if r['complete']]
  analytics=[json.loads(l) for l in (trial/'recording/analytics.jsonl').read_text().splitlines()]
  memory=[json.loads(l) for l in (trial/'memory.jsonl').read_text().splitlines()]
  def stats(values):return dict(min=min(values),median=statistics.median(values),max=max(values))
  summary[robot]=dict(**quality,artifact_validation=maps,recorded_analytics=len(analytics),live_analytics_fraction=len(analytics)/len(rows),closure_reasons={k:sum(x['finish_reason']==k for x in completed) for k in sorted({x['finish_reason'] for x in completed})},duration_s=stats([(x['last_member_ns']-x['begin_ns'])/1e9 for x in completed]),area_m2=stats([x['occupied_area_m2'] for x in completed]),new_area_m2=stats([x['new_area_m2'] for x in completed]),overlap_ratio=stats([x['overlap_ratio'] for x in completed]),shared_area_m2=stats([x['shared_area_m2'] for x in completed]),processing_s=stats([x['processing_s'] for x in rows]),max_correspondence_age_s=max(x['age_max_s'] for x in rows if x['lidar_updated']),mapper_peak_rss_mib=max(x['VmHWM'] for x in memory)/1024,recovery_events=sum('recovery' in x['submap_event'] for x in rows),max_active_points=max(x['active_points'] for x in rows))
  output=W/f'prepared-{robot}'
  if not output.exists():
   call([sys.executable,'-m','s3e_pipeline.recent_submaps','--source',trial/'frontend/submaps','--output',output,'--config',W/'config.yaml'],W/f'{robot}-prepare.log',300)
  artifacts[f'keyframes.ellipselio.{robot}']=str(output);artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(output/'ellipsoid')
 save(W/'frontend-summary.json',summary);save(W/'artifacts.json',artifacts)
 state['phase']='backend';save(W/'diagnostic-status.json',state)
 call([sys.executable,B/'smoke.py','backend'],W/'backend.log',300)
 call([sys.executable,S/'rollout_report.py','--work',W,'--trials',W/'trials.json'],W/'report.log',300)
 call([ROOT/'.ros2/rerun-venv/bin/python',S/'rollout_rerun.py',W],W/'rerun.log',120)
 call([ROOT/'.ros2/rerun-venv/bin/rerun','rrd','verify',W/'report/result.rrd'],W/'derived-rrd.log',120)
 call([sys.executable,S/'render_spatial_bevs.py','--work',W,'--output',W/'bevs'],W/'bevs.log',300)
 frozen=read(W/'source-hashes.json');assert all(sha(Path(p))==h for p,h in frozen.items())
 state.update(phase='complete',diagnostic=True,wall_s=time.monotonic()-started,frozen_sources_verified=True);save(W/'diagnostic-status.json',state)
except Exception as error:
 import traceback
 state.update(phase='failed',error=repr(error),traceback=traceback.format_exc());save(W/'diagnostic-status.json',state);raise
