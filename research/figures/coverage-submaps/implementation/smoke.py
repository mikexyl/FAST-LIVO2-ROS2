import json,sys,os,time,hashlib,subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import yaml,numpy as np
ROOT=Path(__file__).resolve().parents[2];B=Path(__file__).resolve().parent;W=B/'smoke';S=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(S),str(ROOT/'FAST-LIVO2-ROS2/research')]
from graco_aerial import call,save,read,sha
if len(sys.argv)>1 and sys.argv[1]=='backend':
 from s3e_pipeline.dpgo import run
 (W/'dpgo').mkdir();run(yaml.safe_load((W/'config.yaml').read_text()),read(W/'artifacts.json'),ROOT,W/'dpgo');sys.exit()
W.mkdir(exist_ok=False);state=dict(phase='frontends',ground_truth_used=False);save(W/'status.json',state)
started=time.monotonic()
try:
 import s3e_mapclosures_native as native
 cfg=yaml.safe_load((S/'coverage_rollout.yaml').read_text());cfg.update(robots=['aerial05','aerial08'],dataset=str(W/'reference'),output_root=str(W),experiment_name='120-second coverage implementation smoke; no ground truth')
 cfg['dpgo']['ros_domain_id']=210;cfg['backend']['read_paths']=[native.__file__]
 (W/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));(W/'reference').mkdir()
 watched={p.resolve() for directory in [ROOT/'ellipselio/src',ROOT/'ellipselio/include',ROOT/'ellipselio/msg',S,ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline'] for p in directory.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.h','.hpp','.msg','.yaml','.sh')}
 watched.update([B/'install/ellipselio/lib/libellipselio_mapping.so',B/'launcher/ellipselio_mapping_mt',ROOT/'ellipselio/CMakeLists.txt',ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',Path(native.__file__)])
 frozen={str(p):sha(p) for p in sorted(watched)};save(W/'source-hashes.json',frozen)
 trials={r:str(W/'frontends'/r) for r in cfg['robots']};save(W/'trials.json',trials)
 def frontend(item):
  i,robot=item;env=dict(os.environ,ROS_DOMAIN_ID=str(212+2*i))
  call(['/usr/bin/python3',S/'run_trial.py',robot,'--robot',robot,'--submap-strategy','coverage','--duration','120','--bag',ROOT/'.ros2/graco-aerial-temporal-20260920/inputs'/robot,'--mapping-config',ROOT/'.ros2/graco-aerial-temporal-20260920/configs'/f'{robot}.yaml','--output-root',W/'frontends','--mapper-executable',B/'launcher/ellipselio_mapping_mt','--mapping-library',B/'install/ellipselio/lib/libellipselio_mapping.so'],W/f'{robot}-frontend.log',300,env)
  call([sys.executable,S/'validate.py',trials[robot]],W/f'{robot}-validation.log',300,env)
  call([ROOT/'.ros2/rerun-venv/bin/rerun','rrd','verify',Path(trials[robot])/'recording/live.rrd'],W/f'{robot}-rrd.log',120,env)
 with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(frontend,enumerate(cfg['robots'])))
 state['phase']='descriptors';save(W/'status.json',state)
 artifacts={}
 for robot,trial in trials.items():
  updates=[json.loads(l) for l in (Path(trial)/'frontend/native_updates.jsonl').read_text().splitlines()]
  stamp=np.array([r['sensor_stamp_ns'] for r in updates]);pose=np.array([r['pose'] for r in updates]);success=stamp[[r['lidar_updated'] for r in updates]]
  gap=float(np.max(np.diff(np.r_[stamp[0],success,stamp[-1]]))/1e9)
  speed=float(np.max(np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff([r['stamp_ns'] for r in updates])/1e9)))
  assert np.isfinite(pose).all() and np.all(np.diff(stamp)>0) and gap<1 and speed<20
  save(W/f'{robot}-quality.json',dict(native_poses=len(updates),finite_chronological=True,max_successful_update_gap_s=gap,max_speed_m_s=speed,smoke_passed=True,full_sequence=False,ground_truth_used=False))
  output=W/f'prepared-{robot}'
  call([sys.executable,'-m','s3e_pipeline.recent_submaps','--source',Path(trial)/'frontend/submaps','--output',output,'--config',W/'config.yaml'],W/f'{robot}-prepare.log',300)
  artifacts[f'keyframes.ellipselio.{robot}']=str(output);artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(output/'ellipsoid')
 save(W/'artifacts.json',artifacts)
 state['phase']='backend';save(W/'status.json',state)
 call([sys.executable,Path(__file__),'backend'],W/'backend.log',300)
 call([sys.executable,S/'rollout_report.py','--work',W,'--trials',W/'trials.json'],W/'report.log',300)
 call([ROOT/'.ros2/rerun-venv/bin/python',S/'rollout_rerun.py',W],W/'rerun.log',120)
 call([ROOT/'.ros2/rerun-venv/bin/rerun','rrd','verify',W/'report/result.rrd'],W/'derived-rrd.log',120)
 call([sys.executable,S/'render_spatial_bevs.py','--work',W,'--output',W/'bevs'],W/'bevs.log',300)
 assert all(sha(Path(p))==h for p,h in frozen.items())
 state.update(phase='complete',wall_s=time.monotonic()-started,frozen_sources_verified=True);save(W/'status.json',state)
except Exception as error:
 import traceback
 state.update(phase='failed',error=repr(error),traceback=traceback.format_exc());save(W/'status.json',state);raise
