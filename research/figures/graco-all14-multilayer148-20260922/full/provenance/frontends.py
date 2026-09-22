"""Fresh GRACO captures, four at a time on 148; missing sensors can be added later."""
from pathlib import Path
import sys,os,json,time,threading,traceback
from concurrent.futures import ThreadPoolExecutor
import yaml
B=Path(__file__).resolve().parent;ROOT=Path('/workspace');W=B/'full';S=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(S),str(ROOT/'FAST-LIVO2-ROS2/research')]
from upstream_area_batch import read,save,sha,call,quality
from s3e_pipeline.graco import mapping_config
ROBOTS=[f'aerial{i:02}' for i in range(1,9)]+[f'ground{i:02}' for i in range(1,7)]
PY=ROOT/'.ros2/research-venv/bin/python';RR=ROOT/'.ros2/rerun-venv/bin'
LIB=ROOT/'.ros2/upstream-area-s3e-20260921/capacity-run/install/ellipselio/lib/libellipselio_mapping.so'
LAUNCHER=ROOT/'.ros2/upstream-area-s3e-20260921/capacity-run/launcher/ellipselio_mapping_mt'
W.mkdir(exist_ok=True);(W/'configs').mkdir(exist_ok=True);(W/'frontends').mkdir(exist_ok=True)
lock=threading.RLock()
state=read(W/'frontend-status.json') if (W/'frontend-status.json').exists() else dict(phase='preparing',robots={r:dict(status='awaiting_sensors') for r in ROBOTS})
def mark(robot=None,**values):
 with lock:
  (state['robots'][robot] if robot else state).update(values);state['updated_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
  save(W/'frontend-status.json',state);print(robot,values,flush=True)
inputs=read(W/'inputs.json') if (W/'inputs.json').exists() else {}
calhash={}
for r in ROBOTS:
 n=int(r[-2:]);a=r.startswith('aerial')
 if a:
  bag=(B/'inputs'/r if n<4 else ROOT/'.ros2/graco-aerial-five148-20260921/inputs'/r if n==4 else ROOT/'.ros2/graco-aerial-four148-20260920/inputs'/r)
  calib=ROOT/'.ros2/graco-aerial-four148-20260920/calibration'
 else:
  bag=ROOT/'.ros2/graco/inputs/sensors'/f'robot{n}';calib=ROOT/'.ros2/graco/inputs/calibration'
 if not (bag/'COMPLETE.json').exists():continue
 if r not in inputs:
  mark(r,status='checking_sensor_hashes')
  marker=read(bag/'COMPLETE.json');assert all(sha(bag/k)==v for k,v in marker['files'].items())
  meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
  assert {x['topic_metadata']['name'] for x in meta['topics_with_message_count']}=={'/velodyne/points','/gnss/imu'}
  inputs[r]=dict(path=str(bag),sha256=marker['files'],sensor_duration_s=meta['duration']['nanoseconds']/1e9)
  save(W/'inputs.json',inputs)
 if not (W/'configs'/f'{r}.yaml').exists():
  cfg=mapping_config(ROOT,calib,r,dict(translation_m=1.,rotation_deg=10.,max_interval_s=2.))
  p=cfg['/**']['ros__parameters'];p.pop('research',None)
  p['mapping']['submaps']=dict(enabled=False)
  p['mapping']['area_maps']=dict(yaml.safe_load((S/'area_maps.yaml').read_text()),odometry=False)
  p['publish']=dict(map=True,scan=True,markers=True,odometry=True,analytics=True,tf=True)
  assert p['input']['reliable']
  (W/'configs'/f'{r}.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
 calhash[r]={str(p):sha(p) for p in calib.glob('*.yaml') if p.name!='groundtruth.yaml'}
 if state['robots'][r]['status']!='complete':mark(r,status='queued')
save(W/'calibration-hashes.json',calhash)
save(W/'trials.json',{r:str(W/'frontends'/r) for r in ROBOTS})
save(W/'frontend-binaries.json',{str(p):sha(p) for p in [LIB,LAUNCHER]})
mark(phase='frontends',concurrent_robots=4,replay_rate=1.,resource_quotas=False)
def run(r):
 i=ROBOTS.index(r);env=dict(os.environ,ROS_DOMAIN_ID=str(100+2*i));trial=W/'frontends'/r
 try:
  mark(r,status='recording',ros_domain_id=int(env['ROS_DOMAIN_ID']))
  call(['/usr/bin/python3',S/'run_trial.py',r,'--robot',r,'--area-maps','--persistent-odometry','--bag',inputs[r]['path'],
   '--mapping-config',W/'configs'/f'{r}.yaml','--output-root',W/'frontends','--mapper-executable',LAUNCHER,'--mapping-library',LIB],
   W/f'{r}-frontend.log',inputs[r]['sensor_duration_s']+360,env)
  mark(r,status='validating')
  call([PY,S/'validate.py',trial],W/f'{r}-validation.log',600,env)
  call(['/usr/bin/python3',S/'sensor_coverage.py',trial],W/f'{r}-coverage.log',120,env)
  q=quality(trial);save(W/f'{r}-quality.json',q)
  assert q['passed'],q
  assert read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
  call([RR/'rerun','rrd','verify',trial/'recording/live.rrd'],W/f'{r}-recording-verification.log',600,env)
  mark(r,status='complete',quality=q)
 except Exception as e:
  mark(r,status='failed',error=repr(e),traceback=traceback.format_exc())
selected=[r for r in ROBOTS if state['robots'][r]['status']=='queued']
selected.sort(key=lambda r:-inputs[r]['sensor_duration_s'])
with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(run,selected))
mark(phase='complete' if all(s['status']=='complete' for s in state['robots'].values()) else 'pending_or_failed')
