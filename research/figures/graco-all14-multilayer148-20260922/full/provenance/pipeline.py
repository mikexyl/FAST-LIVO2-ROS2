"""Resumable preparation and full fourteen-robot runtime; evaluate only afterwards."""
from pathlib import Path
import sys,os,json,time,threading,traceback,zlib
from concurrent.futures import ThreadPoolExecutor
import numpy as np,yaml
B=Path(__file__).resolve().parent;ROOT=Path('/workspace');W=B/'full';S=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(S),str(ROOT/'FAST-LIVO2-ROS2/research')]
from upstream_area_batch import read,save,sha,call
from s3e_pipeline.artifacts import read_jsonl
PY=ROOT/'.ros2/research-venv/bin/python';RR=ROOT/'.ros2/rerun-venv/bin'
ROBOTS=[f'aerial{i:02}' for i in range(1,9)]+[f'ground{i:02}' for i in range(1,7)]
state=read(W/'status.json') if (W/'status.json').exists() else dict(robots=ROBOTS,ground_truth_used=False)
lock=threading.RLock()
def mark(phase,**values):
 with lock:
  state.update(phase=phase,updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),**values)
  save(W/'status.json',state);print(phase,values,flush=True)
def sources():
 paths=[p for folder in [ROOT/'FAST-LIVO2-ROS2/research',S,B/'native'] for p in folder.rglob('*')
  if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h','.so','.yaml') and '__pycache__' not in str(p)]
 paths += [B/'pipeline.py',B/'frontends.py',B/'env.sh',W/'config.yaml',
  ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node',
  ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so']
 return {str(p):sha(p) for p in sorted(set(paths))}
def configure():
 if (W/'config.yaml').exists():return yaml.safe_load((W/'config.yaml').read_text())
 cfg=yaml.safe_load((ROOT/'.ros2/graco-aerial-singleton148-20260921/full/config.yaml').read_text())
 cfg.update(robots=ROBOTS,dataset=str(W/'reference'),output_root=str(W),
  experiment_name='GRACO all 14: updated persistent EllipseLIO, accumulated-area multilayer ellipsoid BEVs, PCM/CBS')
 cfg['backend']['mapclosures']['multilayer']={'enabled':True,'version':'terrain-relative-orb-v1'}
 cfg['backend']['read_paths']=['/opt/upstream-area-mapclosures']
 cfg['evaluation']['expected_connected_robots']=ROBOTS
 cfg['odometry'].pop('imu_noise',None)
 cfg['odometry']['imu_noise_source']='Per-robot configs: native GRACO ground or aerial calibration imu.yaml'
 cfg['dpgo'].update(ros_domain_id=180,timeout_s=14400)
 (W/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
 return cfg
def descriptor(r):
 output=W/f'prepared-{r}'
 if (output/'summary.json').exists():return
 mark('descriptors',current_robot=r)
 call([PY,'-m','s3e_pipeline.recent_submaps','--source',W/f'frontends/{r}/frontend/area_maps',
       '--output',output,'--config',W/'config.yaml'],W/f'{r}-descriptors.log',3600)
 summary=read(output/'summary.json');print(r,'prepared',summary,flush=True)
def audit():
 keys={};unavailable={};counts={}
 for r in ROBOTS:
  native={row['submap_id']:row for row in read_jsonl(W/f'frontends/{r}/frontend/area_maps/index.jsonl') if row['complete'] and row['retrievable']}
  rows=read_jsonl(W/f'prepared-{r}/store/keyframes.jsonl');assert len(native)==len(rows)
  unavailable[r]=0;counts[r]=len(rows)
  for row in rows:
   assert all(row[k]==v for k,v in native[row['submap_id']].items() if k!='keyframe_id')
   d=json.loads(zlib.decompress((W/f"prepared-{r}/ellipsoid/{row['keyframe_id']:06d}.json.zlib").read_bytes()))
   assert d['member_scan_ids']==row['member_scan_ids'] and d['payload_sha256']==row['sha256']
   from s3e_pipeline.multilayer_mapclosures import decode_layers
   decode_layers(d);unavailable[r]+=int(not d['mapclosures_multilayer']['terrain']['available'])
   keys[r,row['keyframe_id']]=row
 return keys,dict(descriptor_evidence_memberships=len(keys),submaps=counts,terrain_unavailable=unavailable)
def main():
 cfg=configure();mode=sys.argv[1] if len(sys.argv)>1 else 'prepare'
 if mode=='prepare':
  status=read(W/'frontend-status.json')['robots'];ready=[r for r in ROBOTS if status[r]['status']=='complete']
  with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(descriptor,ready))
  mark('prepared_available',ready_robots=ready,remaining_robots=[r for r in ROBOTS if r not in ready]);return
 if mode=='backend':
  assert (B/'smoke14-v2/verified.json').exists(),'14-worker runtime preflight has not passed'
  assert all(read(W/'frontend-status.json')['robots'][r]['status']=='complete' for r in ROBOTS)
  assert all((W/f'prepared-{r}/summary.json').exists() for r in ROBOTS)
  keys,audit_result=audit();save(W/'descriptor-audit.json',audit_result)
  artifacts={}
  for r in ROBOTS:
   artifacts[f'keyframes.ellipselio.{r}']=str(W/f'prepared-{r}')
   artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(W/f'prepared-{r}/ellipsoid')
  save(W/'artifacts.json',artifacts);save(W/'source-hashes.json',sources())
  mark('distributed_pcm_cbs');(W/'dpgo').mkdir(exist_ok=False)
  from s3e_pipeline.dpgo import run
  run(cfg,artifacts,ROOT,W/'dpgo')
  assert sources()==read(W/'source-hashes.json')
  causal=0
  for r in ROBOTS:
   for e in read_jsonl(W/f'dpgo/{r}/events.jsonl'):
    if e['type']!='ranked':continue
    assert e['query_stamp_ns']==keys[tuple(e['query'])]['available_ns']<=e['delivery_ns']
    assert all(keys[e['candidate_robot'],x['keyframe_id']]['available_ns']<=e['delivery_ns'] for x in e['candidates'])
    causal+=1
  poses=read_jsonl(W/'dpgo/poses.jsonl');assert len(poses)==len(keys)
  for p in poses:assert p['stamp_ns']==keys[p['robot_id'],p['keyframe_id']]['stamp_ns'] and np.isfinite(p['T_world_body']).all()
  audit_result.update(causal_ranked_events=causal,graph_anchors_verified=True,frozen_sources_verified=True)
  save(W/'runtime-audit.json',audit_result)
  mark('backend_complete',loops=read(W/'dpgo/summary.json')['loops']);return
 if mode=='evaluate':
  assert state['phase']=='backend_complete'
  assert all((W/f'reference/{r}_gt.txt').exists() for r in ROBOTS)
  mark('evaluation')
  call([PY,S/'rollout_report.py','--work',W,'--trials',W/'trials.json'],W/'evaluation.log',7200)
  call([RR/'python',S/'rollout_rerun.py',W],W/'rerun.log',1800)
  call([RR/'rerun','rrd','verify',W/'report/result.rrd'],W/'recording-verification.log',1800)
  q=read(W/'report/report.json');mark('complete',loops=q['runtime']['loops'],connectivity=q['connectivity'],
    raw_ate={r:v['rmse_m'] for r,v in q['raw'].items()},cbs_individual_ate={r:v['rmse_m'] for r,v in q['cbs_individual'].items()},
    cbs_shared_ate={r:v['rmse_m'] for r,v in q['cbs'].items()});return
 raise ValueError('Unknown stage')
try:main()
except Exception as e:mark('failed',error=repr(e),traceback=traceback.format_exc());raise
