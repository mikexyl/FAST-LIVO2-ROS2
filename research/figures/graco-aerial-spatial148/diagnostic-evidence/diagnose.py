"""Frozen-run checks. No GT, no retrieval changes, no writes to experiment outputs."""
from pathlib import Path
import json,sys,time,hashlib
from collections import Counter
import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
ROOT=Path('/workspace');B=ROOT/'.ros2/graco-aerial-spatial148-20260920';W=B/'full';OUT=B/'diagnostic-evidence'
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.registration import bounded_cloud,refine
from s3e_pipeline.geometry import transform

def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def save(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def summary(a):return dict(min=float(np.min(a)),median=float(np.median(a)),max=float(np.max(a)))
frozen=read(W/'source-hashes.json');assert all(sha(Path(p))==h for p,h in frozen.items())
cfg=yaml.safe_load((W/'config.yaml').read_text())['backend'];reg=cfg['registration'];ev=cfg['evidence']
started=time.monotonic();clouds={};packed={};keys={};stats=[]
for robot in ('aerial05','aerial06','aerial07','aerial08'):
 for key in rows(W/f'prepared-{robot}/store/keyframes.jsonl'):
  identity=(robot,key['keyframe_id']);keys[identity]=key
  with np.load(W/f'prepared-{robot}/store'/f'{identity[1]:06d}.npz') as z:cloud=z['cloud'].copy()
  coarse,res=bounded_cloud(cloud,ev['voxel_m'],ev['max_points'],80.)
  clouds[identity]=cloud;packed[identity]=coarse
  stats.append(dict(robot=robot,key=identity[1],member_scans=len(key['member_scan_ids']),native_points=key['geometry_count'],ellipsoids=key['ellipsoid_count'],stored_points=len(cloud),evidence_points=len(coarse),effective_evidence_voxel_m=res,extent_m=key['extent_m']))
save(OUT/'point-counts.json',stats)
print('Point counts', {r:dict(stored=summary([x['stored_points'] for x in stats if x['robot']==r]),evidence=summary([x['evidence_points'] for x in stats if x['robot']==r]),voxel=summary([x['effective_evidence_voxel_m'] for x in stats if x['robot']==r])) for r in ('aerial05','aerial06','aerial07','aerial08')},flush=True)

def assess(t,s,T):
 d=cKDTree(t).query(transform(T,s),workers=4)[0];back=cKDTree(transform(T,s)).query(t,workers=4)[0];m=d<reg['inlier_m']
 return dict(rmse_m=float(np.sqrt(np.mean(d[m]**2))),overlap=min(float(m.mean()),float((back<reg['inlier_m']).mean())),median_distance_m=float(np.median(d)),source_points=len(s),target_points=len(t))

def slim(r):return {k:r[k] for k in ('accepted','reason','rmse_m','overlap','inliers','initial_overlap','converged','iterations','target_points','source_points','target_voxel_m','source_voxel_m','T_i_j') if k in r}

results=[]
for robot in ('aerial05','aerial06','aerial07','aerial08'):
 for event in rows(W/f'dpgo/{robot}/events.jsonl'):
  if event['type']!='verification':continue
  q,c=tuple(event['query']),tuple(event['candidate']);initial=np.array(event['initial_T_i_j'])
  coarse=refine(packed[q],packed[c],initial,reg)
  assert coarse['reason']==event['reason'] and np.isclose(coarse['rmse_m'],event['rmse_m'],atol=1e-8), (q,c,'replay mismatch')
  dense_cfg=dict(reg,max_points=10000000)
  dense=refine(clouds[q],clouds[c],initial,dense_cfg)
  tq,tv=bounded_cloud(clouds[q],.4,10000000,80.);sc,sv=bounded_cloud(clouds[c],.4,10000000,80.)
  result=dict(query=q,candidate=c,frozen_result_reproduced=True,original=slim(coarse),fixed_04m_geometry=slim(dense),
              frozen_transform_on_dense=assess(tq,sc,np.array(coarse['T_i_j'])),dense_transform_on_coarse=assess(packed[q],packed[c],np.array(dense.get('T_i_j',initial))))
  results.append(result);save(OUT/'replays.json',results)
  print(q,c,'coarse',coarse['reason'],round(coarse['rmse_m'],3),'fixed .4m',dense['reason'],round(dense.get('rmse_m',0),3),flush=True)
# A positive control: identical physical points expressed in different local frames.
original=clouds['aerial07',0];original=original[np.linalg.norm(original,axis=1)<65]
T=np.eye(4);T[:3,:3]=Rotation.from_euler('zyx',[27,5,-3],degrees=True).as_matrix();T[:3,3]=[7,-4,3]
source=transform(np.linalg.inv(T),original)
t,vt=bounded_cloud(original,ev['voxel_m'],ev['max_points'],80.);s,vs=bounded_cloud(source,ev['voxel_m'],ev['max_points'],80.)
control=dict(description='Identical physical cloud transformed into another frame; known exact transform; no GT',original_points=len(original),evidence_voxel_m=[vt,vs],at_exact_transform=assess(t,s,T),
             coarse_refine=slim(refine(t,s,T,reg)),fixed_04m_refine=slim(refine(original,source,T,dict(reg,max_points=10000000))))
save(OUT/'identical-geometry-control.json',control)
# Isolate the 80 m range filter from voxelization for the first submap of each flight.
from s3e_pipeline.geometry import voxel_downsample
range_stats=[]
for robot in ('aerial05','aerial06','aerial07','aerial08'):
 key=keys[robot,0];p=W/'frontends'/robot/'frontend/submaps'/key['payload'];assert sha(p)==key['sha256']
 with np.load(p) as z:pts=z['points'].copy()
 voxel=voxel_downsample(pts,.25);mask=np.linalg.norm(voxel,axis=1)<=80
 range_stats.append(dict(robot=robot,key=0,native_member_points=len(pts),voxel_025_before_range=len(voxel),retained_within_80m=int(mask.sum()),range_retained_fraction=float(mask.mean())))
save(OUT/'range-filter.json',range_stats)
assert all(sha(Path(p))==h for p,h in frozen.items())
save(OUT/'summary.json',dict(complete=True,frozen_sources_verified=True,ground_truth_used=False,original_experiment_unchanged=True,original_replays=len(results),dense_accepted=sum(r['fixed_04m_geometry']['accepted'] for r in results),wall_s=time.monotonic()-started,script_sha256=sha(Path(__file__))))
print('COMPLETE',read(OUT/'summary.json'),flush=True)
