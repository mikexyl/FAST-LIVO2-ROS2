"""Synthetic all-14 runtime transport/PCM/CBS test; no dataset or GT inputs."""
import sys,json,zlib,copy,time
from pathlib import Path
import numpy as np,yaml
ROOT=Path('/workspace');B=Path(__file__).resolve().parent;W=B/'smoke14-v2'
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.artifacts import canonical,write_jsonl,write_json,read_json,read_jsonl
from s3e_pipeline.backends import pack_array
from s3e_pipeline.multilayer_mapclosures import NAMES,VERSION
from s3e_pipeline.dpgo import run
W.mkdir(exist_ok=False)
robots=[f'aerial{i:02}' for i in range(1,9)]+[f'ground{i:02}' for i in range(1,7)]
cfg=yaml.safe_load((ROOT/'.ros2/graco-aerial-singleton148-20260921/full/config.yaml').read_text())
cfg.update(robots=robots,output_root=str(W),dataset=str(W/'no-reference'))
cfg['backend']['mapclosures']['multilayer']={'enabled':True}
cfg['backend']['read_paths']=['/opt/upstream-area-mapclosures']
cfg['dpgo'].update(ros_domain_id=170,timeout_s=1200)
(W/'config.yaml').write_text(yaml.safe_dump(cfg))
rng=np.random.default_rng(12);cloud=[]
for axis in range(3):
 p=rng.uniform(-5,5,(1600,3));p[:,axis]=0;cloud.append(p)
cloud=np.vstack(cloud).astype(np.float32)
f=dict(ground=np.eye(4),xy=rng.uniform(-80,80,(15,2)),bits=rng.integers(0,256,(15,32),dtype=np.uint8))
fp={k:pack_array(v) for k,v in f.items()};artifacts={}
for i,r in enumerate(robots):
 out=W/f'prepared-{r}';store=out/'store';d=out/'ellipsoid';store.mkdir(parents=True);d.mkdir()
 rows=[]
 for k in range(2):
  stamp=int((1+10*k+.1*i)*1e9)
  row=dict(robot_id=r,keyframe_id=k,submap_id=k,stamp_ns=stamp,available_ns=stamp+10_000_000,
    T_world_body=np.eye(4).tolist(),image_available=False,member_scan_ids=[k],sha256='fixture',
    submap_start_ns=stamp-100_000_000,submap_end_ns=stamp,
    cloud_frame=r+'/imu',body_frame=r+'/imu',geometry_preprocessing='causal accumulated area map in snapshot IMU frame',
    strategy='area',geometry_id_ranges=[[0,len(cloud)-1]])
  rows.append(row);np.savez_compressed(store/f'{k:06d}.npz',cloud=cloud,scan=cloud)
  desc=dict(mapclosures=fp,mapclosures_features=len(f['xy']),representation='ellipsoid',
    submap_id=k,member_scan_ids=[k],payload_sha256='fixture',
    projection_alignment=dict(projection='orthographic gravity-horizontal'),
    mapclosures_multilayer=dict(version=VERSION,terrain=dict(available=True),layers={n:fp for n in NAMES}))
  (d/f'{k:06d}.json.zlib').write_bytes(zlib.compress(canonical(desc)))
 write_jsonl(store/'keyframes.jsonl',rows)
 artifacts[f'keyframes.ellipselio.{r}']=str(out);artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(d)
(W/'dpgo').mkdir();started=time.monotonic();run(cfg,artifacts,ROOT,W/'dpgo')
s=read_json(W/'dpgo/summary.json');poses=read_jsonl(W/'dpgo/poses.jsonl')
assert len(poses)==28 and {p['robot_id'] for p in poses}==set(robots)
assert s['loops']>0
assert all(np.isfinite(p['T_world_body']).all() for p in poses)
assert len({p['component'] for p in poses})==1
assert all(np.allclose(p['T_world_body'],np.eye(4),atol=1e-3) for p in poses)
write_json(W/'verified.json',dict(passed=True,robots=14,poses=28,loops=s['loops'],pcm=read_json(W/'dpgo/pcm.json'),
 registration_factor_count=read_json(W/'dpgo/registration.json')['registration_factor_count'],wall_s=time.monotonic()-started))
print('All fourteen synthetic workers completed with one component and correct poses',flush=True)
