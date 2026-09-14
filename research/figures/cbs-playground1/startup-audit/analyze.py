"""Preserve small, reproducible evidence from the bounded startup diagnosis."""
import copy
import hashlib
import json
from pathlib import Path
import re
import shutil
import sqlite3
import subprocess
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D
from evo.core import sync, metrics
from evo.main_ape import ape

source=Path('/home/mikexyl/workspaces/fast_livo2_ws/src')
repo=source/'FAST-LIVO2-ROS2'
runs=source/'.ros2/playground-start-audit'
out=repo/'research/figures/cbs-playground1/startup-audit'
out.mkdir(parents=True,exist_ok=True)
bag_start=1661163877.933
names=['Alpha','Alpha-visual-cov100','Bob','Bob-visual-cov100','Bob-before-gap-cov100']
summary={'scope':'bounded raw FAST-LIVO2 startup diagnosis, no loops or CBS',
 'evo_version':'1.36.5','association_max_diff_s':.05,'scale_fitting':False,
 'alignment':'independent SE(3) alignment per replay; prefix diagnostics, not full-sequence or multi-robot ATE',
 'ground_truth_orientation_used':False,'bag_start_s':bag_start,'runs':{},
 'mapper_source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()}
summary['source_sha256']={str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [repo/'src/LIVMapper.cpp',repo/'src/IMU_Processing.cpp',repo/'src/vio.cpp',repo/'scripts/s3e_adapter.py']}
pairs={}
with sqlite3.connect('file:/data/s3e/S3E_Playground_1/S3E_Playground_1.db3?mode=ro',uri=True) as db:
 for name in names:
  path=runs/name;dest=out/name;dest.mkdir(exist_ok=True);robot=name.split('-')[0]
  rows=[json.loads(s) for s in (path/'export/frames.jsonl').read_text().splitlines()]
  poses=np.array([r['T_world_body'] for r in rows]).reshape(-1,4,4)
  est=PoseTrajectory3D(poses_se3=poses,timestamps=np.array([r['stamp_ns'] for r in rows])/1e9)
  ref=file_interface.read_tum_trajectory_file(f'/data/s3e/S3E_Playground_1/{robot.lower()}_gt.txt')
  raw=copy.deepcopy(est);ref,est=sync.associate_trajectories(ref,est,max_diff=.05)
  file_interface.write_tum_trajectory_file(dest/'raw.tum',raw)
  file_interface.write_tum_trajectory_file(dest/'matched-estimate.tum',est)
  file_interface.write_tum_trajectory_file(dest/'matched-reference.tum',ref)
  pairs[name]=(copy.deepcopy(ref),copy.deepcopy(est))
  result=ape(ref,est,metrics.PoseRelation.translation_part,align=True,correct_scale=False)
  file_interface.save_res_file(dest/'ape.zip',result)
  file_interface.write_tum_trajectory_file(dest/'aligned-estimate.tum',est)
  for filename in ['mapping_config.yaml','camera_config.yaml','summary.json']:
   shutil.copy2(path/filename,dest/filename)
  log=(path/'mapping.log').read_text()
  features=np.array([int(x) for x in re.findall(r'\[ VIO \] Retrieve (\d+) points',log)])
  initialized=re.findall(r'IMU Initials: Gravity: ([^\n]+)',log)
  image_stamps=set()
  tid=db.execute('select id from topics where name=?',(f'/{robot}/left_camera/compressed',)).fetchone()[0]
  for ts, in db.execute('select timestamp from messages where topic_id=?',(tid,)):image_stamps.add(ts)
  image_matches=sum(r['stamp_ns'] in image_stamps for r in rows)
  mat=np.loadtxt(path/'mapper/Log/mat_out.txt')
  # Upstream records LiDAR then visual updates at the same timestamp.
  lidar=mat[mat[:,-1]>0];visual=mat[mat[:,-1]==0]
  paired=min(len(lidar),len(visual))
  assert np.allclose(lidar[:paired,0],visual[:paired,0])
  corrections=np.linalg.norm(visual[:paired,4:7]-lidar[:paired,4:7],axis=1)
  details={'robot':robot,'summary':json.loads((path/'summary.json').read_text()),
   'frames':len(rows),'matched_gt_samples':ref.num_poses,'export_image_stamp_exact_matches':image_matches,
   'first_export_offset_s':float(raw.timestamps[0]-bag_start),'last_export_offset_s':float(raw.timestamps[-1]-bag_start),
   'visual_img_point_cov':yaml.safe_load((path/'mapping_config.yaml').read_text())['/**']['ros__parameters']['vio']['img_point_cov'],
   'ate_m':result.stats,'visual_tracked_points_median':float(np.median(features)),
   'visual_tracked_points_quartiles':np.percentile(features,[0,25,50,75,100]).tolist(),
   'visual_translation_correction_median_m':float(np.median(corrections)),
   'imu_initialization_log':initialized,
   'mapping_log_sha256':hashlib.sha256((path/'mapping.log').read_bytes()).hexdigest(),
   'export_manifest_sha256':hashlib.sha256((path/'export/manifest.json').read_bytes()).hexdigest()}
  summary['runs'][name]=details
  print(name,'ATE',result.stats['rmse'],'tracked',np.median(features),'exact image stamps',image_matches,len(rows))
for robot in ['Alpha','Bob']:
 assert np.array_equal(pairs[robot][0].timestamps,pairs[robot+'-visual-cov100'][0].timestamps)
 a=yaml.safe_load((out/robot/'mapping_config.yaml').read_text())
 b=yaml.safe_load((out/(robot+'-visual-cov100')/'mapping_config.yaml').read_text())
 b['/**']['ros__parameters']['vio']['img_point_cov']=1000
 assert a==b
summary['paired_controls']='identical matched GT timestamps; only mapping parameter changed is vio.img_point_cov 1000 to 100'
for file in ['sensor_timing.json','raw_scan_registration.json']:
 shutil.copy2(runs/file,out/file)
shutil.copy2(__file__,out/'analyze.py')
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
fig,axs=plt.subplots(1,3,figsize=(14.5,4.4),layout='constrained')
for ax,group,title in zip(axs,[['Alpha','Alpha-visual-cov100'],['Bob','Bob-visual-cov100'],['Bob-before-gap-cov100']],['Alpha: original start','Bob: restart at 22 s','Bob: start before the IMU gap']):
 ref,_=pairs[group[-1]]
 ax.plot(ref.timestamps-bag_start,np.linalg.norm(ref.positions_xyz-ref.positions_xyz[0],axis=1),color='black',ls='--',lw=2,label='Position ground truth')
 for name in group:
  ref,est=pairs[name];cov=summary['runs'][name]['visual_img_point_cov'];err=summary['runs'][name]['ate_m']['rmse']
  ax.plot(est.timestamps-bag_start,np.linalg.norm(est.positions_xyz-est.positions_xyz[0],axis=1),color='#c43c39' if cov==1000 else '#148a60',lw=1.7,label=f'Variance {cov}; ATE {err:.3f} m')
 ax.set_title(title);ax.set_xlabel('Time from bag start [s]');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
axs[0].set_ylabel('Displacement from first matched pose [m]')
fig.suptitle('Playground 1: under-motion is sensitive to visual weighting and the start interval',fontsize=14)
fig.savefig(out/'startup_motion.png',dpi=240)
fig.savefig(out/'startup_motion.pdf')
plt.close(fig)
summary['files_sha256']={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file() and p.name!='diagnosis.json'}
(out/'diagnosis.json').write_text(json.dumps(summary,indent=2)+'\n')
