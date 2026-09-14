"""Save native evo raw-odometry diagnostics and a report figure before cleanup."""
from pathlib import Path
import copy,sys
import numpy as np
import yaml
from evo.core import metrics,sync
from evo.core.trajectory import PoseTrajectory3D
from evo.main_ape import ape
from evo.tools import file_interface
from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,file_hash
from s3e_pipeline.report_figures import plt,configure,COLORS
registry=read_json(Path(sys.argv[1]));cfg=registry['config'];output=Path('FAST-LIVO2-ROS2/research/figures/cbs-library1');output.mkdir(exist_ok=True)
start=yaml.safe_load((Path(cfg['dataset'])/'metadata.yaml').read_text())['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']/1e9
configure();fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained');result={}
for i,robot in enumerate(cfg['robots']):
 stage=Path(registry['artifacts'][f'odometry.livo.{robot}']);rows=read_jsonl(stage/'run/export/frames.jsonl');dest=output/'raw-odometry'/robot;dest.mkdir(parents=True,exist_ok=True)
 estimate=PoseTrajectory3D(poses_se3=[np.array(r['T_world_body']).reshape(4,4) for r in rows],timestamps=[r['stamp_ns']/1e9 for r in rows]);raw=copy.deepcopy(estimate)
 reference=file_interface.read_tum_trajectory_file(Path(cfg['dataset'])/f'{robot.lower()}_gt.txt');reference,estimate=sync.associate_trajectories(reference,estimate,max_diff=.05)
 before=copy.deepcopy(estimate);score=ape(reference,estimate,metrics.PoseRelation.translation_part,align=True,correct_scale=False)
 file_interface.save_res_file(dest/'ape.zip',score)
 for name,track in [('raw',raw),('matched-estimate',before),('matched-reference',reference),('aligned-estimate',estimate)]:file_interface.write_tum_trajectory_file(dest/f'{name}.tum',track)
 origin=reference.positions_xyz[0];ref=reference.positions_xyz-origin;est=estimate.positions_xyz-origin
 gap=np.flatnonzero(np.diff(reference.timestamps)>2)+1
 ax=axes[0,i];ax.plot(*np.insert(ref[:,:2],gap,np.nan,axis=0).T,'--',color='.3',label='Position GT');ax.plot(*np.insert(est[:,:2],gap,np.nan,axis=0).T,color=COLORS[robot],label='FAST-LIVO2');ax.scatter(*est[0,:2],color=COLORS[robot]);ax.axis('equal');ax.grid(alpha=.25);ax.set_title(f'{robot}: raw odometry\nevo ATE {score.stats["rmse"]:.3f} m');ax.set_xlabel('East from first matched GT [m]');ax.set_ylabel('North [m]')
 ax=axes[1,i];times=np.insert(reference.timestamps-start,gap,np.nan)
 ax.plot(times,np.insert(np.linalg.norm(ref,axis=1),gap,np.nan),'--',color='.3',label='Position GT');ax.plot(times,np.insert(np.linalg.norm(before.positions_xyz-before.positions_xyz[0],axis=1),gap,np.nan),color=COLORS[robot],label='FAST-LIVO2');ax.set_xlabel('Time from bag start [s]');ax.set_ylabel('Displacement from first matched pose [m]');ax.grid(alpha=.25);ax.legend()
 result[robot]=dict(samples=estimate.num_poses,statistics=score.stats,summary=read_json(stage/'run/summary.json'),stage_hash=read_json(stage/'COMPLETE.json')['stage_hash'],mapping_config_sha256=file_hash(stage/'run/mapping_config.yaml'))
fig.suptitle('Library 1: raw odometry diagnostics; independent evo SE(3) alignment per robot, no scale fitting')
for suffix in ['png','pdf']:fig.savefig(output/f'raw_odometry_diagnostic.{suffix}',dpi=300)
plt.close(fig)
write_json(output/'raw_odometry.json',dict(engine='evo 1.36.5',association_max_diff_s=.05,scale_fitting=False,alignment='independent SE(3) alignment per robot; diagnostic only, not directly comparable to CBS shared alignment',orientation_ground_truth_used=False,robots=result,files={str(p.relative_to(output)):file_hash(p) for p in (output/'raw-odometry').rglob('*') if p.is_file()}))
print({r:d['statistics']['rmse'] for r,d in result.items()},flush=True)
