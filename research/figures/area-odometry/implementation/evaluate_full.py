"""Ground truth is read only after the full sensor-only capture has finished."""
import json
from pathlib import Path
import sys
import numpy as np

root=Path(__file__).resolve().parents[2];base=Path(__file__).resolve().parent;work=base/'full'
sys.path[:0]=[str(root/'FAST-LIVO2-ROS2/scripts/recent_submaps'),str(root/'FAST-LIVO2-ROS2/research')]
from rollout_report import native_rows
from graco_aerial import sha
from s3e_pipeline.evaluation import save_tum,trajectory_metrics
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth

state=json.loads((work/'status.json').read_text());assert state['phase']=='evaluation'
cfg=dict(ground_truth_format='graco_imu_enu',evo_max_diff_s=.05,
         trajectory_limitation='GRACO RTK/INS T_Base_Imu positions in base-station ENU; GT orientations unused; original timestamps; rigid alignment without scale')
rows=native_rows(work/'aerial08','aerial08')
reference=root/'.ros2/graco-aerial-temporal-20260920/reference'
gt,status=evaluation_ground_truth(['aerial08'],reference,rows,cfg)
result=trajectory_metrics(dict(poses=rows),gt,cfg,work/'evo')
assert 'aerial08' in result
save_tum(work/'aerial08-raw.tum',rows)
(work/'evaluation.json').write_text(json.dumps(dict(metrics=result,ground_truth=status,
    reference_sha256=sha(reference/'aerial08_gt.txt'),config=cfg),indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
estimate=np.loadtxt(work/'evo/aerial08-estimate-aligned.tum');truth=np.loadtxt(work/'evo/aerial08-reference-matched.tum')
origin=truth[0,1:4];t=estimate[:,0]-estimate[0,0]
fig,axes=plt.subplots(1,2,figsize=(11,4.2),constrained_layout=True)
axes[0].plot(truth[:,1]-origin[0],truth[:,2]-origin[1],'k--',label='Position GT')
axes[0].plot(estimate[:,1]-origin[0],estimate[:,2]-origin[1],label='Accumulated-area odometry')
axes[0].axis('equal');axes[0].legend();axes[0].set(xlabel='ENU X relative to start (m)',ylabel='ENU Y relative to start (m)')
axes[1].plot(t,np.linalg.norm(estimate[:,1:4]-truth[:,1:4],axis=1))
axes[1].set(xlabel='Sensor time (s)',ylabel='Position error after evo rigid alignment (m)')
fig.suptitle(f"Full Aerial 08 · evo 1.36.5 position ATE {result['aerial08']['rmse_m']:.4f} m")
for ax in axes:ax.grid(alpha=.2)
fig.savefig(work/'trajectory-error.png',dpi=180)
print(json.dumps(result,indent=2))
