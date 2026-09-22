#!/usr/bin/env python3
"""Native-state audit; all accuracy calculations are delegated to evo 1.36.5."""
import argparse, json, math, re, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
workspace=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(workspace/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.evo_evaluation import evaluate
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--truth',type=Path,required=True);a=p.parse_args()
trial=a.trial;out=trial/'evaluation';out.mkdir(exist_ok=True)
summary=json.loads((trial/'frontend/summary.json').read_text())
# C++ streams spell NaN as nan/-nan. Preserve it for finite diagnostics, never silently discard poses.
rows=[json.loads(re.sub(r'(?<![A-Za-z])[-+]?nan', 'NaN', s, flags=re.I)) for s in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
stamp=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
sensor=np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)
pose=np.array([r['pose'] for r in rows]);finite=bool(np.isfinite(pose).all())
chronological=bool(np.all(np.diff(stamp)>0) and np.all(np.diff(sensor)>0))
reported=np.array([r['lidar_updated'] for r in rows],bool)
evidence=np.array([r['num_feats']>0 and math.isfinite(r['residual']) for r in rows],bool)
success=reported&evidence&np.isfinite(pose).all(axis=1)
gaps=np.diff(np.r_[sensor[0],sensor[success],sensor[-1]])*1e-9
speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(stamp)*1e-9)
processing=np.array([r['processing_s'] for r in rows])
metadata=__import__('yaml').safe_load((trial/'bag-metadata.yaml').read_text())['rosbag2_bagfile_information']
start=metadata['starting_time']['nanoseconds_since_epoch'];end=start+metadata['duration']['nanoseconds']
metrics=dict(sequence='S3E Library 2 / Bob',upstream_commit='6506f46f1947b4ef86cfba402f11f10a6ef520ee',
 full_completion=bool(summary['success'] and summary['requested_duration']==0),
 finite=finite,chronological=chronological,native_poses=len(rows),
 first_sensor_stamp_ns=int(sensor[0]),last_sensor_stamp_ns=int(sensor[-1]),
 trajectory_span_s=float((stamp[-1]-stamp[0])*1e-9),final_sensor_to_bag_end_s=float((end-sensor[-1])*1e-9),
 successful_lidar_updates=int(success.sum()),native_reported_successes=int(reported.sum()),
 reported_success_without_valid_final_features=int((reported&~evidence).sum()),
 failed_updates_after_initialization=int((~success[1:]).sum()),
 max_successful_update_gap_s=float(gaps.max()),max_speed_m_s=float(speed.max()),
 first_speed_violation_s=next((float((stamp[i+1]-stamp[0])*1e-9) for i,v in enumerate(speed) if v>20),None),
 processing_mean_ms=float(processing.mean()*1000),processing_median_ms=float(np.median(processing)*1000),
 processing_p95_ms=float(np.quantile(processing,.95)*1000),processing_max_ms=float(processing.max()*1000),
 wall_s=summary['wall_s'],map_points_final=rows[-1]['map_size'],
 update_success_policy='Upstream iEKF returned success, positive final feature count, finite final residual and pose',
 pose_policy='Every native completed scan state, before ROS publication; no subsampling or pose filtering',
 numerical_changes='None: upstream estimator, thresholds, and -Ofast flags retained; isolated logging and reliable ROS input adapter only',
 native_summary=summary)
memory=[json.loads(s) for s in (trial/'memory.jsonl').read_text().splitlines()]
metrics['peak_rss_mib']=max((r.get('VmHWM',r.get('VmRSS',0)) for r in memory),default=0)/1024
rmse=None
if finite and chronological:
    graph=[]
    with (out/'native-trajectory.tum').open('w') as f:
        for r in rows:
            ns=r['stamp_ns'];sec,rem=divmod(ns,10**9)
            f.write(f'{sec}.{rem:09d} '+' '.join(f'{v:.17g}' for v in r['pose'])+'\n')
            T=np.eye(4);T[:3,:3]=Rotation.from_quat(r['pose'][3:]).as_matrix();T[:3,3]=r['pose'][:3]
            graph.append(dict(robot_id='Bob',component='Bob',stamp_ns=ns,T_world_body=T.tolist()))
    truth=np.loadtxt(a.truth)
    results=evaluate(dict(poses=graph),{'Bob':(np.rint(truth[:,0]*1e9).astype(np.int64),truth[:,1:4])},dict(evo_max_diff_s=.05),out/'evo')
    rmse=results.get('Bob',{}).get('rmse_m')
metrics.update(ate_rmse_m=rmse,evo_version='1.36.5',association_s=.05,scale_fitting=False)
bounds_path=trial.parent/'input-bounds.json'
if bounds_path.exists():
    bounds=json.loads(bounds_path.read_text())
    last_imu=bounds['/Bob/imu/data']['last_header_stamp_ns']
    metrics['input_bounds']=bounds
    metrics['final_sensor_to_last_imu_s']=(last_imu-int(sensor[-1]))*1e-9
    metrics['full_completion']=bool(metrics['full_completion'] and abs(metrics['final_sensor_to_last_imu_s'])<.3)
    if success.any():
        tail=max(0,(last_imu-int(sensor[success][-1]))*1e-9)
        metrics['last_success_to_input_end_s']=tail
        # Partial smoke trials are intentionally stopped before input end.
        if summary['requested_duration']==0:
            metrics['max_successful_update_gap_s']=max(metrics['max_successful_update_gap_s'],tail)
metrics['gate_pass']=bool(metrics['full_completion'] and finite and chronological and metrics['max_speed_m_s']<=20 and metrics['max_successful_update_gap_s']<1 and rmse is not None and rmse<=5)
(out/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:v for k,v in metrics.items() if k!='native_summary'},indent=2))
