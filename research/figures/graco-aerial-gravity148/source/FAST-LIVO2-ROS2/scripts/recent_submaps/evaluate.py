#!/usr/bin/env python3
"""Evaluate frontend trials with evo; ground truth is used only for evaluation."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.evo_evaluation import evaluate


def audit(trial, truth, output, robot="Bob", gt_reference="s3e"):
    trial=Path(trial);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    summary=json.loads((trial/'frontend/summary.json').read_text())
    rows=[json.loads(line) for line in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
    stamp=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    sensor=np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)
    pose=np.array([r['pose'] for r in rows],dtype=float)
    finite=bool(np.isfinite(pose).all());chronological=bool(np.all(np.diff(stamp)>0) and np.all(np.diff(sensor)>0))
    speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(stamp)*1e-9)
    success=np.array([r['lidar_updated'] for r in rows],dtype=bool)
    # Include the interval from initialization and the final unsuccessful tail.
    gaps=np.diff(np.r_[sensor[0],sensor[success],sensor[-1]])*1e-9
    handovers=np.flatnonzero(np.diff([r['active_submap_id'] for r in rows])!=0)+1
    graph=[]
    with (output/'native-trajectory.tum').open('w') as tum:
        for row in rows:
            T=np.eye(4);T[:3,:3]=Rotation.from_quat(row['pose'][3:]).as_matrix();T[:3,3]=row['pose'][:3]
            graph.append(dict(robot_id=robot,component=robot,stamp_ns=row['stamp_ns'],T_world_body=T.tolist()))
            sec,ns=divmod(row['stamp_ns'],10**9)
            tum.write(f'{sec}.{ns:09d} '+ ' '.join(f'{v:.15g}' for v in row['pose'])+'\n')
    gt=np.loadtxt(truth)
    evaluation_config=dict(evo_max_diff_s=.05)
    if gt_reference=='graco':
        evaluation_config['trajectory_limitation']='translation APE against supplied GRACO T_Base_Imu; IMU reference and estimate; GT orientations unused'
    result=evaluate(dict(poses=graph),{robot:(np.rint(gt[:,0]*1e9).astype(np.int64),gt[:,1:4])},
                    evaluation_config,output/'evo')
    rmse=result.get(robot,{}).get('rmse_m')
    memory=[json.loads(line) for line in (trial/'memory.jsonl').read_text().splitlines()]
    metrics=dict(full_completion=summary['success'] and summary['requested_duration']==0,
                 finite=finite,chronological=chronological,native_poses=len(rows),successful_lidar_updates=int(success.sum()),
                 max_speed_m_s=float(speed.max()),max_successful_update_gap_s=float(gaps.max()),
                 ate_rmse_m=rmse,evo_version='1.36.5',association_s=.05,scale_fitting=False,
                 wall_s=summary['wall_s'],processing_mean_s=float(np.mean([r['processing_s'] for r in rows])),
                 processing_max_s=float(max(r['processing_s'] for r in rows)),
                 peak_rss_kib=max((r.get('VmHWM',r.get('VmRSS',0)) for r in memory),default=0),
                 max_correspondence_age_s=max(r['age_max_s'] for r in rows if r['lidar_updated']),
                 handovers=[dict(scan_id=int(i),stamp_ns=int(stamp[i]),step_m=float(np.linalg.norm(pose[i,:3]-pose[i-1,:3])),
                                 speed_m_s=float(speed[i-1])) for i in handovers],
                 first_speed_violation_s=next(((stamp[i+1]-stamp[0])*1e-9 for i,s in enumerate(speed) if s>20),None))
    metrics['gate_pass']=bool(metrics['full_completion'] and finite and chronological and
                             metrics['max_speed_m_s']<=20 and metrics['max_successful_update_gap_s']<1 and
                             rmse is not None and rmse<=5)
    metrics['motion_pass']=bool(finite and chronological and metrics['max_speed_m_s']<=20)
    metrics['update_continuity_pass']=bool(metrics['max_successful_update_gap_s']<1)
    metrics['accuracy_pass']=None if rmse is None else bool(rmse<=5)
    metrics['gt_reference']=gt_reference
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--robot',default='Bob');p.add_argument('--gt-reference',choices=['s3e','graco'],default='s3e');p.add_argument('trial',type=Path);p.add_argument('--truth',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(audit(args.trial,args.truth,args.output,args.robot,args.gt_reference),indent=2))
