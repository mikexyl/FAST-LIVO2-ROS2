#!/usr/bin/env python3
"""Update the remote-results section from retained reports only."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
research=ROOT/'FAST-LIVO2-ROS2/research'
def fmt(x):return 'unavailable' if x is None else f'{x:.4f}'
def robot_error(metrics,robot):
    return next((v['per_robot'][robot]['statistics']['rmse'] for v in metrics.values()
                 if robot in v['per_robot']),None)

lines=['## Workstation 148','',
       'These tests use the user-approved multistage setup: serial EllipseLIO exports, '
       'three simultaneous native Swarm-SLAM robot instances, then evo evaluation. '
       'They are not an untouched official online launch. No fresh paired MapClosures result is claimed.', '',
       'The isolated ROS Humble build and final adapter checks passed **32 tests**. The duplicate-input smoke test processed '
       '**1,341/1,341 scans** and **285/285 keyframes/descriptors**, with no optimizer errors and '
       'maximum corresponding-position disagreement **0.0612 m**. '
       '[Smoke report](figures/swarm148-smoke/REPORT.md); '
       '[remote setup](../../Swarm-SLAM/s3e/remote/README.md).','',
       '| Sequence | Native run | Exact input audit | Shared ATE [m] | Alpha | Bob | Carol |',
       '|---|---|---|---:|---:|---:|---:|']
reports=[]
for seq,title in [('library1','Library 1'),('campusroad1','Campus Road 1')]:
    p=research/f'figures/swarm148-{seq}/report.json'
    if not p.exists():
        lines.append(f'| {title} | pending | pending | pending | pending | pending | pending |');continue
    report=json.loads(p.read_text());s=report['swarm'];metrics=s['trajectory'];reports.append((seq,title,report))
    if report.get('frontend_failure'):
        lines.append(f'| [{title}](figures/swarm148-{seq}/REPORT.md) | not run: frontend failed | unavailable | unavailable | unavailable | unavailable | unavailable |')
        continue
    combined=(next(iter(metrics.values()))['rmse_m'] if len(metrics)==1 and
              len(next(iter(metrics.values()))['robots'])==3 else None)
    lines.append(f'| [{title}](figures/swarm148-{seq}/REPORT.md) | '+
                 ('complete' if report['swarm_run']['success'] else 'incomplete')+
                 f' | {s["selected_inputs_complete"]} | {fmt(combined)} | '+
                 ' | '.join(fmt(robot_error(metrics,r)) for r in ('Alpha','Bob','Carol'))+' |')
lines+=['','Swarm ATE uses evo 1.36.5, 50 ms timestamp association, one shared rigid alignment '
        'per output component, no scale fitting, and no extra robot alignment. '
        'Supplied GT orientations are unused. Missing native robot trajectories are not replaced.', '',
        '| Sequence | Raw Alpha ATE [m] | Raw Bob | Raw Carol | Registrations accepted / inter | GT-checkable / flagged | Native time [min] |',
        '|---|---:|---:|---:|---:|---:|---:|']
for seq,title,r in reports:
    raw=r['ours']['raw_odometry'];s=r['swarm']
    if r.get('frontend_failure'):
        lines.append(f'| {title} | '+' | '.join(fmt(robot_error(raw,robot)) for robot in ('Alpha','Bob','Carol'))+' | not run | not run | not run |')
        continue
    lines.append(f'| {title} | '+' | '.join(fmt(robot_error(raw,robot)) for robot in ('Alpha','Bob','Carol'))+
                 f' | {s["accepted"]} / {s["accepted_inter"]} | {s["gt_checkable"]} / {s["flagged"]} | {r["swarm_run"].get("total_wall_s",0)/60:.2f} |')
lines+=['','Raw odometry uses independent robot fits, so its alignment scope differs from the connected Swarm result. '
        'Flagged registrations have a GT endpoint-distance discrepancy above 2 m; this is not a full 6-DoF outlier label. '
        'Accepted counts precede native GNC, whose weights are not exported. Native time includes paced replay, backpressure and settling, but excludes frontend generation.', '',
        'Reports retain PNG/PDF plots, native outputs, source provenance, evo evidence and compact verified Rerun recordings. '
        'Completed generated clouds are retired after verification; original S3E datasets remain intact. '
        'Remote workspace: `/data3/mikexyl/swarm_s3e_ws/src`. '
        'Live status: `ssh 148 python3 /data3/mikexyl/swarm_s3e_ws/src/Swarm-SLAM/s3e/remote/status.py`.']
for seq,title,r in reports:
    if r.get('frontend_failure'):
        lines += ['',f'**{title} failure:** '+r['frontend_failure']+' This occurred before Swarm-SLAM.']
        if r.get('imu_onset_audit'):
            lines += ['The recorded IMU stream had no gap over 50 ms around the onset; the frontend failure mechanism remains undiagnosed.']
path=research/'RESULTS-SWARM-SLAM.md'
prefix=path.read_text().split('## Workstation 148')[0].rstrip()
path.write_text(prefix+'\n\n'+'\n'.join(lines)+'\n')
print(path)
