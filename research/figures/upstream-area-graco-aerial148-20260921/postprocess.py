"""Figures and a compact report from completed GRACO outputs; no new ATE calculation."""
from pathlib import Path
import json
import hashlib
import sys
from collections import Counter
import numpy as np
B=Path('/workspace/.ros2/upstream-area-graco-aerial148-20260921');W=B/'full'
p=json.loads((W/'report/report.json').read_text());a=json.loads((W/'retention-audit.json').read_text());state=json.loads((W/'status.json').read_text())
assert state['phase']=='complete'
robots=list(a['frontends']);labels={r:r.replace('aerial','Aerial ') for r in robots}
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=W/'diagnostics';D.mkdir(exist_ok=True)
fig,axs=plt.subplots(len(robots),3,figsize=(15,3*len(robots)),layout='constrained')
for i,r in enumerate(robots):
 t=Path(a['frontends'][r]['path']);rows=[json.loads(l) for l in (t/'frontend/native_updates.jsonl').read_text().splitlines()]
 stamps=np.array([x['sensor_stamp_ns'] for x in rows],dtype=np.int64);seconds=(stamps-stamps[0])/1e9
 success=np.array([x['lidar_updated'] for x in rows],bool);xyz=np.array([x['pose'][:3] for x in rows]);pose_t=np.array([x['stamp_ns'] for x in rows],dtype=np.int64)
 axs[i,0].plot(seconds,[x['processing_s']*1000 for x in rows],lw=.6,label='Total')
 axs[i,0].plot(seconds,[x['snapshot_s']*1000 for x in rows],lw=.6,label='Snapshot/export');axs[i,0].legend(fontsize=8)
 axs[i,0].set_ylabel(labels[r]+'\nProcessing [ms]')
 axs[i,1].plot(seconds[1:],np.linalg.norm(np.diff(xyz,axis=0),axis=1)/(np.diff(pose_t)*1e-9),lw=.6);axs[i,1].set_ylabel('Scan-derived speed [m/s]')
 times=seconds[success];axs[i,2].plot(times[1:],np.diff(times),lw=.6);axs[i,2].set_ylabel('Successful-update gap [s]')
 for ax in axs[i]:ax.set_xlabel('Sensor elapsed time [s]');ax.grid(alpha=.2)
fig.suptitle('GRACO aerial 05–08 · updated persistent-map EllipseLIO');fig.savefig(D/'frontend-performance.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained');x=np.arange(len(robots))
for shift,key,label,color in [(-.18,'raw','Raw','#4477aa'),(.18,'cbs_individual','CBS','#ee7733')]:
 v=[p[key][r]['rmse_m'] for r in robots];bars=ax.bar(x+shift,v,.34,label=label,color=color);ax.bar_label(bars,fmt='%.3f',padding=3)
ax.set_xticks(x,[labels[r] for r in robots]);ax.set_ylabel('Position ATE RMSE [m]');ax.set_title('Independent rigid alignment per robot · evo 1.36.5 · no scale fitting')
ax.set_ylim(0,max(p[k][r]['rmse_m'] for k in ('raw','cbs_individual') for r in robots)*1.25);ax.legend();ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.savefig(D/'ate-comparison.png',dpi=180);plt.close(fig)
shared={r:(c,v['per_robot'][r]['statistics']['rmse']) for c,v in p['cbs'].items() for r in v['per_robot']}
lines=['**Updated EllipseLIO + accumulated-area MapClosures / PCM / CBS — GRACO aerial 05–08, 21 September 2026**','',
'Four fresh full-flight captures ran concurrently at 1× on workstation 148, without CPU, memory or GPU quotas. Updated upstream EllipseLIO uses its persistent map for odometry; accumulated-area snapshots supply MapClosures and registration evidence.','',
f"All four connected: **{p['connectivity']['all_robots_connected']}**. Retained loops: **{p['pcm']['retained_loops']}**, rejected by PCM: **{p['pcm']['excluded_loops']}**. Backend wall time: **{p['runtime']['wall_s']:.2f} s**. Total capture/preparation/backend/evaluation/audit time: **{state['wall_s']:.2f} s**, excluding gallery generation.",'',
'| Flight | Raw ATE (m) | CBS, individual alignment (m) | CBS, component alignment (m) | Component | Matched poses |','|---|---:|---:|---:|---|---:|']
for r in robots:lines.append(f"| {labels[r]} | {p['raw'][r]['rmse_m']:.4f} | {p['cbs_individual'][r]['rmse_m']:.4f} | {shared[r][1]:.4f} | {shared[r][0]} | {p['raw'][r]['samples']} |")
lines+=['','For context, the earlier four-robot temporal-submap run had raw ATEs of 0.8601 / 3.5598 / 0.2034 / 0.1668 m for aerial 05 / 06 / 07 / 08. Both the upstream odometry version and backend map construction changed, so these runs do not isolate the effect of submapping.','',
'Raw and individual CBS ATE each use one independent rigid SE(3) alignment per robot. Component ATE uses one rigid alignment shared by all robots in that component. Evaluation uses evo 1.36.5, 50 ms timestamp association, no scale fitting, and supplied GRACO RTK/INS IMU-frame positions. Ground truth is accessed only after backend completion.','']
for c,v in p['cbs'].items():lines.append(f"Component `{c}` ({', '.join(v['robots'])}): **{v['rmse_m']:.4f} m** shared position ATE.")
edges=[json.loads(l) for l in (W/'dpgo/constraints.jsonl').read_text().splitlines()]
pairs=Counter(tuple(sorted((e['i'][0],e['j'][0]))) for e in edges)
lines+=['','| Retained loop pair | Count |','|---|---:|']
for pair,count in sorted(pairs.items()):lines.append(f"| {' ↔ '.join(pair)} | {count} |")
lines+=['',f"Geometric verification outcomes: `{json.dumps(p['verification_reasons'],sort_keys=True)}`.",
'Backend completion means the configured 100 local settling iterations finished after peer input completion; it does not establish mathematical convergence. A disconnected flight keeps its own coordinate frame.','']
lines+=['','| Flight | Stable | Native poses | Failed corrections after initialization | Max update gap (s) | Max speed (m/s) | Mean / p95 processing (ms) | Peak mapper RSS (MiB) | Area snapshots |','|---|---|---:|---:|---:|---:|---:|---:|---:|']
for r,f in a['frontends'].items():
 q=f['quality'];lines.append(f"| {labels[r]} | {q['passed']} | {f['native_poses']} | {f['failed_updates_after_init']} | {q['max_successful_update_gap_s']:.3f} | {q['max_speed_m_s']:.3f} | {f['processing_mean_ms']:.2f} / {f['processing_p95_ms']:.2f} | {f['peak_mapper_rss_mib']:.1f} | {f['area_snapshots']} |")
lines+=['',a['processing_timing_scope']+'. Update gaps use sensor timestamps and are distinct from processing time. Stability requires full completion, finite chronological poses, maximum speed ≤20 m/s and update gaps <1 s. Any failed update-gap gate is retained even if the complete finite capture enters the backend diagnostically.','',
'The selected area is an 80 m horizontal disk around the corrected IMU pose in the estimated gravity plane, with no height or point-age cutoff. Snapshots occur after 20 m horizontal displacement or 10 seconds, plus the final nonduplicate snapshot; these triggers do not define scan membership. Odometry has no submap handovers or area query crop. Export uses native processed map representatives and native fitted ellipsoids, not full-resolution raw scans.','',
'Gravity-horizontal ellipsoid BEVs, geometric verification and CBS registration factors all use the same immutable area snapshot and anchor frame. Retrieval uses availability timestamps; graph poses retain anchor timestamps. Same-robot candidates retain the 30 s exclusion and reject shared persistent point IDs. Existing retrieval, verification, PCM and CBS thresholds are unchanged. Four-thread launchers, reliable input, GRACO calibration and supplied IMU noise are retained.','',
'Upstream revision is `6506f46f1947b4ef86cfba402f11f10a6ef520ee`, with the already-tested persistent-map export integration and octree/CUDA capacity fixes from the S3E experiment. Those binaries and source hashes were reused unchanged. This is a single current-configuration trial, not a controlled submapping-only comparison or a parameter sweep.','',
'IMU acc/gyr noise is 0.018744963 / 0.00054259815, with acc/gyr bias terms 0.0006480891 / 1.0949571e-05. MapClosures retains 0.5 m density pixels, threshold 0.05, Hamming threshold 50, five inliers and twenty hypotheses. Verification retains 0.4 m preprocessing, at least 100 points, overlap 0.3, RMSE 0.35 m and 1.5 m correspondence distance. PCM retains probability 0.99, minimum clique size 2 and 60 s timeout.','',
f"The final audit verified **{a['descriptor_evidence_memberships']} descriptor/evidence memberships**, **{a['causal_ranked_events']} causal retrieval events**, graph anchor timestamps, frozen source hashes, unchanged sensor bags, four live recordings and the derived recording. Correspondence age is unmeasured and stored as null. All bulk geometry remains on 148.",'',
'![Position ATE](diagnostics/ate-comparison.png)','',
'[Trajectories and maps](report/trajectories-maps.png) · [Frontend diagnostics](diagnostics/frontend-performance.png) · [BEV gallery](bev-gallery/index.html) · [Rerun](report/result.rrd) · [Numeric report](report/report.json) · [Artifact audit](retention-audit.json)','',
'Full output root on 148: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-graco-aerial148-20260921/full`. Previous experiments and the paused CU-Multi download are preserved.','']
(W/'REPORT.md').write_text('\n'.join(lines))
print(json.dumps(dict(raw={r:v['rmse_m'] for r,v in p['raw'].items()},cbs_individual={r:v['rmse_m'] for r,v in p['cbs_individual'].items()},shared={r:v['rmse_m'] for r,v in p['cbs'].items()},connectivity=p['connectivity'],loops=p['pcm']['retained_loops'])),flush=True)
