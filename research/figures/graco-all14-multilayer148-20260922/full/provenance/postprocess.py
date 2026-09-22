"""Numeric report and gravity-horizontal figures after the all-14 optimization/evo run."""
from pathlib import Path
from collections import Counter
import json,sys,time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.robot_colors import robot_colors
from s3e_pipeline.gravity_bev import gravity_ground
B=Path(__file__).resolve().parent;W=B/'full';D=W/'diagnostics';D.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_text())
rows=lambda p:[json.loads(x) for x in p.read_text().splitlines()]
assert read(W/'status.json')['phase']=='complete'
report=read(W/'report/report.json');trials=read(W/'trials.json');robots=list(trials)
loops=rows(W/'dpgo/constraints.jsonl');proposed=rows(W/'dpgo/proposed-constraints.jsonl')
events=[e for r in robots for e in rows(W/f'dpgo/{r}/events.jsonl')]
verifications=[e for e in events if e['type']=='verification']
category=lambda a,b:'aerial–ground' if a.startswith('aerial')!=b.startswith('aerial') else 'aerial–aerial' if a.startswith('aerial') else 'ground–ground'
counts={}
for label,edges in [('proposed',proposed),('retained',loops)]:
 counts[label]=dict(Counter(category(e['i'][0],e['j'][0]) for e in edges))
pair_counts=dict(Counter(' ↔ '.join(sorted([e['i'][0],e['j'][0]])) for e in loops))
summary=dict(frontend={},loop_categories=counts,loop_pairs=pair_counts,connectivity=report['connectivity'],
 verification_reasons=dict(Counter(e['reason'] for e in verifications)),verification_attempts=len(verifications),
 rejected_verifications=sum(not e['accepted'] for e in verifications),pcm_rejected=report['pcm']['excluded_loops'],
 backend_wall_s=report['runtime']['wall_s'],descriptor_audit=read(W/'descriptor-audit.json'))
summary['registration_factors']=report['registration']['registration_factor_count']
summary['skipped_registration_factors']=[dict(owner=r,**p) for r,v in report['registration']['robots'].items() for p in v['pairs'] if not p['accepted']]
summary['network_cdr_bytes']={k:v for k,v in report['runtime'].items() if k.endswith('_network_cdr_bytes')}
summary['convergence']={}
for r in robots:
 stats=rows(W/f'dpgo/{r}/stats.jsonl')
 summary['convergence'][r]=dict(samples=len(stats),
  final_pose_change=stats[-1]['result_logmap_change'],
  last_10_max_pose_change=max(x['result_logmap_change'] for x in stats[-10:]),
  last_10_received_beliefs=sum(x['num_received_beliefs'] for x in stats[-10:]))
for r in robots:
 trial=Path(trials[r]);updates=rows(trial/'frontend/native_updates.jsonl');q=read(W/f'{r}-quality.json')
 times=np.array([x['processing_s'] for x in updates])*1000
 memory=rows(trial/'memory.jsonl')
 summary['frontend'][r]=dict(q,failed_updates_after_init=sum(not x['lidar_updated'] for x in updates[1:]),
  processing_mean_ms=float(times.mean()),processing_p95_ms=float(np.quantile(times,.95)),
  mapper_observed_wall_s=(memory[-1]['wall_ns']-memory[0]['wall_ns'])/1e9,
  native_pose_sensor_span_s=(updates[-1]['stamp_ns']-updates[0]['stamp_ns'])/1e9,
  peak_mapper_rss_mib=max(x.get('VmHWM',x.get('VmRSS',0)) for x in memory)/1024,
  snapshots=len(rows(W/f'prepared-{r}/store/keyframes.jsonl')),odometry_map_source=updates[-1]['odometry_map_source'])
(D/'summary.json').write_text(json.dumps(summary,indent=2));(D/'verification-events.json').write_text(json.dumps(verifications))
components=report['components'];groups=sorted(set(components.values()));colors={r:np.array(c)/255 for r,c in robot_colors(robots).items()}
nodes={(n['robot_id'],n['keyframe_id']):np.array(n['T_world_body']) for n in rows(W/'dpgo/poses.jsonl')}
fig,axes=plt.subplots(len(groups),2,figsize=(16,6*len(groups)),squeeze=False,layout='constrained')
for i,c in enumerate(groups):
 anchor=rows(W/f'prepared-{c}/store/keyframes.jsonl')[0]
 G=gravity_ground(nodes[c,0][:3,:3]@np.array(anchor['gravity_imu_m_s2']))
 for r in robots:
  if components[r]!=c:continue
  track=np.loadtxt(W/f'report/{r}-cbs.tum')[:,1:4]@G[:3,:3].T
  axes[i,0].plot(track[:,0],track[:,1],color=colors[r],label=r)
  with np.load(W/f'report/{r}-maps.npz') as data:points=data['cbs']
  points=points[::max(1,len(points)//100000)]@G[:3,:3].T
  axes[i,1].scatter(points[:,0],points[:,1],s=.25,alpha=.35,color=colors[r],label=r,rasterized=True)
 for e in loops:
  if components[e['i'][0]]!=c:continue
  p=np.array([nodes[tuple(e['i'])][:3,3],nodes[tuple(e['j'])][:3,3]])@G[:3,:3].T
  axes[i,0].plot(p[:,0],p[:,1],color='gray',lw=.45,alpha=.2)
 for ax in axes[i]:ax.set_aspect('equal');ax.set_xlabel('Horizontal x [m]');ax.set_ylabel('Horizontal y [m]');ax.legend(fontsize=8,ncol=2);ax.grid(alpha=.15)
 axes[i,0].set_title(f'Component {c}: trajectories and loops');axes[i,1].set_title(f'Component {c}: accumulated geometry')
fig.suptitle('GRACO all 14 · updated EllipseLIO / accumulated-area multilayer BEVs / PCM / CBS\nGravity-horizontal display from estimated gravity; no ground-truth display alignment')
fig.savefig(D/'components-map-topdown.png',dpi=150);plt.close(fig)
# ATE bars preserve the distinction between individual and shared alignment.
shared={r:v['per_robot'][r]['statistics']['rmse'] for v in report['cbs'].values() for r in v['per_robot']}
fig,ax=plt.subplots(figsize=(15,6),layout='constrained');x=np.arange(len(robots));w=.25
for offset,label,values,color in [(-w,'Raw: individual alignment',[report['raw'][r]['rmse_m'] for r in robots],'#6985a8'),
 (0,'CBS: individual alignment',[report['cbs_individual'][r]['rmse_m'] for r in robots],'#eaa957'),
 (w,'CBS: shared component alignment',[shared[r] for r in robots],'#509c88')]:ax.bar(x+offset,values,w,label=label,color=color)
ax.set_xticks(x,robots,rotation=45,ha='right');ax.set_ylabel('Position ATE RMSE [m]');ax.legend();ax.grid(axis='y',alpha=.2)
fig.savefig(D/'ate-comparison.png',dpi=160);plt.close(fig)
lines=['# GRACO all 14: multilayer accumulated-area pipeline — 22 September 2026','',
 f"**{len(groups)} connected component(s), {len(loops)} retained loops, {summary['pcm_rejected']} PCM rejections.** All fourteen fresh full-length 1× captures passed the sensor-only stability gate.",'',
 'Workstation 148 ran without CPU, memory or GPU quotas. Updated EllipseLIO matches scans against its persistent map. Every 20 m or 10 s, a snapshot retains all accumulated native map representatives inside an 80 m horizontal radius, across scan ages and heights. Snapshots feed five terrain-relative ellipsoid BEV layers and the corresponding native geometry. This is processed map geometry, not full-resolution raw scans.','',
 'Same-band ORB/HBST matches are pooled and deduplicated, then fitted with the tested joint SE(2) estimator. The full-height descriptor is an inspection control. Missing terrain support produces empty layers. The existing geometric height initializer, GICP acceptance thresholds, singleton-accepting PCM and distributed CBS/GICP factors remain unchanged.','',
 '| Robot | Raw ATE (m) | CBS individual ATE (m) | CBS shared-component ATE (m) | Component |','|---|---:|---:|---:|---|']
for r in robots:lines.append(f"| {r} | {report['raw'][r]['rmse_m']:.4f} | {report['cbs_individual'][r]['rmse_m']:.4f} | {shared[r]:.4f} | {components[r]} |")
lines+=['','ATE uses **evo 1.36.5**, 50 ms association, rigid SE(3) alignment and no scale fitting. Individual alignment measures trajectory shape after each robot gets its own rigid alignment; shared-component alignment also measures errors in relative placement. Separate components receive separate alignments and do not constitute a recovered shared frame. Ground truth was introduced only after optimization.','',
 '| Component | Robots | Shared ATE (m) |','|---|---|---:|']
for c,v in report['cbs'].items():lines.append(f"| {c} | {', '.join(v['robots'])} | {v['rmse_m']:.4f} |")
lines+=['','| Loop category | Geometrically accepted | PCM retained |','|---|---:|---:|']
for c in ['aerial–aerial','ground–ground','aerial–ground']:lines.append(f"| {c} | {counts['proposed'].get(c,0)} | {counts['retained'].get(c,0)} |")
lines+=['',f"CBS added {summary['registration_factors']} native registration factors. {len(summary['skipped_registration_factors'])} retained pose constraint(s) lacked an additional registration factor; reasons and preprocessing diagnostics are preserved in `diagnostics/summary.json`.",
 'Loop counts are submap-pair constraints, not independent places: overlapping area snapshots reuse geometry.', '',
 f"Network payload accounting (serialized CDR bytes, excluding RTPS/discovery/retransmission): `{json.dumps(summary['network_cdr_bytes'])}`. Loop exchange includes both descriptors and verification geometry."]
lines+=['',f"{len(verifications)} geometric verification attempts; outcomes: `{json.dumps(summary['verification_reasons'])}`. Backend wall time: {summary['backend_wall_s']:.1f} s.",'',
 '| Robot | Native poses | Failed updates after initialization | Largest update gap (s) | Mean / p95 processing (ms) | Peak mapper RSS (MiB) | Submaps | Terrain unavailable |',
 '|---|---:|---:|---:|---:|---:|---:|---:|']
for r,v in summary['frontend'].items():lines.append(f"| {r} | {v['native_poses']} | {v['failed_updates_after_init']} | {v['max_successful_update_gap_s']:.3f} | {v['processing_mean_ms']:.2f} / {v['processing_p95_ms']:.2f} | {v['peak_mapper_rss_mib']:.1f} | {v['snapshots']} | {summary['descriptor_audit']['terrain_unavailable'][r]} |")
lines+=['','Update gaps use sensor timestamps between successful LiDAR corrections. Processing time measures native per-scan work and is a different quantity. Stability requires complete finite chronological output, speed ≤20 m/s and successful-update gaps <1 s.','',
 'CBS uses the unchanged 100-iteration settling budget. Completion alone does not prove convergence; final pose-change and last-ten-iteration statistics are retained in `diagnostics/summary.json`.','',
 'The native adapter and Python regression checks cover disabled-mode compatibility, raw correspondence eligibility, gravity-frame pose direction, layer serialization, deduplication, and peer isolation. A separate synthetic 14-worker DDS/PCM/CBS run retained 53 loops and 53 registration factors with correct poses in one connected component. This preflight does not establish dataset accuracy.','',
 '![ATE comparison](diagnostics/ate-comparison.png)','', '![Recovered components](diagnostics/components-map-topdown.png)','',
 '[BEV gallery](gallery/index.html) · [Rerun recording](report/result-gravity.rrd) · [Numeric report](report/report.json) · [Detailed diagnostics](diagnostics/summary.json) · [Runtime audit](runtime-audit.json) · [Configuration](config.yaml)','',
 'Original datasets, previous experiments, and the paused CU-Multi download were preserved. Bulk geometry is retained. Source, sensor and configuration hashes accompany this run.']
(W/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(dict(components=components,loop_categories=counts,loops=len(loops)),indent=2))
