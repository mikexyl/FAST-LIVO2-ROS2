import json
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.robot_colors import robot_colors
from s3e_pipeline.gravity_bev import gravity_ground
from scipy.spatial import cKDTree
B=Path(__file__).resolve().parent;W=B/'source/output'
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]
assert read(W/'status.json')['phase']=='complete'
report=read(W/'report/report.json');trials=read(W/'trials.json');robots=list(trials)
colors={r:np.array(c)/255 for r,c in robot_colors(robots).items()}
D=W/'diagnostics';D.mkdir(exist_ok=False)
loops=rows(W/'dpgo/constraints.jsonl');proposed=rows(W/'dpgo/proposed-constraints.jsonl')
events=[e for r in robots for e in rows(W/f'dpgo/{r}/events.jsonl')]
verifications=[e for e in events if e['type']=='verification']
ranked=[e for e in events if e['type']=='ranked' and e['candidate_robot']!=e['query'][0]]
candidates=[dict(query=e['query'],candidate=[e['candidate_robot'],c['keyframe_id']],**c)
    for e in ranked for c in e['candidates']]
summary=dict(connectivity=report['connectivity'],verification_outcomes=dict(Counter(e['reason'] for e in verifications)),
    candidate_entries=sum(len(e['candidates']) for e in ranked),
    eligible_candidate_entries=sum(c.get('eligible',False) for e in ranked for c in e['candidates']),
    rejection_events=dict(Counter(e.get('reason','unknown') for e in events if e['type']=='rejection')),
    loops=len(loops),proposed_loops=len(proposed),pcm_rejected=report['pcm']['excluded_loops'],
    frontend={},backend_wall_s=report['runtime']['wall_s'])
summary['retrieval_inliers_histogram']=dict(Counter(c['mapclosures_hypothesis']['inliers'] for c in candidates))
summary['retrieval_max_inliers']=max((c['mapclosures_hypothesis']['inliers'] for c in candidates),default=0)
summary['retrieval_gate']='valid pose and strictly more than 5 MapClosures RANSAC inliers'
summary['verification_attempts']=len(verifications)
(D/'cross-robot-candidates.json').write_text(json.dumps(candidates,indent=2)+'\n')
for r in robots:
    trial=Path(trials[r]);updates=rows(trial/'frontend/native_updates.jsonl');q=read(W/f'{r}-quality.json')
    times=np.array([x['processing_s'] for x in updates])*1000
    summary['frontend'][r]=dict(q,failed_updates_after_init=sum(not x['lidar_updated'] for x in updates[1:]),
        processing_mean_ms=float(times.mean()),processing_p95_ms=float(np.quantile(times,.95)),
        peak_mapper_rss_mib=max(x.get('VmHWM',x.get('VmRSS',0)) for x in rows(trial/'memory.jsonl'))/1024,
        snapshots=len(rows(W/f'prepared-{r}/store/keyframes.jsonl')),
        odometry_map_source=updates[-1]['odometry_map_source'])
(D/'verification-events.json').write_text(json.dumps(verifications,indent=2)+'\n')
components=report['components'];groups=sorted(set(components.values()))
nodes={(n['robot_id'],n['keyframe_id']):np.array(n['T_world_body']) for n in rows(W/'dpgo/poses.jsonl')}
fig,axes=plt.subplots(len(groups),2,figsize=(14,6*len(groups)),squeeze=False,layout='constrained')
for i,c in enumerate(groups):
    anchor=rows(W/f'prepared-{c}/store/keyframes.jsonl')[0]
    G=gravity_ground(nodes[c,0][:3,:3]@np.array(anchor['gravity_imu_m_s2']))
    for r in robots:
        if components[r]!=c:continue
        track=np.loadtxt(W/f'report/{r}-cbs.tum')[:,1:4]@G[:3,:3].T
        axes[i,0].plot(track[:,0],track[:,1],color=colors[r],label=r)
        with np.load(W/f'report/{r}-maps.npz') as data:points=data['cbs']
        points=points[::max(1,len(points)//200000)]@G[:3,:3].T
        axes[i,1].scatter(points[:,0],points[:,1],s=.3,alpha=.35,color=colors[r],label=r)
    for e in loops:
        if components[e['i'][0]]!=c:continue
        p=np.array([nodes[tuple(e['i'])][:3,3],nodes[tuple(e['j'])][:3,3]])@G[:3,:3].T
        axes[i,0].plot(p[:,0],p[:,1],color='gray',lw=.6,alpha=.3)
    for ax in axes[i]:ax.set_aspect('equal');ax.set_xlabel('Horizontal x [m]');ax.set_ylabel('Horizontal y [m]');ax.legend();ax.grid(alpha=.15)
    axes[i,0].set_title(f'Component {c}: trajectories and loops');axes[i,1].set_title(f'Component {c}: accumulated geometry')
fig.suptitle('Ground-06 + aerial-06 · local EllipseLIO / area BEV / PCM / CBS\nGravity-horizontal display; separate components have separate frames; no GT alignment')
fig.savefig(D/'components-map-topdown.png',dpi=170);plt.close(fig)
# Ground truth is used here only for a post-run evaluation diagnostic. It does
# not transform estimator output or supply a loop hypothesis to any worker.
references={r:np.loadtxt(W/f'report/evo/raw/{r}-reference-matched.tum')[:,1:4] for r in robots}
ground=references['ground06'];aerial=references['aerial06']
dist,nearest=cKDTree(ground[:,:2]).query(aerial[:,:2]);close=dist<20
summary['evaluation_only_route_overlap']=dict(ground_truth_used=True,
    minimum_horizontal_path_distance_m=float(dist.min()),
    aerial_samples_within_20m_of_ground_path_fraction=float(close.mean()),
    median_vertical_separation_for_those_samples_m=float(np.median(aerial[close,2]-ground[nearest[close],2])) if close.any() else None,
    limitation='Path proximity does not establish common observed surfaces; no simultaneity requirement.')
fig,ax=plt.subplots(figsize=(10,7),layout='constrained')
for r in robots:
    ref=references[r]
    ax.plot(ref[:,0],ref[:,1],label=r,color=colors[r],lw=2)
    ax.scatter(ref[0,0],ref[0,1],color=colors[r],marker='o',s=45)
    ax.scatter(ref[-1,0],ref[-1,1],color=colors[r],marker='x',s=60)
ax.set_aspect('equal');ax.set_xlabel('East [m]');ax.set_ylabel('North [m]');ax.grid(alpha=.2);ax.legend()
ax.set_title('Evaluation only: published ground-truth routes in shared ENU\nNot a recovered multi-robot alignment · circles: start, crosses: end')
fig.savefig(D/'evaluation-only-route-overlap.png',dpi=170);plt.close(fig)
(D/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
shared={r:v['per_robot'][r]['statistics']['rmse'] for v in report['cbs'].values() for r in v['per_robot']}
connected=report['connectivity']['all_robots_connected']
lines=['**GRACO ground-06 + aerial-06 (20 m): local ground–aerial experiment — 21 September 2026**','',
    '**The two robots connected.**' if connected else '**The two robots remained disconnected.**','',
    f"The complete local run retained **{len(loops)} loops** from **{len(proposed)} geometrically accepted proposals**; PCM rejected **{summary['pcm_rejected']}**. Both full 1× frontend replays passed the sensor-only stability gate.",'',
    f"The failure occurred at **BEV retrieval**: {len(candidates)} cross-robot candidate entries, maximum {summary['retrieval_max_inliers']} RANSAC inliers, with the unchanged gate requiring **strictly more than 5**. **{len(verifications)} candidates reached geometric verification**. Consequently, the runtime vertical initializer and inter-robot registration factors were not exercised. This run does not test whether they could align a correct ground–aerial candidate.",'',
    'The pair was selected from the official published route diagrams: both traverse the northeastern street, and aerial-06 has nominal 20 m altitude. This is a visual route-based choice, not an exhaustive overlap ranking. Numerical ground-truth trajectories were not consulted for selection, initialization, retrieval, verification or optimization.','',
    '[Official ground routes](https://github.com/SYSU-RoboticsLab/GrAco/blob/main/doc/sequence-ground.png) · [Official aerial routes](https://github.com/SYSU-RoboticsLab/GrAco/blob/main/doc/sequence-aerial.png)','',
    'Both bags were already ROS2. Sensor staging preserved the native LiDAR/IMU payloads and all record/header/point timestamps. Ground and aerial extrinsics and all four IMU noise values came from their respective supplied calibration files. The two frontends ran concurrently on this laptop; workstation 148 was not used.','',
    '| Robot | Raw ATE (m) | Individual CBS ATE (m) | Component-aligned CBS ATE (m) | Component |','|---|---:|---:|---:|---|']
for r in robots:lines.append(f"| {r} | {report['raw'][r]['rmse_m']:.4f} | {report['cbs_individual'][r]['rmse_m']:.4f} | {shared[r]:.4f} | {components[r]} |")
for c,v in report['cbs'].items():lines+=['',f"Component `{c}` ({', '.join(v['robots'])}): **{v['rmse_m']:.4f} m** ATE."]
lines+=['','All ATE computation uses evo 1.36.5 with 50 ms association, rigid SE(3) alignment and no scale fitting. **There is no recovered shared ground–aerial alignment or joint ATE in this disconnected result.** Each row is aligned independently. CBS changes only the arbitrary frame with no accepted loops; its individual ATE is unchanged. Ground-truth files were read after the backend finished.','',
    '| Robot | Native poses | Failed updates after initialization | Largest successful-update gap (s) | Mean / p95 processing (ms) | Peak mapper RSS (MiB) | Area snapshots |','|---|---:|---:|---:|---:|---:|---:|']
for r,v in summary['frontend'].items():lines.append(f"| {r} | {v['native_poses']} | {v['failed_updates_after_init']} | {v['max_successful_update_gap_s']:.3f} | {v['processing_mean_ms']:.2f} / {v['processing_p95_ms']:.2f} | {v['peak_mapper_rss_mib']:.1f} | {v['snapshots']} |")
lines+=['',f"Inter-robot retrieval returned {summary['candidate_entries']} candidate entries, of which {summary['eligible_candidate_entries']} were eligible. Verification outcomes: `{json.dumps(summary['verification_outcomes'],sort_keys=True)}`. Backend time: **{summary['backend_wall_s']:.2f} s**.",'',
    'The estimator is updated upstream EllipseLIO with persistent-map odometry and the tested octree storage fix. MapClosures uses 80 m horizontal accumulated-area snapshots with all historical in-area points, no height cutoff, 20 m/10 s snapshot triggers, and gravity-horizontal ellipsoid BEVs. Vertical translation is initialized from geometry. Evidence consists of processed native map representatives, not full-resolution raw scans.','',
    'Geometric thresholds match the five-drone singleton experiment: 0.4 m evidence voxels, 1.5 m correspondence distance, 0.6 m inlier distance, minimum symmetric overlap 0.30, maximum inlier RMSE 0.35 m, and unchanged observability checks. Two MapClosures candidates per query may reach verification. PCM minimum clique size is one; isolated accepted loops are labelled `singleton_unchecked` and have no pairwise consistency confirmation. Distributed robot isolation and live GICP registration factors remain enabled.','',
    'The local build passed all three native tests, 19 Python regression tests and the native PCM suite. CUDA surface sampling was compiled for the RTX 4070 (sm_89) to avoid a compiler/driver PTX mismatch. Earlier preflight failures are retained; neither started a dataset replay. The final audit verifies frozen sources, sensor bag hashes, descriptor/evidence membership, causal availability, graph anchors, and live/derived Rerun recordings.','',
    '![Trajectories and maps](diagnostics/components-map-topdown.png)','',
    '![Evaluation-only route overlap](diagnostics/evaluation-only-route-overlap.png)','',
    f"Post-run ground-truth evaluation confirms route proximity: minimum horizontal path distance **{dist.min():.2f} m**; **{100*close.mean():.1f}%** of aerial trajectory samples lie within 20 m horizontally of the ground route. This is a trajectory-only overlap check, not evidence of common LiDAR surfaces, and it was not used by retrieval or optimization.",'',
    '[BEV gallery](bev-gallery/index.html) · [Rerun recording](report/result.rrd) · [Numeric report](report/report.json) · [Verification events](diagnostics/verification-events.json) · [Audit](retention-audit.json)','',
    f'Full local output: `{W}`. All original data and bulk geometry are retained.']
(W/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(summary,indent=2))
