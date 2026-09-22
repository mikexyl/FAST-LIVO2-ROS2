"""Five-flight report and honest component-wise map views from completed outputs."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.robot_colors import robot_colors
from s3e_pipeline.gravity_bev import gravity_ground
B=Path('/workspace/.ros2/graco-aerial-five148-20260921');W=B/'full'
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
p=read(W/'report/report.json');audit=read(W/'retention-audit.json');state=read(W/'status.json')
assert state['phase']=='complete'
robots=list(read(W/'trials.json'));components=p['components'];groups=sorted(set(components.values()))
loops=rows(W/'dpgo/constraints.jsonl');proposals=rows(W/'dpgo/proposed-constraints.jsonl')
events=[e for r in robots for e in rows(W/f'dpgo/{r}/events.jsonl') if e['type']=='verification']
nodes={(r['robot_id'],r['keyframe_id']):np.array(r['T_world_body']) for r in rows(W/'dpgo/poses.jsonl')}
colors={r:np.array(c)/255 for r,c in robot_colors(robots).items()}
D=W/'diagnostics';D.mkdir(exist_ok=False)
fig,axes=plt.subplots(len(groups),2,figsize=(15,7*len(groups)),squeeze=False,layout='constrained')
frames={}
for n,component in enumerate(groups):
    key=rows(W/f'prepared-{component}/store/keyframes.jsonl')[0]
    G=gravity_ground(nodes[component,0][:3,:3]@np.array(key['gravity_imu_m_s2']))
    frames[component]=G.tolist()
    for r in robots:
        if components[r]!=component:continue
        track=np.loadtxt(W/f'report/{r}-cbs.tum')[:,1:4]@G[:3,:3].T
        axes[n,0].plot(track[:,0],track[:,1],color=colors[r],label=r,lw=1.3)
        with np.load(W/f'report/{r}-maps.npz') as f:points=f['cbs']
        points=points[::max(1,len(points)//200000)]@G[:3,:3].T
        axes[n,1].scatter(points[:,0],points[:,1],s=.25,alpha=.35,color=colors[r],label=r,rasterized=True)
    for e in loops:
        if components[e['i'][0]]!=component:continue
        segment=np.array([nodes[tuple(e['i'])][:3,3],nodes[tuple(e['j'])][:3,3]])@G[:3,:3].T
        axes[n,0].plot(segment[:,0],segment[:,1],color='#777777',lw=.6,alpha=.25,zorder=0)
    axes[n,0].set_title(f'Component {component}: trajectories and retained loops')
    axes[n,1].set_title(f'Component {component}: combined map')
    for ax in axes[n]:
        ax.set_aspect('equal');ax.set_xlabel('Horizontal x [m]');ax.set_ylabel('Horizontal y [m]');ax.legend();ax.grid(alpha=.15)
fig.suptitle('GRACO aerial 04–08 · accumulated-area MapClosures → PCM → CBS\nGravity-horizontal display; separate components have separate frames; no GT alignment',fontsize=15)
fig.savefig(D/'components-map-topdown.png',dpi=170);plt.close(fig)
(D/'gravity-display-transforms.json').write_text(json.dumps(frames,indent=2)+'\n')

fig,ax=plt.subplots(figsize=(10,5),layout='constrained');display=sorted(robots);x=np.arange(len(display))
for shift,key,label,color in [(-.18,'raw','Raw','#4477aa'),(.18,'cbs_individual','CBS','#ee7733')]:
    bars=ax.bar(x+shift,[p[key][r]['rmse_m'] for r in display],.34,label=label,color=color)
    ax.bar_label(bars,fmt='%.3f',padding=3)
ax.set_xticks(x,[r.replace('aerial','A') for r in display]);ax.set_ylabel('Position ATE RMSE [m]')
ax.set_title('Independent rigid alignment per robot · evo 1.36.5 · no scale');ax.legend();ax.grid(axis='y',alpha=.2)
ax.set_ylim(0,ax.get_ylim()[1]*1.2);fig.savefig(D/'individual-ate.png',dpi=170);plt.close(fig)

def is_a4(e):return 'aerial04' in (e.get('query',e.get('i'))[0],e.get('candidate',e.get('j'))[0])
a4_events=[e for e in events if is_a4(e)];a4_loops=[e for e in loops if is_a4(e)]
a4_proposals=[e for e in proposals if is_a4(e)]
pairs=Counter(tuple(sorted([e['i'][0],e['j'][0]])) for e in loops)
key_counts={r:len(rows(W/f'prepared-{r}/store/keyframes.jsonl')) for r in robots}
summary=dict(aerial04_connected=any(e['i'][0]!=e['j'][0] for e in a4_loops),
    aerial04_verification_attempts=len(a4_events),aerial04_verification_outcomes=dict(Counter(e['reason'] for e in a4_events)),
    aerial04_proposed_loops=len(a4_proposals),aerial04_retained_loops=len(a4_loops),
    aerial04_pcm_excluded=len(a4_proposals)-len(a4_loops),aerial04_component=components['aerial04'],
    components=components,descriptors=key_counts,pairs={'--'.join(k):v for k,v in pairs.items()},
    backend_wall_s=p['runtime']['wall_s'],fresh_capture='aerial04',reused_captures=robots[:-1])
retrieval=read(W/'aerial04-retrieval-diagnostic.json')
reference_check=read(W/'aerial04-skipped-candidate-reference-check.json')
summary['aerial04_retrieval']=retrieval
summary['aerial04_skipped_candidate_reference_check']=reference_check
(D/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(D/'aerial04-verification.json').write_text(json.dumps(a4_events,indent=2)+'\n')
shared={r:v['per_robot'][r]['statistics']['rmse'] for v in p['cbs'].values() for r in v['per_robot']}
outcome='connected to the existing group' if summary['aerial04_connected'] else 'remained disconnected'
lines=['**GRACO aerial 04 added to aerial 05–08 — 21 September 2026**','',
f"**A04 {outcome}.** It produced {key_counts['aerial04']} accumulated-area descriptors, {len(a4_events)} geometric verification attempts, {len(a4_proposals)} accepted loop proposals and {len(a4_loops)} retained loops after PCM. A disconnected A04 is an accepted experimental outcome, not an execution failure.",'',
'A04 received a fresh full-flight 1× replay on workstation 148. The verified A05–A08 frontend captures and descriptor/evidence payloads were reused unchanged; all five isolated workers ran fresh retrieval, geometric verification, PCM and CBS. The earlier four-robot result is preserved. No hardware resource quotas were imposed.','',
'| Flight | Raw ATE (m) | CBS, individual (m) | CBS, component alignment (m) | Component |','|---|---:|---:|---:|---|']
for r in display:lines.append(f"| {r} | {p['raw'][r]['rmse_m']:.4f} | {p['cbs_individual'][r]['rmse_m']:.4f} | {shared[r]:.4f} | {components[r]} |")
lines+=['','All metrics use evo 1.36.5, nearest timestamp association within 50 ms, rigid SE(3) alignment and no scale fitting. Individual ATE fits one transform per flight; component ATE fits one transform per connected component. Disconnected components are never displayed or evaluated as if their relative placement were known. Ground truth was first accessed for A04 after backend completion.','']
for c,v in p['cbs'].items():lines.append(f"Component `{c}` ({', '.join(v['robots'])}): **{v['rmse_m']:.4f} m** shared ATE.")
lines+=['','| Retained loop pair | Count |','|---|---:|']
for pair,count in sorted(pairs.items()):lines.append(f"| {' ↔ '.join(pair)} | {count} |")
lines+=['',f"All robots: {len(proposals)} geometrically accepted proposals, {len(loops)} retained loops, {p['pcm']['excluded_loops']} PCM exclusions. Verification outcomes: `{json.dumps(p['verification_reasons'],sort_keys=True)}`.",
f"A04 verification outcomes: `{json.dumps(summary['aerial04_verification_outcomes'],sort_keys=True)}`. Backend wall time: **{p['runtime']['wall_s']:.2f} s**.",'',
'A04 retrieval requires a qualification: 746 inter-robot candidate entries were returned. Of these, 745 failed the retrieval threshold. A08:11 → A04:22 was eligible with six BEV inliers, but the existing one-candidate-per-query rule selected A08:11 → A07:31, which had nine inliers. The A04 candidate was explicitly skipped as `verification_budget`; it did not fail a runtime 3D test.','',
f"A separate diagnostic applied the unchanged production verifier to that single skipped candidate. It **{retrieval['diagnostic']['result']['reason']}**: overlap **{retrieval['diagnostic']['result'].get('overlap',float('nan')):.4f}**, residual RMSE **{retrieval['diagnostic']['result'].get('rmse_m',float('nan')):.4f} m**. The diagnostic read no ground truth and did not modify the graph. The configured PCM minimum clique size is two, so one isolated A04–A08 proposal would still lack the required consistency support. No singleton exception or threshold change was introduced.",'',
f"A post-hoc reference check of that candidate gives {reference_check['translation_discrepancy_m']:.3f} m translation and {reference_check['rotation_discrepancy_deg']:.3f}° rotation discrepancy against independently position-GT-aligned raw odometry. This is an approximate relative-anchor reference, not exact six-DoF GT; it was not fed back to estimation. The result supports further examination of this match, while the completed runtime result remains disconnected.",'',
'The ranked-candidate metadata retains a stale default `rejection_reason` even for the eligible candidate. The counts above use the final rejection events, which correctly distinguish `retrieval_threshold` from `verification_budget`.','',
'| Flight | Stable | Native poses | Failed updates after initialization | Max successful-update gap (s) | Mean / p95 processing (ms) | Peak mapper RSS (MiB) | Snapshots |','|---|---|---:|---:|---:|---:|---:|---:|']
for r in display:
    f=audit['frontends'][r];q=f['quality'];lines.append(f"| {r} | {q['passed']} | {f['native_poses']} | {f['failed_updates_after_init']} | {q['max_successful_update_gap_s']:.3f} | {f['processing_mean_ms']:.2f} / {f['processing_p95_ms']:.2f} | {f['peak_mapper_rss_mib']:.1f} | {f['area_snapshots']} |")
lines+=['','A05–A08 performance numbers describe the reused captures. Processing time is native scan processing, including map insertion and snapshot export; successful-update gaps use sensor timestamps.','',
'Configuration matches the four-robot height-initialization run: updated persistent-map EllipseLIO without odometry submapping; 80 m horizontal area snapshots with no height or age cutoff; 20 m movement / 10 s snapshot triggers; gravity-horizontal ellipsoid BEVs; geometry-only vertical initialization; unchanged GICP gates, PCM and CBS registration factors. Geometry consists of native processed accumulated-map representatives. The supplied aerial calibration, IMU noise, reliable input and four-thread launcher are unchanged.','',
'A04 conversion preserved all 2,915 LiDAR and 36,431 IMU measurements and their record, header and point timestamps. Only these sensor streams entered estimation. Frozen source hashes, descriptor/evidence membership, causal availability, graph anchors, input hashes and live/derived Rerun recordings were checked. The configured 100 settling iterations completed; this does not claim global optimality.','',
'The four-robot investigation found a discrepancy between A05-related loop placement and the trajectory reference, while cloud registration appeared geometrically consistent. This experiment tests A04 connectivity; it does not resolve that earlier map/reference discrepancy.','',
'![Component trajectories and maps](diagnostics/components-map-topdown.png)','',
'![Individual ATE](diagnostics/individual-ate.png)','',
'[BEV gallery](bev-gallery/index.html) · [Rerun](report/result.rrd) · [Numeric report](report/report.json) · [A04 retrieval and skipped-candidate diagnostic](aerial04-retrieval-diagnostic.json) · [Post-hoc reference check](aerial04-skipped-candidate-reference-check.json) · [Audit](retention-audit.json)','',
'Full output on 148: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-five148-20260921/full`. No bulk geometry was deleted.']
(W/'REPORT.md').write_text('\n'.join(lines)+'\n');print(json.dumps(summary,indent=2))
