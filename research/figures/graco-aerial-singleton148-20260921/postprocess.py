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
B=Path('/workspace/.ros2/graco-aerial-singleton148-20260921');W=B/'full'
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
p=read(W/'report/report.json');audit=read(W/'retention-audit.json');state=read(W/'status.json')
assert state['phase']=='complete'
audit.update(source_integration='Unchanged updated persistent-map estimator and geometry thresholds; PCM minimum clique size 1, two verification candidates, and corrected numeric/name endpoint conversion and registration ownership.',
             endpoint_order_regression='11 bridge and registration protocol tests passed in ROS; native PCM regression suite passed',
             failed_attempt_preserved=str(B/'failed-endpoint-order'))
(W/'retention-audit.json').write_text(json.dumps(audit,indent=2)+'\n')
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
fig.suptitle('GRACO aerial 04–08 · accumulated-area MapClosures → PCM → CBS\nGravity-horizontal display; singleton PCM enabled; no GT alignment',fontsize=15)
fig.savefig(D/'components-map-topdown.png',dpi=170);plt.close(fig)
(D/'gravity-display-transforms.json').write_text(json.dumps(frames,indent=2)+'\n')

fig,ax=plt.subplots(figsize=(10,5),layout='constrained');display=sorted(robots);x=np.arange(len(display))
for shift,key,label,color in [(-.18,'raw','Raw','#4477aa'),(.18,'cbs_individual','CBS','#ee7733')]:
    bars=ax.bar(x+shift,[p[key][r]['rmse_m'] for r in display],.34,label=label,color=color)
    ax.bar_label(bars,fmt='%.3f',padding=3)
ax.set_xticks(x,[r.replace('aerial','A') for r in display]);ax.set_ylabel('Position ATE RMSE [m]')
ax.set_title('Independent rigid alignment per robot · evo 1.36.5 · no scale');ax.legend();ax.grid(axis='y',alpha=.2)
ax.set_ylim(0,ax.get_ylim()[1]*1.2);fig.savefig(D/'individual-ate.png',dpi=170);plt.close(fig)

"""Executed after the common map/ATE plotting preamble in postprocess.py."""
import hashlib,re,shutil
OLD=Path('/workspace/.ros2/graco-aerial-five148-20260921/full')
before=read(OLD/'report/report.json')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
gallery=W/'bev-gallery';shutil.copytree(OLD/'bev-gallery',gallery)
g=read(gallery/'manifest.json')
assert g['exact_cached_features'] and len(g['maps'])==168
for m in g['maps']:
    assert sha(gallery/m['image'])==m['png_sha256']
    assert sha(W/f"prepared-{m['robot']}/ellipsoid/{m['key']:06d}.json.zlib")==m['descriptor_sha256']
g.update(dataset=p['dataset'],loops=[dict(i=e['i'],j=e['j']) for e in loops],
         pcm_minimum_clique_size=1,image_reuse_source=str(OLD/'bev-gallery'),image_and_descriptor_hashes_reverified=True)
(gallery/'manifest.json').write_text(json.dumps(g,indent=2)+'\n')
html=(gallery/'index.html').read_text()
data=dict(maps=g['maps'],loops=g['loops'],resolution=g['density_map_resolution_m'])
html=re.sub(r'const DATA=.*?;const el=',lambda _: 'const DATA='+json.dumps(data)+';const el=',html,count=1)
html=html.replace('The selected pairs survived geometric verification and PCM.',
    'The selected pairs passed geometric verification and the configured PCM gate. Singleton acceptance is enabled; one-loop pairs have no pairwise consistency check.')
(gallery/'index.html').write_text(html)
a4_events=[e for e in events if 'aerial04' in (e['query'][0],e['candidate'][0])]
a4_loops=[e for e in loops if 'aerial04' in (e['i'][0],e['j'][0])]
pairs=Counter(tuple(sorted([e['i'][0],e['j'][0]])) for e in loops)
shared={r:v['per_robot'][r]['statistics']['rmse'] for v in p['cbs'].values() for r in v['per_robot']}
assert any(set((tuple(e['i']),tuple(e['j'])))=={('aerial04',22),('aerial08',11)} for e in a4_loops)
summary=dict(all_connected=p['connectivity']['all_robots_connected'],components=components,
    aerial04_verifications=len(a4_events),aerial04_retained_loops=len(a4_loops),
    aerial04_verification_results=[dict(query=e['query'],candidate=e['candidate'],accepted=e['accepted'],reason=e['reason'],
        overlap=e.get('overlap'),rmse_m=e.get('rmse_m')) for e in a4_events],
    pairs={'--'.join(k):v for k,v in pairs.items()},loops=len(loops),proposed_loops=len(proposals),
    pcm_excluded=p['pcm']['excluded_loops'],backend_wall_s=p['runtime']['wall_s'],
    raw_ate={r:p['raw'][r]['rmse_m'] for r in display},individual_cbs_ate={r:p['cbs_individual'][r]['rmse_m'] for r in display},
    shared_cbs_ate=shared,shared_components={c:v['rmse_m'] for c,v in p['cbs'].items()},
    changed_settings=dict(pcm_minimum_clique_size=[2,1],verification_candidates_per_query=[1,2]))
(D/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(D/'aerial04-runtime-verification.json').write_text(json.dumps(a4_events,indent=2)+'\n')
outcome='All five drones are connected.' if summary['all_connected'] else 'The result has multiple components.'
lines=['**GRACO aerial 04–08: PCM singleton acceptance — 21 September 2026**','',
f"**{outcome}** A04 has **{len(a4_loops)} retained loop(s)**, including A04:22 ↔ A08:11. The candidate was retrieved and geometrically verified by the live distributed workers in this run; no diagnostic constraint was injected.",'',
'The verified five-flight captures and all 168 area descriptors/evidence payloads were reused byte-for-byte. Fresh workers ran retrieval, verification, PCM and CBS on workstation 148 without hardware quotas. Two settings changed: `dpgo.pcm.minimum_clique_size` **2 → 1**, and `loops.branch_verification_limits.mapclosures` **1 → 2**, allowing the previously skipped A04 candidate to reach verification. All geometric acceptance thresholds, calibration, odometry, evidence and CBS registration settings are unchanged.','',
'Singleton loops still pass 3D registration, but cannot be checked against a second loop by PCM. Native PCM records such decisions as `singleton_unchecked`. This is an explicit acceptance policy, not a claim of pairwise confirmation. Candidate sets with multiple loops still undergo the existing consistency/clique procedure, with a one-vertex clique allowed.','',
'The first attempt exposed an endpoint-order bug when A04 was appended after A08: native numeric robot order differed from Python name order. The bridge now converts PCM verdicts consistently, assigns registration ownership by native robot ID, and compares collected pairs independently of orientation. All 11 bridge/registration regression tests passed in the ROS environment, including an unsorted robot list. The failed attempt and its original frozen sources are preserved under `failed-endpoint-order`; the results below come from a fresh complete run with the fix.','',
'| Flight | Raw ATE (m) | Previous CBS, individual (m) | New CBS, individual (m) | New CBS, shared alignment (m) |','|---|---:|---:|---:|---:|']
for r in display:lines.append(f"| {r} | {p['raw'][r]['rmse_m']:.4f} | {before['cbs_individual'][r]['rmse_m']:.4f} | {p['cbs_individual'][r]['rmse_m']:.4f} | {shared[r]:.4f} |")
lines+=['','evo 1.36.5 performs all trajectory association/alignment/ATE: 50 ms nearest timestamp association, rigid SE(3), scale fixed to one. Raw and individual CBS columns fit a separate transform for each flight; shared ATE uses one transform per component. Ground truth was accessed only after backend completion.','']
for c,v in p['cbs'].items():lines.append(f"Component `{c}` ({', '.join(v['robots'])}): **{v['rmse_m']:.4f} m** shared ATE.")
lines+=['','The previous five-robot run had A04 separate and a four-robot A05–A08 component at 0.4381 m shared ATE. That four-robot metric is not directly comparable to a five-robot metric.','',
'| Retained robot pair | Loops |','|---|---:|']
for pair,count in sorted(pairs.items()):lines.append(f"| {' ↔ '.join(pair)} | {count} |")
lines+=['',f"Geometric verification accepted **{len(proposals)}** proposals; PCM retained **{len(loops)}** and excluded **{p['pcm']['excluded_loops']}**. Verification outcomes: `{json.dumps(p['verification_reasons'],sort_keys=True)}`. Backend wall time: **{p['runtime']['wall_s']:.2f} s**; rerun through evaluation/audit: **{state['wall_s']:.2f} s**, excluding original frontends and gallery/report generation.",'']
for e in a4_events:lines.append(f"A04 candidate {e['query']} → {e['candidate']}: **{e['reason']}**, overlap **{e.get('overlap',float('nan')):.4f}**, residual RMSE **{e.get('rmse_m',float('nan')):.4f} m**.")
lines+=['','The reused A04 capture completed with 2,760 native poses, zero unsuccessful LiDAR updates after initialization, maximum successful-update gap 0.120 s, mean/p95 native processing 5.02/9.64 ms, and 32 area snapshots. All five captures passed the sensor-only stability gate. A04 was converted from its original ROS1 bag to ROS2 with all 2,915 LiDAR and 36,431 IMU measurements and timestamps verified.','',
'Odometry uses updated persistent-map EllipseLIO without submapping. MapClosures uses 80 m horizontal accumulated-area snapshots with no height or age cutoff, gravity-horizontal ellipsoid BEVs and geometric vertical initialization. Registration evidence consists of native processed map representatives, not full-resolution raw scans.','',
'Validation: the native PCM regression suite passed, including default singleton rejection, explicit singleton acceptance, outlier selection, covariance, reversal and missing-evidence checks. This full five-robot run verifies actual distributed singleton handling. The audit checks unchanged raw trajectories, immutable descriptor/evidence hashes, causal retrieval, graph timestamps, historical outputs, frozen sources and Rerun recording integrity. All 168 gallery images and descriptors were rechecked against their saved hashes. The 100 configured settling iterations completed; this does not establish global optimality.','',
'![Connected trajectories and map](diagnostics/components-map-topdown.png)','',
'![Individual ATE](diagnostics/individual-ate.png)','',
'[BEV and retained-loop gallery](bev-gallery/index.html) · [Rerun recording](report/result.rrd) · [Numeric report](report/report.json) · [A04 runtime verification](diagnostics/aerial04-runtime-verification.json) · [PCM decisions](dpgo/pcm.json) · [Audit](retention-audit.json)','',
'Full output on 148: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-singleton148-20260921/full`. Previous five-robot output: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-five148-20260921/full`. All bulk geometry remains retained.']
(W/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(summary,indent=2))
