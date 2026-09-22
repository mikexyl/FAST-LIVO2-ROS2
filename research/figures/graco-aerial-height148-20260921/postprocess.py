"""Report, unchanged-BEV gallery with fresh loops, and gravity-horizontal maps."""
from pathlib import Path
from collections import Counter
import hashlib,json,re,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.gravity_bev import gravity_ground
from s3e_pipeline.robot_colors import robot_colors

B=Path('/workspace/.ros2/graco-aerial-height148-20260921');W=B/'full'
OLD=Path('/workspace/.ros2/upstream-area-graco-aerial148-20260921/full')
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=read(W/'report/report.json');before=read(OLD/'report/report.json');state=read(W/'status.json');a=read(W/'retention-audit.json')
assert state['phase']=='complete'
robots=list(p['components']);loops=rows(W/'dpgo/constraints.jsonl');nodes=rows(W/'dpgo/poses.jsonl')
events=[e for r in robots for e in rows(W/f'dpgo/{r}/events.jsonl') if e['type']=='verification']
out=W/'diagnostics';out.mkdir(exist_ok=False)

# The same 136 images were already reconstructed and checked against cached ORB.
# Reuse their bytes, but publish only the NEW retained loop endpoints in this gallery.
gallery=W/'bev-gallery';shutil.copytree(OLD/'bev-gallery',gallery)
g=read(gallery/'manifest.json');assert g['exact_cached_features']
for m in g['maps']:
    assert sha(gallery/m['image'])==m['png_sha256']
    assert sha(W/f"prepared-{m['robot']}/ellipsoid/{m['key']:06d}.json.zlib")==m['descriptor_sha256']
g.update(dataset=p['dataset'],loops=[dict(i=e['i'],j=e['j']) for e in loops],
    image_reuse_source=str(OLD/'bev-gallery'),image_and_descriptor_hashes_reverified=True,
    runtime_vertical_initialization=True,new_backend_loops=True)
(gallery/'manifest.json').write_text(json.dumps(g,indent=2)+'\n')
html=(gallery/'index.html').read_text()
data=dict(maps=g['maps'],loops=g['loops'],resolution=g['density_map_resolution_m'])
html=re.sub(r'const DATA=.*?;const el=',lambda _: 'const DATA='+json.dumps(data)+';const el=',html,count=1)
html=html.replace('<h1>GRACO aerial 05–08 — accumulated-area ellipsoid BEVs</h1>',
    '<h1>GRACO aerial 05–08 — runtime height initialization</h1><p>The images are unchanged verified BEVs. The loop-pair list below comes from the new distributed PCM/CBS run with geometric vertical initialization.</p>')
(gallery/'index.html').write_text(html)

# Common-frame map views use native gravity, never GT alignment.
colors={r:np.array(c)/255 for r,c in robot_colors(robots).items()}
groups=sorted(set(p['components'].values()));fig,axes=plt.subplots(len(groups),2,figsize=(15,7*len(groups)),squeeze=False,layout='constrained')
gravity_frames={};lookup={(n['robot_id'],n['keyframe_id']):np.array(n['T_world_body']) for n in nodes}
for i,component in enumerate(groups):
    reference=rows(W/f'prepared-{component}/store/keyframes.jsonl')[0]
    anchor=lookup[component,0]
    G=gravity_ground(anchor[:3,:3]@np.array(reference['gravity_imu_m_s2']))
    gravity_frames[component]=G.tolist()
    for robot in robots:
        if p['components'][robot]!=component:continue
        track=np.loadtxt(W/'report'/f'{robot}-cbs.tum')[:,1:4]@G[:3,:3].T
        axes[i,0].plot(track[:,0],track[:,1],color=colors[robot],lw=1.4,label=robot)
        with np.load(W/'report'/f'{robot}-maps.npz') as f:points=f['cbs']
        points=points[::max(1,len(points)//200000)]@G[:3,:3].T
        axes[i,1].scatter(points[:,0],points[:,1],s=.25,color=colors[robot],alpha=.35,label=robot,rasterized=True)
    for e in loops:
        if p['components'][e['i'][0]]!=component:continue
        segment=np.array([lookup[tuple(e['i'])][:3,3],lookup[tuple(e['j'])][:3,3]])@G[:3,:3].T
        axes[i,0].plot(segment[:,0],segment[:,1],color='#777777',alpha=.25,lw=.6,zorder=0)
    axes[i,0].set_title(f'Component {component}: trajectories and retained loop links')
    axes[i,1].set_title(f'Component {component}: combined native map, top-down')
    for ax in axes[i]:ax.set_aspect('equal');ax.set_xlabel('Horizontal x [m]');ax.set_ylabel('Horizontal y [m]');ax.legend();ax.grid(alpha=.15)
fig.suptitle('GRACO aerial 05–08 · MapClosures + height initialization → PCM → CBS\nGravity-horizontal display; no GT alignment',fontsize=16)
fig.savefig(out/'connected-map-topdown.png',dpi=170);plt.close(fig)
(out/'gravity-display-transforms.json').write_text(json.dumps(gravity_frames,indent=2)+'\n')

fig,ax=plt.subplots(figsize=(10,5),layout='constrained');x=np.arange(len(robots))
for shift,key,label,color in [(-.25,'raw','Raw','#4477aa'),(0,'previous','Previous CBS','#bbbbbb'),(.25,'new','CBS with height initialization','#ee7733')]:
    vals=[(p['raw'] if key=='raw' else before['cbs_individual'] if key=='previous' else p['cbs_individual'])[r]['rmse_m'] for r in robots]
    bars=ax.bar(x+shift,vals,.24,label=label,color=color);ax.bar_label(bars,fmt='%.3f',padding=3,fontsize=9)
ax.set_xticks(x,[r.replace('aerial','Aerial ') for r in robots]);ax.set_ylabel('Position ATE RMSE [m]');ax.set_title('Independent rigid alignment per robot · evo 1.36.5 · no scale');ax.legend();ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);ax.set_ylim(0,ax.get_ylim()[1]*1.2)
fig.savefig(out/'ate-comparison.png',dpi=170);plt.close(fig)

pairs=Counter(tuple(sorted([e['i'][0],e['j'][0]])) for e in loops)
shared={r:(c,v['per_robot'][r]['statistics']['rmse']) for c,v in p['cbs'].items() for r in v['per_robot']}
heights=[e['vertical_initialization'] for e in events]
a5=[e for e in events if 'aerial05' in [e['query'][0],e['candidate'][0]]]
summary=dict(all_connected=p['connectivity']['all_robots_connected'],loops=len(loops),pcm_rejected=p['pcm']['excluded_loops'],
    a5_geometric_attempts=len(a5),a5_geometric_accepted=sum(e['accepted'] for e in a5),
    a5_retained_loops=sum('aerial05' in [e['i'][0],e['j'][0]] for e in loops),
    height_initializations=len(heights),height_runtime_mean_s=float(np.mean([d['runtime_s'] for d in heights])),
    height_runtime_p95_s=float(np.quantile([d['runtime_s'] for d in heights],.95)),
    height_nonzero=sum(abs(d['correction_m'])>1e-9 for d in heights),backend_wall_s=p['runtime']['wall_s'],
    individual_cbs_ate={r:v['rmse_m'] for r,v in p['cbs_individual'].items()},
    shared_cbs_ate={r:v['rmse_m'] for r,v in p['cbs'].items()})
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
connected='All four drones are connected in one optimized component.' if summary['all_connected'] else 'The output remains split into multiple components; see connectivity below.'
lines=['**GRACO aerial 05–08: full distributed backend with runtime height initialization — 21 September 2026**','',
f"**{connected}** A05 has **{summary['a5_retained_loops']} retained inter-robot loops**. This is the completed runtime MapClosures → PCM → CBS result, including native GICP registration factors.",'',
'The four previously verified updated-EllipseLIO captures were reused byte-for-byte. Odometry uses its persistent map without submapping; 80 m horizontal accumulated-area snapshots supply BEVs and registration evidence. All 136 descriptors and their native processed geometry are unchanged. Fresh isolated workers performed retrieval, runtime verification, PCM and CBS; no diagnostic loop list was injected. Workstation 148 had no CPU, memory or GPU quotas.','',
'| Flight | Raw ATE (m) | Previous CBS, individual (m) | New CBS, individual (m) | New CBS, component alignment (m) | Matched poses |','|---|---:|---:|---:|---:|---:|']
for r in robots:lines.append(f"| {r.replace('aerial','Aerial ')} | {p['raw'][r]['rmse_m']:.4f} | {before['cbs_individual'][r]['rmse_m']:.4f} | {p['cbs_individual'][r]['rmse_m']:.4f} | {shared[r][1]:.4f} | {p['raw'][r]['samples']} |")
lines+=['','Raw and individual CBS ATE each use an independent rigid SE(3) alignment per robot. Component ATE uses one alignment shared by all robots in that component. Evaluation is entirely evo 1.36.5, 50 ms timestamp association, no scale fitting. GRACO IMU-frame position GT is accessed after backend completion and never used by the height initializer.','']
for c,v in p['cbs'].items():lines.append(f"Component `{c}` ({', '.join(v['robots'])}): **{v['rmse_m']:.4f} m** shared position ATE.")
lines+=['','The previous run had two components: A05 alone (0.1775 m ATE) and A06–A07–A08 (0.1023 m shared ATE). Its three-robot shared ATE is not directly comparable to a new four-robot shared ATE. Connecting A05 does not itself establish an accuracy improvement.','',
f"Geometric verification accepted **{p['pcm']['proposed_loops']}** loop proposals; PCM retained **{p['pcm']['retained_loops']}** and rejected **{p['pcm']['excluded_loops']}**. A05: {summary['a5_geometric_accepted']}/{summary['a5_geometric_attempts']} geometric candidates accepted; {summary['a5_retained_loops']} survived PCM.",'',
'| Retained pair | Loops |','|---|---:|']
for pair,n in sorted(pairs.items()):lines.append(f"| {' ↔ '.join(pair)} | {n} |")
lines+=['',f"Verification outcomes: `{json.dumps(p['verification_reasons'],sort_keys=True)}`.",'',
'The initializer fills the unobserved vertical component of a gravity-horizontal BEV pose. It votes on height differences of horizontally adjacent 3D points, scores five separated modes plus zero by symmetric coarse overlap, and changes only vertical translation before GICP. It runs on all robot-pair candidates, not an A05-specific rule. No nominal flight heights, GNSS or GT enter it. Default-disabled configuration preserves existing behavior elsewhere.','',
'Height estimation uses 0.8 m voxels, eight XY neighbors within 1 m, 0.5 m bins, >2 m mode separation and 1.5 m coarse overlap distance. Evidence/registration remain fixed 0.4 m; GICP keeps 1.5 m correspondence distance, 0.6 m inlier distance, minimum overlap 0.30 and maximum RMSE 0.35 m. Retrieval, same-robot exclusions, PCM probability 0.99 / minimum clique 2, and CBS settings are unchanged.','',
f"Backend wall time: **{p['runtime']['wall_s']:.2f} s** (previously {before['runtime']['wall_s']:.2f} s). Rerun preparation, regression, backend, evaluation and audit: **{state['wall_s']:.2f} s**; this excludes the original odometry capture and this gallery/report generation. Runtime height initialization ran {summary['height_initializations']} times, with mean / p95 {1000*summary['height_runtime_mean_s']:.1f} / {1000*summary['height_runtime_p95_s']:.1f} ms per candidate. These are backend costs, not per-scan odometry times.",'',
'All reused frontends completed with finite chronological poses and zero unsuccessful LiDAR updates after initialization. Max update gaps for 05/06/07/08 remain 0.192 / 0.168 / 0.152 / 0.136 s; mean native processing remains 11.34 / 11.98 / 12.02 / 12.42 ms. No new frontend performance claim is made.','',
'Verification: 38 tests passed, including gravity/reversal and actual distributed integration. The production verifier reproduced all fourteen offline regression outcomes (twelve recovered A05 candidates and two accepted controls). The final audit checked unchanged raw trajectories, input/descriptor/evidence hashes, causal availability, graph anchor timestamps, unchanged historical results, runtime initializer diagnostics, frozen sources and Rerun integrity. The 136 gallery images retain their verified original bytes and cached ORB features; its loop-pair list uses this new PCM result.','',
'Backend completion means the configured 100 local settling iterations finished after peer input completion; it is not a claim of mathematical convergence. This is one run with fixed settings.','',
'![Connected trajectories and map](diagnostics/connected-map-topdown.png)','',
'![Individual ATE comparison](diagnostics/ate-comparison.png)','',
'[BEV and retained-loop gallery](bev-gallery/index.html) · [Rerun recording](report/result.rrd) · [Trajectories and maps with evaluation](report/trajectories-maps.png) · [Numeric report](report/report.json) · [Artifact audit](retention-audit.json) · [Runtime summary](diagnostics/summary.json)','',
'Full output on 148: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-height148-20260921/full`. The original disconnected run remains at `/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-graco-aerial148-20260921/full`.','']
(W/'REPORT.md').write_text('\n'.join(lines));print(json.dumps(summary,indent=2))
