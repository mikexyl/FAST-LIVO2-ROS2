"""Render audited experiment results without recomputing any ATE."""
import json
from pathlib import Path
root=Path('/home/mikexyl/workspaces/fast_livo2_ws/src/FAST-LIVO2-ROS2/research')
assets=root/'figures/upstream-area-s3e-20260921';data=json.loads((assets/'final-summary.json').read_text())
assert not data['pending_recordings']
prefix='figures/upstream-area-s3e-20260921'
lines=['**Updated EllipseLIO + accumulated-area MapClosures + distributed PCM/CBS — S3E, 21 September 2026**','',
'Campus Road 1, 2 and 3 completed the three-robot pipeline. Laboratory 4 failed the frontend stability check because Bob diverged; its backend was not run. All twelve requested robot sequences were attempted fresh at 1×. Campus Road 2 / Carol required one replay after fixing an upstream octree storage-capacity crash.','',
'CBS substantially improves Bob on Campus Roads 2 and 3, but does not solve Campus Road 2 / Alpha: its individual ATE remains 10.63 m. Campus Road 2 / Carol is unchanged in individual ATE to displayed precision. Laboratory 4 / Bob still diverges.','',
'Odometry uses updated upstream EllipseLIO’s persistent map, with temporal submapping and area-cropped odometry disabled. Accumulated area snapshots supply only MapClosures and registration evidence.','',
'**Position ATE (RMSE, metres)**','',
'Raw and individual CBS trajectories each receive an independent rigid SE(3) alignment. The shared result uses one SE(3) alignment for the entire connected three-robot component. All numbers come from evo 1.36.5, 50 ms association, scale fixed to one. Ground truth is confined to evaluation. Supplied GT orientation is unused; the antenna lever arm is uncorrected.','',
'| Sequence | Robot | Raw | CBS, individual | CBS, shared alignment | Matched GT samples |','|---|---|---:|---:|---:|---:|']
for name,g in data['groups'].items():
 if g['status']!='complete':continue
 p=g['report'];shared=next(iter(p['cbs'].values()))
 for r in ('Alpha','Bob','Carol'):
  lines.append(f"| {name.replace('S3E_','').replace('_',' ')} | {r} | {p['raw'][r]['rmse_m']:.3f} | {p['cbs_individual'][r]['rmse_m']:.3f} | {shared['per_robot'][r]['statistics']['rmse']:.3f} | {p['raw'][r]['samples']} |")
lines+=['','| Sequence | Shared-component ATE | Verified loops before PCM | Retained / rejected | Inter-robot pairs A–B / A–C / B–C | Connected | Backend wall time |',
'|---|---:|---:|---:|---|---|---:|']
for name,g in data['groups'].items():
 if g['status']!='complete':continue
 p=g['report'];c=p['connectivity'];pcm=p['pcm'];shared=next(iter(p['cbs'].values()))
 pairs=' / '.join(str(c['inter_robot_loop_counts'].get(k,0)) for k in ('Alpha--Bob','Alpha--Carol','Bob--Carol'))
 lines.append(f"| {name.replace('S3E_','').replace('_',' ')} | {shared['rmse_m']:.3f} | {pcm['proposed_loops']} | {pcm['retained_loops']} / {pcm['excluded_loops']} | {pairs} | {c['all_robots_connected']} | {p['runtime']['wall_s']:.1f} s |")
lines+=['','| Sequence | Descriptor preparation A / B / C (s) | Evaluation and map reconstruction (s) | Largest backend process RSS (GiB) |','|---|---:|---:|---:|']
for name,g in data['groups'].items():
 if g['status']!='complete':continue
 p=g['report'];prep=' / '.join(f"{g['frontends'][r]['descriptor_preparation']['wall_s']:.1f}" for r in ('Alpha','Bob','Carol'))
 rss=max(p['runtime']['sampled_peak_rss_kib'].values())/(1024**2)
 lines.append(f"| {name.replace('S3E_','').replace('_',' ')} | {prep} | {p['evaluation_wall_s']:.1f} | {rss:.2f} |")
lines+=['','Descriptor times are elapsed per robot; Road 2 prepared robots concurrently, while Roads 1 and 3 prepared them sequentially. The largest backend process RSS is the maximum sampled process peak, not aggregate memory. Initial batch playback ran from 10:20:44 to 10:55:38 UTC; later storage repairs, the Carol replay, backends and verification are separate stages.']
lines+=['','Backend completion means the configured 100 local settling iterations finished after peer input completion; it is not a claim of mathematical convergence. These are single runs, not a controlled comparison against earlier temporal or area-odometry experiments. A connected graph does not guarantee better accuracy for every robot.','',
'**Frontend completion, update gaps and processing**','',
'An update gap is elapsed sensor time between successful LiDAR corrections, including the end interval. It differs from wall-clock processing time. The stability check requires full sensor-tail coverage, finite chronological poses, scan-derived speed ≤20 m/s and no successful-update gap ≥1 s. It does not impose an ATE cutoff on this requested batch.','',
'| Sequence | Robot | Stability | Poses | Max update gap (s) | Max speed (m/s) | Processing mean / p95 (ms) | Peak mapper RSS (GiB) | Replay wall (s) | Area snapshots |',
'|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
for name,g in data['groups'].items():
 for r,f in g['frontends'].items():
  q=f['quality'];lines.append(f"| {name.replace('S3E_','').replace('_',' ')} | {r} | {'pass' if q['passed'] else 'FAIL'} | {f['native_poses']} | {q['max_successful_update_gap_s']:.3f} | {q['max_speed_m_s']:.3f} | {f['processing_mean_ms']:.1f} / {f['processing_p95_ms']:.1f} | {f['peak_mapper_rss_mib']/1024:.2f} | {f['summary']['wall_s']:.1f} | {f['area_snapshots']} |")
lines+=['',data['processing_timing_scope']+'. Times include snapshot/writer backpressure where it occurs. RSS is the native mapper process high-water mark, not whole-machine peak usage. Six original frontend workers shared workstation 148; the corrected Carol replay ran separately. These conditions do not provide an isolated throughput benchmark. Correspondence age is explicitly unmeasured, with null fields in diagnostics. All native diagnostics report persistent-map odometry and zero handovers.','',
'Laboratory 4 / Bob completed playback but produced a trajectory-implied peak speed of 602.11 m/s, a 28.90 s successful-update gap, and 665 unsuccessful updates after initialization. Alpha and Carol passed stability. Laboratory 4 GT files contain only two samples at 0/1 s with no overlap with sensor timestamps, so no valid ATE is reported. This remains a genuine frontend failure; no tuning sweep was performed.','',
f'[Laboratory 4 diagnostic plots]({prefix}/S3E_Laboratory_4/diagnostics/frontend-performance.png) and [unchanged raw trajectory files]({prefix}/S3E_Laboratory_4/raw-evaluation-after-failure/) are retained.','','**Area definition and fixed settings**','',
'After matching and inserting each corrected scan, the exporter can snapshot all stored native point representatives within an 80 m horizontal disk around the current corrected IMU position in the estimated gravity plane. Membership has no height or scan-age limit. A new snapshot is triggered after 20 m horizontal displacement or 10 s, plus a nonduplicate shutdown tail. The trigger is not a temporal membership window. Native tensors keep their persistent-map neighborhood support; export does not refit them. Ellipsoid centers and sampled surfaces are restricted to the horizontal area.','',
'Both descriptors and geometric evidence use the same snapshot, represented in its IMU anchor frame. These are native processed map representatives, not full-resolution raw scans. Schema 5 records point/scan membership, anchor time, later availability time and payload hashes. Retrieval is scheduled by availability; graph poses retain anchor timestamps. Same-robot endpoints retain the 30 s exclusion and reject any shared persistent point IDs.','',
'Calibration comes from each dataset version’s Alpha/Bob/Carol files. All captures retain reliable input, the four-thread launcher and IMU acc/gyr noise 0.1 / 0.1 with bias terms 0.0001 / 0.0001. The separate research deskew exporter is disabled. Workstation 148 containers have no CPU quota, CPU affinity restriction or memory cap.','',
'MapClosures uses the existing gravity-aligned ellipsoid surface projection, 0.5 m density pixels, density threshold 0.05, Hamming threshold 50, five inliers and twenty hypotheses. Verification retains 0.4 m preprocessing, minimum 100 points, overlap 0.3, RMSE 0.35 m and 1.5 m correspondence distance, without a 3D spherical range filter. Distributed PCM retains probability 0.99, minimum clique size 2 and 60 s timeout. CBS and optional geometric registration factors use the existing settings; no threshold was tuned.','',
'**Failures repaired without changing estimator or backend thresholds**','',
'1. Campus Road 2 / Carol hit the upstream ten-million-octant allocation limit at about 22.4 min, after retaining 7.44 million map points. Stable dynamically allocated octree leaf blocks preserve existing pointers and query behavior. The full replay then finished with 8,831,645 retained map points and passed the frontend checks. Its original partial capture remains retained and is excluded from the final ATE table.','2. Descriptor preparation exceeded the CUDA sampler’s fixed voxel hash capacity. The table now grows and rehashes integer sums/counts. Forced-growth tests matched the original sampler’s centroids bit-for-bit across repeated batches and reset/reuse; voxel resolution and sampling were unchanged. All reported backends use this fix.','3. Some large live Rerun recordings passed data-stream verification but exceeded the footer Arrow table limit. Lossless entity partitions retain all source entities and recording IDs; each partition passes full verification. Recombined data were compared with the source, using equal compaction settings where needed. Original recordings remain untouched. Open every part together for the complete recording.','',
'**Verification and provenance**','',
'Native accumulated-area and bounded-writer tests passed; after the capacity fix the native suite passed 3/3. The octree growth test also passed AddressSanitizer, UndefinedBehaviorSanitizer and LeakSanitizer. Python integration tests passed 40/40, including actual DDS/PCM/CBS operation. Source comparison checks retain the upstream estimator function bodies; the later native capacity change is confined to octree storage and test registration.','']
for name,g in data['groups'].items():
 if g['status']=='complete':lines.append(f"- {name.replace('S3E_','').replace('_',' ')}: {g['audit']['descriptor_evidence_memberships']} descriptor/evidence memberships, {g['audit']['causal_ranked_events']} causal retrieval events, finite timestamp-matched graph poses, and verified derived Rerun recording. Gallery: {g['gallery']['images']} sampled images reproduce cached ORB features exactly.")
lines+=['',f"Upstream EllipseLIO revision: `{data['upstream_commit']}`. Source worktrees are on `dev/upstream-ellipselio-area-mapclosures` and `dev/upstream-ellipselio-octree-growth`; integration changes are retained but not committed or pushed by this experiment. [Source patches, new files, tests and hashes]({prefix}/provenance/implementation/sha256.json), [complete audited results]({prefix}/final-summary.json), and [initial failure record]({prefix}/provenance/initial-batch-summary.json) are retained.",'',
'All full trajectories, native diagnostics, area payloads, maps, descriptors, constraints, PCM decisions and recordings remain on 148 under `/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-s3e-20260921/`. `full/` preserves initial captures; `capacity-run/frontends/Carol/` holds the corrected replay; `recovered-cuda/` holds the three complete backends. Bulk geometry has not been retired. Existing experiments and the paused CU-Multi download were preserved.','',
'**Figures and galleries**','',f'![Raw and individual CBS ATE]({prefix}/ate-comparison.png)','']
for name,g in data['groups'].items():
 if g['status']!='complete':continue
 label=name.replace('S3E_','').replace('_',' ')
 lines+= [f'[{label}: trajectories and maps]({prefix}/{name}/report/trajectories-maps.png) · [BEV gallery]({prefix}/{name}/bev-gallery/index.html) · [frontend diagnostics]({prefix}/{name}/diagnostics/frontend-performance.png) · [numeric report]({prefix}/{name}/report/report.json) · [Rerun recording]({prefix}/{name}/report/result.rrd)','']
(root/'RESULTS-UPSTREAM-AREA-S3E.md').write_text('\n'.join(lines)+'\n')
print(root/'RESULTS-UPSTREAM-AREA-S3E.md')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
fig,axs=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for ax,(name,g) in zip(axs,((n,g) for n,g in data['groups'].items() if g['status']=='complete')):
 p=g['report'];robots=['Alpha','Bob','Carol'];x=np.arange(3)
 for offset,key,label,color in [(-.18,'raw','Raw','#4477aa'),(.18,'cbs_individual','CBS','#ee7733')]:
  values=[p[key][r]['rmse_m'] for r in robots]
  bars=ax.bar(x+offset,values,.34,color=color,label=label)
  ax.bar_label(bars,fmt='%.2f',padding=3,fontsize=9)
 ax.set_xticks(x,robots);ax.set_ylabel('Position ATE RMSE [m]');ax.set_ylim(0,max([p[k][r]['rmse_m'] for k in ('raw','cbs_individual') for r in robots])*1.22)
 shared=next(iter(p['cbs'].values()))['rmse_m']
 ax.set_title(f"{name.replace('S3E_','').replace('_',' ')}\nShared CBS ATE: {shared:.2f} m")
 ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axs[0].legend(frameon=False)
fig.suptitle('Updated persistent-map EllipseLIO + accumulated-area MapClosures / PCM / CBS\nBars: independent rigid alignment per robot; no scale fitting',fontsize=12)
fig.savefig(assets/'ate-comparison.png',dpi=180);plt.close(fig)
import html
cards=[]
for name,g in data['groups'].items():
 if g['status']!='complete':continue
 p=g['report'];ate=next(iter(p['cbs'].values()))['rmse_m'];loops=p['pcm']['retained_loops']
 label=html.escape(name.replace('S3E_','').replace('_',' '))
 cards.append(f'<article><h2>{label}</h2><a href="{name}/bev-gallery/index.html"><img src="{name}/bev-gallery/overview.png" alt="{label} BEV overview"></a><p>{loops} retained loops · shared CBS ATE {ate:.3f} m</p><p><a href="{name}/bev-gallery/index.html">Interactive BEV gallery</a> · <a href="{name}/report/trajectories-maps.png">Trajectories and maps</a> · <a href="{name}/report/result.rrd">Rerun</a></p></article>')
(assets/'index.html').write_text('''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Updated EllipseLIO — S3E results</title><style>body{font:16px system-ui;margin:32px;background:#f3f5f8;color:#182330}main{max-width:1400px;margin:auto}p{line-height:1.5}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:20px}article{padding:20px;background:white;border-radius:10px}img{max-width:100%;height:auto}a{color:#1859a1}</style><main><h1>Updated EllipseLIO — accumulated-area MapClosures / PCM / CBS</h1><p>21 September 2026 · Alpha, Bob and Carol · workstation 148 without CPU or memory quotas.</p><p>Odometry uses the updated persistent map. Retrieval uses all stored native representatives within an 80 m horizontal radius, without a scan-age or height cutoff. Galleries show selected snapshots whose images reproduce the cached ORB features exactly.</p><p><strong>Laboratory 4 failed:</strong> Bob diverged, so that group’s backend was not run. The three Campus Road groups completed; CBS did not improve every robot.</p><p><a href="../../RESULTS-UPSTREAM-AREA-S3E.md">Full report</a> · <a href="final-summary.json">Audited numeric results</a></p><div class="cards">'''+''.join(cards)+'''</div><p><img src="ate-comparison.png" alt="Raw and individual CBS position ATE"></p></main></html>''')
