"""Portable result page, using only frozen post-run numeric outputs."""
from pathlib import Path
from collections import Counter
from html import escape
import json,math
from s3e_pipeline.robot_colors import robot_colors
B=Path(__file__).resolve().parent;W=B/'full'
read=lambda p:json.loads(p.read_text())
r=read(W/'report/report.json');d=read(W/'diagnostics/summary.json');a=read(W/'retention-audit.json')
robots=list(r['raw']);components=r['components'];groups=sorted(set(components.values()))
colors={k:'#%02x%02x%02x'%tuple(v) for k,v in robot_colors(robots).items()}
loops=[json.loads(x) for x in (W/'dpgo/constraints.jsonl').read_text().splitlines()]
pairs=Counter(tuple(sorted((e['i'][0],e['j'][0]))) for e in loops)
shared={k:v['per_robot'][k]['statistics']['rmse'] for v in r['cbs'].values() for k in v['per_robot']}
loc={}
for names,x in [(robots[:8],220),(robots[8:],800)]:
 for i,k in enumerate(names):loc[k]=(x,55+i*65)
svg=['<svg viewBox="0 0 1020 570" role="img" aria-label="Retained loop connectivity between all fourteen robots">']
for (u,v),count in pairs.items():
 if u==v:continue
 x,y=loc[u];xx,yy=loc[v];cross=u.startswith('aerial')!=v.startswith('aerial')
 path=f'M{x},{y} L{xx},{yy}' if cross else f'M{x},{y} Q{x+(-145 if x<500 else 145)},{(y+yy)/2} {xx},{yy}'
 svg.append(f'<path d="{path}" fill="none" stroke="{"#58c9bc" if cross else "#758baa"}" opacity=".48" stroke-width="{1+math.log2(count+1)*.4:.2f}"><title>{u} ↔ {v}: {count} retained loops</title></path>')
for k,(x,y) in loc.items():
 labelx=x-28 if x<500 else x+28;anchor='end' if x<500 else 'start'
 svg.append(f'<circle cx="{x}" cy="{y}" r="9" fill="{colors[k]}"/><text x="{labelx}" y="{y+5}" text-anchor="{anchor}" fill="#e6edf5">{k}</text>')
svg.append('</svg>')
table=[]
for k in robots:
 f=d['frontend'][k]
 table.append(f'<tr data-robot="{k}"><td><i style="background:{colors[k]}"></i>{k}</td><td>{r["raw"][k]["rmse_m"]:.3f}</td><td>{r["cbs_individual"][k]["rmse_m"]:.3f}</td><td>{shared[k]:.3f}</td><td>{components[k]}</td><td>{f["max_successful_update_gap_s"]:.3f}</td><td>{f["processing_mean_ms"]:.1f} / {f["processing_p95_ms"]:.1f}</td><td>{f["snapshots"]}</td></tr>')
cats=''.join(f'<tr><td>{k}</td><td>{d["loop_categories"]["proposed"].get(k,0)}</td><td>{d["loop_categories"]["retained"].get(k,0)}</td></tr>' for k in ['aerial–aerial','ground–ground','aerial–ground'])
comps=''.join(f'<li><b>{escape(g)}</b>: {", ".join(k for k in robots if components[k]==g)} — shared ATE {r["cbs"][g]["rmse_m"]:.3f} m</li>' for g in groups)
cross=d['loop_categories']['retained'].get('aerial–ground',0)
loop_errors=read(W/'diagnostics/loop-errors.json')
err=loop_errors['summary']['retained'];errtext='No evaluated retained loops.'
if err['count']:errtext=f'Retained loop translation error: median {err["translation_error_m"]["median"]:.3f} m, p95 {err["translation_error_m"]["p95"]:.3f} m; rotation error: median {err["rotation_error_deg"]["median"]:.3f}°, p95 {err["rotation_error_deg"]["p95"]:.3f}°. This is a separate post-run RTK/INS diagnostic and does not change loop acceptance.'
html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>GRACO · all 14 robots</title>
<style>:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#101720;color:#e6edf5}body{max-width:1260px;margin:0 auto;padding:38px 25px}h1{font-size:2.4rem;margin:.25em 0}h2{margin-top:2.3em}p,li{line-height:1.65;color:#bac7d8}small{color:#9eb0c8}a{color:#6edacb}nav{display:flex;gap:22px;flex-wrap:wrap;margin:24px 0}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}.card{background:#192431;border:1px solid #314054;border-radius:12px;padding:20px}.card strong{display:block;font-size:2rem;color:#71dccb}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-size:.9rem}th,td{text-align:left;padding:12px;border-bottom:1px solid #304054;white-space:nowrap}th{color:#91a8c3}td i{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:8px}img{width:100%;background:white;border-radius:8px}svg{width:100%;max-height:630px;background:#151f2b;border:1px solid #304054;border-radius:10px}button{background:#24374b;color:white;border:1px solid #49617b;border-radius:6px;padding:8px 15px;cursor:pointer;margin-right:8px}.note{border-left:3px solid #e9b46b;padding-left:16px}@media(max-width:760px){.cards{grid-template-columns:repeat(2,1fr)}h1{font-size:1.9rem}}</style>
<small>WORKSTATION 148 · 22 SEPTEMBER 2026 · FRESH FULL-LENGTH 1× CAPTURES</small><h1>GRACO: eight aerial + six ground robots</h1><p>Updated EllipseLIO → accumulated-area multilayer ellipsoid BEVs → MapClosures → distributed PCM → CBS</p>
'''
html+=f'<div class="cards"><div class="card"><strong>14 / 14</strong>stable frontend captures</div><div class="card"><strong>{len(groups)}</strong>connected components</div><div class="card"><strong>{len(loops)}</strong>retained loops</div><div class="card"><strong>{cross}</strong>aerial–ground loops</div></div>'
improved=sum(r['cbs_individual'][k]['rmse_m']<r['raw'][k]['rmse_m'] for k in robots)
html+=f'<p class="note">Shared-component ATE: {", ".join(format(r["cbs"][g]["rmse_m"], ".3f")+" m" for g in groups)}. CBS improves individually aligned ATE for {improved} robots and worsens it for {len(robots)-improved}. This run establishes connectivity for these sequences; it is not a controlled comparison against the full-height BEV method.</p>'
html+='<nav><a href="gallery/index.html">Browse all 504 BEVs</a><a href="report/result-gravity.rrd">Rerun recording</a><a href="REPORT.md">Full written report</a><a href="report/report.json">Numeric results</a><a href="retention-audit.json">Verification audit</a></nav>'
html+='<h2>Trajectory accuracy</h2><p>Position ATE RMSE in metres, measured entirely with evo 1.36.5: 50 ms association, rigid alignment, no scale fitting. Individual alignment gives each trajectory its own rigid fit. Shared alignment gives every robot in a connected component the same rigid fit, exposing relative placement errors.</p><div><button onclick="filterRows(\'all\')">All robots</button><button onclick="filterRows(\'aerial\')">Aerial</button><button onclick="filterRows(\'ground\')">Ground</button></div><div class="scroll"><table><thead><tr><th>Robot</th><th>Raw ATE [m]</th><th>CBS individual [m]</th><th>CBS shared [m]</th><th>Component</th><th>Max update gap [s]</th><th>Processing mean / p95 [ms]</th><th>Submaps</th></tr></thead><tbody>'+''.join(table)+'</tbody></table></div><p>Update gaps are sensor-time intervals between successful LiDAR corrections. Processing time is native per-scan work. They measure different quantities.</p><img src="diagnostics/ate-comparison.png" alt="Raw, CBS individual, and CBS shared ATE comparison">'
html+='<h2>Connectivity and loop verification</h2><ul>'+comps+'</ul>'+''.join(svg)+'<p>Turquoise edges connect aerial and ground robots; grey edges connect robots of the same platform. Hover over an edge for the retained-loop count. Connectivity alone does not establish accurate alignment.</p><table><thead><tr><th>Loop category</th><th>Geometrically accepted</th><th>PCM retained</th></tr></thead><tbody>'+cats+'</tbody></table>'
html+=f'<p>{d["verification_attempts"]} geometric verification attempts; {d["rejected_verifications"]} rejected. PCM excluded {d["pcm_rejected"]} proposed loops. Backend wall time: {d["backend_wall_s"]:.1f} s.</p><p>{errtext} <a href="diagnostics/loop-errors.json">All loop-transform errors</a>.</p>'
html+=f'<p>CBS used {d["registration_factors"]} extra native registration factors. {len(d["skipped_registration_factors"])} retained pose constraint(s) did not receive an extra factor because of the separate CBS geometry gate. Loop counts describe submap pairs, not independent places: overlapping snapshots reuse geometry.</p>'
html+='<h2>Recovered maps</h2><p>Gravity-horizontal views computed from estimated IMU gravity. Each connected component has its own recovered frame. Ground truth does not align these displayed maps.</p><img src="diagnostics/components-map-topdown.png" alt="Recovered robot trajectories and map geometry by component">'
html+='<h2>Method and verification</h2><p>Odometry uses the updated persistent map. The loop pipeline snapshots all available native processed map representatives within an 80 m horizontal radius, every 20 m or 10 s. Five terrain-relative bands supply same-band ORB matches; pooled, deduplicated matches receive joint SE(2) RANSAC, geometric height initialization, and unchanged GICP verification. Full-height BEVs are inspection controls. PCM allows a singleton loop; native CBS includes registration factors.</p><p class="note">Terrain fitting is independent for each snapshot. A supported low surface can be a roof or another level. Failed terrain support produces empty layers. These are processed map representatives, not full-resolution raw scans. This run is an experiment with the layered adaptation, not a reproduction of ForestLPR.</p>'
html+=f'<p>All 14 live Rerun recordings and the result recording verified. Every one of the 504 gallery images reproduces its saved ORB features; descriptor/evidence memberships, causal availability, anchor timestamps, source hashes, calibration hashes and sensor hashes checked. {a["latest_tests_passed"]} distinct regression tests passed after fixture corrections; a separate 14-worker transport/PCM/CBS preflight passed. Ground truth entered only after optimization and is used for evaluation. Workstation 148 had no CPU, memory or GPU quotas. Bulk geometry and prior experiments are retained.</p>'
html+='<nav><a href="config.yaml">Configuration</a><a href="source-hashes.json">Source hashes</a><a href="runtime-audit.json">Runtime audit</a><a href="diagnostics/summary.json">Detailed diagnostics</a><a href="artifact-hashes.json">Artifact hashes</a></nav><script>function filterRows(kind){document.querySelectorAll("tr[data-robot]").forEach(e=>e.hidden=kind!=="all"&&!e.dataset.robot.startsWith(kind))}</script></html>'
(W/'index.html').write_text(html)
print(W/'index.html')
