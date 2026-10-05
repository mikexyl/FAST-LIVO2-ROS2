#!/usr/bin/env python3
"""Compact report and gravity-frame previews for a completed circle pipeline."""
import argparse
from collections import Counter
import html
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/circle-all14-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.circle_map_export import DTYPE


def preview(ply, target):
    # Independent PLY reader verifies standard XYZ/RGB interoperability.
    import open3d as o3d
    cloud=o3d.io.read_point_cloud(str(ply))
    xyz=np.asarray(cloud.points); rgb=np.asarray(cloud.colors)
    if not len(xyz) or not cloud.has_colors() or not np.isfinite(xyz).all():
        raise ValueError('Independent PLY readback failed')
    count=len(xyz)
    sample=np.linspace(0,count-1,min(count,350000),dtype=np.int64)
    points=xyz[sample].copy();colors=rgb[sample].copy()
    bounds=[xyz.min(0).tolist(),xyz.max(0).tolist()]
    del cloud,xyz,rgb
    yaw,elevation=np.deg2rad([35.,55.])
    across=np.array([-np.sin(yaw),np.cos(yaw),0.])
    camera=np.stack([[np.cos(yaw),np.sin(yaw),0.],
        across*np.sin(elevation)+[0.,0.,np.cos(elevation)],
        -across*np.cos(elevation)+[0.,0.,np.sin(elevation)]])
    q=(points-np.mean(points,axis=0))@camera.T
    order=np.argsort(q[:,2],kind='stable')
    fig,axes=plt.subplots(1,2,figsize=(16,7),facecolor='white')
    for ax,view,title in [(axes[0],points,'Top view'),(axes[1],q,'Oblique view')]:
        indices=np.argsort(points[:,2]) if ax is axes[0] else order
        ax.scatter(view[indices,0],view[indices,1],c=colors[indices],s=.16,linewidths=0,rasterized=True)
        ax.set_aspect('equal');ax.set_title(title);ax.set_axis_off()
    fig.tight_layout();fig.savefig(target,dpi=180,bbox_inches='tight');plt.close(fig)
    return dict(reader='Open3D '+o3d.__version__,vertices=count,has_rgb=True,bounds_m=bounds,
        preview_points=len(points),preview_only_thinned=True)


def run(args):
    root=args.run;out=args.output;out.mkdir(parents=True,exist_ok=False)
    done=read_json(root/'COMPLETE.json');assert done['success']
    cfg=read_json(root/'configuration.json');prep=read_json(root/'capture/preparation-summary.json')
    detection=read_json(root/'capture/central/summary.json');summary=read_json(root/'solution/cbs/summary.json')
    evaluation=read_json(root/'solution/evaluation/result.json');exported=read_json(root/'exports/export.json')
    graph=read_json(root/'solution/central/graph.json');resources=read_json(root/'resources.json')
    graph_resources=read_json(root/'solution/resources.json');comm=read_json(root/'solution/communication.json')
    rows=[]
    for r in cfg['robots']:
        p=prep['robots'][r]['placement'];s=summary['robots'][r]
        rows.append([r,str(p['footprints']),f"{p['metrics']['coverage']:.2%}",
            str(exported['robots'][r]['points']),s['component'],
            str(s['registration']['registration_factor_count']),
            f"{evaluation['raw_ate'][r]['rmse_m']:.3f}",f"{evaluation['individual_ate'][r]['rmse_m']:.3f}"])
    components={k:v['robots'] for k,v in evaluation['joint_ate'].items()}
    intro=(f"Fresh circle placement, point-cloud BEVs, multilayer loop verification and persistent PCM/CBS "
        f"ran for all {len(cfg['robots'])} GRACO robots on 148. There are {sum(int(r[1]) for r in rows)} "
        f"40 m circles, {detection['proposed_loops']} verified loops, {summary['loops']} PCM-retained loops, "
        f"and {len(components)} final connected component(s).")
    joint='; '.join(f"{k} ({len(v['robots'])} robots): {v['rmse_m']:.3f} m" for k,v in evaluation['joint_ate'].items())
    scope=('Retained validated EllipseLIO odometry and native map observations were reused. Circle geometry '
        'was sealed at capture end. Retrieval used the existing staged descriptor/evidence services; PCM, '
        'geometry-factor exchange and CBS ran in independent robot processes. No centralized graph solve '
        'or ground-truth initialization was used. This is not a fresh bag replay or online geometry-freezing trial.')
    geometry=('PLY geometry contains every unique retained native map representative once per robot, '
        're-posed using its owning optimized submap. No extra voxel downsampling was applied. Points '
        'outside all circles are retained with the nearest-circle correction and marked uncovered. '
        'Each component uses one gravity rotation from native IMU data; ground truth is not used for the PLY. '
        'Disconnected components have separate files and must not be overlaid as an aligned global map.')
    recovered=prep.get('reused_after_worker_crash',False)
    timing=(("Recovery retrieval through export (preparation reused): " if recovered else "Fresh backend through export: ")+
        f"{resources['wall_s']:.1f} s wall, "
        f"{resources['parent_cpu_s']+resources['child_cpu_s']:.1f} CPU s, "
        f"{resources['peak_simultaneous_process_tree_rss_bytes']/2**30:.2f} GiB peak simultaneous process-tree RSS. "
        f"Circle/BEV preparation: {prep['wall_s']:.1f} s. Detection/verification: {detection['wall_s']:.1f} s. "
        f"Persistent graph stage: {graph_resources['wall_s']:.1f} s, including waiting for the 10-second cadence "
        "and 30 final iterations per robot. No host resource quota was imposed. Shared pages may count twice in RSS.")
    if recovered:
        timing+=' Preparation was performed in the first attempt; its time is reported separately. The first native retrieval process segfaulted after 38 verified loops. Its outputs/logs are retained; no thresholds were changed for the restart. These are not fresh end-to-end or steady-state timing benchmarks.'
    traffic=(f"Staged descriptor/evidence IPC: {detection['communication']['total_bytes']/2**20:.2f} MiB. "
        f"Distributed graph traffic: {comm['total_graph_application_cdr_bytes']/2**20:.2f} MiB "
        f"(CBS {comm['cbs_requests_plus_responses_cdr_bytes']/2**20:.2f}, "
        f"PCM {comm['pcm_network_cdr_bytes']/2**20:.2f}, geometry {comm['registration_sent_cdr_bytes']/2**20:.2f}). "
        "These are actual serialized application messages, excluding DDS/IP overhead, discovery, odometry, "
        "and visualization/download traffic. The two stages use different transports and are reported separately.")
    residuals=graph['published_loop_residuals']
    audit=(f"All admitted geometry factors passed final native checks. Maximum published loop mismatch: "
        f"{max(r['translation_m'] for r in residuals):.3f} m / {max(r['rotation_deg'] for r in residuals):.3f} degrees. "
        "The PLY count, finite coordinates and robot IDs were checked on readback; Open3D independently reads XYZ/RGB. "
        "Evo 1.36.5 evaluates corrected IMU-reference GT with 50 ms association, one rigid alignment per component, no scale fit.")
    headers=['Robot','Circles','Cell coverage','Map points','Reference','GPU factors','Raw ATE [m]','Individual CBS ATE [m]']
    table='\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(r)+' |' for r in rows])
    copies={'configuration.json':root/'configuration.json','source-hashes.json':root/'source-hashes.json',
        'resources.json':root/'resources.json','export.json':root/'exports/export.json',
        'published-transform-audit.json':root/'exports/published-transform-audit.json',
        'evaluation.json':root/'solution/evaluation/result.json','graph-communication.json':root/'solution/communication.json',
        'graph-resources.json':root/'solution/resources.json','native-hashes.json':root/'solution/native-hashes.json',
        'cbs-summary.json':root/'solution/cbs/summary.json','pcm.json':root/'solution/cbs/pcm.json',
        'registration.json':root/'solution/cbs/registration.json','detection-summary.json':root/'capture/central/summary.json',
        'preparation-summary.json':root/'capture/preparation-summary.json',
        'reference-barrier.json':root/'solution/reference-barrier.json',
        'COMPLETE.json':root/'COMPLETE.json'}
    for name,src in copies.items():shutil.copy2(src,out/name)
    if recovered:
        failed=Path(prep['source']).parent/'FAILED.json'
        if failed.exists():shutil.copy2(failed,out/'first-attempt-failure.json')
    native=importlib.util.find_spec('s3e_mapclosures_native')
    write_json(out/'inspection-provenance.json',dict(
        inspector_sha256=file_hash(__file__),
        versions={p:importlib.metadata.version(p) for p in ('numpy','scipy','evo','open3d','matplotlib')},
        mapclosures_module=native.origin,
        mapclosures_sha256=file_hash(native.origin),
        measured_scope='Recovery retrieval through export; prior preparation reported separately' if recovered else resources['scope']))
    checks={};figures=[]
    for i,f in enumerate(exported['files']):
        checks[Path(f['path']).name]=preview(Path(f['path']),out/f'map-{i}.png')
        if checks[Path(f['path']).name]['vertices']!=f['points']:raise ValueError('Independent vertex count mismatch')
        figures.append((f'map-{i}.png',f))
    write_json(out/'independent-ply-validation.json',checks)
    paragraphs=[intro,'Component joint ATE: '+joint,scope,geometry,timing,traffic,audit]
    (out/'REPORT.md').write_text('# GRACO all 14 · point-cloud circle submaps\n\n'+'\n\n'.join(paragraphs)+'\n\n'+table+'\n')
    page='<!doctype html><meta charset="utf-8"><title>GRACO all 14 · point-cloud circles</title><style>body{font:16px/1.6 system-ui;max-width:1250px;margin:32px auto;padding:0 24px;color:#243746;background:#f6f8fa}img{width:100%;background:white}table{border-collapse:collapse;font-size:13px}td,th{padding:8px;border-bottom:1px solid #dce2e7;text-align:right}a{color:#246e91}</style><h1>GRACO all 14 · point-cloud circle submaps</h1>'
    page+=''.join('<p>'+html.escape(p)+'</p>' for p in paragraphs[:2])
    for image,f in figures:
        name=Path(f['path']).name
        page+=f'<p><a href="exports/{name}">Download {len(f["robots"])}-robot map PLY</a> · {f["points"]:,} points · {f["bytes"]/2**20:.1f} MiB</p><img src="{image}" alt="Gravity-aligned corrected map">'
    page+='<table><tr>'+''.join('<th>'+h+'</th>' for h in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+v+'</td>' for v in r)+'</tr>' for r in rows)+'</table>'
    page+=''.join('<p>'+html.escape(p)+'</p>' for p in paragraphs[2:])
    page+='<p><a href="REPORT.md">Report</a> · <a href="export.json">Export provenance</a> · <a href="independent-ply-validation.json">Independent PLY validation</a></p>'
    (out/'index.html').write_text(page)
    write_json(out/'FILES.sha256.json',{str(p.relative_to(out)):file_hash(p) for p in sorted(out.rglob('*')) if p.is_file()})


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
