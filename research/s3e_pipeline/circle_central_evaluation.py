"""Post-run evaluation/inspection only. Ground truth never reaches placement or PGO."""
from collections import Counter, defaultdict
from decimal import Decimal
import argparse
import html
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from .artifacts import file_hash, read_json, read_jsonl, write_json, write_jsonl
from .evaluation import corrected_dense_rows, ground_truth, save_tum, trajectory_metrics
from .geometry import inv, pose, transform, voxel_downsample
from .gravity_bev import gravity_ground
from .trajectory_continuity import continuity_summary

COLORS={'aerial05':'#cb5c57','aerial07':'#dc9f32','ground05':'#378ca4','ground06':'#6554a4'}


def tum_rows(path,robot):
    rows=[]
    for k,line in enumerate(Path(path).read_text().splitlines()):
        if not line.strip() or line.startswith('#'):continue
        values=line.split();T=np.eye(4)
        T[:3,3]=[float(v) for v in values[1:4]]
        T[:3,:3]=Rotation.from_quat([float(v) for v in values[4:8]]).as_matrix()
        rows.append(dict(robot_id=robot,frame_id=k,stamp_ns=int(Decimal(values[0])*10**9),T_world_body=T.tolist()))
    if not rows or any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])):raise ValueError('Nonchronological trajectory')
    return rows


def correction_anchors(keys,optimized):
    """Average corrections at duplicate creation times before temporal interpolation.

    Multiple circles born in one update share a native IMU anchor. Their optimized
    nodes can differ; averaging is an explicit trajectory reconstruction choice,
    while map geometry retains each circle's actual optimized node.
    """
    by_key={r['keyframe_id']:r for r in optimized};groups=defaultdict(list)
    for row in keys:groups[row['stamp_ns']].append(row)
    collapsed=[]; graph=[]; diagnostics=[]
    for stamp,group in sorted(groups.items()):
        corrections=np.stack([pose(by_key[r['keyframe_id']]['T_world_body'])@inv(pose(r['T_world_body'])) for r in group])
        C=np.eye(4);C[:3,3]=np.mean(corrections[:,:3,3],axis=0)
        C[:3,:3]=Rotation.from_matrix(corrections[:,:3,:3]).mean().as_matrix()
        key=len(collapsed);raw=dict(group[0],keyframe_id=key)
        collapsed.append(raw);graph.append(dict(keyframe_id=key,T_world_body=(C@pose(raw['T_world_body'])).tolist()))
        diagnostics.append(dict(stamp_ns=stamp,nodes=len(group),
            max_translation_correction_deviation_m=float(np.max(np.linalg.norm(corrections[:,:3,3]-C[:3,3],axis=1)))))
    return collapsed,graph,diagnostics


def corrected_union(world,keys,optimized):
    """Assign each point to its closest footprint center; use nearest outside R too.

    Shared geometry is output once. Uncovered points are preserved using the
    nearest circle correction and flagged, never silently discarded or called
    covered. No geometry registration or new measurements are introduced here.
    """
    frame=np.asarray(keys[0]['T_horizontal_map']);xy=transform(frame,world)[:,:2]
    distance,owner=cKDTree([r['footprint_center_xy'] for r in keys]).query(xy)
    by_key={r['keyframe_id']:r for r in optimized};result=np.empty_like(world,dtype=float)
    for k,row in enumerate(keys):
        mask=owner==k
        C=pose(by_key[row['keyframe_id']]['T_world_body'])@inv(pose(row['T_world_body']))
        result[mask]=transform(C,world[mask])
    return result,owner,distance>keys[0]['footprint_radius_m']


def detailed_tables(entries):
    """Report measured scopes separately; never add unrelated memory peaks."""
    def table(headers, rows):
        return '\n'.join(['| '+' | '.join(headers)+' |',
            '| '+' | '.join(['---']*len(headers))+' |']+
            ['| '+' | '.join(map(str,row))+' |' for row in rows])

    coverage=[]; timing=[]; communication=[]; resources=[]; accuracy=[]
    for label,entry in entries.items():
        summary=entry['summary']; prep=entry['preparation']['robots']; native=entry.get('native_result',{})
        for robot,row in prep.items():
            placement=row['placement']; m=placement['metrics']
            coverage.append([label,robot,m['circle_count'],f"{m['coverage']:.2%}",
                f"{placement['actual_point_coverage']:.2%}",
                f"{m['mean_memberships_per_covered_cell']:.2f}",
                f"{m['nearest_overlap_median']:.1%}",f"{m['nearest_overlap_in_band_fraction']:.1%}"])
            accuracy.append([label,robot,
                f"{entry['evaluation']['raw_ate'][robot]['rmse_m']:.3f}",
                f"{entry['evaluation']['individual_ate'][robot]['rmse_m']:.3f}"])
        stage=summary['stage']
        full=sum(row['preparation']['full_s'] for row in prep.values())
        layers=sum(row['preparation']['layers_s'] for row in prep.values())
        timing.append([label,f"{sum(row['wall_s'] for row in prep.values()):.2f}",
            f'{full:.3f}',f'{layers:.2f}',f"{stage['publication_s']:.2f}",
            f"{stage['retrieval_s']:.2f}",f"{stage['verification_and_geometry_transfer_s']:.2f}",
            f"{stage['pose_graph_s']:.3f}",f"{stage['registration_graph_s']:.2f}",
            f"{native['preparation_s']:.3f}" if native else 'unavailable',
            f"{native['optimization_s']:.3f}" if native else 'unavailable'])
        comm=summary['communication']; kinds=comm['by_kind']
        communication.append([label,summary['downloaded_endpoints'],f"{kinds.get('manifest',0)/2**20:.2f}",
            f"{kinds.get('geometry',0)/2**20:.2f}",
            f"{(comm['total_bytes']-kinds.get('manifest',0)-kinds.get('geometry',0))/2**20:.3f}",
            f"{comm['total_bytes']/2**20:.2f}"])
        r=entry['resources']
        resources.append([label,f"{r['wall_s']:.2f}",f"{r['parent_cpu_s']+r['child_cpu_s']:.2f}",
            f"{r['peak_simultaneous_process_tree_rss_bytes']/2**20:.0f}",
            f"{r['whole_device_peak_gpu_mib']:.0f}" if r.get('whole_device_peak_gpu_mib') is not None else 'unavailable'])
    graph_only=all('graph_replay_resources' in e['summary'] for e in entries.values())
    return {
        'Coverage and overlap': table(['Mode','Robot','Nodes','Occupied-cell coverage','Observed-point coverage',
            'Memberships / covered cell','Median greatest disk overlap','Circles in 20–40% overlap band'],coverage)+
            '\n\nCoverage uses equally weighted fixed-grid cells. Observed-point coverage weights native retained representatives. '
            'Disk overlap is intersection area divided by one circle’s area. Memberships count every containing disk; '
            'the final map still emits each observed representative once.',
        'Measured computation stages': table(['Mode','Preparation total [s]','Full BEV + ORB [s]',
            'Terrain + layered BEV + ORB [s]','Publication [s]','Retrieval [s]','Verification / transfer [s]',
            'Pose graph [s]','Registration stage [s]','Native preparation [s]','Native optimization [s]'],timing)+
            '\n\nPreparation totals are sequential robot preparation times including placement, union/crop construction, '
            'payload writing and BEVs. Full-height images are inspection controls. Layer time includes terrain estimation '
            'and feature extraction, so it is not pure rendering time. Native preparation and optimization are nested '
            'inside the registration stage and must not be added again. These are single measured controls, without '
            'repeated-run uncertainty estimates.',
        'Application communication': table(['Mode','Cached endpoints','Manifest [MiB]','Geometry [MiB]',
            'Session / solution [MiB]','Total [MiB]'],communication)+
            '\n\nBytes are actual serialized requests/replies plus IPC framing. On-demand geometry is transferred once '
            'per requested endpoint and reused for verification and registration. This excludes network headers, '
            'frontend traffic and inspection downloads.',
        'Graph-stage resources' if graph_only else 'Backend resources': table(['Mode','Observed wall [s]',
            'Process-tree CPU [s]','Simultaneous RSS [MiB]','Whole-device GPU peak [MiB]'],resources)+
            ('\n\nThese measurements cover only the fresh corrected graph solve and solution delivery. Detection-capture '
             'resource measurements remain in the original run. No sum of historical memory peaks is reported.' if graph_only else
             '\n\nThese measurements cover the measured backend process lifetime.')+
            ' RSS is sampled every 20 ms and may count shared pages twice. GPU is sampled every 100 ms and includes '
            'driver overhead and any unrelated device activity. CPU seconds include ready/solution services and native child processes.',
        'Individual trajectory accuracy': table(['Mode','Robot','Raw individual ATE [m]',
            'Corrected individual ATE [m]'],accuracy)+
            '\n\nEach individual row uses its own rigid alignment. The headline joint ATE instead uses one shared '
            'alignment for all four robots; individual errors cannot substitute for joint alignment quality.'
    }


def evaluate_run(run,cfg):
    run=Path(run);output=run/'evaluation';output.mkdir(exist_ok=False)
    graph=read_json(run/'central/graph.json'); dense=[]; raw=[]; maps={}; diagnostics={}; truth={}
    for robot in cfg['robots']:
        keys=read_jsonl(run/robot/'store/keyframes.jsonl');optimized=[r for r in graph['poses'] if r['robot_id']==robot]
        track=tum_rows(cfg['dense_trajectories'][robot],robot)
        # The preserved dense log must belong to exactly the same odometry capture.
        stamps=np.array([r['stamp_ns'] for r in track],dtype=np.int64)
        errors=[]
        for row in keys:
            nearest=int(np.argmin(np.abs(stamps-row['stamp_ns'])))
            if abs(int(stamps[nearest])-row['stamp_ns'])>10**6:raise ValueError('Anchor not present in retained native trajectory')
            errors.append(float(np.max(np.abs(pose(track[nearest]['T_world_body'])-pose(row['T_world_body'])))))
        if max(errors)>1e-4:raise ValueError('Dense odometry/anchor capture mismatch')
        anchors,poses,duplicates=correction_anchors(keys,optimized)
        corrected=list(corrected_dense_rows(track,anchors,poses))
        for row in corrected:row['component']=graph['components'][robot]
        raw.extend(dict(row,component=robot) for row in track);dense.extend(corrected)
        save_tum(output/f'{robot}-corrected.tum',corrected);write_jsonl(output/f'{robot}-corrected.jsonl',corrected)
        with np.load(run/robot/'union.npz',allow_pickle=False) as data:world=np.asarray(data['world'],dtype=float)
        points,owner,uncovered=corrected_union(world,keys,optimized)
        np.savez_compressed(output/f'{robot}-map.npz',points=points.astype(np.float32),owner=owner,uncovered=uncovered)
        maps[robot]=points
        diagnostics[robot]=dict(anchor_max_matrix_error=max(errors),duplicate_anchor_groups=duplicates,
            continuity=continuity_summary(track,corrected),
            map_points=len(points),uncovered_points=int(uncovered.sum()),map_policy='closest spatial center owner; outside-radius points use nearest correction and are flagged')
        truth[robot]=ground_truth(Path(cfg['dataset'])/f'{robot}_gt.txt')
    metrics=trajectory_metrics(dict(poses=dense),truth,cfg['evaluation'],output/'evo/joint')
    individual=trajectory_metrics(dict(poses=[dict(r,component=r['robot_id']) for r in dense]),truth,cfg['evaluation'],output/'evo/individual')
    raw_metrics=trajectory_metrics(dict(poses=raw),truth,cfg['evaluation'],output/'evo/raw')
    result=dict(joint_ate=metrics,individual_ate=individual,raw_ate=raw_metrics,components=graph['components'],
        reconstruction=diagnostics,trajectory_reconstruction='mean correction at simultaneous creation anchors, then temporal SE(3) correction interpolation',
        map_reconstruction='per-node spatial correction; closest-center ownership; flagged uncovered points preserved',
        source_trajectories={r:dict(path=cfg['dense_trajectories'][r],sha256=file_hash(cfg['dense_trajectories'][r])) for r in cfg['robots']},
        reference_hashes={r:file_hash(Path(cfg['dataset'])/f'{r}_gt.txt') for r in cfg['robots']})
    write_json(output/'result.json',result)
    return result


def make_report(runs,output,cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    output=Path(output);output.mkdir(exist_ok=False)
    entries={};texts=[]
    for label,run in runs.items():
        run=Path(run)
        result=read_json(run/'evaluation/result.json') if (run/'evaluation/result.json').is_file() else evaluate_run(run,cfg)
        summary=read_json(run/'central/summary.json'); prep=read_json(run/'preparation-summary.json'); resources=read_json(run/'resources.json')
        samples=[float(line.split(',')[1]) for line in (run/'gpu.csv').read_text().splitlines() if len(line.split(','))==3]
        resources['whole_device_peak_gpu_mib']=max(samples) if samples else None
        events=read_jsonl(run/'central/events.jsonl')
        reasons=Counter(e['reason'] for e in events if e['type']=='verification' and not e['accepted'])
        native=read_json(run/'central/native-result.json')
        entry=dict(run=str(run),summary=summary,preparation=prep,resources=resources,evaluation=result,
            native_result=native,verification_rejections=dict(reasons))
        entries[label]=entry
        components=defaultdict(list)
        for robot,component in result['components'].items():components[component].append(robot)
        fig,axes=plt.subplots(1,len(components),figsize=(8*len(components),7),squeeze=False)
        for ax,(component,robots) in zip(axes[0],sorted(components.items())):
            alignment=np.asarray(result['joint_ate'][component]['alignment_SE3'])
            for robot in robots:
                with np.load(run/'evaluation'/f'{robot}-map.npz') as f:points=np.asarray(f['points'],dtype=float)
                points=transform(alignment,points)
                if len(points)>150000:points=points[::int(np.ceil(len(points)/150000))]
                ax.scatter(points[:,0],points[:,1],s=.22,c=COLORS[robot],alpha=.25,rasterized=True)
                track=tum_rows(run/'evaluation'/f'{robot}-corrected.tum',robot)
                xyz=transform(alignment,np.array([np.asarray(r['T_world_body'])[:3,3] for r in track]))
                ax.plot(xyz[:,0],xyz[:,1],lw=1.5,c=COLORS[robot],label=robot)
                truth=ground_truth(Path(cfg['dataset'])/f'{robot}_gt.txt')[1]
                ax.plot(truth[:,0],truth[:,1],lw=.7,c=COLORS[robot],ls='--',alpha=.8)
            ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(frameon=False)
            ax.set_xlabel('ENU x [m]');ax.set_ylabel('ENU y [m]')
            ax.set_title(f"{label} · {' + '.join(robots)}\nJoint ATE {result['joint_ate'][component]['rmse_m']:.2f} m; GT dashed")
        fig.tight_layout();fig.savefig(output/f'{label}-joint-map.png',dpi=160);plt.close(fig)
        # Footprints in each robot's original horizontal map frame, not a GT frame.
        fig,axes=plt.subplots(2,2,figsize=(12,10))
        for ax,robot in zip(axes.ravel(),cfg['robots']):
            keys=read_jsonl(run/robot/'store/keyframes.jsonl')
            with np.load(run/robot/'union.npz') as f:world=np.asarray(f['world'],float)
            xy=transform(np.asarray(keys[0]['T_horizontal_map']),world)[:,:2]
            if len(xy)>100000:xy=xy[::int(np.ceil(len(xy)/100000))]
            ax.scatter(xy[:,0],xy[:,1],s=.25,color='#999999',alpha=.35)
            for row in keys:
                ax.add_patch(Circle(row['footprint_center_xy'],row['footprint_radius_m'],fill=False,lw=.8,color=COLORS[robot],alpha=.8))
            p=prep['robots'][robot]['placement'];ax.set_title(f"{robot}: {len(keys)} circles; {p['metrics']['coverage']:.2%} cells")
            ax.set_aspect('equal');ax.autoscale();ax.grid(alpha=.15)
        fig.tight_layout();fig.savefig(output/f'{label}-footprints.png',dpi=150);plt.close(fig)
        image_root=output/label;image_root.mkdir();bev=[]
        for robot in cfg['robots']:
            (image_root/robot).mkdir()
            for p in sorted((run/robot/'images').glob('*.png')):
                shutil.copy2(p,image_root/robot/p.name)
                bev.append(f'<figure><img loading="lazy" src="{robot}/{p.name}"><figcaption>{robot} / {p.stem}</figcaption></figure>')
        (image_root/'index.html').write_text('<meta charset="utf-8"><style>body{font-family:system-ui;background:#fafafa}.grid{display:grid;grid-template-columns:repeat(4,1fr)}figure{margin:10px}img{width:100%;image-rendering:pixelated}</style><h1>'+html.escape(label)+' · native point-cloud full-height BEVs</h1><p>Retrieval uses unchanged terrain-relative multilayer ORB; these full-height images are inspection controls.</p><div class="grid">'+''.join(bev)+'</div>')
        joint='; '.join(f"{'+'.join(v['robots'])}: {v['rmse_m']:.2f}" for v in result['joint_ate'].values())
        total=summary['wall_s'] if 'graph_replay_resources' in summary else resources['wall_s']
        texts.append(f"| {label} | {summary['graph_nodes']} | {summary['proposed_loops']} / {summary['retained_loops']} | {summary['registration_factors']} | {joint} | {summary['communication']['total_bytes']/2**20:.2f} | {total:.1f} | {resources['peak_simultaneous_process_tree_rss_bytes']/2**20:.0f} |")
    write_json(output/'comparison.json',entries)
    replayed=all('graph_replay_resources' in e['summary'] for e in entries.values())
    memory_header='Graph-stage RSS [MiB]' if replayed else 'Backend RSS [MiB]'
    table_header=('| Mode | Nodes | Verified / retained loops | GPU factors | Component joint ATE [m] | '
        f'Communication [MiB] | Measured backend stages [s] | {memory_header} |')
    scope_note=(
        'The centralized initializer now admits a verified singleton bridge, matching the user’s singleton-loop policy. '
        'The original two-distinct-endpoint initializer rejected valid ground bridges in the circle run. Only graph/solution '
        'delivery was repeated after fixing this policy: BEVs, retrieval proposals, verification outcomes and transferred '
        'geometry are immutable captures. The time column is the sum of separately measured publication, retrieval, '
        'verification/transfer and corrected graph stages. It excludes placement and BEV preparation and is **not** a '
        'new end-to-end latency measurement. The memory column is actual simultaneous RSS during the corrected graph '
        'stage only. Full detection-capture resources remain in the retained original runs; their peaks are not summed '
        'with graph-stage peaks.' if replayed else
        'Each time and RSS value covers that measured backend process lifetime, excluding placement and BEV preparation. '
        'Mixed fresh-capture and graph-replay reports retain the explicit per-entry measurement scopes in comparison.json; '
        'such scopes must not be treated as interchangeable end-to-end latency measurements.')
    communication_note=(
        'Communication combines actually measured pre-graph publication/retrieval/evidence streams with the actual '
        'corrected graph-session and final-pose-delivery messages. It excludes the obsolete initializer’s pose '
        'delivery/closing messages. ' if replayed else 'Communication records actual robot/server messages. ')
    report=(
        '# Centralized circle-submap comparison\n\n'
        'GRACO A05, A07, G05 and G06. Fixed circles: 40 m, 99% occupied-cell target, 1 m grid. Placement is incremental '
        'over the growing union of retained snapshots; geometry is frozen once at each capture end. This is a staged '
        'backend test, not an online replay. Identical native odometry, input union, point-cloud BEVs, terrain layers, '
        'vertical initialization and verification thresholds are reused. The centralized solver uses GNC-TLS selected '
        'pose loops followed by joint LM with live GPU VGICP factors, instead of PCM/CBS.\n\n'
        +table_header+'\n|---|---:|---:|---:|---|---:|---:|---:|\n'+'\n'.join(texts)+'\n\n'+scope_note+'\n\n'
        +communication_note+
        'Descriptors are uploaded once; selected endpoint geometry is cached and reused for registration. Network '
        'transport headers, odometry replay traffic and inspection-map downloads are excluded. RSS can double-count '
        'shared pages. GPU memory is whole-device sampling; native per-endpoint geometry counters omit buffers/driver '
        'overhead.\n\nEach connected component has one evo 1.36.5 rigid alignment, 50 ms association, no scale. Disconnected '
        'components are never pooled into a fictitious joint ATE. GRACO RTK/INS IMU-reference positions are used only '
        'during evaluation. Dense trajectories average corrections from nodes born at the same timestamp before '
        'interpolation. Map points use the optimized pose of their nearest spatial footprint, so shared points are '
        'emitted once. Outside-radius points are kept using the nearest correction and explicitly marked uncovered.\n\n'
        'The 40 m trajectory control retains original sensor-time snapshot cadence and freezes final geometry like '
        'the circle run. The 80 m reference restores the original radius under the same sealing/centralized protocol; '
        'it is not a replay of historical CBS traffic. Radius and placement effects must be distinguished.\n')
    details=detailed_tables(entries)
    report+='\n'+''.join(f'## {title}\n\n{body}\n\n' for title,body in details.items())
    (output/'REPORT.md').write_text(report)
    content='<h1>Centralized circle-submap experiment</h1><p>GRACO A05, A07, G05, G06 · staged native point-cloud backend · fixed odometry</p><p><a href="REPORT.md">Detailed protocol and results</a> · <a href="comparison.json">Numeric results</a></p>'
    content+='<table><thead><tr>'+''.join(f'<th>{html.escape(x.strip())}</th>' for x in table_header.strip('|').split('|'))+'</tr></thead><tbody>'
    for text in texts:content+='<tr>'+''.join(f'<td>{html.escape(x.strip())}</td>' for x in text.strip('|').split('|'))+'</tr>'
    content+='</tbody></table>'
    content+='<details><summary>Detailed coverage, computation, communication and accuracy tables</summary>'
    content+='<pre style="white-space:pre-wrap;font-size:13px">'+html.escape('\n\n'.join(
        title+'\n\n'+body for title,body in details.items()))+'</pre></details>'
    for label in entries:
        content+=f'<h2>{html.escape(label)}</h2><a href="{label}/index.html">BEV gallery</a><img src="{label}-joint-map.png"><img src="{label}-footprints.png">'
    (output/'index.html').write_text('<meta charset="utf-8"><style>body{font-family:system-ui;max-width:1300px;margin:40px auto;color:#202b36;background:#fafbfc}img{max-width:100%;display:block;margin:20px 0}table{border-collapse:collapse}td,th{padding:10px;border-bottom:1px solid #ddd}th{text-align:left}</style>'+content)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--output',required=True)
    p.add_argument('--run',action='append',required=True,help='label=run directory')
    args=p.parse_args();cfg=yaml.safe_load(Path(args.config).read_text())
    make_report(dict(s.split('=',1) for s in args.run),args.output,cfg)


if __name__=='__main__':main()
