#!/usr/bin/env python3
"""Compact figures and Rerun for the completed paired Swarm-SLAM comparison."""
import argparse
import json
from pathlib import Path
import numpy as np

ROBOTS=('Alpha','Bob','Carol')
COLORS={'Alpha':'#df554f','Bob':'#3b97dc','Carol':'#52ac76'}
LABELS={'raw':'Our raw BEV + PGO','ellipsoid':'Our ellipsoid BEV + PGO','swarm':'Swarm-SLAM LiDAR'}

def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(x) for x in p.read_text().splitlines()]
def combined(m):
    return next(iter(m.values()))['rmse_m'] if len(m)==1 and len(next(iter(m.values()))['robots'])==3 else None
def error(m,r):return next((v['per_robot'][r]['statistics']['rmse'] for v in m.values() if r in v['per_robot']),None)
def fmt(x):return 'unavailable' if x is None else f'{x:.4f}'

def branches(report):
    return {k:report['ours']['branches'][k] for k in ('raw','ellipsoid')} | {'swarm':report['swarm']}

def figures(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from s3e_pipeline.evaluation import ground_truth
    report=read(output/'report.json');models=branches(report);tracks={};groups={};truth={}
    gt_available=bool(report['ours']['raw_odometry'])
    if gt_available:
        for robot in ROBOTS:truth[robot]=ground_truth(Path(report['dataset'])/f'{robot.lower()}_gt.txt')[1]
        origin=truth['Alpha'][0]
    else:origin=np.zeros(3)
    for b,v in models.items():
        source=output/('swarm' if b=='swarm' else f'ours/{b}')/'poses.jsonl'
        groups[b]={c:[r for r in ROBOTS if v['components'].get(r)==c] for c in sorted(set(v['components'].values()))}
        if not source.exists():continue
        data=rows(source)
        for robot in ROBOTS:
            xyz=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in data if r['robot_id']==robot])
            component=v['components'][robot]
            if component in v['trajectory']:
                T=np.asarray(v['trajectory'][component]['alignment_SE3']);xyz=xyz@T[:3,:3].T+T[:3,3]-origin
            tracks[f'{b}_{robot}']=xyz
    height=max(1,max(map(len,groups.values())))
    fig,axes=plt.subplots(height,3,figsize=(16,4.5*height),squeeze=False,layout='constrained')
    for col,(b,components) in enumerate(groups.items()):
        for row in range(height):
            ax=axes[row,col]
            if row>=len(components):
                if row==0 and not components:
                    ax.text(.5,.5,'Result unavailable\nSee report',ha='center',va='center',transform=ax.transAxes)
                    ax.set_title(LABELS[b])
                ax.axis('off');continue
            component=list(components)[row];robots=components[component]
            for robot in robots:
                xyz=tracks[f'{b}_{robot}'];ax.plot(xyz[:,0],xyz[:,1],color=COLORS[robot],lw=1.1,label=robot)
                if gt_available and component in models[b]['trajectory']:
                    gt=truth[robot]-origin;ax.plot(gt[:,0],gt[:,1],color=COLORS[robot],ls=':',lw=.8,alpha=.65)
            value=models[b]['trajectory'].get(component,{}).get('rmse_m')
            detail=f'ATE {value:.4f} m' if value is not None else 'ATE unavailable'
            ax.set_title(LABELS[b]+'\n'+' + '.join(robots)+f' · {detail}')
            ax.set_aspect('equal');ax.set_xlabel('X [m]');ax.set_ylabel('Y [m]');ax.legend();ax.grid(alpha=.2)
    sequence=Path(report['dataset']).name.removeprefix('S3E_').replace('_',' ')
    if report.get('integration_fixture'):sequence='INTEGRATION FIXTURE — NOT A BENCHMARK'
    fig.suptitle(sequence+' · identical fresh EllipseLIO inputs\n'+('Dotted: position GT; one evo SE(3) alignment per component, no scale fit' if gt_available else 'Independent component frames; no timestamped trajectory GT'))
    fig.savefig(output/'trajectories.png',dpi=180);fig.savefig(output/'trajectories.pdf');plt.close(fig)
    if gt_available and any(v['trajectory'] for v in models.values()):
        fig,ax=plt.subplots(figsize=(10,5),layout='constrained');x=np.arange(3)
        for offset,(b,v) in enumerate(models.items()):
            values=[error(v['trajectory'],r) for r in ROBOTS]
            if any(x is None for x in values):continue
            bars=ax.bar(x+(offset-1)*.24,values,width=.24,label=LABELS[b])
            ax.bar_label(bars,fmt='%.3f',padding=3)
        ax.set_xticks(x,ROBOTS);ax.set_ylabel('Position ATE RMSE [m]');ax.legend();ax.grid(axis='y',alpha=.2)
        ax.set_title(sequence+' · individual errors after each component’s shared alignment')
        fig.savefig(output/'ate.png',dpi=180);fig.savefig(output/'ate.pdf');plt.close(fig)
    np.savez_compressed(output/'trajectory-view.npz',**tracks,**{f'gt_{r}':p-origin for r,p in truth.items()})
    (output/'visualization.json').write_text(json.dumps(dict(groups=groups,gt_available=gt_available),indent=2)+'\n')

def markdown(output):
    report=read(output/'report.json');models=branches(report);run=report['swarm_run'];swarm=report['swarm']
    sequence=Path(report['dataset']).name.removeprefix('S3E_').replace('_',' ')
    lines=[f'# {sequence}: Swarm-SLAM LiDAR comparison','',
        'All systems consume the same fresh EllipseLIO odometry and full deskewed LiDAR exports.',
        '[Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) runs its actual ROS2 LiDAR nodes for each robot.',
        'Our raw and ellipsoid BEV paths use MapClosures and centralized GNC-TLS PGO.',
        'This compares complete loop/optimization paths; keyframes, map history and factor noise differ.', '',
        f'Native run status: **{"complete" if run["success"] else "incomplete"}**.', '']
    if not run['success']:lines += [f'Reason: `{run.get("error","not available")}`. The strict all-scan replay check did not pass.','']
    if swarm.get('usable_keyframe_result'):
        lines += ['**Selected-keyframe input audit passed:** every native selected timestamp and odometry pose matches an independent replay of the native selector over all exported scans;',
                  'every selected descriptor and optimized keyframe is present. The trajectory below uses those native results.','']
        if not run['success']:
            lines += ['Unacknowledged scans did not change any selected keyframe. This supports the loop/PGO comparison on the selected inputs,',
                      'but does not establish lossless all-scan transport. The failed strict replay status is preserved.','']
    else:
        lines += ['A complete three-robot native trajectory is unavailable. No missing robot estimates are replaced with odometry and no complete-run ATE is invented.','']
    if report.get('integration_fixture'):lines[0]='# Integration fixture only — not a sequence benchmark'
    if 'failure' in report['ours']:
        lines += ['**Paired BEV comparison unavailable:** '+report['ours']['failure']['reason'],
                  'The shared frontend exports and Swarm run are evaluated independently. No historical ATE is substituted for this fresh run.','']
    lines+=['## Position ATE','',
        'RMSE in metres, evaluated entirely through evo 1.36.5 on the same dense frontend timestamps.',
        'One shared rigid alignment per connected component, no scale fitting, no additional robot fit.', '',
        '| Method | Combined | Alpha | Bob | Carol |','|---|---:|---:|---:|---:|']
    lines.append('| Raw odometry (separate robot fits) | unavailable | '+' | '.join(fmt(error(report['ours']['raw_odometry'],r)) for r in ROBOTS)+' |')
    for b,v in models.items():lines.append('| '+LABELS[b]+' | '+fmt(combined(v['trajectory']))+' | '+' | '.join(fmt(error(v['trajectory'],r)) for r in ROBOTS)+' |')
    if not report['ours']['raw_odometry']:
        lines+=['','Laboratory 1 has endpoint records labeled 0 and 1, without sensor timestamps.',
            '**ATE and GT-based loop accuracy are unavailable.** Endpoints were not interpolated or remapped into a trajectory.',
            'Connectivity and appearance do not establish a trajectory-accuracy improvement.']
    if run.get('native_optimizer_error_count',0):lines+=['',f'**Native optimizer warnings:** {run["native_optimizer_error_count"]} solver errors were logged; upstream may return its initial estimates on failure. Inspect the retained native logs.']
    lines+=['','![Trajectories](trajectories.png)']
    if (output/'ate.png').exists():lines+=['','![Individual ATE](ate.png)']
    lines+=['','## Loop and graph outcomes','','| Method | Keyframes | Verification attempts | Accepted registrations | Accepted inter | Output frames |','|---|---:|---:|---:|---:|---:|']
    for b,v in models.items():
        if v.get('available') is False:
            lines.append(f'| {LABELS[b]} | unavailable | unavailable | unavailable | unavailable | unavailable |')
            continue
        if b=='swarm':n=sum(run['received_keyframes']);accepted=v['accepted'];inter=v['accepted_inter']
        else:
            n=sum(report['ours']['keyframes'].values());edges=rows(output/f'ours/{b}/constraints.jsonl');accepted=len(edges);inter=sum(e['i'][0]!=e['j'][0] for e in edges)
        lines.append(f'| {LABELS[b]} | {n} | {v["verification_attempts"]} | {accepted} | {inter} | {len(set(v["components"].values())) or "unavailable"} |')
    lines+=['','Accepted registrations are counted before robust optimization. Swarm-SLAM does not export GNC weights in its native result messages;',
        'its output frames reflect native origin IDs, and should not be interpreted as a verified graph of GNC-selected inter-robot factors.',
        f'Our GNC-selected loops: raw **{models["raw"].get("selected_loops","unavailable")}**, ellipsoid **{models["ellipsoid"].get("selected_loops","unavailable")}**.', '',
        f'Swarm GT-checkable registrations: **{swarm["gt_checkable"]}/{swarm["accepted"]}**; endpoint-distance discrepancies above 2 m: **{swarm["flagged"] if swarm["gt_checkable"] else "unavailable"}**.',
        'This position-only test compares translation magnitude against GT endpoint separation. It is not a complete 6-DoF outlier test.', '',
        '## Runtime and delivery','']
    if run.get('total_wall_s') is not None:
        lines += [f'Swarm wall time: **{run["total_wall_s"]:.2f} s**, including **{run["sensor_replay_wall_s"]:.2f} s** of sensor replay and',
            f'**{run.get("backpressure_s",0):.2f} s** spent waiting for processing capacity during that replay.',
            'These values include native timer budgets and final settling; sequence jobs can overlap on the same CPU.']
    lines += ['',f'Our detection-only wall times: raw **{fmt(models["raw"].get("detection",{}).get("wall_runtime_s"))} s**, ellipsoid **{fmt(models["ellipsoid"].get("detection",{}).get("wall_runtime_s"))} s**.',
        '**These wall-time definitions are different and do not support a direct speedup ratio.**', '',
        f'Swarm observed coordination payload: **{swarm["observed_cdr_bytes"]/2**20:.2f} MiB** of serialized CDR.',
        'This counts each observed publication once on the listed topics, including local deliveries; DDS transport/discovery overhead and broadcast fanout are excluded.',
        'Our communication figures use compressed simulated envelopes, so the byte totals have different wire semantics.', '',
        f'Scans published/acknowledged: **{run["published_scans"]} / {run.get("processed_scans","unavailable")}**.',
        f'Keyframes expected/observed: **{run["expected_keyframes"]} / {run["received_keyframes"]}**; descriptors: **{run["received_descriptors"]}**.', '',
        '## Setup and adaptations','',
        'The pinned checkout uses native Scan Context, FPFH/TEASER++ with ICP refinement, spectral candidate selection,',
        'and elected-robot GTSAM optimization. The LiDAR pose-factor direction is corrected by inverting the',
        'source-to-target registration transform; forward/reversed synthetic tests validate that convention.',
        'GTSAM 4.3 compatibility changes replace removed shared-pointer, quaternion and value-filter APIs.',
        'A LiDAR-only build avoids compiling the unrelated visual frontend. A processing acknowledgement',
        'adds bounded replay backpressure without changing keyframe selection or registration.', '',
        'Native LiDAR settings come from upstream graco_lidar.yaml: 0.5 m keyframe distance/voxel size,',
        'similarity threshold 0.8, more than 60 registration inliers, 20-keyframe intra exclusion,',
        'one selected inter-robot candidate per five-second detection period. Synchronization tolerance is 1 ms',
        'for exact timestamp pairs; the operational backend wait timeout is 60 seconds. Logs are enabled.', '',
        'Swarm uses a full scan per keyframe; our raw BEVs use trailing five-second submaps and ellipsoid BEVs use persistent spatial maps.',
        'All loop selection and optimization exclude GT. GT orientations are unused; the antenna lever arm is uncorrected.', '',
        '## Artifacts','',
        '[Rerun trajectories](trajectories.rrd), [machine-readable comparison](report.json),',
        '[native run](swarm/summary.json), [native loop checks](swarm/loop-quality.jsonl), and [our paired report](ours/REPORT.md).',
        'The project setup and adaptation details are in `Swarm-SLAM/s3e/README.md`.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')

def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    metadata=read(output/'visualization.json');groups=metadata['groups']
    with np.load(output/'trajectory-view.npz') as f:data={k:f[k] for k in f.files}
    rr.init(output.name);rr.save(str(output/'trajectories.rrd'))
    columns=[]
    for branch,components in groups.items():
        views=[]
        for component,robots in components.items():
            base=f'/{branch}/{component}'
            views.append(rrb.Spatial3DView(name=LABELS[branch]+' · '+' + '.join(robots),origin=base))
            rr.log(base,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
            for robot in robots:
                color=[int(COLORS[robot][i:i+2],16) for i in (1,3,5)]
                rr.log(base+'/'+robot+'/trajectory',rr.LineStrips3D([data[f'{branch}_{robot}']],colors=color,radii=.04),static=True)
                if 'gt_'+robot in data:rr.log(base+'/'+robot+'/GT',rr.LineStrips3D([data['gt_'+robot]],colors=[*color,90],radii=.018),static=True)
        if views:columns.append(rrb.Vertical(*views))
    layout=(rrb.Vertical(rrb.Horizontal(*columns),rrb.TextDocumentView(origin='/report'),row_shares=[.8,.2])
            if columns else rrb.TextDocumentView(origin='/report'))
    rr.send_blueprint(rrb.Blueprint(layout,collapse_panels=True))
    rr.log('/report',rr.TextDocument((output/'REPORT.md').read_text(),media_type=rr.MediaType.MARKDOWN),static=True)
    rr.get_data_recording().flush()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--rerun',action='store_true');args=p.parse_args()
    if args.rerun:rerun(args.output.resolve())
    else:figures(args.output.resolve());markdown(args.output.resolve())
