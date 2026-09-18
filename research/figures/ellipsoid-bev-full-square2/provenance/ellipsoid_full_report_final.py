"""Trajectory figures and a compact full-sequence BEV comparison report."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROBOTS=('Alpha','Bob','Carol')
COLORS={'Alpha':'#df554f','Bob':'#3b97dc','Carol':'#52ac76'}


def read(path):return json.loads(path.read_text())
def rows(path):return [json.loads(x) for x in path.read_text().splitlines()]
def xyz(T,points):return points@T[:3,:3].T+T[:3,3]
def robot_error(metrics,robot):
    return next((v['per_robot'][robot]['statistics']['rmse'] for v in metrics.values() if robot in v['per_robot']),None)
def combined(metrics):return next(iter(metrics.values()))['rmse_m'] if len(metrics)==1 and len(next(iter(metrics.values()))['robots'])==3 else None
def fmt(value):return 'unavailable' if value is None else f'{value:.4f}'


def figures(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from decimal import Decimal
    report=read(output/'report.json');inputs=read(output/'inputs.json')
    truth={}
    for r in ROBOTS:
        fields=[x.split() for x in (Path(report['dataset'])/(r.lower()+'_gt.txt')).read_text().splitlines() if x.strip() and not x.startswith('#')]
        truth[r]=np.array([[float(x) for x in f[1:4]] for f in fields])
    origin=truth['Alpha'][0]
    fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
    aligned={}
    for ax,branch in zip(axes,('raw','ellipsoid')):
        metrics=report['branches'][branch]['trajectory'];track=rows(output/branch/'poses.jsonl');aligned[branch]={}
        for robot in ROBOTS:
            selected=[r for r in track if r['robot_id']==robot];component=selected[0]['component']
            T=np.asarray(metrics[component]['alignment_SE3']);p=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in selected])
            p=xyz(T,p)-origin;aligned[branch][robot]=p
            ax.plot(p[:,0],p[:,1],color=COLORS[robot],lw=1.4,label=robot)
            ref=truth[robot]-origin;ax.plot(ref[:,0],ref[:,1],color=COLORS[robot],lw=.8,ls=':',alpha=.7)
        label='Raw-cloud BEVs' if branch=='raw' else 'Ellipsoid-surface BEVs'
        value=combined(metrics);detail=f'Combined ATE {value:.4f} m' if value is not None else f'{len(metrics)} separately aligned components'
        ax.set_title(f'{label} → centralized PGO\n{detail}');ax.set_xlabel('East from GT origin [m]');ax.set_ylabel('North from GT origin [m]')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend()
    sequence=Path(report['dataset']).name.removeprefix('S3E_').replace('_',' ')
    fig.suptitle(sequence+' · same EllipseLIO odometry and GNC-TLS settings\nDotted: position ground truth; evo SE(3) alignment, no scale fit')
    fig.savefig(output/'trajectories.png',dpi=180);fig.savefig(output/'trajectories.pdf');plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,5),layout='constrained');x=np.arange(3)
    for offset,(branch,label,color) in enumerate((('raw','Raw BEV + PGO','#527aa3'),('ellipsoid','Ellipsoid BEV + PGO','#ca9957'))):
        values=[robot_error(report['branches'][branch]['trajectory'],r) for r in ROBOTS]
        bars=ax.bar(x+(offset-.5)*.32,values,width=.32,label=label,color=color)
        ax.bar_label(bars,fmt='%.3f',padding=3)
    ax.set_xticks(x,ROBOTS);ax.set_ylabel('Position ATE RMSE [m]');ax.legend();ax.grid(axis='y',alpha=.2)
    ax.set_title('Individual robot errors after each graph’s component-wide evo alignment')
    fig.savefig(output/'ate.png',dpi=180);fig.savefig(output/'ate.pdf');plt.close(fig)
    np.savez_compressed(output/'trajectory-view.npz',origin=origin,
        **{f'{b}_{r}':p for b,d in aligned.items() for r,p in d.items()},
        **{f'gt_{r}':v-origin for r,v in truth.items()})
    # Provide plot/visualizer inputs independently of the temporary MCAP data.
    metadata=dict(graphs={b:report['branches'][b]['trajectory'] for b in ('raw','ellipsoid')},report=report)
    (output/'visualization.json').write_text(json.dumps(metadata,indent=2)+'\n')


def markdown(output):
    report=read(output/'report.json');inputs=read(output/'inputs.json')
    sequence=Path(report['dataset']).name.removeprefix('S3E_').replace('_',' ')
    lines=[f'# {sequence}: raw versus ellipsoid BEVs', '',
        'Both branches use identical fresh EllipseLIO odometry, keyframes and raw geometric',
        'verification evidence. MapClosures runs in three isolated robot workers with causal',
        'simulated delivery; centralized GTSAM 4.2 GNC-TLS and a selected-inlier LM refit',
        'optimize the saved pose graphs. These results use pose factors, without CBS or live',
        'registration factors in the optimizer.', '', '## Trajectory ATE', '',
        'Position RMSE in metres, evaluated entirely through evo 1.36.5.', '',
        '| Configuration | Combined | Alpha | Bob | Carol |', '|---|---:|---:|---:|---:|']
    for label,metrics in [('Raw odometry (independent robot alignment)',report['raw_odometry']),
            ('Raw BEV + PGO',report['branches']['raw']['trajectory']),
            ('Ellipsoid BEV + PGO',report['branches']['ellipsoid']['trajectory'])]:
        lines.append(f'| {label} | {fmt(combined(metrics))} | '+' | '.join(fmt(robot_error(metrics,r)) for r in ROBOTS)+' |')
    raw=combined(report['branches']['raw']['trajectory']);ellipsoid=combined(report['branches']['ellipsoid']['trajectory'])
    if raw is not None and ellipsoid is not None:
        direction='lower' if ellipsoid<raw else 'higher'
        lines+=['',f'Ellipsoid combined ATE is **{abs(ellipsoid-raw):.4f} m ({100*abs(ellipsoid-raw)/raw:.2f}%) {direction}**',
                'than raw BEV ATE in this one paired run. This is not a repeated-trial estimate.']
    lines+=['', 'Connected trajectories receive one shared SE(3) fit across all robots, with no',
        'scale fitting or subsequent per-robot realignment. Disconnected components are',
        'independently anchored and aligned; no combined three-robot ATE is reported for',
        'a disconnected graph. Raw-odometry diagnostics use separate robot alignments.',
        'The timestamp tolerance is 0.05 s. Placeholder GT orientations are unused and the',
        'antenna lever arm is uncorrected. Ground truth enters only after optimization.', '',
        '![Trajectories](trajectories.png)', '', '![Individual ATE](ate.png)', '',
        '## Loops and computation', '',
        '| BEV input | Verification attempts | Accepted | Selected intra | Selected inter | Components |',
        '|---|---:|---:|---:|---:|---:|']
    for b in ('raw','ellipsoid'):
        v=report['branches'][b]
        lines.append(f'| {b} | {v["verification_attempts"]} | {v["constraints"]} | {v["selected_loops"]-v["selected_inter_robot_loops"]} | {v["selected_inter_robot_loops"]} | {len(set(v["components"].values()))} |')
    lines+=['', 'Verification attempts count candidate cloud pairs, not individual ORB matches.',
        'Accepted registrations and GNC-selected graph factors are distinct decisions.',
        'A larger total loop count need not imply more inter-robot matches.', '',
        '| BEV input | Detection wall time | PGO wall time | Serialized exchange | Verification task time sum |',
        '|---|---:|---:|---:|---:|']
    for b in ('raw','ellipsoid'):
        v=report['branches'][b]
        d=v.get('detection') or (read(output/b/'detection-summary.json') if (output/b/'detection-summary.json').exists() else None)
        if d:
            task=v.get('verification_task_time_sum_s')
            task_label=f'{task:.2f} s' if task is not None else 'unavailable'
            lines.append(f'| {b} | {d["wall_runtime_s"]:.2f} s | {v["optimization_s"]:.3f} s | {d["network_bytes"]/2**20:.2f} MiB | {task_label} |')
    lines+=['', 'Detection time excludes preparation. Verification task times include evidence',
        'packing and registration; their sum can exceed wall time because robot tasks',
        'overlap. It is not a pure GICP-kernel timing. Delivery is reliable/unrestricted.',
        'MapClosures and small_gicp use CPU; CUDA accelerates ellipsoid surface sampling.']
    preparation=report.get('preparation')
    if preparation:
        lines+=['', '### Descriptor workload', '',
            '| Robot | Keyframes | Raw retained ORB | Ellipsoid retained ORB | Raw describe | Ellipsoid describe | CUDA sampling |',
            '|---|---:|---:|---:|---:|---:|---:|']
        for r,v in preparation.items():
            lines.append(f'| {r} | {v["keyframes"]} | {v["raw_orb_total"]:,} | {v["ellipsoid_orb_total"]:,} | {v["raw_descriptor_s"]:.2f} s | {v["ellipsoid_descriptor_s"]:.2f} s | {v["sampling_s"]:.2f} s |')
        lines+=['', 'ORB counts sum retained features over all keyframes, including repeated',
            'observations of the same structures; they are not correspondence counts.',
            'Describe times cover native MapClosures ground alignment, density construction',
            'and ORB extraction. They exclude raw-submap assembly, ellipsoid reconstruction,',
            'surface sampling, debug rendering and artifact I/O.', '',
            f'Serial odometry replay: **{report["odometry_runtime_s"]/60:.2f} min**. Preparation of both',
            f'branches: **{sum(v["preparation_s"] for v in preparation.values())/60:.2f} min**, including',
            f'**{sum(v["sampling_s"] for v in preparation.values())/60:.2f} min** of CUDA sampling.',
            'Preparation overlaps later robot replays, so those stage totals are not additive.']
        if (output/'workload.json').exists():
            workload=read(output/'workload.json');n=sum(v['keyframes'] for v in workload.values())
            raw_points=sum(v['raw_input_points_sum'] for v in workload.values())/n
            ellipsoid_points=sum(v['ellipsoid_input_points_sum'] for v in workload.values())/n
            lines+=['',f'Native descriptor inputs average **{raw_points:,.0f} raw points** versus',
                f'**{ellipsoid_points:,.0f} sampled ellipsoid points** per keyframe. Thus construction',
                'time includes substantially different point-processing workloads; it should',
                'not be interpreted as feature-matching speed for equal inputs.']
    lines+=['', '## Fixed settings and provenance', '',
        f'Keyframes: {report["keyframes"]}. Dense frames: {report["odometry_frames"]}.', '',
        'Native ellipsoid centers, geometric semi-axes and axis directions define the',
        'rendered surfaces: 0.125 m nominal sampling, 0.25 m voxel centroids, deterministic',
        'CUDA accumulation at 0.1 micrometre resolution. Earlier validation on three real',
        'maps reproduced CPU density pixels and ORB descriptors exactly. The ellipsoid',
        'map is persistent and causal, cropped at 80 m. Raw BEVs use trailing 5-second',
        'submaps with the same crop. Different map histories remain part of the comparison.', '',
        'MapClosures uses 0.5 m pixels, density threshold 0.05, Hamming threshold 50 and',
        'more than five RANSAC inliers. Retrieval keeps a top-20 shortlist with one selected',
        'LiDAR candidate per query, a two-second cooldown and 30-second same-robot exclusion.',
        'Common raw-evidence GICP uses the existing 0.35 m RMSE, 30% overlap and observability',
        'gates. No thresholds were tuned on this sequence. Full settings are in inputs.json.']
    schedule=report.get('schedule_provenance',{})
    if schedule:
        lines+=['',f'The fixed 1 m / 10 degree / 2 s schedule comes from saved **{schedule.get("source_frontend","odometry")}**',
            'raw trajectories, without loop labels or ground truth. Both branches use fresh',
            'EllipseLIO states and the same actual export timestamps. A requested snapshot',
            'before filter initialization is associated with the first valid export, with',
            'a two-second startup allowance; later associations retain the 250 ms guard.',
            'Observed association delays and merged schedule entries are retained in the audit.']
    validation=read(output/'validation.json') if (output/'validation.json').exists() else None
    if validation:
        lines+=['', '## Validation and loop quality', '',
            f'Frozen PGO was rerun for both branches. All {validation["checked_input_files"]:,} watched',
            'odometry/keyframe/descriptor input hashes stayed unchanged. Pose differences:',
            ', '.join(f'{b}: {v["max_pose_matrix_difference"]:.3g}' for b,v in validation['pgo_reproduction'].items())+'.',
            f'The regression selection passed {validation["regression_tests_passed"]} tests.', '',
            '| BEV input | GT-checkable loops | Flagged distance discrepancies >2 m |', '|---|---:|---:|']
        for b,q in validation['loop_quality'].items():
            lines.append(f'| {b} | {q["gt_checkable"]}/{report["branches"][b]["constraints"]} | {q["flagged"]} |')
        lines+=['', 'Checks compare constraint translation magnitude with GT endpoint separation.',
            'Both endpoints need GT interpolation across gaps of at most two seconds. Flags',
            'are position-only diagnostics, not proof of full 6-DoF outliers. These post hoc',
            'checks did not select or remove factors. Intra/inter-robot mixtures differ.']
    attempts_path=output/'provenance/replay-attempts.json'
    if attempts_path.exists():
        lines+=['', '## Replay failures and retries', '']
        for robot,attempt in read(attempts_path).items():
            first=attempt['attempt1']
            lines += [f'{robot}: the first replay failed after {first["wall_s"]:.2f} s wall time.',
                attempt['reason'],
                f'The retained full replay uses {attempt["retry_rate"]}x playback, with estimator and',
                'loop parameters unchanged. Both BEV branches share this same replay.',
                'The failed attempt and original-bag IMU audit are retained in provenance.']
    lines+=['', '## Retained artifacts', '',
        'Full poses, constraints, evo ZIPs and shared-alignment evidence, graph residuals',
        'and weights, source/configuration provenance and figures are retained.',
        'The compact [Rerun recording](trajectories.rrd) shows both trajectory sets and GT.']
    if (output/'cleanup.json').exists():
        c=read(output/'cleanup.json')
        if c['complete']:
            lines+=['',f'Cleanup removed **{c["total_bytes"]/2**30:.2f} GiB** of generated MCAP, ellipsoid deltas',
                'and geometry stages. Frozen PGO remains reproducible from the retained evidence;',
                'reconstructing descriptors requires replay. The original dataset is unchanged.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    report=read(output/'report.json')
    with np.load(output/'trajectory-view.npz') as f:data={k:f[k] for k in f.files}
    rr.init(Path(report['dataset']).name+'_ellipsoid_BEV_ATE');rr.save(str(output/'trajectories.rrd'))
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(rrb.Horizontal(
        rrb.Spatial3DView(name='Raw BEVs + PGO',origin='/raw'),
        rrb.Spatial3DView(name='Ellipsoid BEVs + PGO',origin='/ellipsoid')),
        rrb.TextDocumentView(name='ATE and method',origin='/report'),row_shares=[.8,.2]),collapse_panels=True))
    for branch in ('raw','ellipsoid'):
        rr.log('/'+branch,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
        for robot in ROBOTS:
            color=[int(COLORS[robot][i:i+2],16) for i in (1,3,5)]
            rr.log(f'/{branch}/{robot}/optimized',rr.LineStrips3D([data[f'{branch}_{robot}']],colors=color,radii=.08),static=True)
            rr.log(f'/{branch}/{robot}/GT',rr.Points3D(data[f'gt_{robot}'],colors=[*color,90],radii=.07),static=True)
    rr.log('/report',rr.TextDocument((output/'REPORT.md').read_text(),media_type=rr.MediaType.MARKDOWN),static=True)
    rr.get_data_recording().flush()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--rerun',action='store_true');args=p.parse_args()
    if args.rerun:rerun(args.output)
    else:figures(args.output);markdown(args.output)
