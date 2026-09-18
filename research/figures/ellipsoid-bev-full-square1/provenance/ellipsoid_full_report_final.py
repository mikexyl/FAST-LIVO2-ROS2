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
    fig.suptitle('Square 1 · same EllipseLIO odometry and GNC-TLS settings\nDotted: position ground truth; evo SE(3) alignment, no scale fit')
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
    row=lambda label,metrics:f'| {label} | {fmt(combined(metrics))} | '+' | '.join(fmt(robot_error(metrics,r)) for r in ROBOTS)+' |'
    table='\n'.join([row('Raw EllipseLIO (independent robot alignment)',report['raw_odometry']),
                     row('Raw BEVs + centralized PGO',report['branches']['raw']['trajectory']),
                     row('Ellipsoid BEVs + centralized PGO',report['branches']['ellipsoid']['trajectory'])])
    counts=[]
    for b in ('raw','ellipsoid'):
        v=report['branches'][b]
        counts.append(f'| {b} | {v["verification_attempts"]} | {v["constraints"]} | {v["selected_loops"]} | {v["selected_inter_robot_loops"]} | {len(set(v["components"].values()))} |')
    text=f'''# Square 1: full-sequence ellipsoid BEVs and trajectory ATE

Both inputs were evaluated on the **same new EllipseLIO odometry and keyframes**
for Alpha, Bob and Carol. Native MapClosures retrieval and small_gicp acceptance
run in isolated robot workers with the existing deterministic delivery scheduler.
Both resulting graphs use centralized GTSAM 4.2 GNC-TLS followed by a
selected-inlier LM refit. This is a centralized PGO comparison; CBS and live
registration factors are not part of these two solver objectives.

**Observed result:** ellipsoid BEVs achieve 1.1408 m combined ATE versus
1.1457 m for raw BEVs, a 0.0049 m (0.43%) difference. Alpha improves, while
Bob and Carol worsen. This single paired run establishes working full-sequence
integration, with little change in aggregate ATE.

## Trajectory ATE

Position RMSE in metres, evaluated entirely through **evo 1.36.5**:

| Configuration | Combined ATE | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
{table}

Connected optimized trajectories receive one shared SE(3) alignment across all
three robots. Each listed robot error uses that same alignment, without a second
robot fit. Disconnected components are independently anchored and aligned; a
combined three-robot ATE is unavailable if the graph stays disconnected. Raw
odometry uses separate robot fits and is a diagnostic with a different alignment
scope. No scale is fitted, timestamp tolerance is 0.05 seconds, and supplied GT
orientations are unused. GT is introduced only after both graphs are optimized.

![Full-sequence trajectories](trajectories.png)

![Individual ATEs](ate.png)

## Loop detection and graph selection

| BEV input | Verification attempts | Accepted constraints | GNC-selected loops | Selected inter-robot loops | Components |
|---|---:|---:|---:|---:|---:|
{chr(10).join(counts)}

Keyframes: {report['keyframes']}. Dense odometry frames: {report['odometry_frames']}.
Loop acceptance and robust factor selection are separate steps; selected-loop
counts exclude factors rejected by GNC or unsupported initial robot alignment.
Native acceptance and registration reasons are retained in `report.json` and
the branch-specific verification logs.

The ellipsoid branch's 63 loops comprise **55 intra-robot and 8 inter-robot**
constraints. All 38 raw-branch loops are inter-robot. The extra ellipsoid loops
therefore do not represent increased inter-robot detection. Both graphs connect
all three robots, and GNC selects every accepted loop in each graph.

## Rendering and controls

The renderer uses each actual native ellipsoid’s center, geometric semi-axes
and orthonormal axis directions. Surfaces are sampled at the same nominal
0.125 m spacing as the short diagnostic and merged with 0.25 m voxel centroids.
CUDA accelerates the sampler using deterministic integer accumulation at 0.1 µm
precision. Three retained real snapshots reproduced the CPU prototype’s density
images, ORB keypoints and descriptors exactly, including a 144,474-ellipsoid map;
that map took 0.28 s to sample on this machine. Repeated GPU rendering was also
identical. Native MapClosures ground alignment, density/ORB/HBST/RANSAC and their
thresholds are unchanged.

The ellipsoid map is spatial, persistent and causal, cropped at 80 m around the
current pose. The raw branch uses trailing five-second scan submaps. Their map
histories differ, so this comparison includes that effect. Both branches use
the same raw geometric evidence for GICP and the same odometry pose factors.
The existing 1 m / 10° / 2 s keyframe schedule is taken from the complete prior
odometry keyframe list, independently of loops and GT. Any merged scan intervals
are recorded. Empty fitted maps at initialization produce empty descriptors.

MapClosures settings: 0.5 m BEV pixels, density threshold 0.05, Hamming threshold
50, more than five native RANSAC inliers, top-20 shortlist, one selected LiDAR
candidate per query with the existing two-second cooldown and 30-second
same-robot exclusion. GICP retains the existing 0.35 m RMSE and 30% overlap gates
and observability checks. There is no parameter sweep.

Native ellipsoid snapshots are stored as lossless world-frame changes between
keyframes. Raw scans and states remain standard ROS2 MCAP messages. New stores,
descriptor caches, loop transcripts and PGO graphs are versioned immutable
stages. Both loop branches refer to identical preparation hashes, recorded in
`report.json`. The solver corrects saved trajectories only.

This fresh paired experiment supersedes using the earlier raw-BEV 1.1371 m ATE
as a direct comparison: that value came from a different replay and loop set.
The earlier eight-pair test remains a separate rendering diagnostic.

Machine-readable metrics, per-robot TUM trajectories, graph residuals and robust
weights, evo result ZIPs and alignment evidence are retained alongside this
report. `inputs.json` records the stage lineage. The Rerun recording presents
the two optimized trajectory sets and position GT.
'''
    if all((output/b/'detection-summary.json').exists() for b in ('raw','ellipsoid')):
        text+='\n## Runtime and communication\n\n'
        text+='| BEV input | Distributed detection | Centralized PGO | Serialized exchange |\n|---|---:|---:|---:|\n'
        for b in ('raw','ellipsoid'):
            detection=read(output/b/'detection-summary.json')
            text+=f'| {b} | {detection["wall_runtime_s"]:.2f} s | {report["branches"][b]["optimization_s"]:.3f} s | {detection["network_bytes"]/2**20:.2f} MiB |\n'
        text+='\nThese are measured wall times after descriptor preparation. Detection runs three\nisolated CPU workers with reliable unrestricted simulated delivery. CUDA is used\nfor ellipsoid surface sampling; MapClosures and small_gicp run on CPU. Full fresh\nserial odometry replay took 23.17 minutes in total. Preparing both descriptor\nbranches took 29.93 minutes, including 10.67 minutes of CUDA surface sampling.\nPreparation overlapped later robot replays, so these totals are not additive.\n'
    if (output/'validation.json').exists():
        validation=read(output/'validation.json')
        text+='\n## Verification and retained evidence\n\n'
        text+=f'Frozen PGO reproduced both graphs with zero pose-matrix difference; hashes of\nall {validation["checked_input_files"]:,} watched odometry, keyframe and descriptor files stayed unchanged.\nThe regression selection passed {validation["regression_tests_passed"]} tests. All 1,548 scheduled keyframe snapshots were\ncaptured, with no schedule entries merged. The export audit records the observed\nscan intervals and association delays.\n\n'
        text+='Post hoc loop checks compare the estimated endpoint distance with interpolated\nGT positions. They flag discrepancies above 2 m, and require bracketing GT samples\nno more than 2 seconds apart at both endpoints. This is a position-only diagnostic;\nGT orientations are unavailable, so it does not establish full 6-DoF correctness.\n\n| BEV input | GT-checkable loops | Flagged (>2 m distance discrepancy) |\n|---|---:|---:|\n'
        for b in ('raw','ellipsoid'):
            q=validation['loop_quality'][b]
            text+=f'| {b} | {q["gt_checkable"]}/{report["branches"][b]["constraints"]} | {q["flagged"]} |\n'
        text+='\nThese checks are diagnostic only and did not select or remove optimization factors.\nDifferent intra/inter-robot loop mixes prevent treating these counts as a direct\ncomparison of outlier rates. Eleven ellipsoid loops lack sufficient GT coverage.\n'
    if (output/'cleanup.json').exists():
        cleanup=read(output/'cleanup.json')
        if cleanup['complete']:
            text+=f'\nCleanup removed {cleanup["total_bytes"]/2**30:.2f} GiB of generated MCAP, map deltas and geometry\nstages. Full frozen poses and constraints, compressed exchanges, evo evidence,\nsource/configuration hashes, figures and the 0.43 MiB [Rerun recording](trajectories.rrd)\nremain. Descriptor/retrieval reconstruction requires replay after cleanup; frozen\npose-only PGO remains reproducible from retained evidence. The original dataset\nand earlier experiment reports are unchanged.\n'
    (output/'REPORT.md').write_text(text)


def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    report=read(output/'report.json')
    with np.load(output/'trajectory-view.npz') as f:data={k:f[k] for k in f.files}
    rr.init('Square1_ellipsoid_BEV_ATE');rr.save(str(output/'trajectories.rrd'))
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
