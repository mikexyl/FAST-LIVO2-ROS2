#!/usr/bin/env python3
"""Standalone remote-run report, with compact inputs for later re-evaluation."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import numpy as np

ROBOTS=('Alpha','Bob','Carol')
COLORS={'Alpha':'#df554f','Bob':'#3b97dc','Carol':'#52ac76'}


def read(path):return json.loads(path.read_text())
def rows(path):return [json.loads(line) for line in path.read_text().splitlines()]
def individual(metrics,robot):
    return next((m['per_robot'][robot]['statistics']['rmse'] for m in metrics.values() if robot in m['per_robot']),None)
def fmt(value):return 'unavailable' if value is None else f'{value:.4f}'


def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    rr.init(output.name);rr.save(str(output/'trajectories.rrd'))
    rr.log('/world',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    with np.load(output/'trajectory-view.npz') as data:
        for name in data.files:
            kind,robot=name.split('_',1);color=[int(COLORS[robot][i:i+2],16) for i in (1,3,5)]
            if kind=='gt':color+=[90]
            rr.log('/world/'+robot+'/'+kind,rr.LineStrips3D([data[name]],colors=color,radii=.03),static=True)
    rr.log('/report',rr.TextDocument((output/'REPORT.md').read_text(),media_type=rr.MediaType.MARKDOWN),static=True)
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(rrb.Spatial3DView(origin='/world'),
        rrb.TextDocumentView(origin='/report'),row_shares=[.8,.2]),collapse_panels=True))
    rr.get_data_recording().flush()


def report(work,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from s3e_pipeline.evaluation import ground_truth
    from s3e_pipeline.artifacts import file_hash
    root=Path(__file__).resolve().parents[3]
    data=read(output/'report.json');native=data['swarm'];run=data['swarm_run'];raw=data['ours']['raw_odometry']
    metrics=native['trajectory'];seq=Path(data['dataset']).name
    evidence=output/'provenance';evidence.mkdir(exist_ok=True)
    hashes={};truth={};odometry={}
    for robot in ROBOTS:
        path=Path(data['dataset'])/f'{robot.lower()}_gt.txt';truth[robot]=ground_truth(path)[1]
        shutil.copy2(path,evidence/path.name);hashes[path.name]=file_hash(path)
        src=work/'frontend'/robot;dest=evidence/robot;dest.mkdir(exist_ok=True)
        for name in ('summary.json','mapping_config.yaml','runtime.yaml','mapping.log','playback.log'):
            if (src/name).is_file():shutil.copy2(src/name,dest/name)
        for name in ('frames.jsonl','manifest.json'):
            path=src/'export'/name;hashes[f'{robot}/{name}']=file_hash(path)
            with path.open('rb') as inp,gzip.open(dest/(name+'.gz'),'wb') as out:shutil.copyfileobj(inp,out)
        odometry[robot]=rows(src/'export/frames.jsonl')
    shutil.copy2(work/'frontend/config.yaml',evidence/'frontend-config.yaml')
    shutil.copytree(root/'Swarm-SLAM/s3e',evidence/'adapter',dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copytree(root/'.ros2/swarm/provenance',evidence/'setup',dirs_exist_ok=True)
    hashes['evaluate.py']=file_hash(root/'Swarm-SLAM/s3e/evaluate.py')
    hashes['report.py']=file_hash(Path(__file__))
    (evidence/'sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    origin=truth['Alpha'][0];view={f'gt_{r}':truth[r]-origin for r in ROBOTS}
    fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
    for robot in ROBOTS:
        gt=truth[robot]-origin
        for ax in axes:ax.plot(gt[:,0],gt[:,1],':',color=COLORS[robot],lw=1,alpha=.7)
        xyz=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in odometry[robot]])
        if robot in raw:
            T=np.array(raw[robot]['alignment_SE3']);xyz=xyz@T[:3,:3].T+T[:3,3]-origin
            axes[0].plot(xyz[:,0],xyz[:,1],color=COLORS[robot],lw=1,label=robot)
    if metrics:
        poses=rows(output/'swarm/poses.jsonl')
        for robot in ROBOTS:
            if robot not in native['components']:continue
            component=native['components'][robot]
            if component not in metrics:continue
            T=np.array(metrics[component]['alignment_SE3'])
            xyz=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in poses if r['robot_id']==robot])
            xyz=xyz@T[:3,:3].T+T[:3,3]-origin;view[f'swarm_{robot}']=xyz
            axes[1].plot(xyz[:,0],xyz[:,1],color=COLORS[robot],lw=1,label=robot)
    else:axes[1].text(.5,.5,'Native trajectory ATE unavailable',transform=axes[1].transAxes,ha='center')
    axes[0].set_title('Raw EllipseLIO — independent robot alignments')
    axes[1].set_title('Swarm-SLAM — one rigid alignment per output component')
    for ax in axes:
        ax.set_aspect('equal');ax.set_xlabel('X [m]');ax.set_ylabel('Y [m]');ax.grid(alpha=.2)
        if ax.get_legend_handles_labels()[0]:ax.legend()
    fig.suptitle(seq.replace('_',' ')+' · workstation 148\nDotted lines: supplied position ground truth; scale fixed to 1')
    fig.savefig(output/'trajectories.png',dpi=180);fig.savefig(output/'trajectories.pdf');plt.close(fig)
    np.savez_compressed(output/'trajectory-view.npz',**view)
    combined=next(iter(metrics.values()))['rmse_m'] if len(metrics)==1 and len(next(iter(metrics.values()))['robots'])==3 else None
    text=[f'# {seq}: Swarm-SLAM on workstation 148','',
          'Three native Swarm-SLAM LiDAR workers consume fresh EllipseLIO odometry and complete deskewed scans. '
          'Scan Context, FPFH/TEASER++/ICP, native candidate allocation, and elected-robot GNC are retained.', '',
          f'Native run: **{"complete" if run["success"] else "incomplete"}**. '
          f'Exact selected-input audit: **{native["selected_inputs_complete"]}**. '
          f'Full optimized coverage: **{native["optimized_coverage_complete"]}**.', '']
    if not run['success']:text += [f'Reason: `{run.get("error")}`.','']
    text += ['ATE uses evo 1.36.5, 50 ms nearest-timestamp association, translation error, and no scale fitting. '
             'Swarm uses one shared SE(3) alignment per native output component; robot errors use that same fit. '
             'Raw odometry uses independent robot alignments because its coordinate frames are unrelated. '
             'GT orientations are unused; antenna lever arm is uncorrected.','',
             '| Method | Combined ATE [m] | Alpha | Bob | Carol |','|---|---:|---:|---:|---:|',
             '| Raw EllipseLIO | — | '+' | '.join(fmt(individual(raw,r)) for r in ROBOTS)+' |',
             '| Swarm-SLAM | '+fmt(combined)+' | '+' | '.join(fmt(individual(metrics,r)) for r in ROBOTS)+' |','',
             'Trajectory measurement and replay-comparison validity are reported separately. '
             'No missing optimized robot is replaced with raw odometry. '
             'If only some robots have full optimized coverage, only those robots are evaluated and the combined result remains unavailable. '
             'Dense trajectories interpolate the native keyframe correction field over the saved odometry.','',
             '![Trajectories and position GT](trajectories.png)','',
             f'- Keyframes: {run["received_keyframes"]}; descriptors: {run["received_descriptors"]}.',
             f'- Scans published / processed: {run["published_scans"]} / {run["processed_scans"]}.',
             f'- Verification attempts: {native["verification_attempts"]}; accepted registrations: {native["accepted"]}, including {native["accepted_inter"]} inter-robot.',
             f'- GT-checkable accepted loops: {native["gt_checkable"]}; endpoint-distance discrepancies over 2 m: {native["flagged"]}. This is a position-only diagnostic, not a full 6-DoF outlier test.',
             f'- Native output components: {native["components"]}. GNC weights are not exported; native origin IDs do not prove which factors survived GNC.',
             f'- Native optimizer errors: {run.get("native_optimizer_error_count",0)}.',
             f'- Native wall time including paced replay and settling: {run.get("total_wall_s",0):.1f} s; backpressure: {run.get("backpressure_s",0):.1f} s.',
             f'- Observed coordination CDR payload: {native["observed_cdr_bytes"]/2**20:.2f} MiB; excludes DDS overhead and broadcast fanout.', '',
             'The replay uses reliable input QoS and exact per-scan acknowledgements, at most two outstanding scans per robot, '
             'and a bounded 120 s drain. The native registration direction fix and GTSAM 4.3 compatibility adaptations are recorded in the retained source patches. '
             'The original Python Scan Context implementation and upstream graco LiDAR thresholds remain unchanged.','',
             'Remote workspace: `/data3/mikexyl/swarm_s3e_ws/src`. '
             'ROS Humble runs in an isolated existing Docker image; the host ROS installation is unchanged. '
             'All loop and optimization decisions exclude ground truth. No fresh paired MapClosures result is claimed for this run.','',
             '[Machine-readable report](report.json), [evo evidence](swarm/evo/README.md), '
             '[native replay summary](swarm/summary.json), [input audit](swarm/keyframe-input-audit.json).']
    (output/'REPORT.md').write_text('\n'.join(text)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rerun',action='store_true');args=p.parse_args()
    if args.rerun:rerun(args.output.resolve())
    else:report(args.work.resolve(),args.output.resolve())
