"""Identical scan samples under two CBS solutions; static and Rerun views."""
import argparse
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from .artifacts import read_json, read_jsonl, file_hash, write_json
from .geometry import pose, transform, voxel_downsample

COLORS = {'Alpha': '#e2544a', 'Bob': '#388ed1', 'Carol': '#34a36a'}
MODES = ('pose_only', 'mixed')
LABELS = ('CBS pose factors', 'CBS pose + live GICP')


def collect(output):
    output = Path(output).resolve(); report = read_json(output/'report.json')
    run = read_json(output/'mixed/run.json'); artifacts = run['artifacts']; robots = run['config']['robots']
    poses = {m:{(r['robot_id'],r['keyframe_id']):pose(r['T_world_body'])
                for r in read_jsonl(output/m/'poses.jsonl')} for m in MODES}
    # A single viewing transform for both solutions. No independent map alignment.
    view = pose(report['trajectory']['pose_only']['Alpha']['alignment_SE3'])
    origin = np.loadtxt(Path(run['config']['dataset'])/'alpha_gt.txt')[0,1:4]
    view[:3,3] -= origin
    # Three geographically separated loop regions, selected without comparing results.
    anchors = [('Start overlap', ('Alpha', 10)), ('Junction overlap', ('Bob', 384)),
               ('End overlap', ('Alpha', 538))]
    regions = [dict(name=n, endpoint=list(e), center=(view@poses['pose_only'][e])[:3,3].tolist(),
                    half_width_m=14., height_above_center_m=[.5, 1.5]) for n,e in anchors]
    overview = {m:{r:[] for r in robots} for m in MODES}
    details = [{m:{r:[] for r in robots} for m in MODES} for _ in regions]
    hashes = {}; counts = {r:0 for r in robots}
    for robot in robots:
        store = Path(artifacts[f'keyframes.livo.{robot}'])/'store'
        rows = read_jsonl(store/'keyframes.jsonl')
        for row in rows:
            endpoint = (robot, row['keyframe_id']); path = store/f'{row["keyframe_id"]:06d}.npz'
            hashes[str(path)] = file_hash(path)
            with np.load(path, allow_pickle=False) as data: scan = data['scan'][:,:3].astype(np.float64)
            assert row['cloud_frame'] == row['body_frame'] == robot+'/imu'
            scan = scan[np.isfinite(scan).all(axis=1) & (np.linalg.norm(scan,axis=1) <= 80)]
            counts[robot] += len(scan)
            # Source-frame sampling is shared: different solutions never get different points.
            sample = voxel_downsample(scan, .4)
            sample = sample[np.linspace(0,len(sample)-1,min(900,len(sample)),dtype=int)]
            world = {m:transform(view@poses[m][endpoint], scan) for m in MODES}
            for m in MODES: overview[m][robot].append(transform(view@poses[m][endpoint], sample).astype(np.float32))
            for index, region in enumerate(regions):
                center = np.asarray(region['center']); p = world['pose_only']-center
                keep = ((np.abs(p[:,:2]) <= region['half_width_m']+1).all(axis=1) &
                        (p[:,2] >= .5) & (p[:,2] <= 1.5))
                if np.any(keep):
                    for m in MODES: details[index][m][robot].append(world[m][keep].astype(np.float32))
        for m in MODES: overview[m][robot] = np.concatenate(overview[m][robot])
        for item in details:
            for m in MODES: item[m][robot] = np.concatenate(item[m][robot]) if item[m][robot] else np.empty((0,3))
        print(f'Map: {robot}, {len(rows)} complete scans, {counts[robot]} range-cropped points', flush=True)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(12,8), sharex=True, sharey=True)
    for ax,mode,label in zip(axes,MODES,LABELS):
        for robot in robots:
            p=overview[mode][robot]; ax.scatter(p[:,0],p[:,1],s=.08,alpha=.35,color=COLORS[robot],rasterized=True)
            ax.plot([],[],color=COLORS[robot],label=robot)
        for k,reg in enumerate(regions):
            x,y,_=reg['center']; h=reg['half_width_m']
            ax.add_patch(Rectangle((x-h,y-h),2*h,2*h,fill=False,edgecolor='#222222',linewidth=1))
            ax.text(x+h+1,y,str(k+1),fontsize=11,fontweight='bold')
        ax.set_aspect('equal');ax.set(title=label,xlabel='East [m]',ylabel='North [m]');ax.legend(loc='upper left');ax.grid(alpha=.15)
    fig.suptitle('Square 1 — identical full-LiDAR scan samples, one shared viewing frame')
    fig.tight_layout();fig.savefig(output/'maps.png',dpi=200);fig.savefig(output/'maps.pdf');plt.close(fig)

    fig,axes=plt.subplots(3,2,figsize=(11,15))
    for index,(reg,item) in enumerate(zip(regions,details)):
        center=np.asarray(reg['center']);h=reg['half_width_m']
        for ax,mode,label in zip(axes[index],MODES,LABELS):
            # Interleave robots deterministically so draw order does not hide one robot.
            points=np.concatenate([item[mode][r] for r in robots])
            colors=np.concatenate([np.tile(np.array(matplotlib.colors.to_rgb(COLORS[r])),(len(item[mode][r]),1)) for r in robots])
            order=np.random.default_rng(0).permutation(len(points));p=points[order]-center
            ax.scatter(p[:,0],p[:,1],s=.12,alpha=.35,c=colors[order],rasterized=True)
            ax.set(xlim=(-h,h),ylim=(-h,h),xlabel='Local east [m]',ylabel='Local north [m]',
                   title=f'{index+1}. {reg["name"]} | {label}')
            ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.suptitle('Matching map close-ups — same raw points; fixed 1 m height slabs\nAlpha red · Bob blue · Carol green',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(output/'map-closeups.png',dpi=220);fig.savefig(output/'map-closeups.pdf');plt.close(fig)
    source=Path(__file__).resolve().parents[3]
    with tempfile.TemporaryDirectory(prefix='cbs-map-view-',dir=output) as temp:
        payload=Path(temp)/'maps.npz'
        arrays={}
        for m in MODES:
            for r in robots:
                p=overview[m][r];indices=np.linspace(0,len(p)-1,min(250000,len(p)),dtype=int)
                arrays[m+'_'+r]=p[indices]
        np.savez_compressed(payload,**arrays)
        subprocess.run([str(source/'.ros2/rerun-venv/bin/python'),'-m','s3e_pipeline.cbs_registration_maps',
            '--rerun-payload',str(payload),'--output',str(output)],check=True,timeout=60)
    assert all(file_hash(p)==h for p,h in hashes.items())
    write_json(output/'map-view.json',dict(source_scan_hashes=hashes,full_scan_points=counts,regions=regions,
        view_transform=view.tolist(),overview='all keyframe full scans; 80 m crop; source-frame 0.4 m voxels, at most 900 points per scan',
        closeups='all raw full-scan points selected by fixed pose-only-frame XY boxes and 1 m height slabs; identical selected points in both solutions; no smoothing',
        rerun='same overview samples capped deterministically at 250,000 points per robot per solution',
        interpretation='qualitative map overlap inspection; no independent map-sharpness ground truth or accuracy metric',
        input_scans_unchanged=True))
    write_json(output/'files.json',{str(p.relative_to(output)):file_hash(p) for p in sorted(output.rglob('*'))
        if p.is_file() and p.name!='files.json'})


def rerun_view(payload, output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    rr.init('Square 1 CBS pose and GICP maps');rr.save(str(Path(output)/'maps.rrd'))
    rr.log('/',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*[rrb.Spatial3DView(name=label,origin='/'+m)
        for m,label in zip(MODES,LABELS)])))
    with np.load(payload) as data:
        for m in MODES:
            for r,hexcolor in COLORS.items():
                color=[int(hexcolor[k:k+2],16) for k in (1,3,5)]
                rr.log('/'+m+'/'+r,rr.Points3D(data[m+'_'+r],colors=color,radii=.025),static=True)
    rr.get_data_recording().flush()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--rerun-payload',type=Path);args=parser.parse_args()
    if args.rerun_payload:rerun_view(args.rerun_payload,args.output)
    else:collect(args.output)
