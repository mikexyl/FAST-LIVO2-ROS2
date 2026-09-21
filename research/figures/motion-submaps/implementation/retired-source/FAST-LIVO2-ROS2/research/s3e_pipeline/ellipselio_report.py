"""Retain compact, inspectable evidence for the EllipseLIO S3E experiment."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import shutil
import zlib

import numpy as np

from .artifacts import read_json, read_jsonl, write_json, write_jsonl, file_hash, validate_stage
from .evaluation import ground_truth, gt_position, save_tum
from .frontends import frontend_name
from .geometry import pose, transform

COLORS={'Alpha':'#df5752','Bob':'#388fd0','Carol':'#3faa76'}


def collect(registry, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    run=read_json(registry);cfg=run['config'];artifacts=run['artifacts']
    assert frontend_name(cfg)=='ellipselio' and cfg['backend']['name']=='mapclosures'
    dpgo=Path(artifacts['dpgo']);evaluation=Path(artifacts['dpgo_evaluate'])
    report=read_json(evaluation/'report.json');robots=cfg['robots']
    def retain(source,relative):
        dest=output/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    retain(registry,'run.json')
    for name in ('report.json','metrics.csv','trajectories.png','result.rrd'):
        retain(evaluation/name,name)
    for p in (evaluation/'evo').rglob('*'):
        if p.is_file():retain(p,'evo/'+str(p.relative_to(evaluation/'evo')))
    for name in ('summary.json','poses.jsonl','constraints.jsonl','proposed-constraints.jsonl','pcm.json','registration.json'):
        retain(dpgo/name,'dpgo/'+name)
    for label,p in artifacts.items():
        validate_stage(p,verify_files=False)
        retain(Path(p)/'COMPLETE.json','manifests/'+label+'.json')
    gt={r:ground_truth(Path(cfg['dataset'])/(r.lower()+'_gt.txt')) for r in robots}
    keys={};raw={};corrected={};odometry={};descriptor_stats={};wire=Counter();events=[]
    for robot in robots:
        root=Path(artifacts[f'odometry.ellipselio.{robot}'])/'run'
        odometry[robot]=read_json(root/'summary.json')
        for name in ('summary.json','mapping_config.yaml','export/manifest.json'):
            retain(root/name,robot+'/odometry/'+name)
        raw[robot]=read_jsonl(root/'export/frames.jsonl');save_tum(output/f'{robot}/raw.tum',raw[robot])
        xyz=np.array([pose(r['T_world_body'])[:3,3] for r in raw[robot]])
        steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
        odometry[robot]['synchronized_motion']=dict(path_length_m=float(steps.sum()),max_step_m=float(steps.max()),
            scan_pose_offset_max_ms=max(r['stamp_ns']-r['scan_end_ns'] for r in raw[robot])/1e6)
        odometry[robot]['long_export_intervals']=[dict(frame=b['frame_id'],gap_s=(b['stamp_ns']-a['stamp_ns'])/1e9,
            processed_interval_s=(b['scan_end_ns']-b['scan_start_ns'])/1e9,
            uncovered_boundary_s=(b['scan_start_ns']-a['scan_end_ns'])/1e9,cloud_points=b['cloud_points'])
            for a,b in zip(raw[robot],raw[robot][1:]) if b['stamp_ns']-a['stamp_ns']>200_000_000]
        retain(evaluation/f'{robot}-corrected.tum',robot+'/corrected.tum')
        corrected[robot]=read_jsonl(evaluation/f'{robot}-corrected.jsonl')
        store=Path(artifacts[f'keyframes.ellipselio.{robot}'])/'store'
        rows=read_jsonl(store/'keyframes.jsonl');keys.update({(robot,r['keyframe_id']):r for r in rows})
        assert not list(store.glob('*.png')) and all(r.get('image_available') is False for r in rows)
        write_jsonl(output/robot/'keyframes.jsonl',[{k:r[k] for k in ('robot_id','keyframe_id','frame_id','stamp_ns','T_world_body','body_frame','cloud_frame','submap_start_ns','submap_end_ns','submap_points')} for r in rows])
        descriptors=Path(artifacts[f'descriptors.ellipselio.mapclosures.{robot}'])
        for p in descriptors.glob('*.json.zlib'):
            assert 'visual' not in json.loads(zlib.decompress(p.read_bytes()))
        timings=read_jsonl(descriptors/'timing.jsonl')
        descriptor_stats[robot]=dict(read_json(descriptors/'summary.json'),
            nonempty=sum(r['features']>0 for r in timings),median_features=float(np.median([r['features'] for r in timings])))
        retain(descriptors/'timing.jsonl',robot+'/descriptor-timing.jsonl')
        retain(dpgo/robot/'stats.jsonl',robot+'/cbs-stats.jsonl')
        for e in read_jsonl(dpgo/robot/'wire.jsonl'):wire[e['kind']]+=e['network_bytes']
        events+=read_jsonl(dpgo/robot/'events.jsonl')
    causal_errors=[]
    for e in events:
        if e['type']!='ranked':continue
        q=tuple(e['query']);stamp=keys[q]['stamp_ns']
        for candidate in e['candidates']:
            endpoint=(e['candidate_robot'],candidate['keyframe_id']);age=(stamp-keys[endpoint]['stamp_ns'])/1e9
            if age<0 or (endpoint[0]==q[0] and age<cfg['loops']['same_robot_exclusion_s']):
                causal_errors.append([list(q),list(endpoint),age])
    assert not causal_errors,causal_errors[:5]
    loops=read_jsonl(dpgo/'constraints.jsonl');seen=set();quality=[]
    for e in loops:
        i,j=tuple(e['i']),tuple(e['j']);canonical=tuple(sorted((i,j)))
        assert canonical not in seen;seen.add(canonical)
        assert e['diagnostics']['retrieval_sources']==['mapclosures']
        points=[gt_position(gt[k[0]],keys[k]['stamp_ns'],round(cfg['evaluation']['gt_max_gap_s']*1e9)) for k in (i,j)]
        distance=float(np.linalg.norm(pose(e['T_i_j'])[:3,3]));checkable=all(p is not None for p in points)
        truth_distance=float(np.linalg.norm(points[0]-points[1])) if checkable else None
        discrepancy=abs(distance-truth_distance) if checkable else None
        quality.append(dict(i_robot=i[0],i_key=i[1],j_robot=j[0],j_key=j[1],
            measured_distance_m=distance,gt_distance_m=truth_distance,absolute_discrepancy_m=discrepancy,
            gt_checkable=checkable,flag_over_2m=discrepancy>2 if checkable else None))
    with (output/'loop-quality.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(quality[0]) if quality else ['gt_checkable']);writer.writeheader();writer.writerows(quality)
    verification=[e for e in events if e['type']=='verification']
    write_jsonl(output/'verifications.jsonl',verification)
    audit=dict(odometry=odometry,descriptors=descriptor_stats,causal_candidate_violations=len(causal_errors),
        unique_accepted_loops=len(seen),loop_pairs=dict(Counter('—'.join(sorted([e['i'][0],e['j'][0]])) for e in loops)),
        loop_quality=dict(gt_checkable=sum(q['gt_checkable'] for q in quality),
            flagged_over_2m=sum(q['flag_over_2m'] is True for q in quality),
            criterion='absolute difference between measured translation length and GT endpoint distance > 2 m; diagnostic, not a confirmed 6-DoF outlier',
            gt_association='linear position interpolation within bracketing GT gaps <= 2 s; used only for loop diagnostics, not evo trajectory evaluation'),
        wire_cdr_bytes_by_kind=dict(wire),image_files=0,visual_descriptors=0,
        evaluation_scope='evo 1.36.5; raw individual SE3 fits; CBS shared component SE3 fit; no scale fitting')
    write_json(output/'audit.json',audit)
    plot(cfg,report,raw,corrected,gt,evaluation,output)
    write_json(output/'files.json',{str(p.relative_to(output)):file_hash(p) for p in sorted(output.rglob('*')) if p.is_file() and p.name!='files.json'})
    print(json.dumps(dict(trajectory=report['trajectory'],raw=report['raw_odometry'],audit=audit),indent=2))


def plot(cfg, report, raw, corrected, gt, evaluation, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin=gt['Alpha'][1][0]
    fig,axes=plt.subplots(1,3,figsize=(17,6),sharex=True,sharey=True)
    for robot in cfg['robots']:
        for ax,tracks,metrics,component in ((axes[0],raw,report['raw_odometry'],robot),
                (axes[1],corrected,report['trajectory'],report['components'][robot])):
            if component not in metrics:continue
            alignment=pose(metrics[component]['alignment_SE3'])
            xyz=transform(alignment,np.array([pose(r['T_world_body'])[:3,3] for r in tracks[robot]]))-origin
            ax.plot(xyz[:,0],xyz[:,1],color=COLORS[robot],label=robot,linewidth=1.2)
        truth=np.insert(gt[robot][1]-origin,np.flatnonzero(np.diff(gt[robot][0])>2e9)+1,np.nan,axis=0)
        for ax in axes[:2]:ax.plot(truth[:,0],truth[:,1],':',color=COLORS[robot],alpha=.6)
        axes[2].plot(truth[:,0],truth[:,1],color=COLORS[robot],label=robot,linewidth=1.2)
    for ax,title in zip(axes,['Raw EllipseLIO — independent robot fits','MapClosures + PCM + CBS — shared fit','Supplied multi-robot position ground truth']):
        ax.set(title=title,xlabel='East from local origin [m]',ylabel='North from local origin [m]')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend()
    fig.suptitle('S3E Square 1 | dotted: position GT; no scale fitting')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=180);fig.savefig(output/'comparison.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,7),sharex=True,sharey=True)
    for robot in cfg['robots']:
        component=report['components'][robot]
        alignment=pose(report['trajectory'][component]['alignment_SE3']) if component in report['trajectory'] else np.eye(4)
        with np.load(evaluation/f'{robot}-maps.npz') as data:
            for ax,label in zip(axes,['original','optimized']):
                points=data[label];sample=points[np.linspace(0,len(points)-1,min(250000,len(points)),dtype=int)]
                xyz=transform(alignment,sample)-origin
                ax.scatter(xyz[:,0],xyz[:,1],s=.12,color=COLORS[robot],alpha=.35,rasterized=True)
                ax.plot([],[],color=COLORS[robot],label=robot)
    for ax,title in zip(axes,['EllipseLIO maps placed at each first CBS pose','CBS corrected maps']):
        ax.set(title=title,xlabel='East [m]',ylabel='North [m]');ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend()
    fig.suptitle('Square 1 LiDAR maps | colors identify robots; 0.25 m map voxels')
    fig.tight_layout();fig.savefig(output/'maps.png',dpi=200);fig.savefig(output/'maps.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input-run',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();collect(args.input_run,args.output)
