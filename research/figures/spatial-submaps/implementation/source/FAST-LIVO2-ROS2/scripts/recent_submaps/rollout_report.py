#!/usr/bin/env python3
"""Evaluate native-submap CBS output; GT is read only after optimization."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,write_jsonl
from s3e_pipeline.geometry import pose,transform,voxel_downsample
from s3e_pipeline.evaluation import corrected_dense_rows,save_tum,trajectory_metrics
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth
from s3e_pipeline.robot_colors import robot_colors
from s3e_pipeline.graco import connectivity
from collections import Counter


def native_rows(trial,robot):
    result=[]
    for row in read_jsonl(Path(trial)/'frontend/native_updates.jsonl'):
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(row['pose'][3:]).as_matrix();T[:3,3]=row['pose'][:3]
        result.append(dict(robot_id=robot,component=robot,frame_id=row['scan_id'],stamp_ns=row['stamp_ns'],T_world_body=T.tolist()))
    return result


def report(work,trials):
    started=time.monotonic();work=Path(work);out=work/'report';out.mkdir(exist_ok=False)
    cfg=yaml.safe_load((work/'config.yaml').read_text())
    graph=read_jsonl(work/'dpgo/poses.jsonl');loops=read_jsonl(work/'dpgo/constraints.jsonl')
    components={p['robot_id']:p['component'] for p in graph}
    connected=connectivity(cfg['robots'],loops,graph)
    connected['all_robots_connected']=connected.pop('all_six_connected')
    write_json(work/'connectivity.json',connected)
    if any(components[e['i'][0]]!=components[e['j'][0]] for e in loops):raise ValueError('CBS failed common-frame contract')
    raw=[];corrected=[]
    for robot,trial in trials.items():
        dense=native_rows(trial,robot);raw+=dense
        keys=read_jsonl(work/f'prepared-{robot}/store/keyframes.jsonl')
        optimized=sorted((p for p in graph if p['robot_id']==robot),key=lambda r:r['keyframe_id'])
        if [(r['keyframe_id'],r['stamp_ns']) for r in optimized]!=[(r['keyframe_id'],r['stamp_ns']) for r in keys]:
            raise ValueError('Graph endpoint/anchor mismatch')
        track=[dict(r,component=components[robot]) for r in corrected_dense_rows(dense,keys,optimized)]
        corrected+=track;save_tum(out/f'{robot}-raw.tum',dense);save_tum(out/f'{robot}-cbs.tum',track)
        write_jsonl(out/f'{robot}-cbs.jsonl',track)
        before=[];after=[]
        for key,opt in zip(keys,optimized):
            with np.load(work/f'prepared-{robot}/store'/f"{key['keyframe_id']:06d}.npz") as data:points=data['cloud']
            before.append(transform(pose(key['T_world_body']),points));after.append(transform(pose(opt['T_world_body']),points))
            if len(before)>=20:
                before=[voxel_downsample(np.concatenate(before),.25)];after=[voxel_downsample(np.concatenate(after),.25)]
        np.savez_compressed(out/f'{robot}-maps.npz',raw=voxel_downsample(np.concatenate(before),.25),
                            cbs=voxel_downsample(np.concatenate(after),.25))
    gt,status=evaluation_ground_truth(cfg['robots'],Path(cfg['dataset']),corrected,cfg['evaluation'])
    metrics=dict(raw=trajectory_metrics(dict(poses=raw),gt,cfg['evaluation'],out/'evo/raw'),
                 cbs=trajectory_metrics(dict(poses=corrected),gt,cfg['evaluation'],out/'evo/cbs'),
                 components=components,ground_truth=status,runtime=read_json(work/'dpgo/summary.json'),
                 pcm=read_json(work/'dpgo/pcm.json'),registration=read_json(work/'dpgo/registration.json'),
                 connectivity=connected, dataset=cfg.get('experiment_name',cfg['dataset']),
                 map_source='native processed member scans in completed submaps; not full-resolution raw geometry')
    events=[e for robot in cfg['robots'] for e in read_jsonl(work/f'dpgo/{robot}/events.jsonl') if e.get('type')=='verification']
    metrics['verification_reasons']=dict(Counter(e['reason'] for e in events))
    write_json(out/'report.json',metrics)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={r:np.array(c)/255. for r,c in robot_colors(cfg['robots']).items()}
    groups=sorted(set(components.values()))
    fig,axes=plt.subplots(2,len(groups),figsize=(7*len(groups),11),layout='constrained',squeeze=False)
    reference_files=list((out/'evo/cbs').glob('*-reference-matched.tum'))
    origin=np.loadtxt(reference_files[0])[0,1:3] if reference_files else np.zeros(2)
    for column,group in enumerate(groups):
        for robot in cfg['robots']:
            if components[robot]!=group:continue
            aligned=out/f'evo/cbs/{robot}-estimate-aligned.tum'
            if aligned.exists():
                estimate=np.loadtxt(aligned);reference=np.loadtxt(out/f'evo/cbs/{robot}-reference-matched.tum')
                axes[0,column].plot(reference[:,1]-origin[0],reference[:,2]-origin[1],color=colors[robot],ls='--',alpha=.5)
                axes[0,column].plot(estimate[:,1]-origin[0],estimate[:,2]-origin[1],color=colors[robot],label=robot)
            elif not reference_files:
                estimate=np.loadtxt(out/f'{robot}-cbs.tum')
                axes[0,column].plot(estimate[:,1],estimate[:,2],color=colors[robot],label=robot+' (native frame; no GT)')
            with np.load(out/f'{robot}-maps.npz') as data:
                points=data['cbs'];points=points[::max(1,len(points)//200000)]
                axes[1,column].scatter(points[:,0],points[:,1],s=.2,color=colors[robot],alpha=.4,label=robot,rasterized=True)
        axes[0,column].set_title(f'CBS component {group}'+(' and position GT' if reference_files else ': native frame; GT unavailable'))
        axes[1,column].set_title(f'Component {group}: native member-scan map')
    for ax in axes.flat:ax.axis('equal');ax.legend();ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
    fig.savefig(out/'trajectories-maps.png',dpi=170);plt.close(fig)
    metrics['evaluation_wall_s']=time.monotonic()-started;write_json(out/'report.json',metrics)
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True)
    p.add_argument('--trials',type=Path,help='JSON mapping robot IDs to native trial directories')
    for robot in ('alpha','bob','carol'):p.add_argument('--'+robot,type=Path)
    args=p.parse_args()
    trials=read_json(args.trials) if args.trials else dict(Alpha=args.alpha,Bob=args.bob,Carol=args.carol)
    if any(v is None for v in trials.values()):p.error('Supply --trials or all three robot paths')
    report(args.work,trials)
