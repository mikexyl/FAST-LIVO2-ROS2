"""Compact maps for inspecting paired BEV graphs without trajectory ground truth."""
from pathlib import Path
import numpy as np
from .artifacts import read_json,read_jsonl,write_json,file_hash
from .geometry import pose,transform,voxel_downsample


def build_maps(work,output,voxel=.15,max_points=150000):
    work=Path(work);output=Path(output);solved=read_json(work/'solved.json')
    graphs={b:read_json(output/b/'graph.json') for b in ('raw','ellipsoid')}
    all_maps={};counts={}
    for robot,p in solved['prepared'].items():
        store=Path(p)/'store';keys=read_jsonl(store/'keyframes.jsonl')
        transforms={b:{r['keyframe_id']:pose(r['T_world_body']) for r in graph['poses'] if r['robot_id']==robot}
                    for b,graph in graphs.items()}
        accumulated={b:[] for b in graphs};input_points=0
        for index,key in enumerate(keys):
            with np.load(store/f'{key["keyframe_id"]:06d}.npz') as f:scan=f['scan'][:,:3]
            input_points+=len(scan)
            for branch in graphs:
                accumulated[branch].append(transform(transforms[branch][key['keyframe_id']],scan))
                if (index+1)%20==0:
                    accumulated[branch]=[voxel_downsample(np.concatenate(accumulated[branch]),voxel)]
        for branch in graphs:
            cloud=voxel_downsample(np.concatenate(accumulated[branch]),voxel)
            before_cap=len(cloud)
            if len(cloud)>max_points:cloud=cloud[np.linspace(0,len(cloud)-1,max_points,dtype=int)]
            all_maps[f'{branch}_{robot}']=cloud.astype('f4')
            counts[f'{branch}_{robot}']=dict(component=graphs[branch]['components'][robot],
                input_points=input_points,voxel_points=before_cap,retained_points=len(cloud))
        print(f'{robot}: compact corrected maps retained',flush=True)
    np.savez_compressed(output/'maps.npz',**all_maps)
    write_json(output/'maps.json',dict(voxel_m=voxel,max_points_per_robot=max_points,counts=counts,
        code_sha256=file_hash(Path(__file__)),
        source='Same raw full scans at the frozen keyframes, transformed by each optimized graph',
        coordinates='Independent component frames; no GT alignment and no alignment between disconnected components',
        interpretation='Visualization only; apparent sharpness is not a ground-truth accuracy metric',
        graph_hashes={b:file_hash(output/b/'graph.json') for b in graphs}))


def retain_bevs(work,output):
    """Select each robot's middle keyframe independently of matching and GT."""
    work=Path(work);output=Path(output);solved=read_json(work/'solved.json');saved={};selected={}
    for robot,p in solved['prepared'].items():
        p=Path(p);keys=read_jsonl(p/'store/keyframes.jsonl');key=keys[len(keys)//2]
        selected[robot]=dict(keyframe_id=key['keyframe_id'],stamp_ns=key['stamp_ns'])
        for branch in ('raw','ellipsoid'):
            with np.load(p/'bevs'/f'{key["keyframe_id"]:06d}-{branch}.npz') as f:
                for name in ('image','orb_xy','kept_indices','cached_xy','ground','lower_bound'):
                    saved[f'{branch}_{robot}_{name}']=f[name]
    np.savez_compressed(output/'bev-examples.npz',**saved)
    write_json(output/'bev-examples.json',dict(selection='Middle keyframe of each robot, without loop or GT selection',
        density_pixel_m=.5,keyframes=selected))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();build_maps(args.work,args.output);retain_bevs(args.work,args.output)
