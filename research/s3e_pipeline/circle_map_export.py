"""Export corrected native map representatives, once per robot-local point ID.

Uses optimized submap corrections and IMU gravity only. No ground truth alignment
or map registration is used. Disconnected components are never overlaid.
"""
from collections import defaultdict
from pathlib import Path

import numpy as np

from .artifacts import file_hash, read_json, read_jsonl, write_json
from .circle_central_evaluation import corrected_union
from .geometry import transform
from .gravity_bev import gravity_ground

PALETTE = ['#2166ac','#4393c3','#018571','#5e9c66','#c23b32','#dc8f26','#8d5d9f',
           '#b24d7a','#466b29','#8b782d','#2b8090','#6957a3','#ae643b','#626a75']
DTYPE = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),
                  ('green','u1'),('blue','u1'),('robot_id','u1'),
                  ('submap_id','<u4'),('uncovered','u1')])


def export(run, config, output):
    run=Path(run); output=Path(output); output.mkdir(parents=True,exist_ok=False)
    graph=read_json(run/'central/graph.json')
    residuals=graph['published_loop_residuals']
    limits=dict(translation_m=config['pgo']['alignment_translation_m'],
                rotation_deg=config['pgo']['alignment_rotation_deg'])
    bad=[r for r in residuals if any(not np.isfinite(r[k]) or r[k]>v for k,v in limits.items())]
    write_json(output/'published-transform-audit.json',dict(success=not bad,limits=limits,failed_loops=bad))
    if bad: raise ValueError('Refusing joint map: inconsistent published loop transforms')
    groups=defaultdict(list)
    for robot in config['robots']: groups[graph['components'][robot]].append(robot)
    files=[]; provenance={}; colors={}
    for root,robots in groups.items():
        keys={r:read_jsonl(run/r/'store/keyframes.jsonl') for r in robots}
        optimized={r:[p for p in graph['poses'] if p['robot_id']==r] for r in robots}
        first=next(p for p in optimized[root] if p['keyframe_id']==0)
        gravity=np.asarray(first['T_world_body'])[:3,:3]@keys[root][0]['gravity_imu_m_s2']
        level=gravity_ground(gravity)
        total=sum(read_json(run/r/'placement.json')['points'] for r in robots)
        filename=('graco-all14-circle40-pointcloud-global-dark-robots.ply'
                  if len(robots)==14 else f'graco-circle40-{root}-{len(robots)}robots-dark.ply')
        path=output/filename
        header=('ply\nformat binary_little_endian 1.0\n'
            'comment Optimized circle submap corrections; native gravity aligned; no GT\n'
            f'comment Component root {root}; colors and native frame in export.json\n'
            f'element vertex {total}\nproperty float x\nproperty float y\nproperty float z\n'
            'property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar robot_id\n'
            'property uint submap_id\nproperty uchar uncovered\nend_header\n').encode('ascii')
        offset=0
        with path.open('wb') as stream:
            stream.write(header)
            for robot in robots:
                with np.load(run/robot/'union.npz',allow_pickle=False) as f:
                    world=np.asarray(f['world'],float); ids=f['point_ids']
                    if len(np.unique(ids))!=len(ids): raise ValueError('Duplicate native map IDs')
                points,owner,uncovered=corrected_union(world,keys[robot],optimized[robot])
                xyz=transform(level,points)
                if not np.isfinite(xyz).all(): raise ValueError('Nonfinite map coordinates')
                index=config['robots'].index(robot); color=PALETTE[index%len(PALETTE)]
                rgb=[int(color[k:k+2],16) for k in (1,3,5)]; colors[robot]=rgb
                rows=np.empty(len(xyz),DTYPE)
                for k,name in enumerate(('x','y','z')): rows[name]=xyz[:,k]
                for k,name in enumerate(('red','green','blue')): rows[name]=rgb[k]
                rows['robot_id']=index;rows['submap_id']=owner;rows['uncovered']=uncovered
                rows.tofile(stream)
                provenance[robot]=dict(robot_id=index,points=len(rows),first_vertex=offset,
                    uncovered_points=int(uncovered.sum()),component=root,
                    union_sha256=file_hash(run/robot/'union.npz'),
                    submap_index_sha256=file_hash(run/robot/'store/keyframes.jsonl'))
                offset+=len(rows)
        if offset!=total or path.stat().st_size!=len(header)+total*DTYPE.itemsize:
            raise ValueError('PLY vertex count or byte-size mismatch')
        check=np.memmap(path,dtype=DTYPE,mode='r',offset=len(header),shape=(total,))
        if not all(np.isfinite(check[k]).all() for k in ('x','y','z')):
            raise ValueError('PLY readback is nonfinite')
        counts={int(k):int(n) for k,n in zip(*np.unique(check['robot_id'],return_counts=True))}
        if counts!={config['robots'].index(r):provenance[r]['points'] for r in robots}:
            raise ValueError('PLY readback robot counts differ')
        del check
        files.append(dict(path=str(path),points=total,bytes=path.stat().st_size,
            sha256=file_hash(path),robots=robots,T_level_cbs=level.tolist(),
            xyz_frame=f'{root} CBS anchor frame, rotated using native IMU gravity'))
    result=dict(files=files,components=graph['components'],robots=provenance,colors_rgb=colors,
        ground_truth_used=False,voxel_downsampling=False,
        geometry='All unique retained native representatives, re-posed by optimized circle submaps',
        ownership='Closest footprint center; uncovered points retained with nearest correction and flag',
        graph_sha256=file_hash(run/'central/graph.json'))
    write_json(output/'export.json',result)
    return result
