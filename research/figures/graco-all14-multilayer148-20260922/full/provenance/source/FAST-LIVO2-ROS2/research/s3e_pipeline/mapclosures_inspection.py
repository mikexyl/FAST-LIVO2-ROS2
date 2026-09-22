"""Recover native intermediates from a frozen run, checking cached features and poses."""
from pathlib import Path
import json
import subprocess
import time
import zlib
import numpy as np
from .artifacts import read_jsonl,write_json,write_jsonl
from .backends import unpack_array
from .registration import bounded_cloud


def density_pixels(xy,lower_bound):
    """Native coordinates are (LiDAR x,row; LiDAR y,column); image is (u,v)."""
    return (np.asarray(xy).reshape(-1,2)-np.asarray(lower_bound).reshape(1,2))[:,::-1]


def check_features(debug,frozen):
    kept=np.asarray(debug['kept_indices'],dtype=int)
    if not np.array_equal(np.asarray(debug['orb_bits'])[kept],frozen['bits']):
        raise ValueError('Density reconstruction changed cached ORB descriptors')
    if not np.allclose(np.asarray(debug['orb_xy'])[kept],density_pixels(frozen['xy'],debug['lower_bound']),atol=1e-5):
        raise ValueError('Density reconstruction changed cached ORB keypoints')


def build(cfg,artifacts,output):
    import s3e_mapclosures_inspection as native
    from .cli import SOURCE
    started=time.monotonic();output=Path(output);options=cfg['backend']['mapclosures']
    if native.upstream_commit!=options['upstream_commit']:raise ValueError('Inspection source mismatch')
    def engine():return native.Inspector(options['density_map_resolution'],options['density_threshold'],options['hamming_distance_threshold'])
    loops=Path(artifacts['loops.livo.megaloc_mapclosures'])
    events=[e for e in read_jsonl(loops/'events.jsonl') if e['type']=='verification']
    rows={r:read_jsonl(Path(artifacts[f'keyframes.livo.{r}'])/'store/keyframes.jsonl') for r in cfg['robots']}
    descriptors={};needed={tuple(e[k]) for e in events for k in ('query','candidate')}
    def features(robot,key):
        endpoint=(robot,key)
        if endpoint not in descriptors:
            path=Path(artifacts[f'descriptors.livo.megaloc_mapclosures.{robot}'])/f'{key:06d}.json.zlib'
            d=json.loads(zlib.decompress(path.read_bytes()))
            descriptors[endpoint]={k:unpack_array(v) for k,v in d['mapclosures'].items()}
        return descriptors[endpoint]
    inspector=engine();maps=output/'maps';maps.mkdir()
    for index,(robot,key) in enumerate(sorted(needed)):
        store=Path(artifacts[f'keyframes.livo.{robot}'])/'store';frozen=features(robot,key)
        with np.load(store/f'{key:06d}.npz') as archive:cloud=archive['cloud'][:,:3].astype(np.float64)
        cloud=cloud[np.isfinite(cloud).all(1)&(np.linalg.norm(cloud,axis=1)<=options['max_range_m'])]
        debug=inspector.density(cloud,frozen['ground']);check_features(debug,frozen)
        evidence=cfg['backend']['evidence'];points,_=bounded_cloud(cloud,evidence['voxel_m'],evidence['max_points'],80.)
        np.savez_compressed(maps/f'{robot}-{key:06d}.npz',**debug,cloud=points.astype(np.float32),
                            ground=frozen['ground'],cached_xy=frozen['xy'],cached_bits=frozen['bits'])
        (maps/f'{robot}-{key:06d}.png').write_bytes((store/f'{key:06d}.png').read_bytes())
        if index%50==0:print(f'Inspected density/ORB: {index+1}/{len(needed)} local maps',flush=True)
    # Rebuild each native HBST index in the original query-delivery order. No
    # candidate search, payload delivery, GICP or PGO is rerun here.
    engines={r:engine() for r in rows};added={r:set() for r in rows};requests={}
    for index,e in enumerate(events):requests.setdefault((tuple(e['query']),e['candidate'][0]),[]).append(index)
    messages=sorted((m for m in read_jsonl(loops/'exchanges.jsonl') if m['kind']=='query'),
                    key=lambda m:(m['delivery_ns'],m['dst'],m['id']))
    pairs=output/'pairs';pairs.mkdir();completed=set()
    for m in messages:
        robot=m['dst'];q=(m['src'],m['query_id']);stamp=rows[q[0]][q[1]]['stamp_ns']
        eligible=[r['keyframe_id'] for r in rows[robot] if r['stamp_ns']<=stamp and
            (robot!=q[0] or stamp-r['stamp_ns']>=round(cfg['loops']['same_robot_exclusion_s']*1e9))]
        for key in eligible:
            if key not in added[robot]:engines[robot].add(key,features(robot,key));added[robot].add(key)
        for index in requests.get((q,robot),[]):
            if index in completed:raise ValueError('Duplicate inspection query')
            e=events[index];candidate=tuple(e['candidate']);expected=e['native']['hypothesis']
            if 'mapclosures' in e['retrieval_sources']:
                if candidate[1] not in eligible:raise ValueError('Noncausal inspection pair')
                debug=engines[robot].correspondences(features(*q),candidate[1]);mode='original causal HBST database'
            else:
                pair=engine();pair.add(0,features(*candidate));debug=pair.correspondences(features(*q),0)
                mode='original visual-proposal two-map HBST index'
            if len(debug['query_xy'])!=expected['matches'] or len(debug['ransac_inlier_indices'])!=expected['inliers']:
                raise ValueError(f'Native match/inlier replay differs at verification {index}')
            if expected.get('valid_pose') and not np.allclose(debug['T_i_j'],expected['T_i_j'],atol=1e-8):
                raise ValueError(f'Native pose replay differs at verification {index}')
            np.savez_compressed(pairs/f'{index:06d}.npz',**debug)
            e['inspection']=dict(original_event_index=index,index_mode=mode,exact_native_replay=True)
            completed.add(index)
    if len(completed)!=len(events):raise ValueError('Missing inspection events')
    # Put a strong independent LiDAR loop first, followed by all remaining events
    # in their saved order. Original indices and timestamps stay visible.
    showcase=sorted(range(len(events)),key=lambda i:(not events[i]['accepted'],
        events[i]['retrieval_sources']!=['mapclosures'],-events[i]['native']['hypothesis']['inliers'],i))[:1]
    ordered=showcase+[i for i in range(len(events)) if i not in showcase]
    write_jsonl(output/'events.jsonl',[dict(events[i],inspection_pair=k) for k,i in enumerate(ordered)])
    write_json(output/'summary.json',dict(schema_version=1,verified_pairs=len(events),local_maps=len(needed),
        accepted=sum(e['accepted'] for e in events),exact_cached_features=True,exact_native_match_replay=True,
        ground_truth_used=False,extraction_runtime_s=time.monotonic()-started,
        density_coordinate_convention='native image u=LiDAR y / resolution - lower_y; v=LiDAR x / resolution - lower_x'))
    subprocess.run([str(SOURCE/'.ros2/rerun-venv/bin/python'),str(Path(__file__).with_name('mapclosures_rerun.py')),str(output)],check=True)
