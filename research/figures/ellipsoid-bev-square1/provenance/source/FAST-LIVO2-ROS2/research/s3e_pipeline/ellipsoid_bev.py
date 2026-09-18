"""Bounded EllipseLIO-map BEV test using unchanged native MapClosures.

Export native ellipsoid snapshots at frozen reference keyframe timestamps. Render
their actual surfaces, voxelize, and use the upstream density/ORB/HBST/RANSAC
implementation. No ground truth is read and no parameter search is performed.
"""
import argparse
from collections import deque
from functools import lru_cache
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .artifacts import file_hash, read_jsonl, write_json, write_jsonl
from .data import frames
from .geometry import extrinsic, inv, pose, transform, voxel_downsample
from .registration import bounded_cloud, refine

ROBOTS = ('Alpha', 'Bob', 'Carol')
RESEARCH = Path(__file__).resolve().parents[1]
SOURCE = RESEARCH.parent.parent


def prepare(reference, work, duration):
    """Freeze a small positive-pair diagnostic before rendering any new images."""
    reference, work = Path(reference), Path(work)
    work.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((RESEARCH/'configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    metadata = yaml.safe_load((Path(cfg['dataset'])/'metadata.yaml').read_text())
    start = metadata['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']
    rows = {r: read_jsonl(reference/r/'keyframes.jsonl') for r in ROBOTS}
    pairs = [dict(i=e['i'], j=e['j']) for e in read_jsonl(reference/'dpgo/constraints.jsonl')
             if e.get('accepted') and all(rows[e[k][0]][e[k][1]]['stamp_ns'] < start+round(duration*1e9)
                                        for k in ('i', 'j'))]
    if not pairs:
        raise ValueError('No reference loops inside diagnostic interval')
    needed = sorted({tuple(p[k]) for p in pairs for k in ('i', 'j')})
    schedule = {r: [dict(keyframe_id=k, stamp_ns=rows[r][k]['stamp_ns']) for robot,k in needed if robot == r]
                for r in ROBOTS}
    for robot in ROBOTS:
        mapping = yaml.safe_load((reference/robot/'odometry/mapping_config.yaml').read_text())
        mapping['/**']['ros__parameters']['research'] = dict(
            ellipsoid_stamps_ns=[s['stamp_ns'] for s in schedule[robot]], ellipsoid_range_m=80.0)
        (work/f'{robot}.yaml').write_text(yaml.safe_dump(mapping, sort_keys=False))
    write_json(work/'test.json', dict(schema_version=1, reference=str(reference.resolve()),
        duration_s=duration, bag_start_ns=start, pairs=pairs, schedule=schedule, config=cfg,
        selection='All previously accepted pairs with both endpoints before fixed replay cutoff; positive-pair diagnostic only',
        reference_hashes={str(p.relative_to(reference)):file_hash(p) for p in
                          [reference/'dpgo/constraints.jsonl', *[reference/r/'keyframes.jsonl' for r in ROBOTS]]}))
    print(json.dumps(dict(pairs=len(pairs), snapshots=len(needed), work=str(work)), indent=2))


def decode_ellipsoids(msg):
    names = ['x','y','z','a','b','c',*[f'v{r}{c}' for r in range(3) for c in range(3)]]
    fields = {f.name:f for f in msg.fields}
    def column(name, dtype):
        field = fields[name]
        expected = 7 if dtype == 'f4' else 6
        if field.datatype != expected or field.count != 1:
            raise ValueError('Invalid ellipsoid field schema')
        if msg.width*msg.height == 0:
            return np.empty(0,dtype=dtype)
        return np.ndarray((msg.height,msg.width), dtype=('>' if msg.is_bigendian else '<')+dtype,
            buffer=msg.data, offset=field.offset, strides=(msg.row_step,msg.point_step)).ravel().copy()
    values = np.column_stack([column(n,'f4') for n in names]).astype(np.float64)
    centers, axes, basis = values[:,:3], values[:,3:6], values[:,6:].reshape(-1,3,3)
    if len(centers):validate_ellipsoids(centers, axes, basis)
    return dict(centers=centers, axes=axes, basis=basis,
                primitive=column('primitive','u4'), map_id=column('map_id','u4'))


def validate_ellipsoids(centers, axes, basis):
    n = len(centers)
    if centers.shape != (n,3) or axes.shape != (n,3) or basis.shape != (n,3,3) or not n:
        raise ValueError('Invalid/empty ellipsoid array shape')
    if not all(np.isfinite(x).all() for x in (centers,axes,basis)) or np.any(axes <= 0):
        raise ValueError('Invalid ellipsoid geometry')
    # Eigenvector matrices can be improper orthogonal bases; no quaternion is
    # needed for sampling an ellipsoid, and reflection does not change its shape.
    if not np.allclose(basis.transpose(0,2,1)@basis, np.eye(3), atol=2e-5):
        raise ValueError('Ellipsoid axes are not orthonormal')


@lru_cache(maxsize=16)
def unit_surface(count):
    """Sign-symmetric sphere samples plus exact extremal points of every axis."""
    n = max(1, (count-6+7)//8)
    z = (np.arange(n)+.5)/n
    theta = np.remainder(np.arange(n)*(np.sqrt(5)-1)/2, 1)*np.pi/2
    octant = np.column_stack((np.sqrt(1-z*z)*np.cos(theta), np.sqrt(1-z*z)*np.sin(theta), z))
    signs = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    return np.concatenate(((octant[:,None,:]*signs[None,:,:]).reshape(-1,3),np.eye(3),-np.eye(3)))


def surface_points(centers, axes, basis, spacing=.125, voxel=.25, max_range=80.):
    """Sample native surfaces; do not inflate axes or treat them as covariances.

    Sampling count uses the largest-axis enclosing sphere, rounded up to a power
    of two, then voxel centroids merge overlapping surfaces. Chunked sampling
    bounds temporary memory; the resolution is fixed for the whole experiment.
    """
    import small_gicp
    validate_ellipsoids(centers,axes,basis)
    if not np.isfinite([spacing,voxel]).all() or min(spacing,voxel) <= 0:
        raise ValueError('Invalid sampling resolution')
    required = np.maximum(32, np.ceil(4*np.pi*(axes.max(axis=1)/spacing)**2)).astype(int)
    counts = (2**np.ceil(np.log2(required))).astype(int)
    if counts.max() > 8192:
        raise ValueError('Ellipsoid size exceeds bounded diagnostic sampling budget')
    parts=[]; generated=0
    for count in np.unique(counts):
        indices=np.flatnonzero(counts==count); sphere=unit_surface(int(count))
        batch=max(1,250000//len(sphere))
        for start in range(0,len(indices),batch):
            idx=indices[start:start+batch]
            local=axes[idx,None,:]*sphere[None,:,:]
            points=np.einsum('nij,nkj->nki',basis[idx],local)+centers[idx,None,:]
            points=points.reshape(-1,3); generated+=len(points)
            if max_range is not None:points=points[np.linalg.norm(points,axis=1)<=max_range]
            if len(points):parts.append(small_gicp.voxelgrid_sampling(points,voxel,num_threads=1).points()[:,:3].copy())
    if not parts:raise ValueError('All ellipsoid geometry lies outside crop')
    points=small_gicp.voxelgrid_sampling(np.concatenate(parts),voxel,num_threads=1).points()[:,:3].copy()
    return points, dict(surface_samples=generated, voxel_points=len(points), spacing_m=spacing, voxel_m=voxel)


def extract_robot(work, robot, schedule):
    """Use raw causal submaps and ellipsoids from exactly the same updated state."""
    from mcap.reader import NonSeekingReader
    from rosbags.typesys import Stores, get_typestore
    export=work/robot/'export'; trailing=deque(); snapshots={}
    lookup={s['stamp_ns']:s['keyframe_id'] for s in schedule}
    for row,cloud,_ in frames(export):
        T=pose(row['T_world_body'])
        points=voxel_downsample(transform(extrinsic(row),cloud[:,:3]),.25)
        trailing.append((row['stamp_ns'],T,points))
        while trailing and trailing[0][0]<row['stamp_ns']-5*10**9:trailing.popleft()
        if 'ellipsoids' not in row:continue
        requested=row['ellipsoids']['requested_stamps_ns']
        if len(requested)!=1 or requested[0] not in lookup:
            raise ValueError('Missing or merged snapshot endpoint')
        delta=row['stamp_ns']-requested[0]
        if not 0<=delta<=250000000:raise ValueError('Snapshot timestamp association exceeds 250 ms')
        raw=voxel_downsample(np.concatenate([transform(inv(T)@oldT,p) for _,oldT,p in trailing]),.25)
        raw=raw[np.linalg.norm(raw,axis=1)<=80.]
        snapshots[row['stamp_ns']]=dict(row=row,raw=raw,keyframe_id=lookup[requested[0]],association_delta_ns=delta)
    types=get_typestore(Stores.ROS2_HUMBLE)
    with (export/'sensors.mcap').open('rb') as stream:
        for schema,channel,msg in NonSeekingReader(stream).iter_messages(log_time_order=False):
            if not channel.topic.endswith('/ellipsoids'):continue
            native=types.deserialize_cdr(msg.data,schema.name)
            item=snapshots[msg.log_time]
            if native.header.frame_id!=item['row']['body_frame']:
                raise ValueError('Ellipsoid frame differs from registration body frame')
            item.update(decode_ellipsoids(native))
            if len(item['centers'])!=item['row']['ellipsoids']['count']:raise ValueError('Ellipsoid count mismatch')
    if len(snapshots)!=len(schedule) or any('centers' not in s for s in snapshots.values()):
        raise ValueError('Replay did not export every requested snapshot')
    return list(snapshots.values())


def evaluate(work, output):
    import s3e_mapclosures_native as native
    import s3e_mapclosures_inspection as inspection
    from .mapclosures_inspection import check_features
    work,output=Path(work),Path(output);output.mkdir(parents=True,exist_ok=False)
    spec=json.loads((work/'test.json').read_text());cfg=spec['config']['backend'];mc=cfg['mapclosures']
    if native.upstream_commit!=mc['upstream_commit']:raise ValueError('Native MapClosures version mismatch')
    def engine():return native.MapClosures(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    def inspector():return inspection.Inspector(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    start=time.monotonic();maps={};mapstats=[]
    (output/'maps').mkdir();(output/'pairs').mkdir()
    for robot in ROBOTS:
        snapshots=extract_robot(work,robot,spec['schedule'][robot])
        for item in snapshots:
            key=item['keyframe_id'];endpoint=(robot,key);begin=time.monotonic()
            if len(item['centers']):
                ellipsoid,sampling=surface_points(item['centers'],item['axes'],item['basis'])
            else:
                # The initialization scan seeds the map before native tensor
                # fitting is available. An empty fitted map is a real failure
                # to describe this endpoint, not a reason to fabricate spheres.
                ellipsoid=np.empty((0,3));sampling=dict(surface_samples=0,voxel_points=0,
                    reason='no_fitted_ellipsoids_at_initialization')
            sampling['runtime_s']=time.monotonic()-begin
            evidence,_=bounded_cloud(item['raw'],cfg['evidence']['voxel_m'],cfg['evidence']['max_points'],80.)
            maps[endpoint]=dict(evidence=evidence, row=item['row'], branches={})
            record=dict(robot_id=robot,keyframe_id=key,ellipsoids=len(item['centers']),
                        association_delta_ns=item['association_delta_ns'],sampling=sampling,branches={})
            np.savez_compressed(output/'maps'/f'{robot}-{key:06d}-geometry.npz',
                centers=item['centers'].astype('f4'),axes=item['axes'].astype('f4'),basis=item['basis'].astype('f4'),
                primitive=item['primitive'],map_id=item['map_id'],raw=evidence.astype('f4'),
                ellipsoid_cloud=ellipsoid.astype('f4'))
            for branch,points in [('raw',item['raw']),('ellipsoid',ellipsoid)]:
                begin=time.monotonic()
                if len(points)>=30:
                    f=engine().describe(points)
                    debug=inspector().density(points,f['ground']);check_features(debug,f)
                else:
                    f=dict(ground=np.eye(4),xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8))
                    debug=dict(image=np.zeros((1,1),dtype=np.uint8),lower_bound=np.zeros(2,dtype=int),
                        orb_xy=np.empty((0,2)),orb_angles=np.empty(0),orb_responses=np.empty(0),
                        orb_bits=f['bits'],kept_indices=np.empty(0,dtype=int))
                maps[endpoint]['branches'][branch]=f
                np.savez_compressed(output/'maps'/f'{robot}-{key:06d}-{branch}.npz',**debug,
                                    ground=f['ground'],cached_xy=f['xy'],cached_bits=f['bits'])
                record['branches'][branch]=dict(points=len(points),orb_detected=len(debug['orb_xy']),
                    orb_retained=len(f['xy']),runtime_s=time.monotonic()-begin)
            mapstats.append(record)
            print(f'{robot} {key}: {len(item["centers"])} ellipsoids, ORB raw/ellipse '+
                  f'{record["branches"]["raw"]["orb_retained"]}/{record["branches"]["ellipsoid"]["orb_retained"]}',flush=True)
    def verify_pair(i,j,branch,index):
        q,c=maps[tuple(i)],maps[tuple(j)];qf=q['branches'][branch];cf=c['branches'][branch]
        n=engine().pair(qf,cf)
        ins=inspector();ins.add(0,cf);debug=ins.correspondences(qf,0)
        if len(debug['ransac_inlier_indices'])!=n['inliers']:raise ValueError('Native RANSAC replay mismatch')
        np.savez_compressed(output/'pairs'/f'{index:03d}-{branch}.npz',**debug)
        begin=time.monotonic()
        if n['valid_pose'] and n['inliers']>mc['inliers_threshold']:
            result=refine(q['evidence'],c['evidence'],n['T_i_j'],cfg['registration'])
        else:result=dict(accepted=False,reason='native_insufficient_inliers')
        return dict(pair_index=index,i=i,j=j,branch=branch,native={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in n.items()},
                    registration=result,registration_runtime_s=time.monotonic()-begin)
    events=[]
    for index,pair in enumerate(spec['pairs']):
        for branch in ('raw','ellipsoid'):
            event=verify_pair(pair['i'],pair['j'],branch,index);events.append(event)
            print(f'Pair {index} {branch}: {event["native"]["inliers"]} inliers; {event["registration"]["reason"]}',flush=True)
    # Retrieval sanity check: only saved snapshots available by each query time
    # are indexed. This is a small selected database, not full distributed replay.
    retrieval=[]
    ordered=sorted(maps,key=lambda e:(maps[e]['row']['stamp_ns'],e))
    for branch in ('raw','ellipsoid'):
        indices={r:engine() for r in ROBOTS};added={r:[] for r in ROBOTS}
        for q in ordered:
            indices[q[0]].add(q[1],maps[q]['branches'][branch]);added[q[0]].append(q[1])
            for robot in ROBOTS:
                if robot==q[0]:continue
                ranked=indices[robot].query(maps[q]['branches'][branch],added[robot],20)
                valid=[h for h in ranked if h['valid_pose'] and h['inliers']>mc['inliers_threshold']]
                valid.sort(key=lambda h:(-h['inliers'],-h['matches'],h['keyframe_id']))
                entry=dict(query=list(q),candidate_robot=robot,branch=branch,
                    candidates=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in h.items()} for h in valid])
                if valid:
                    best=valid[0];candidate=(robot,best['keyframe_id'])
                    entry['registration']=refine(maps[q]['evidence'],maps[candidate]['evidence'],best['T_i_j'],cfg['registration'])
                    entry['candidate']=list(candidate)
                retrieval.append(entry)
    write_jsonl(output/'events.jsonl',events);write_jsonl(output/'retrieval.jsonl',retrieval)
    write_jsonl(output/'maps.jsonl',mapstats)
    summary=dict(schema_version=1,complete=True,ground_truth_used=False,source_test=spec,
        renderer=dict(method='deterministic sign-symmetric ellipsoid surface sampling; no axis inflation; voxel centroids; native MapClosures density',
                      surface_spacing_m=.125,voxel_m=.25,spatial_crop_radius_m=80.,
                      raw_reference='causal trailing 5 s raw scans; ellipsoids use the causal persistent native map'),
        snapshots=len(maps),pairs=len(spec['pairs']),runtime_s=time.monotonic()-start,
        positive_pair_results={branch:dict(native_pass=sum(e['native']['valid_pose'] and e['native']['inliers']>5 for e in events if e['branch']==branch),
            accepted=sum(e['registration']['accepted'] for e in events if e['branch']==branch)) for branch in ('raw','ellipsoid')},
        retrieval_results={branch:dict(queries=sum(e['branch']==branch for e in retrieval),
            verified=sum(e['branch']==branch and 'registration' in e for e in retrieval),
            accepted=sum(e['branch']==branch and e.get('registration',{}).get('accepted',False) for e in retrieval)) for branch in ('raw','ellipsoid')},
        limitations=['Selected positive-pair diagnostic; does not estimate full-sequence recall or false-positive rate',
                     'Persistent ellipsoid map and trailing raw submap have different histories',
                     'Common GICP checks fresh raw-cloud evidence; no CBS/PGO rerun'],
        export_manifests={r:json.loads((work/r/'export/manifest.json').read_text()) for r in ROBOTS},
        source_hashes={str(p.relative_to(SOURCE)):file_hash(p) for p in [Path(__file__),
            SOURCE/'ellipselio/src/research_frame.cpp',SOURCE/'ellipselio/src/map_processing.cpp',
            SOURCE/'ellipselio/include/map_processing.h',RESEARCH/'adapters/mapclosures/adapter.cpp',
            RESEARCH/'adapters/mapclosures/inspect.cpp',RESEARCH.parent/'scripts/run_ellipselio.py']},
        native_binary_hash=file_hash(Path(native.__file__)))
    write_json(output/'summary.json',summary)
    print(json.dumps(summary['positive_pair_results'],indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
    prep=s.add_parser('prepare');prep.add_argument('--reference',type=Path,required=True)
    prep.add_argument('--work',type=Path,required=True);prep.add_argument('--duration',type=float,default=43.)
    run=s.add_parser('evaluate');run.add_argument('--work',type=Path,required=True);run.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.command=='prepare':prepare(args.reference,args.work,args.duration)
    else:evaluate(args.work,args.output)


if __name__=='__main__':main()
