"""Full-sequence, paired raw/ellipsoid BEV loop detection and centralized ATE.

Both branches share exact odometry, keyframes, raw geometric evidence and GNC
settings. Descriptor construction is offline, detection uses the existing
isolated robot workers and deterministic causal scheduler. GT enters only after
both graphs have been optimized. There is no parameter sweep.
"""
import argparse
from collections import deque,Counter
from pathlib import Path
import json
import time
import zlib
import numpy as np
import yaml

from .artifacts import canonical,digest,file_hash,read_json,read_jsonl,write_json,write_jsonl,stage_output,validate_stage
from .geometry import pose,inv,transform,extrinsic,voxel_downsample

SOURCE=Path(__file__).resolve().parents[3]
RESEARCH=Path(__file__).resolve().parents[1]
ROBOTS=('Alpha','Bob','Carol')


def config(work=None):
    path=Path(work)/'config.yaml' if work is not None else RESEARCH/'configs/square1-ellipselio-mapclosures-cbs.yaml'
    cfg=yaml.safe_load(path.read_text())
    cfg.pop('dpgo',None)
    return cfg


def source_hashes(names):
    return {str(p.relative_to(SOURCE)):file_hash(p) for p in [Path(__file__),*[RESEARCH/'s3e_pipeline'/n for n in names]]}


def prepare_robot(work,robot,resume=False):
    import s3e_mapclosures_native as native
    import s3e_mapclosures_inspection as inspection
    from .ellipsoid_cuda import SurfaceSampler
    from .ellipsoid_delta_writer import DeltaReader
    from .data import frames
    from .backends import pack_array
    from .mapclosures_inspection import check_features
    from .evaluation import save_tum
    cfg=config(work);export=work/robot/'export';schedule=read_json(work/'schedule.json')['schedule'][robot]
    expected={r['stamp_ns']:r['keyframe_id'] for r in schedule};seen=set();mc=cfg['backend']['mapclosures']
    sources=source_hashes(['ellipsoid_cuda.py','ellipsoid_bev.py','ellipsoid_delta_writer.py','data.py','geometry.py','backends.py','mapclosures.py'])
    sources['native_mapclosures']=file_hash(Path(native.__file__))
    sources['native_cuda']=file_hash(SOURCE/'.ros2/ellipsoid-cuda/libellipsoid_surface.so')
    inputs=dict(export=file_hash(export/'manifest.json'),schedule=file_hash(work/'schedule.json'),config=file_hash(work/'config.yaml'))
    settings=dict(robot=robot,keyframes=cfg['keyframes'],mapclosures=mc,surface_spacing_m=.125,
                  voxel_m=.25,accumulator_resolution_m=1e-7,ground_truth_used=False)
    def engine():return native.MapClosures(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    with stage_output(work/'stages',f'prepare-{robot}',settings,inputs,digest(sources),resume) as (out,cached):
        if not cached:
            write_json(out/'source-hashes.json',sources)
            for name in ('store','raw','ellipsoid','bevs'): (out/name).mkdir()
            sampler=SurfaceSampler();delta=DeltaReader();trailing=deque();rows=[];odom=[];timings=[]
            started=time.monotonic();next_report=started
            try:
                for row,cloud,_ in frames(export):
                    T=pose(row['T_world_body']);odom.append({k:row[k] for k in ('robot_id','frame_id','stamp_ns','T_world_body')})
                    body=transform(extrinsic(row),cloud[:,:3]);reduced=voxel_downsample(body,.25)
                    trailing.append((row['stamp_ns'],T,reduced))
                    while trailing and trailing[0][0]<row['stamp_ns']-5*10**9:trailing.popleft()
                    if 'ellipsoid_delta' not in row:continue
                    begin=time.monotonic();requested=row['ellipsoids']['requested_stamps_ns']
                    # A schedule from another frontend may precede this filter's
                    # first initialized export. Keep the actual frame timestamp;
                    # only that seed frame gets a bounded startup allowance.
                    association_limit_ns=2*10**9 if len(odom)==1 and not row['lidar_updated'] else 250000000
                    if any(s not in expected or s in seen or not 0<=row['stamp_ns']-s<=association_limit_ns for s in requested):
                        raise ValueError('Missing, duplicate or delayed snapshot schedule')
                    seen.update(requested);key=len(rows)
                    raw=voxel_downsample(np.concatenate([transform(inv(T)@oldT,p) for _,oldT,p in trailing]),.25).astype('f4')
                    raw=raw[np.linalg.norm(raw,axis=1)<=80.]
                    np.savez_compressed(out/'store'/f'{key:06d}.npz',cloud=raw,scan=body.astype('f4'))
                    item={k:v for k,v in row.items() if k!='messages'}
                    item.update(keyframe_id=key,source_keyframe_ids=[expected[s] for s in requested],
                        camera=None,cloud_frame=row['body_frame'],submap_start_ns=trailing[0][0],
                        submap_end_ns=row['stamp_ns'],submap_points=len(raw),
                        geometry_preprocessing='same frozen raw trailing 5 s submap for both branch verifiers')
                    rows.append(item)
                    raw_started=time.monotonic();rf=engine().describe(raw.astype(float));raw_s=time.monotonic()-raw_started
                    entry=row['ellipsoid_delta'];g=delta.apply(export/entry['file'],entry['current_count'])
                    if entry['frame']!=row['world_frame']:raise ValueError('Wrong ellipsoid delta coordinate frame')
                    if len(g['centers']):
                        centers=transform(inv(T),g['centers']);basis=T[:3,:3].T@g['basis']
                        started_render=time.monotonic();ellipsoid,sampling=sampler.render(centers,g['axes'],basis)
                        sampling['runtime_s']=time.monotonic()-started_render
                        ellipsoid_started=time.monotonic();ef=engine().describe(ellipsoid);ellipsoid_descriptor_s=time.monotonic()-ellipsoid_started
                    else:
                        ellipsoid=np.empty((0,3));sampling=dict(runtime_s=0.,surface_samples=0,voxel_points=0,reason='no_fitted_ellipsoids_at_initialization')
                        ef=dict(ground=np.eye(4),xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8))
                        ellipsoid_descriptor_s=0.
                    debug=inspection.Inspector(.5,.05,50)
                    for branch,features,points in [('raw',rf,raw.astype(float)),('ellipsoid',ef,ellipsoid)]:
                        packed=dict(mapclosures={k:pack_array(features[k]) for k in ('ground','xy','bits')},
                            mapclosures_features=len(features['xy']),representation=branch)
                        (out/branch/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(packed)))
                        if len(points):
                            image=debug.density(points,features['ground']);check_features(image,features)
                        else:
                            image=dict(image=np.zeros((1,1),dtype=np.uint8),lower_bound=np.zeros(2),orb_xy=np.empty((0,2)),
                                       kept_indices=np.empty(0,dtype=int))
                        np.savez_compressed(out/'bevs'/f'{key:06d}-{branch}.npz',**image,
                            ground=features['ground'],cached_xy=features['xy'],cached_bits=features['bits'])
                    timing=dict(keyframe_id=key,stamp_ns=row['stamp_ns'],ellipsoids=len(g['centers']),
                        raw_orb=len(rf['xy']),ellipsoid_orb=len(ef['xy']),raw_descriptor_s=raw_s,
                        ellipsoid_descriptor_s=ellipsoid_descriptor_s,
                        schedule_max_delay_ns=max(row['stamp_ns']-s for s in requested),
                        initialization_schedule_allowance=association_limit_ns>250000000,
                        sampling=sampling,frame_prepare_s=time.monotonic()-begin)
                    timings.append(timing)
                    if time.monotonic()>=next_report:
                        progress=dict(robot=robot,keyframes=len(rows),scheduled=len(schedule),
                            elapsed_s=time.monotonic()-started,last=timing)
                        write_json(work/f'prepare-{robot}-progress.json',progress)
                        print(f'{robot}: {len(rows)}/{len(schedule)} keyframes, '+
                              f'ellipsoids={len(g["centers"])}, CUDA={sampling["runtime_s"]:.3f}s, '+
                              f'ORB raw/ellipsoid={len(rf["xy"])}/{len(ef["xy"])}',flush=True)
                        next_report=time.monotonic()+20
            finally:sampler.close()
            if seen!=set(expected):raise ValueError('Not all scheduled maps were exported')
            if any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])):raise ValueError('Nonmonotonic keyframes')
            write_jsonl(out/'store/keyframes.jsonl',rows);write_jsonl(out/'odometry.jsonl',odom)
            write_jsonl(out/'timings.jsonl',timings);save_tum(out/'raw.tum',odom)
            write_json(out/'summary.json',dict(robot=robot,keyframes=len(rows),scheduled=len(schedule),
                odometry_frames=len(odom),merged_schedule_snapshots=len(schedule)-len(rows),
                preparation_s=time.monotonic()-started,sampling_s=sum(r['sampling']['runtime_s'] for r in timings),
                ground_truth_used=False,input_hashes=inputs))
    # stage_output renames a successful incomplete directory to its content hash.
    final=out if cached else out.parent/out.name.split('.incomplete-')[0]
    manifest=validate_stage(final)
    write_json(work/f'prepared-{robot}.json',dict(path=str(final.resolve()),stage_hash=manifest['stage_hash']))
    print(f'{robot} complete: {final}',flush=True)


def solve(work,resume=False):
    from .distributed import replay
    from .pgo import optimize
    cfg=config(work);prepared={r:Path(read_json(work/f'prepared-{r}.json')['path']) for r in ROBOTS}
    manifests={r:validate_stage(p) for r,p in prepared.items()};registry={};graphs={};all_rows=[]
    sources=source_hashes(['distributed.py','mapclosures.py','backends.py','registration.py','verification.py','isolation.py','geometry.py','pgo.py'])
    inputs={r:m['stage_hash'] for r,m in manifests.items()}
    for r in ROBOTS:all_rows+=read_jsonl(prepared[r]/'store/keyframes.jsonl')
    for branch in ('raw','ellipsoid'):
        with stage_output(work/'stages',f'loops-{branch}',dict(backend=cfg['backend'],loops=cfg['loops']),
                          inputs,digest(sources),resume) as (out,cached):
            if not cached:
                replay({r:p/'store' for r,p in prepared.items()},{r:p/branch for r,p in prepared.items()},
                    cfg['backend'],dict(cfg['loops'],robots=list(ROBOTS)),out)
                write_json(out/'source-hashes.json',sources)
        loops=out if cached else out.parent/out.name.split('.incomplete-')[0]
        with stage_output(work/'stages',f'pgo-{branch}',cfg['pgo'],dict(prepared=inputs,loops=validate_stage(loops)['stage_hash']),
                          digest(sources),resume) as (out,cached):
            if not cached:
                start=time.monotonic();graph=optimize(all_rows,read_jsonl(loops/'constraints.jsonl'),cfg['pgo'])
                graph['optimization_s']=time.monotonic()-start;write_json(out/'graph.json',graph)
        pgo=out if cached else out.parent/out.name.split('.incomplete-')[0]
        registry[branch]=dict(loops=str(loops.resolve()),pgo=str(pgo.resolve()))
        graphs[branch]=read_json(pgo/'graph.json')
        print(f'{branch}: {graphs[branch]["components"]}, '+
              f'{sum(f["kind"]=="loop" and f["solver_weight"]>0 for f in graphs[branch]["factors"])} selected loops',flush=True)
    write_json(work/'solved.json',dict(branches=registry,prepared={r:str(p.resolve()) for r,p in prepared.items()},config=cfg))


def evaluate(work,output):
    # No GT access is needed by preparation, detection or graph initialization.
    from .evaluation import corrected_dense_rows,save_tum,ground_truth
    from .evo_evaluation import evaluate as evo
    solved=read_json(work/'solved.json');cfg=solved['config'];output.mkdir(parents=True,exist_ok=False)
    prepared={r:Path(p) for r,p in solved['prepared'].items()}
    keys={r:read_jsonl(p/'store/keyframes.jsonl') for r,p in prepared.items()}
    raw={r:read_jsonl(p/'odometry.jsonl') for r,p in prepared.items()}
    graphs={b:read_json(Path(v['pgo'])/'graph.json') for b,v in solved['branches'].items()}
    dense={}
    for branch,graph in graphs.items():
        path=output/branch;path.mkdir();dense[branch]=[]
        for robot in ROBOTS:
            opt=[r for r in graph['poses'] if r['robot_id']==robot]
            rows=[dict(r,component=graph['components'][robot]) for r in corrected_dense_rows(raw[robot],keys[robot],opt)]
            save_tum(path/f'{robot}-corrected.tum',rows);dense[branch]+=rows
        write_jsonl(path/'poses.jsonl',dense[branch]);write_json(path/'graph.json',graph)
    gt={r:ground_truth(Path(cfg['dataset'])/(r.lower()+'_gt.txt')) for r in ROBOTS}
    report=dict(schema_version=1,dataset=cfg['dataset'],frontend='ellipselio',solver=cfg['pgo'],branches={},
        comparison='raw-cloud versus ellipsoid-surface BEVs; identical fresh odometry, keyframes, raw verification evidence and centralized GNC-TLS PGO',
        keyframes={r:len(v) for r,v in keys.items()},odometry_frames={r:len(v) for r,v in raw.items()},
        frozen_preparation={r:validate_stage(p)['stage_hash'] for r,p in prepared.items()})
    schedule=read_json(work/'schedule.json')
    report['schedule_provenance']={k:v for k,v in schedule.items() if k!='schedule'}
    report['preparation']={}
    for robot,p in prepared.items():
        times=read_jsonl(p/'timings.jsonl')
        report['preparation'][robot]=dict(read_json(p/'summary.json'),
            raw_descriptor_s=sum(t['raw_descriptor_s'] for t in times),
            ellipsoid_descriptor_s=sum(t['ellipsoid_descriptor_s'] for t in times),
            raw_orb_total=sum(t['raw_orb'] for t in times),ellipsoid_orb_total=sum(t['ellipsoid_orb'] for t in times))
    report['odometry_runtime_s']=sum(read_json(work/r/'summary.json')['wall_s'] for r in ROBOTS)
    report['raw_odometry']=evo(dict(poses=[dict(x,component=r) for r in ROBOTS for x in raw[r]]),gt,cfg['evaluation'],output/'evo/odometry')
    for branch,graph in graphs.items():
        loops=Path(solved['branches'][branch]['loops']);events=read_jsonl(loops/'events.jsonl')
        constraints=read_jsonl(loops/'constraints.jsonl');verifications=[e for e in events if e['type']=='verification']
        report['branches'][branch]=dict(trajectory=evo(dict(poses=dense[branch]),gt,cfg['evaluation'],output/f'evo/{branch}'),
            components=graph['components'],constraints=len(constraints),
            selected_loops=sum(f['kind']=='loop' and f['solver_weight']>0 for f in graph['factors']),
            selected_inter_robot_loops=sum(f['kind']=='loop' and f['solver_weight']>0 and f['i'][0]!=f['j'][0] for f in graph['factors']),
            verification_attempts=len(verifications),verification_reasons=dict(Counter(e['reason'] for e in verifications)),
            optimization_s=graph['optimization_s'],detection=read_json(loops/'summary.json'),
            verification_task_time_sum_s=sum(e['verification_runtime_s'] for e in verifications))
        write_json(output/branch/'detection-summary.json',report['branches'][branch]['detection'])
        write_jsonl(output/branch/'constraints.jsonl',constraints);write_jsonl(output/branch/'verifications.jsonl',verifications)
    write_json(output/'report.json',report);write_json(output/'inputs.json',solved)
    for branch,v in report['branches'].items():
        print(branch,json.dumps(dict(trajectory=v['trajectory'],constraints=v['constraints'],selected_loops=v['selected_loops'])),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['prepare','solve','evaluate'])
    p.add_argument('--work',type=Path,required=True);p.add_argument('--robot',choices=ROBOTS)
    p.add_argument('--output',type=Path);p.add_argument('--resume',action='store_true');args=p.parse_args()
    work=args.work.resolve()
    if args.stage=='prepare':
        if not args.robot:p.error('--robot is required for preparation')
        prepare_robot(work,args.robot,args.resume)
    elif args.stage=='solve':solve(work,args.resume)
    else:
        if args.output is None:p.error('--output is required for evaluation')
        evaluate(work,args.output.resolve())


if __name__=='__main__':main()
