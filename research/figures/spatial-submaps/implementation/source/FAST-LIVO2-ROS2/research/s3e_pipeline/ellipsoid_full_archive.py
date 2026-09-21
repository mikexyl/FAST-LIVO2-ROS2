"""Verify compact PGO evidence and optionally retire large generated clouds."""
import argparse
from pathlib import Path
import gzip
import shutil
import re
import numpy as np
from .artifacts import file_hash,read_json,read_jsonl,write_json,write_jsonl,validate_stage
from .pgo import optimize
from .geometry import pose


def archive(work,output,retire=False):
    solved=read_json(work/'solved.json');report=read_json(output/'report.json');cfg=solved['config']
    robots=cfg['robots'];prepared={r:Path(p) for r,p in solved['prepared'].items()}
    if set(report['branches'])!={'raw','ellipsoid'}:raise ValueError('Incomplete paired evaluation')
    target=output/'provenance';target.mkdir(exist_ok=True)
    if (work/'provenance').exists():shutil.copytree(work/'provenance',target,dirs_exist_ok=True)
    failed_attempts=[]
    for summary_path in sorted(work.glob('*-attempt*/summary.json')):
        attempt=summary_path.parent
        if read_json(summary_path).get('success'):continue
        failed_attempts.append(attempt)
        retained=target/attempt.name;retained.mkdir(exist_ok=True)
        for name in ('summary.json','mapping.log','playback.log','runtime.yaml','mapping_config.yaml'):
            if (attempt/name).exists():shutil.copy2(attempt/name,retained/name)
        if (attempt/'export/frames.jsonl').exists():
            shutil.copy2(attempt/'export/frames.jsonl',retained/'frames.jsonl')
    for source in [work/'replay-attempts.json',*work.glob('*-imu-audit.json')]:
        if source.exists():shutil.copy2(source,target/source.name)
    export_audit={}
    schedule=read_json(work/'schedule.json')['schedule']
    for robot in robots:
        export=work/robot/'export';frames=read_jsonl(export/'frames.jsonl')
        snapshots=[r for r in frames if 'ellipsoid_delta' in r]
        expected=[r['stamp_ns'] for r in schedule[robot]]
        requested=[s for r in snapshots for s in r['ellipsoids']['requested_stamps_ns']]
        if requested!=expected:raise ValueError('Snapshot schedule coverage differs')
        manifest=read_json(export/'manifest.json')
        if not manifest['complete'] or manifest['frames']!=len(frames):raise ValueError('Incomplete odometry export')
        if any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(frames,frames[1:])):raise ValueError('Nonmonotonic exports')
        export_audit[robot]=dict(frames=len(frames),lidar_updates=sum(r['lidar_updated'] for r in frames),
            snapshots=len(snapshots),scheduled=len(expected),merged_schedule_snapshots=len(expected)-len(snapshots),complete=True,
            max_stamp_gap_s=max((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(frames,frames[1:])),
            max_uncovered_interval_ns=max(max(0,b['scan_start_ns']-a['scan_end_ns']) for a,b in zip(frames,frames[1:])),
            max_snapshot_delay_ns=max(r['stamp_ns']-s for r in snapshots for s in r['ellipsoids']['requested_stamps_ns']),
            max_noninitial_snapshot_delay_ns=max((r['stamp_ns']-s for r in snapshots if r['frame_id']!=frames[0]['frame_id'] for s in r['ellipsoids']['requested_stamps_ns']),default=0),
            maximum_ellipsoids=max(r['ellipsoid_delta']['current_count'] for r in snapshots),
            delta_entries=sum(r['ellipsoid_delta']['changed'] for r in snapshots),
            full_snapshot_entries=sum(r['ellipsoid_delta']['current_count'] for r in snapshots))
    write_json(work/'export-audit.json',export_audit)
    write_json(work/'preparation-summary.json',{r:read_json(p/'summary.json') for r,p in prepared.items()})
    keys={r:read_jsonl(p/'store/keyframes.jsonl') for r,p in prepared.items()};all_keys=[x for r in robots for x in keys[r]]
    key_lookup={(r,x['keyframe_id']):x for r in robots for x in keys[r]}
    workload={}
    for robot,p in prepared.items():
        times=read_jsonl(p/'timings.jsonl')
        workload[robot]=dict(keyframes=len(keys[robot]),
            raw_nonempty_keyframes=sum(t['raw_orb']>0 for t in times),
            ellipsoid_nonempty_keyframes=sum(t['ellipsoid_orb']>0 for t in times),
            raw_input_points_sum=sum(r['submap_points'] for r in keys[robot]),
            ellipsoid_input_points_sum=sum(t['sampling'].get('voxel_points',0) for t in times))
        # The fixed native ORB detector excludes a 31-pixel border. Retain
        # image extents to distinguish indoor coverage from feature quality.
        for branch in ('raw','ellipsoid'):
            shapes=[]
            for key in keys[robot]:
                with np.load(p/'bevs'/f'{key["keyframe_id"]:06d}-{branch}.npz') as image:
                    shapes.append(image['image'].shape)
            shapes=np.asarray(shapes)
            workload[robot][branch+'_image_extents']=dict(
                median_rows_cols=np.median(shapes,axis=0).tolist(),
                min_dimension_at_most_62_pixels=int(np.sum(shapes.min(axis=1)<=62)),
                images=len(shapes),orb_border_pixels=31)
    write_json(output/'workload.json',workload)
    repeats={};watched={}
    for robot,p in prepared.items():
        manifest=validate_stage(p)
        robot_out=output/'odometry'/robot;robot_out.mkdir(parents=True,exist_ok=True)
        for source,destination in [('store/keyframes.jsonl','keyframes.jsonl'),('odometry.jsonl','poses.jsonl'),('raw.tum','raw.tum'),('summary.json','preparation.json'),('timings.jsonl','timings.jsonl')]:
            shutil.copy2(p/source,robot_out/destination)
        write_json(target/f'prepared-{robot}-manifest.json',manifest)
        shutil.copy2(p/'source-hashes.json',target/f'prepared-{robot}-source-hashes.json')
        for name in ('runtime.yaml','mapping_config.yaml','summary.json'):
            shutil.copy2(work/robot/name,target/f'{robot}-{name}')
        shutil.copy2(work/robot/'export/manifest.json',target/f'{robot}-export-manifest.json')
        for source in [p/'odometry.jsonl',p/'store/keyframes.jsonl',*sorted((p/'raw').glob('*.zlib')),*sorted((p/'ellipsoid').glob('*.zlib'))]:
            watched[str(source)]=file_hash(source)
    for branch,paths in solved['branches'].items():
        loops=Path(paths['loops']);graph=read_json(Path(paths['pgo'])/'graph.json')
        validate_stage(loops)
        validate_stage(Path(paths['pgo']))
        constraints=read_jsonl(loops/'constraints.jsonl')
        # Recompute just the inexpensive frozen PGO, verifying the requested
        # separation between solver outputs and immutable front-end artifacts.
        repeat=optimize(all_keys,constraints,cfg['pgo'])
        if len(graph['poses'])!=len(repeat['poses']):raise ValueError('Frozen PGO pose count differs')
        difference=max(np.max(np.abs(pose(a['T_world_body'])-pose(b['T_world_body']))) for a,b in zip(graph['poses'],repeat['poses']))
        if difference>1e-8 or repeat['components']!=graph['components']:raise ValueError('Frozen PGO reproduction differs')
        repeats[branch]=dict(max_pose_matrix_difference=float(difference),components=repeat['components'])
        for name in ('events.jsonl','exchanges.jsonl'):
            with (loops/name).open('rb') as src,gzip.open(output/branch/(name+'.gz'),'wb') as dst:shutil.copyfileobj(src,dst)
        shutil.copy2(loops/'summary.json',output/branch/'detection-summary.json')
        shutil.copy2(loops/'COMPLETE.json',target/f'loops-{branch}-manifest.json')
        shutil.copy2(Path(paths['pgo'])/'COMPLETE.json',target/f'pgo-{branch}-manifest.json')
    unchanged=all(file_hash(Path(p))==h for p,h in watched.items())
    if not unchanged:raise ValueError('PGO changed frozen odometry or descriptors')
    # Position-GT diagnostics are strictly post-optimization and never alter loops.
    from .evaluation import ground_truth,gt_position
    truth={r:ground_truth(Path(cfg['dataset'])/(r.lower()+'_gt.txt')) for r in robots};quality={}
    write_json(target/'ground-truth-hashes.json',{
        r:dict(path=str(Path(cfg['dataset'])/(r.lower()+'_gt.txt')),
               sha256=file_hash(Path(cfg['dataset'])/(r.lower()+'_gt.txt')))
        for r in robots})
    for branch in solved['branches']:
        checks=[];graph=read_json(output/branch/'graph.json')
        selected={tuple(map(tuple,[f['i'],f['j']])) for f in graph['factors'] if f['kind']=='loop' and f['solver_weight']>0}
        for edge in read_jsonl(output/branch/'constraints.jsonl'):
            i,j=tuple(edge['i']),tuple(edge['j']);a,b=[gt_position(truth[k[0]],key_lookup[k]['stamp_ns'],2*10**9) for k in (i,j)]
            item=dict(i=list(i),j=list(j),gt_checkable=a is not None and b is not None,selected_by_pgo=(i,j) in selected)
            if item['gt_checkable']:
                measured=float(np.linalg.norm(pose(edge['T_i_j'])[:3,3]));actual=float(np.linalg.norm(a-b))
                item.update(measured_separation_m=measured,gt_separation_m=actual,separation_discrepancy_m=abs(measured-actual),flagged=abs(measured-actual)>2.)
            checks.append(item)
        write_jsonl(output/branch/'loop-quality.jsonl',checks)
        quality[branch]=dict(gt_checkable=sum(x['gt_checkable'] for x in checks),flagged=sum(x.get('flagged',False) for x in checks),
            flagged_selected=sum(x.get('flagged',False) and x['selected_by_pgo'] for x in checks),
            criterion='absolute error in endpoint separation above 2 m; position-only diagnostic, not proof of a full 6-DoF outlier')
    regression_log=(work/'regression.log').read_text()
    passed=re.search(r'\b(\d+) passed\b',regression_log)
    if not passed or re.search(r'\b[1-9]\d* (?:failed|errors?)\b',regression_log):
        raise ValueError('Missing successful regression test result')
    write_json(output/'validation.json',dict(pgo_reproduction=repeats,odometry_descriptors_unchanged=unchanged,
        checked_input_files=len(watched),regression_tests_passed=int(passed[1]),loop_quality=quality))
    for name in ('schedule.json','cuda-validation.json','cuda-validation.log','regression.log','export-audit.json','preparation-summary.json'):
        if (work/name).exists():shutil.copy2(work/name,target/name)
    for robot in robots:
        p=prepared[robot]
        expected=report['frozen_preparation'][robot]
        if validate_stage(p,verify_files=False)['stage_hash']!=expected:raise ValueError('Evaluation used a different preparation')
    if retire:
        paths=[work/r/'export' for r in robots]+[work/'stages']+[p/'export' for p in failed_attempts if (p/'export').is_dir()]
        entries=[]
        for path in paths:
            if not path.is_dir() or path.is_symlink() or not path.resolve().is_relative_to(work.resolve()):raise ValueError('Unsafe retirement path')
            entries.append(dict(path=str(path),bytes=sum(f.stat().st_size for f in path.rglob('*') if f.is_file())))
        plan=dict(complete=False,retired=entries,total_bytes=sum(x['bytes'] for x in entries),
                  retained='Complete frozen poses/constraints, evo evidence, source/configuration provenance, loop transcripts, report, figures and Rerun')
        write_json(output/'cleanup.json',plan)
        for path in paths:shutil.rmtree(path)
        plan['complete']=True;write_json(output/'cleanup.json',plan)
        write_json(work/'RETIRED.json',dict(archive=str(output.resolve()),reason=plan['retained']))
    shutil.copy2(Path(__file__),target/'ellipsoid_full_archive.py')
    write_json(output/'files.json',{str(p.relative_to(output)):dict(bytes=p.stat().st_size,sha256=file_hash(p)) for p in sorted(output.rglob('*')) if p.is_file() and p.name!='files.json'})
    print('Frozen PGO reproduced, all front-end inputs unchanged, compact archive verified.',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--retire-large',action='store_true');args=p.parse_args()
    archive(args.work.resolve(),args.output.resolve(),args.retire_large)
