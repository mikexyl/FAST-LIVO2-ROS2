#!/usr/bin/env python3
"""Evaluate native Swarm-SLAM outputs with the project's unchanged evo protocol."""
import argparse
from pathlib import Path
import shutil
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,write_jsonl,file_hash
from s3e_pipeline.evaluation import corrected_dense_rows,ground_truth,gt_position,save_tum
from s3e_pipeline.evo_evaluation import evaluate as evo

ROBOTS=('Alpha','Bob','Carol')

def pose_dict(msg):
    T=np.eye(4);q=msg['orientation'];p=msg['position']
    T[:3,:3]=Rotation.from_quat([q[k] for k in ('x','y','z','w')]).as_matrix()
    T[:3,3]=[p[k] for k in ('x','y','z')]
    if not np.isfinite(T).all():raise ValueError('Nonfinite native pose')
    return T.tolist()

def audit_keyframes(odometry,keys,distance):
    """Prove selected inputs match, including timestamps and absolute odometry."""
    result={}
    for robot,rows in odometry.items():
        expected=[];last=None
        for row in rows:
            T=np.asarray(row['T_world_body']).reshape(4,4);xyz=T[:3,3]
            if last is None or np.sum((xyz-last)**2)>distance**2:
                expected.append(row);last=xyz.copy()
        received=keys[robot]
        differences=[dict(keyframe_id=i,expected_stamp_ns=e['stamp_ns'],received_stamp_ns=k['stamp_ns'])
                     for i,(e,k) in enumerate(zip(expected,received)) if e['stamp_ns']!=k['stamp_ns']]
        exact=(len(expected)==len(received) and all(
            k['keyframe_id']==i and k['stamp_ns']==e['stamp_ns'] and
            np.allclose(np.asarray(k['T_world_body']).reshape(4,4),np.asarray(e['T_world_body']).reshape(4,4),rtol=0,atol=1e-12)
            for i,(e,k) in enumerate(zip(expected,received))))
        result[robot]=dict(expected=len(expected),received=len(received),exact_selected_timestamps_and_poses=bool(exact),
                          differing_timestamps=len(differences),first_timestamp_differences=differences[:10])
    return result

def evaluate(work,output):
    native=work/'swarm';summary=read_json(native/'summary.json')
    cfg=yaml.safe_load((work/'frontend/config.yaml').read_text())
    output.mkdir(parents=True,exist_ok=True)
    target=output/'swarm';target.mkdir(exist_ok=True)
    truth={r:ground_truth(Path(cfg['dataset'])/f'{r.lower()}_gt.txt') for r in ROBOTS}
    # Native Swarm evaluation must not depend on successful BEV preparation.
    odometry={r:read_jsonl(work/f'frontend/{r}/export/frames.jsonl') for r in ROBOTS}
    if (output/'ours/report.json').exists():
        ours=read_json(output/'ours/report.json')
    else:
        failure=read_json(work/'comparison-failure.json')
        ours=dict(failure=failure,branches={b:dict(trajectory={},components={},available=False)
                  for b in ('raw','ellipsoid')},keyframes={},raw_odometry=evo(
                  dict(poses=[dict(row,component=r) for r,rows in odometry.items() for row in rows]),
                  truth,cfg['evaluation'],output/'raw-odometry-evo'))
    for source in native.iterdir():
        if source.is_file():shutil.copy2(source,target/source.name)
    report=dict(dataset=cfg['dataset'],frontend='Shared fresh EllipseLIO exports',
        integration_fixture=bool(read_json(native/'inputs.json').get('smoke_duplicate_alpha')),
        swarm_run=summary,ours=ours,swarm=dict(trajectory={},components={},accepted=0,accepted_inter=0),
        evaluation='evo 1.36.5, one SE(3) alignment per component, no scale fit or additional per-robot fit.',
        caveats=['Swarm-SLAM uses native 0.5 m distance keyframes, individual scans, Scan Context, FPFH/TEASER++/ICP and elected-robot GNC optimization.',
                 'Our paths use fixed 1 m/10 degree/2 s keyframes, submaps, MapClosures, small_gicp and centralized GNC-TLS.',
                 'Concurrent sequence jobs share CPU resources. Swarm wall time includes paced playback, backpressure, native timer budgets and final settling; our detection time excludes odometry/preparation.',
                 'Swarm communication measures observed CDR coordination payloads including local deliveries. Our figures use compressed simulated envelopes; the byte totals have different wire semantics.',
                 'Swarm includes the documented LiDAR transform-direction fix and GTSAM 4.3 compatibility changes.',
                 'GT orientations are unused and the antenna lever arm is uncorrected.'])
    keys={r:read_jsonl(native/f'{r}-keyframes.jsonl') for r in ROBOTS}
    native_cfg=yaml.safe_load((native/'config.yaml').read_text())['/**']['ros__parameters']
    input_audit=audit_keyframes(odometry,keys,native_cfg['frontend']['keyframe_generation_ratio_distance'])
    results=read_json(native/'optimized.json')
    selected_complete=all(v['exact_selected_timestamps_and_poses'] for v in input_audit.values())
    descriptors_complete=all(summary['received_descriptors'][i]==len(keys[r]) for i,r in enumerate(ROBOTS))
    optimized_complete=all(str(i) in results and len(results[str(i)]['estimates'])==len(keys[r]) for i,r in enumerate(ROBOTS))
    report['swarm'].update(keyframe_input_audit=input_audit,
        selected_inputs_complete=selected_complete and descriptors_complete,
        optimized_coverage_complete=optimized_complete,
        usable_keyframe_result=selected_complete and descriptors_complete and optimized_complete)
    write_json(target/'keyframe-input-audit.json',input_audit)
    all_keys={(r,k['keyframe_id']):k for r,rows in keys.items() for k in rows}
    loops=read_jsonl(native/'loops.jsonl');checks=[]
    for index,e in enumerate(loops):
        if not e['success']:continue
        if e['robot_id'] is None:
            i=(ROBOTS[e['robot0_id']],e['robot0_keyframe_id']);j=(ROBOTS[e['robot1_id']],e['robot1_keyframe_id'])
        else:
            i=(ROBOTS[e['robot_id']],e['keyframe0_id']);j=(ROBOTS[e['robot_id']],e['keyframe1_id'])
        report['swarm']['accepted']+=1;report['swarm']['accepted_inter']+=int(i[0]!=j[0])
        a,b=[gt_position(truth[k[0]],all_keys[k]['stamp_ns'],2*10**9) if k in all_keys else None for k in (i,j)]
        item=dict(message_index=index,i=list(i),j=list(j),gt_checkable=a is not None and b is not None)
        if item['gt_checkable']:
            v=e['transform']['translation'];measured=float(np.linalg.norm([v[k] for k in ('x','y','z')]))
            actual=float(np.linalg.norm(a-b));discrepancy=abs(measured-actual)
            item.update(measured_separation_m=measured,gt_separation_m=actual,
                        separation_discrepancy_m=discrepancy,flagged=discrepancy>2.)
        elif i not in all_keys or j not in all_keys:
            item['unavailable_reason']='Missing observed endpoint in incomplete native run'
        else:
            item['unavailable_reason']='No timestamp-associated position GT for both endpoints'
        checks.append(item)
    write_jsonl(target/'loop-quality.jsonl',checks)
    report['swarm'].update(verification_attempts=len(loops),gt_checkable=sum(c['gt_checkable'] for c in checks),
        flagged=sum(c.get('flagged',False) for c in checks),
        loop_quality='GT endpoint-separation discrepancy above 2 m; not a full 6-DoF outlier label. Accepted messages are counted before native GNC; GNC weights are not exported.')
    if report['swarm']['usable_keyframe_result']:
        dense=[];optimized=[]
        for index,robot in enumerate(ROBOTS):
            packet=results[str(index)];component=ROBOTS[packet['origin_robot_id']]
            expected={r['keyframe_id'] for r in keys[robot]};actual=set();rows=[]
            for e in packet['estimates']:
                if e['key']['robot_id']!=index:raise ValueError('Foreign robot in native private estimates')
                key=e['key']['keyframe_id']
                if key in actual:raise ValueError('Duplicate optimized keyframe')
                actual.add(key);rows.append(dict(robot_id=robot,keyframe_id=key,T_world_body=pose_dict(e['pose']),component=component))
            if actual!=expected:raise ValueError('Incomplete native optimized trajectory')
            rows.sort(key=lambda r:r['keyframe_id']);optimized+=rows
            corrected=[dict(r,component=component) for r in corrected_dense_rows(odometry[robot],keys[robot],rows)]
            dense+=corrected;save_tum(target/f'{robot}-corrected.tum',corrected)
            report['swarm']['components'][robot]=component
        write_jsonl(target/'poses.jsonl',dense);write_jsonl(target/'optimized-keyframes.jsonl',optimized)
        report['swarm']['trajectory']=evo(dict(poses=dense),truth,cfg['evaluation'],target/'evo')
        report['swarm']['native_optimizer_error_count']=summary.get('native_optimizer_error_count',0)
    report['swarm']['observed_cdr_bytes']=sum(read_json(native/'communication.json')['serialized_cdr_bytes'].values())
    report['source_sha256']=file_hash(Path(__file__))
    write_json(output/'report.json',report)
    print({k:report['swarm'][k] for k in ('accepted','accepted_inter','components','trajectory')},flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();evaluate(args.work.resolve(),args.output.resolve())
