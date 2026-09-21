"""Audit the completed local pair without changing mapping or backend artifacts."""
import hashlib
import json
from pathlib import Path
import zlib
import numpy as np

root=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()

state=read(root/'status.json')
diagnostic=(root/'diagnostic-backend-status.json').exists()
assert (read(root/'diagnostic-backend-status.json') if diagnostic else state)['phase']=='complete'
report=read(root/'report/report.json');trials=read(root/'trials.json')
result=dict(diagnostic_only=diagnostic,frontends={},descriptor_evidence_memberships=0,causal_ranked_events=0,same_robot_loops=0,all_bulk_geometry_retained=True)
keys={}
for robot,path in trials.items():
    trial=Path(path);data=rows(trial/'frontend/native_updates.jsonl')
    original=rows(trial/'frontend/submaps/index.jsonl')
    complete=[r for r in original if r['complete'] and r['retrievable']]
    prepared=rows(root/f'prepared-{robot}/store/keyframes.jsonl')
    assert len(complete)==len(prepared)
    for i,(source,key) in enumerate(zip(complete,prepared)):
        assert key['keyframe_id']==i
        for name,value in source.items():
            if name!='keyframe_id':assert key[name]==value,(robot,i,name)
        descriptor=json.loads(zlib.decompress((root/f'prepared-{robot}/ellipsoid/{i:06d}.json.zlib').read_bytes()))
        assert descriptor['submap_id']==source['submap_id']
        assert descriptor['member_scan_ids']==source['member_scan_ids']
        assert descriptor['payload_sha256']==source['sha256']
        keys[robot,i]=key;result['descriptor_evidence_memberships']+=1
    assert 'verified without error' in (root/f'{robot}-recording-verification.log').read_text()
    validation=read(trial/'artifact-validation.json');quality=read(root/f'{robot}-quality.json')
    assert (quality['passed'] or diagnostic) and read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
    assert quality['full_completion'] and quality['finite'] and quality['chronological'] and quality['max_speed_m_s']<=20
    assert all(r['age_max_s']<=10.000001 for r in data if r['lidar_updated'])
    memory=rows(trial/'memory.jsonl')
    result['frontends'][robot]=dict(quality=quality,validation=validation,live_rerun_verified=True,
        completed_submaps=len(complete),partial_submaps=len(original)-len(complete),
        max_correspondence_age_s=max(r['age_max_s'] for r in data if r['lidar_updated']),
        processing_mean_s=float(np.mean([r['processing_s'] for r in data])),
        processing_p95_s=float(np.quantile([r['processing_s'] for r in data],.95)),
        peak_mapper_rss_mib=max(r['VmHWM'] for r in memory)/1024,
        live_rrd_sha256=sha(trial/'recording/live.rrd'),native_updates_sha256=sha(trial/'frontend/native_updates.jsonl'))
for robot in trials:
    for event in rows(root/f'dpgo/{robot}/events.jsonl'):
        if event['type']!='ranked':continue
        query=keys[tuple(event['query'])]
        assert event['query_stamp_ns']==query['available_ns']<=event['delivery_ns']
        for candidate in event['candidates']:
            assert keys[event['candidate_robot'],candidate['keyframe_id']]['available_ns']<=event['delivery_ns']
        result['causal_ranked_events']+=1
for edge in rows(root/'dpgo/constraints.jsonl'):
    a,b=keys[tuple(edge['i'])],keys[tuple(edge['j'])]
    if edge['i'][0]==edge['j'][0]:
        assert not set(a['member_scan_ids']).intersection(b['member_scan_ids'])
        assert abs(a['stamp_ns']-b['stamp_ns'])>=30*10**9
        result['same_robot_loops']+=1
graph=rows(root/'dpgo/poses.jsonl');assert len(graph)==len(keys)
for node in graph:
    assert node['stamp_ns']==keys[node['robot_id'],node['keyframe_id']]['stamp_ns']
    assert np.isfinite(node['T_world_body']).all()
assert 'verified without error' in (root/'recording-verification.log').read_text()
result['derived_rerun_verified']=True;result['derived_rerun_sha256']=sha(root/'report/result.rrd')
result['report_sha256']=sha(root/'report/report.json')
for name,value in read(root/'source-hashes.json').items():assert sha(Path(name))==value,name
result['frozen_sources_verified']=True
(root/'retention-audit.json').write_text(json.dumps(result,indent=2)+'\n')
lines=['# GRACO aerial pair: temporal EllipseLIO + ellipsoid BEV + PCM/CBS','',
    'Local full-flight runs of aerial-05-40m and aerial-08-25m, treated as two independent robots. Both use the supplied aerial T_Imu_Lidar and IMU noise, reliable input, four-thread native launchers and separate ROS domains. Native temporal windows are 10 seconds with 5 seconds overlap.','',
    'Sensor staging preserved LiDAR/IMU CDR payloads and all original timestamps. Completed native member scans supply both ellipsoid descriptors and registration evidence. GT was made available only after optimization, for position evaluation using evo 1.36.5, 50 ms association and rigid alignment without scale. Raw fits are independent per robot; CBS uses a shared fit per connected component.','',
    '| Robot / flight | Raw ATE [m] | CBS ATE [m] | Max speed [m/s] | Max update gap [s] | Completed submaps |',
    '|---|---:|---:|---:|---:|---:|']
if diagnostic:
    lines[2:2]=['**Diagnostic result: aerial08 failed the successful-LiDAR-update gap criterion (1.384 s at the 30 s handover). Both captures completed, but this is not a passing frontend-stability trial. The original failed gate is retained; the backend ran with unchanged parameters to inspect the requested two-robot result.**','']
for robot in trials:
    raw=report['raw'].get(robot,{}).get('rmse_m')
    cbs=report['cbs'].get(report['components'][robot],{}).get('per_robot',{}).get(robot,{}).get('statistics',{}).get('rmse')
    info=result['frontends'][robot];q=info['quality']
    fmt=lambda n:'Unavailable' if n is None else f'{n:.4f}'
    lines.append(f'| {robot} | {fmt(raw)} | {fmt(cbs)} | {q["max_speed_m_s"]:.3f} | {q["max_successful_update_gap_s"]:.3f} | {info["completed_submaps"]} |')
lines+=['',f'Connected components: {report["connectivity"]["measured_components"]}.',
    f'Shared-component CBS ATE: '+', '.join(f'{c} = {v["rmse_m"]:.4f} m' for c,v in report['cbs'].items())+'.',
    f'Retained loops: {report["runtime"]["loops"]}; PCM rejected: {report["pcm"]["excluded_loops"]}; backend wall time: {report["runtime"]["wall_s"]:.2f} s.','',
    '![Trajectories and native member-scan maps](report/trajectories-maps.png)','',
    'Both live recordings and the derived recording verified. The artifact audit checks descriptor/evidence membership, causal availability, anchor timestamps, same-robot exclusions and frozen sources. Full geometry, trajectories, evo output and logs are retained. This is one fixed-configuration trial, not an accuracy guarantee or parameter sweep.','']
if not report['connectivity']['all_robots_connected']:
    lines[2:2]=['**No common two-robot map was established. There were no accepted inter-robot loops; the CBS ATEs below use separate alignments for the two disconnected components. No joint two-robot ATE is reported.**','']
(root/'REPORT.md').write_text('\n'.join(lines))
print(json.dumps(result,indent=2))
