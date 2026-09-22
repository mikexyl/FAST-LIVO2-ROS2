"""Audit selected full captures and recovered backends; preserve all original failures."""
import copy
import json
from pathlib import Path
import sys
import time
import zlib
import numpy as np
import yaml
ROOT=Path('/workspace');BASE=ROOT/'.ros2/upstream-area-s3e-20260921'
sys.path[:0]=[str(BASE),str(ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'),str(ROOT/'FAST-LIVO2-ROS2/research')]
from final_audit import read,rows,sha,optional,performance_plot
from upstream_area_batch import quality,sources,save
from resume_backends_gpu import frozen
from s3e_pipeline.area_maps import shared_ids


def capture(trial):
    native=rows(trial/'frontend/native_updates.jsonl');assert len(native)>1
    assert all(r['odometry_map_source']=='persistent_map' and r['handovers']==0 for r in native)
    assert all(r['age_max_s'] is None and not r['correspondence_age_measured'] for r in native)
    runtime=yaml.safe_load((trial/'frontend/runtime.yaml').read_text())['/**']['ros__parameters']
    assert not runtime['mapping']['submaps']['enabled']
    assert not runtime['mapping']['area_maps']['odometry']
    processing=np.array([r['processing_s'] for r in native])*1000
    area=rows(trial/'frontend/area_maps/index.jsonl');complete=[r for r in area if r['complete'] and r['retrievable']]
    mem=[r['VmHWM'] for r in rows(trial/'memory.jsonl') if 'VmHWM' in r]
    return dict(path=str(trial),quality=quality(trial),summary=read(trial/'frontend/summary.json'),
        native_poses=len(native),failed_updates_after_init=sum(not r['lidar_updated'] for r in native[1:]),
        processing_mean_ms=float(processing.mean()),processing_p95_ms=float(np.quantile(processing,.95)),processing_max_ms=float(processing.max()),
        snapshot_max_ms=max(r['snapshot_s'] for r in native)*1000,peak_mapper_rss_mib=max(mem)/1024 if mem else None,
        final_map_points=native[-1]['active_points'],odometry_map_source='persistent_map',handovers=0,correspondence_ages_measured=False,
        sensor_coverage=read(trial/'sensor-coverage.json'),artifact_validation=read(trial/'artifact-validation.json'),
        area_snapshots=len(area),retrievable_snapshots=len(complete),
        max_saved_geometry_history_s=max((r['anchor_sensor_ns']-r['begin_ns'])/1e9 for r in area),
        selected_area_points_min=min(r['geometry_count'] for r in area),selected_area_points_max=max(r['geometry_count'] for r in area),
        config_sha256=sha(trial/'frontend/runtime.yaml'),native_updates_sha256=sha(trial/'frontend/native_updates.jsonl'),
        area_index_sha256=sha(trial/'frontend/area_maps/index.jsonl'))


def recording(trial,initial):
    source=trial/'recording/live.rrd';partition=optional(trial/'recording/entity-parts/result.json')
    result=dict(path=str(source),bytes=source.stat().st_size,sha256=sha(source))
    if partition is not None:
        assert partition['source_sha256']==result['sha256']
        assert partition['source_data_verified'] and partition['partitioned_footer_and_data_verified']
        assert partition.get('complete_chunk_comparison_passed') or partition.get('normalized_complete_data_comparison_passed')
        for p in partition['parts']:assert sha(p['path'])==p['sha256']
        result.update(verified=True,mode='entity-partition',partition=partition)
    else:
        if '--allow-pending-recordings' in sys.argv and not initial.get('live_rerun_verified'):
            return dict(result,verified=False,reason='Waiting for completed whole-recording comparison',mode='pending')
        assert initial.get('live_rerun_verified'),f'Missing recording verification: {trial}'
        result.update(verified=True,mode='single-file')
    return result


def audit():
    assert sources()==frozen()
    capacity=read(BASE/'capacity-run/finished.json')
    assert capacity['library_sha256']==sha(BASE/'capacity-run/install/ellipselio/lib/libellipselio_mapping.so')
    initial=read(BASE/'full/batch-summary.json')
    output=dict(created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        upstream_commit='6506f46f1947b4ef86cfba402f11f10a6ef520ee',
        resources=initial['resources'],initial_batch_started_utc=initial['started_utc'],initial_batch_finished_utc=initial['finished_utc'],
        processing_timing_scope=initial['processing_timing_scope'],
        estimator_parity=read(BASE/'upstream-estimator-parity.json'),octree_capacity_parity=read(BASE/'capacity-source-parity.json'),
        cuda_capacity_test=read(BASE/'capacity-cuda/test-result.json'),capacity_replay=capacity,frozen_sources_verified=True,groups={})
    for name,old in initial['groups'].items():
        folder=BASE/'recovered-cuda'/name
        if 'Laboratory' in name:
            result=copy.deepcopy(old);result.update(status='frontend_failed',stage='odometry',backend_not_run=True,
                reason='Bob diverged: maximum 602.11 m/s and 28.90 s successful-update gap; no parameter retry')
            for robot,v in result['frontends'].items():v['recording']=recording(Path(v['live_rerun']['path']).parent.parent,v)
            output['groups'][name]=result;continue
        state=read(folder/'status.json');assert state['status']=='complete',state
        trials=read(folder/'trials.json');cfg=yaml.safe_load((folder/'config.yaml').read_text())
        assert read(folder/'source-hashes.json')==frozen()
        result=dict(status='complete',state=state,frontends={},report=read(folder/'report/report.json'),
            initial_failure=dict(status=old['status'],error=old.get('error')),
            provenance=read(folder/'capture-provenance.json'),audit=dict(descriptor_evidence_memberships=0,causal_ranked_events=0,same_robot_loop_checks=0))
        keys={}
        for robot,path in trials.items():
            trial=Path(path);stats=capture(trial)
            assert stats['quality']['passed'] and stats['sensor_coverage']['full_selected_sensor_tail_reached']
            assert stats['artifact_validation']['accumulated_area_membership_verified']
            stats['recording']=recording(trial,old['frontends'][robot])
            area=[r for r in rows(trial/'frontend/area_maps/index.jsonl') if r['complete'] and r['retrievable']]
            prepared=rows(folder/f'prepared-{robot}/store/keyframes.jsonl');assert len(prepared)==len(area)
            for i,(original,key) in enumerate(zip(area,prepared)):
                assert key['keyframe_id']==i
                assert all(key[n]==v for n,v in original.items() if n!='keyframe_id')
                packet=json.loads(zlib.decompress((folder/f'prepared-{robot}/ellipsoid/{i:06d}.json.zlib').read_bytes()))
                assert packet['submap_id']==original['submap_id'] and packet['member_scan_ids']==original['member_scan_ids']
                assert packet['payload_sha256']==original['sha256'] and key['submap_points']==original['geometry_count']
                assert packet['representation']=='ellipsoid';keys[robot,i]=key
                result['audit']['descriptor_evidence_memberships']+=1
            stats['descriptor_preparation']=read(folder/f'prepared-{robot}/summary.json')
            result['frontends'][robot]=stats
        for robot in cfg['robots']:
            for event in rows(folder/f'dpgo/{robot}/events.jsonl'):
                if event['type']!='ranked':continue
                assert event['query_stamp_ns']==keys[tuple(event['query'])]['available_ns']<=event['delivery_ns']
                assert all(keys[event['candidate_robot'],c['keyframe_id']]['available_ns']<=event['delivery_ns'] for c in event['candidates'])
                result['audit']['causal_ranked_events']+=1
        for edge in rows(folder/'dpgo/constraints.jsonl'):
            a,b=keys[tuple(edge['i'])],keys[tuple(edge['j'])]
            if edge['i'][0]==edge['j'][0]:
                assert abs(a['stamp_ns']-b['stamp_ns'])>=30*10**9
                assert not shared_ids(a['geometry_id_ranges'],b['geometry_id_ranges'])
                result['audit']['same_robot_loop_checks']+=1
        graph=rows(folder/'dpgo/poses.jsonl');assert len(graph)==len(keys)
        for node in graph:
            assert node['stamp_ns']==keys[node['robot_id'],node['keyframe_id']]['stamp_ns']
            assert np.isfinite(node['T_world_body']).all()
        assert 'verified without error' in (folder/'recording-verification.log').read_text()
        result['derived_rerun']=dict(verified=True,path=str(folder/'report/result.rrd'),sha256=sha(folder/'report/result.rrd'))
        gallery=read(folder/'bev-gallery/manifest.json');assert gallery['complete'] and gallery['exact_cached_features']
        result['gallery']=dict(images=len(gallery['maps']),exact_cached_features=True,all_snapshots_rendered=False,manifest_sha256=sha(folder/'bev-gallery/manifest.json'))
        performance_plot(folder/'diagnostics',dict(name=name,frontends={r:dict(path=p) for r,p in trials.items()}))
        output['groups'][name]=result
        print(name,'audited',result['audit'],flush=True)
    assert sources()==frozen()
    output['all_bulk_geometry_retained']=True
    pending=[(n,r) for n,g in output['groups'].items() for r,f in g['frontends'].items() if not f['recording']['verified']]
    output['pending_recordings']=pending
    save(BASE/('partial-audit.json' if pending else 'final-summary.json'),output)
    print('ARTIFACT AUDIT PASSED; RECORDINGS PENDING '+str(pending) if pending else 'FINAL AUDIT PASSED',flush=True)

if __name__=='__main__':audit()
