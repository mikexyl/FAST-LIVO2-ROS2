"""Validate online causality and completion using retained arrival journals."""
import argparse
from collections import Counter
import json
from pathlib import Path
import zlib
import numpy as np
from .artifacts import digest, file_hash, read_json, read_jsonl
from .online_io import atomic_json


def statistics(values):
    values=np.asarray(values,dtype=float)
    return dict(count=len(values),median=float(np.median(values)),p95=float(np.quantile(values,.95)),
                maximum=float(np.max(values))) if len(values) else dict(count=0)


def audit(work):
    status=read_json(work/'status.json'); assert status['phase']=='complete'
    start=read_json(work/'START')['wall_ns']; robots=status['robots']; rows={}; native={}
    for path,expected in read_json(work/'source-hashes.json').items(): assert file_hash(path)==expected,path
    for path,expected in read_json(work/'binary-hashes.json').items(): assert file_hash(path)==expected,path
    frontend={}; preparation=[]; verification=[]; categories=Counter(); arrivals={r:[] for r in robots}
    raw_poses={}; started=[]; finished=[]; counts={}; rejected=Counter()
    for robot in robots:
        trial=work/f'frontends/{robot}'; summary=read_json(trial/'frontend/summary.json')
        assert summary['success'] and summary['requested_duration']==0 and summary['finite'],robot
        started.append(summary['playback_started_wall_ns']); finished.append(summary['playback_started_wall_ns']+round(summary['wall_s']*1e9))
        updates=read_jsonl(trial/'frontend/native_updates.jsonl'); raw_poses[robot]=updates
        stamps=np.array([x['stamp_ns'] for x in updates],dtype=np.int64)
        sensors=np.array([x['sensor_stamp_ns'] for x in updates],dtype=np.int64)
        positions=np.array([x['pose'] for x in updates]); good=sensors[[x['lidar_updated'] for x in updates]]
        assert np.isfinite(positions).all() and np.all(np.diff(stamps)>0) and np.all(np.diff(sensors)>0)
        gap=float(np.max(np.diff(np.r_[sensors[0],good,sensors[-1]]))/1e9)
        speed=float(np.max(np.linalg.norm(np.diff(positions[:,:3],axis=0),axis=1)/(np.diff(stamps)/1e9)))
        assert gap<1 and speed<=20,(robot,gap,speed)
        memory=read_jsonl(trial/'memory.jsonl')
        frontend[robot]=dict(native_poses=len(updates),max_update_gap_s=gap,max_speed_m_s=speed,
            processing_ms=statistics([x['processing_s']*1000 for x in updates]),
            peak_mapper_rss_mib=max(x.get('VmHWM',x.get('VmRSS',0)) for x in memory)/1024,
            failed_updates_after_init=sum(not x['lidar_updated'] for x in updates[1:]))
        prepared=work/f'prepared-{robot}'; rows[robot]=read_jsonl(prepared/'store/keyframes.jsonl')
        native[robot]={x['submap_id']:x for x in read_jsonl(trial/'frontend/area_maps/index.jsonl') if x['complete'] and x['retrievable']}
        assert len(rows[robot])==len(native[robot])
        assert read_json(prepared/'summary.json')['complete']
        counts[robot]=len(rows[robot])
        for row in rows[robot]:
            original=native[robot][row['submap_id']]
            assert all(row[k]==v for k,v in original.items() if k not in ('keyframe_id','available_ns'))
            assert row['native_available_ns']==original['available_ns']
            assert start<=row['snapshot_received_wall_ns']<=row['available_ns']
            descriptor=json.loads(zlib.decompress((prepared/f"ellipsoid/{row['keyframe_id']:06d}.json.zlib").read_bytes()))
            assert descriptor['member_scan_ids']==row['member_scan_ids'] and descriptor['payload_sha256']==row['sha256']
            assert file_hash(trial/'frontend/area_maps'/row['payload'])==row['sha256']
            preparation.append((row['available_ns']-row['snapshot_received_wall_ns'])/1e9)
        for event in read_jsonl(work/f'live/{robot}/events.jsonl'):
            if event['type']=='observe': assert rows[robot][event['keyframe_id']]['available_ns']<=event['logged_wall_ns']
            elif event['type']=='verification':
                assert event['query_stamp_ns']<=event['verification_submitted_wall_ns']<=event['delivery_ns']<=event['logged_wall_ns']
                verification.append(event['wall_detection_latency_s']); rejected[event['reason']]+=1
    # Candidate eligibility is checked against query publication, not later receipt.
    for robot in robots:
        for event in read_jsonl(work/f'live/{robot}/events.jsonl'):
            if event['type']=='ranked':
                assert event['query_stamp_ns']==rows[robot][event['query'][1]]['available_ns']
                for candidate in event['candidates']:
                    assert rows[event['candidate_robot']][candidate['keyframe_id']]['available_ns']<=event['query_stamp_ns']
    publications=[]
    for path in sorted((work/'epochs').glob('*/publication.json')):
        publication=read_json(path); epoch=path.parent; inp=read_json(epoch/'input.json'); cutoff=inp['cutoff_wall_ns']
        assert cutoff<publication['published_wall_ns']
        for robot in robots:
            subset=read_jsonl(epoch/robot/'store/keyframes.jsonl')
            assert subset==rows[robot][:len(subset)] and all(x['available_ns']<=cutoff for x in subset)
        proposals=read_jsonl(epoch/'loops/constraints.jsonl')
        assert all(x['diagnostics']['query_stamp_ns']<=cutoff for x in proposals)
        publications.append(dict(revision=publication['revision'],cutoff_s=(cutoff-start)/1e9,
            published_s=(publication['published_wall_ns']-start)/1e9,
            backend_s=(publication['published_wall_ns']-cutoff)/1e9,loops=publication['loops']))
    displayed=[]; display_latency=[]
    for event in read_jsonl(work/'demo/events.jsonl'):
        if event['type']=='map_displayed':
            assert event['descriptor_available_ns']<=event['logged_wall_ns']
            display_latency.append((event['logged_wall_ns']-event['descriptor_available_ns'])/1e9)
        elif event['type']=='poses_received': arrivals[event['robot']].extend(event['poses'])
        elif event['type']=='revision_displayed':
            assert event['published_wall_ns']<=event['logged_wall_ns']
            displayed.append(dict(revision=event['revision'],elapsed_s=(event['logged_wall_ns']-start)/1e9,
                component_count=len(set(event['components'].values())),components=event['components']))
    for robot in robots:
        native_track=np.loadtxt(work/f'frontends/{robot}/recording/post_lidar_poses.tum')
        assert len(arrivals[robot])==len(native_track)
        np.testing.assert_allclose(np.array([x[1:] for x in arrivals[robot]]),native_track[:,1:],atol=0,rtol=0)
    final=Path(read_json(work/'epochs/latest.json')['path'])
    loops=read_jsonl(final/'constraints.jsonl')
    for edge in loops:
        a,b=(edge[x][0].startswith('aerial') for x in ('i','j'))
        categories['aerial-ground' if a!=b else 'aerial-aerial' if a else 'ground-ground']+=1
    demo=read_json(work/'demo/summary.json')
    assert demo['revisions']==len(publications)==len(displayed)>0
    # Strong online proof: at least one correction precedes the first bag ending.
    first_sensor_end=min(read_json(work/'inputs.json')[r]['sensor_duration_s'] for r in robots)
    assert displayed[0]['elapsed_s']<first_sensor_end
    frames=read_jsonl(work/'demo/frames.jsonl')
    assert np.all(np.diff([x['frame'] for x in frames])>0)
    result=dict(verified=True,ground_truth_used=False,robots=robots,frontend=frontend,submaps=counts,
        start_spread_s=(max(started)-min(started))/1e9,first_correction_s=displayed[0]['elapsed_s'],
        revisions=publications,displayed_revisions=displayed,preparation_latency_s=statistics(preparation),
        verification_latency_s=statistics(verification),display_map_latency_s=statistics(display_latency),
        verification_reasons=dict(rejected),loop_categories=dict(categories),retained_loops=len(loops),
        pcm_rejected=read_json(final/'pcm.json')['excluded_loops'],
        registration_factors=read_json(final/'registration.json')['registration_factor_count'],
        online_corrections_before_first_bag_ended=sum(x['elapsed_s']<first_sensor_end for x in displayed),
        video_frames=demo['frames'],video_fps=demo['fps'],components=demo['components'])
    atomic_json(work/'online-audit.json',result); return result


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('work',type=Path); a=p.parse_args()
    print(json.dumps(audit(a.work),indent=2))
