#!/usr/bin/env python3
"""Read-only evidence audit for a finished native-submap five-group batch."""
import argparse
import hashlib
import json
from pathlib import Path
import zlib
import numpy as np


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(block)
    return dict(bytes=path.stat().st_size, sha256=h.hexdigest())


def frontend(trial, verification_log):
    data = rows(trial / 'frontend/native_updates.jsonl')
    pose = np.asarray([r['pose'] for r in data])
    stamp = np.asarray([r['stamp_ns'] for r in data], dtype=np.int64)
    handovers = np.flatnonzero(np.diff([r['active_submap_id'] for r in data])) + 1
    steps = np.linalg.norm(np.diff(pose[:, :3], axis=0), axis=1)
    speed = steps / (np.diff(stamp) * 1e-9)
    memory = rows(trial / 'memory.jsonl')
    submaps = rows(trial / 'frontend/submaps/index.jsonl')
    completed = [r for r in submaps if r['complete']]
    validation = read(trial / 'artifact-validation.json')
    coverage = read(trial / 'sensor-coverage.json')
    assert coverage['full_selected_sensor_tail_reached']
    assert 'verified without error' in verification_log.read_text()
    files = ['recording/live.rrd', 'frontend/native_updates.jsonl',
             'frontend/submaps/index.jsonl', 'frontend/submaps/manifest.json',
             'frontend/runtime.yaml', 'source-hashes.json']
    return dict(
        validation=validation, coverage=coverage, recording_verified=True,
        wall_s=read(trial / 'frontend/summary.json')['wall_s'],
        processing_mean_s=float(np.mean([r['processing_s'] for r in data])),
        processing_max_s=max(r['processing_s'] for r in data),
        processing_p95_s=float(np.quantile([r['processing_s'] for r in data], .95)),
        peak_mapper_rss_kib=max((r.get('VmHWM', r.get('VmRSS', 0)) for r in memory), default=0),
        max_correspondence_age_s=max((r['age_max_s'] for r in data if r['lidar_updated']), default=None),
        completed_submaps=len(completed), retrievable_submaps=sum(r['retrievable'] for r in completed),
        partial_submaps=len(submaps)-len(completed),
        completed_member_span_s=dict(zip(('min', 'median', 'max'), map(float, np.quantile(
            [(r['last_member_ns']-r['begin_ns'])*1e-9 for r in completed], [0, .5, 1])))) if completed else None,
        handovers=[dict(scan_id=int(data[i]['scan_id']), stamp_ns=int(stamp[i]),
                        step_m=float(steps[i-1]), speed_m_s=float(speed[i-1])) for i in handovers],
        files={str(trial / name): digest(trial / name) for name in files})


def backend(folder, trials, verification_log=None):
    keys = {}
    membership_count = 0
    for robot, trial in trials.items():
        source = Path(trial) / 'frontend/submaps'
        original = [r for r in rows(source / 'index.jsonl') if r['complete'] and r['retrievable']]
        prepared = rows(folder / f'prepared-{robot}/store/keyframes.jsonl')
        assert len(original) == len(prepared)
        for index, (before, after) in enumerate(zip(original, prepared)):
            assert after['keyframe_id'] == index
            for key, value in before.items():
                assert after[key] == value, (robot, index, key)
            packed = json.loads(zlib.decompress((folder / f'prepared-{robot}/ellipsoid/{index:06d}.json.zlib').read_bytes()))
            assert packed['submap_id'] == before['submap_id']
            assert packed['member_scan_ids'] == before['member_scan_ids']
            assert packed['payload_sha256'] == before['sha256']
            assert before['available_ns'] >= before['end_ns']
            keys[robot, index] = after
            membership_count += 1
    ranked = 0
    for robot in trials:
        for event in rows(folder / f'dpgo/{robot}/events.jsonl'):
            if event['type'] != 'ranked':
                continue
            query = keys[tuple(event['query'])]
            assert event['query_stamp_ns'] == query['available_ns']
            assert event['delivery_ns'] >= query['available_ns']
            for candidate in event['candidates']:
                key = keys[event['candidate_robot'], candidate['keyframe_id']]
                assert key['available_ns'] <= event['delivery_ns']
            ranked += 1
    loops = rows(folder / 'dpgo/constraints.jsonl')
    same_robot = 0
    for edge in loops:
        a, b = keys[tuple(edge['i'])], keys[tuple(edge['j'])]
        if edge['i'][0] == edge['j'][0]:
            assert not set(a['member_scan_ids']).intersection(b['member_scan_ids'])
            assert abs(a['stamp_ns'] - b['stamp_ns']) >= 30 * 10**9
            same_robot += 1
    graph = rows(folder / 'dpgo/poses.jsonl')
    assert len(graph) == len(keys)
    for row in graph:
        key = keys[row['robot_id'], row['keyframe_id']]
        assert row['stamp_ns'] == key['stamp_ns']
        assert np.isfinite(row['T_world_body']).all()
    verification_log = verification_log or folder / 'recording-verification.log'
    assert 'verified without error' in verification_log.read_text()
    return dict(descriptor_evidence_memberships_verified=membership_count,
                causal_ranked_events_verified=ranked, same_robot_loops_verified=same_robot,
                graph_anchor_timestamps_verified=len(graph), recording_verified=True,
                files={str(folder / name): digest(folder / name) for name in
                       ['report/result.rrd', 'dpgo/poses.jsonl', 'dpgo/constraints.jsonl',
                        'report/report.json', 'config.yaml']})


def audit(root):
    state = read(root / 'status.json')
    assert state.get('finished_utc'), 'Batch still active'
    groups = {r['name']: r for r in read(root / 'plan.json')}
    result = dict(schema_version=1, batch=str(root), groups={}, all_bulk_geometry_retained=True)
    for status in state['groups']:
        name = status['name']; group = groups[name]; folder = Path(group['folder'])
        current = dict(status=status['status'], frontends={})
        result['groups'][name] = current
        for robot, item in group['trials'].items():
            outcome = status['frontends'][robot]
            if outcome['status'] != 'complete':
                current['frontends'][robot] = dict(status=outcome['status'], error=outcome.get('error'))
                continue
            trial = Path(item['path'])
            log = trial / 'recording-verification.log' if item['reused'] else folder / f'{robot}-recording-verification.log'
            current['frontends'][robot] = dict(status='complete', reused=item['reused'], quality=outcome['quality'], **frontend(trial, log))
        if status['status'] == 'complete':
            current['backend'] = backend(folder, {r: i['path'] for r, i in group['trials'].items()})
        else:
            current['failure'] = read(folder / 'failure.json')
    (root / 'retention-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('root', type=Path)
    args = parser.parse_args(); audit(args.root)
