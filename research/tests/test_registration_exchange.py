"""Exercise the actual peer geometry protocol without ROS or remote store reads."""
import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import read_json
from s3e_pipeline.registration_exchange import RegistrationExchange
from test_pipeline import scene


def network(tmp_path, robots=None):
    robots = robots or ['Alpha', 'Bob', 'Carol']; workers = {}; queue = []
    pairs = [(('Alpha', 0), ('Bob', 0)), (('Bob', 0), ('Carol', 0)),
             (('Alpha', 1), ('Carol', 1))]
    verdicts = [dict(robot_from=robots.index(i[0]), key_from=i[1],
        robot_to=robots.index(j[0]), key_to=j[1], retained=k < 2) for k, (i,j) in enumerate(pairs)]
    for v in verdicts:
        if v['robot_from'] > v['robot_to']:
            v['robot_from'], v['robot_to'] = v['robot_to'], v['robot_from']
            v['key_from'], v['key_to'] = v['key_to'], v['key_from']
    for robot in robots:
        store = tmp_path/robot; store.mkdir()
        rows = [dict(robot_id=robot, keyframe_id=k, stamp_ns=10+k,
            cloud_frame=robot+'/imu', body_frame=robot+'/imu', submap_end_ns=10+k,
            geometry_preprocessing='causal trailing submap in keyframe IMU frame') for k in range(2)]
        # Rejected key 1 deliberately has no cloud: reading it must fail.
        np.savez_compressed(store/'000000.npz', cloud=scene())
        workers[robot] = RegistrationExchange(robot, robots, rows, store, store/'exchange',
            'test-session', dict(enabled=True, max_points=1000), queue.append)
    pcm = dict(state='ready', verdicts=verdicts)
    edges = {r:{p:{} for p in pairs if any(e[0] == r for e in p)} for r in robots}
    return workers, queue, pcm, edges


def drain(workers, queue, pcm, edges):
    sent = []
    for _ in range(20):
        for r,w in workers.items(): w.advance(pcm, edges[r])
        batch = queue[:]; queue.clear(); sent.extend(copy.deepcopy(batch))
        # Reorder links and packets to exercise the supplier completion barrier.
        for event in reversed(batch): workers[event['dst']].receive(event)
        if all(w.published for w in workers.values()): return sent
    raise AssertionError('protocol did not finish')


@pytest.mark.parametrize('robots', [['Alpha', 'Bob', 'Carol'], ['Bob', 'Carol', 'Alpha']])
def test_pcm_selection_unique_ownership_and_delayed_evidence(tmp_path, robots):
    workers, queue, pcm, edges = network(tmp_path, robots)
    for r,w in workers.items(): w.advance(dict(state='waiting'), edges[r])
    assert not queue
    sent = drain(workers, queue, pcm, edges)
    manifests = {r:read_json(w.output/'manifest.json') for r,w in workers.items()}
    pairs = [p for m in manifests.values() for p in m['pairs']]
    assert len(pairs) == 2
    assert all(p['i'] < p['j'] and p['i'][0] == robots.index(r)
               for r,m in manifests.items() for p in m['pairs'])
    named = {tuple(sorted(((robots[p['i'][0]], p['i'][1]),
                           (robots[p['j'][0]], p['j'][1])))) for p in pairs}
    assert named == {(('Alpha', 0), ('Bob', 0)), (('Bob', 0), ('Carol', 0))}
    assert sum(e['kind'] == 'registration_response' for e in sent) == 2
    assert all(e['body']['keys'] == [0] for e in sent if e['kind'] == 'registration_request')
    assert all(len(w.local) == 1 for w in workers.values())
    assert all(len(m['clouds']) == len({tuple(e) for p in m['pairs'] for e in (p['i'], p['j'])})
               for m in manifests.values())
    assert all(c['key'] == 0 for m in manifests.values() for c in m['clouds'])
    assert all((w.output/c['file']).stat().st_size == c['points']*24
        for w,m in zip(workers.values(), manifests.values()) for c in m['clouds'])


@pytest.mark.parametrize('fault', ['rejected', 'wrong_peer', 'configuration', 'payload'])
def test_invalid_peer_evidence_fails_closed(tmp_path, fault):
    workers, queue, pcm, edges = network(tmp_path)
    for r,w in workers.items(): w.advance(pcm, edges[r])
    event = copy.deepcopy(next(e for e in queue if e['src'] == 'Alpha'))
    if fault == 'rejected': event['body']['keys'] = [1]
    elif fault == 'wrong_peer': event['src'] = 'Unknown'
    elif fault == 'configuration': event['body']['configuration'] = 'other-config'
    else:
        payload = copy.deepcopy(workers['Bob'].payload(0)); payload['payload_sha256'] = 'corrupt'
        event.update(src='Bob', dst='Alpha', kind='registration_response')
        event['body'] = dict(session_id='test-session', configuration=workers['Alpha'].fingerprint, payload=payload)
    receiver = workers[event['dst']]
    with pytest.raises(ValueError):
        receiver.receive(event); receiver.advance(pcm, edges[event['dst']])
    assert not receiver.published


def test_stale_session_ignored_and_disconnection_times_out(tmp_path, monkeypatch):
    workers, queue, pcm, edges = network(tmp_path)
    for r,w in workers.items(): w.advance(pcm, edges[r])
    event = copy.deepcopy(queue[0]); event['body']['session_id'] = 'old-run'
    receiver = workers[event['dst']]; receiver.receive(event)
    assert not receiver.pending
    monkeypatch.setattr('s3e_pipeline.registration_exchange.time.monotonic', lambda: receiver.started+181)
    with pytest.raises(TimeoutError): receiver.advance(pcm, edges[event['dst']])
