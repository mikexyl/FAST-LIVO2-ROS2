"""Frame/membership and causal-input checks for the retained-snapshot replay."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.circle_cover_replay import PreparedObservations, replay
from s3e_pipeline.circle_cover import CircleCoverConfig
from s3e_pipeline.circle_cover_benchmark import benchmark


def store(tmp_path):
    rows = []
    world = np.array([[1.2, 2.2, 0], [3.2, 1.2, 50], [40.2, 2.2, -10]])
    for k, ids in enumerate([np.arange(2), np.arange(3)]):
        T = np.eye(4); T[:3, 3] = [5*k, 3*k, -2*k]
        cloud = (world[ids]-T[:3, 3]).astype(np.float32)
        np.savez_compressed(tmp_path/f'{k:06d}.npz', cloud=cloud, scan=cloud)
        rows.append(dict(keyframe_id=k, available_ns=(k+1)*10**9, stamp_ns=(k+1)*10**9,
                         last_member_ns=(k+1)*10**9, gravity_world_m_s2=[0, 0, -9.81],
                         world_frame='r/odom', robot_id='r', strategy='area', frame='snapshot_anchor_imu',
                         cloud_frame='r/imu', body_frame='r/imu', complete=True,
                         T_world_body=T.tolist(), geometry_id_ranges=[[0, len(ids)]], geometry_count=len(ids),
                         archive_point_count=3, area_center_world=T[:3, 3].tolist(), area_radius_m=80.))
    (tmp_path/'keyframes.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    return rows, world


def test_causal_union_anchor_transform_and_identity(tmp_path):
    rows, world = store(tmp_path)
    stream = PreparedObservations(tmp_path)
    iterator = iter(stream)
    first, new = next(iterator)
    assert len(new) == 2 and len(stream.world) == 2
    np.testing.assert_allclose(stream.world, world[:2], atol=1e-6)
    second, new = next(iterator)
    assert len(new) == 1 and len(stream.world) == 3
    np.testing.assert_allclose(stream.world, world, atol=2e-6)
    assert stream.ids.tolist() == [0, 1, 2]
    assert len(stream.payloads) == 2


@pytest.mark.parametrize('problem', ['world_frame', 'availability', 'identity', 'membership'])
def test_reject_inconsistent_inputs(tmp_path, problem):
    rows, _ = store(tmp_path)
    if problem == 'world_frame': rows[1]['world_frame'] = 'different'
    if problem == 'availability': rows[1]['available_ns'] = rows[0]['available_ns']
    if problem == 'membership': rows[1]['geometry_id_ranges'] = [[0, 2]]
    if problem == 'identity':
        with np.load(tmp_path/'000001.npz') as data: cloud = data['cloud'].copy()
        cloud[0, 0] += 1
        np.savez_compressed(tmp_path/'000001.npz', cloud=cloud)
    (tmp_path/'keyframes.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    with pytest.raises(ValueError): list(PreparedObservations(tmp_path))


def test_replay_outputs_and_original_footprints(tmp_path):
    inputs = tmp_path/'inputs'; inputs.mkdir(); store(inputs)
    output = tmp_path/'out'
    history = replay(inputs, output, CircleCoverConfig(), make_gallery=False)
    assert len(history) == 2
    assert history[-1]['native_unique_points'] == 3
    assert history[-1]['trajectory_centered']['circle_count'] == 2
    assert history[-1]['planner']['coverage'] == 1
    assert (output/'final-state.json').is_file()
    assert (output/'inspection'/'placement-history.json').is_file()


def test_fresh_process_control_uses_identical_prefixes(tmp_path):
    inputs = tmp_path/'inputs'; inputs.mkdir(); store(inputs)
    a = benchmark(inputs, 'planner', CircleCoverConfig(), tmp_path/'planner.json')
    b = benchmark(inputs, 'trajectory_centered', CircleCoverConfig(), tmp_path/'baseline.json')
    assert [r['occupied_cells'] for r in a['updates']] == [r['occupied_cells'] for r in b['updates']]
    assert a['input_payloads'] == b['input_payloads']
    assert a['updates'][-1]['circle_count'] == 1 and b['updates'][-1]['circle_count'] == 2
