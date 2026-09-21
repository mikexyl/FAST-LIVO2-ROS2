"""Exercise actual gtsam_points factors in the isolated native solver."""
from pathlib import Path
import copy
import json
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import file_hash, read_json, write_json, write_jsonl
from s3e_pipeline.geometry import inv, transform
from s3e_pipeline.mixed_pgo import settings, native_input, native_provenance, refine_graph
from s3e_pipeline.pgo import optimize
from test_pipeline import T, scene, CFG, graph_fixture

SOURCE = Path(__file__).resolve().parents[3]
BINARY = SOURCE/'.ros2/mixed-pgo-install/bin/s3e_mixed_pgo'


@pytest.fixture
def native():
    if not BINARY.is_file(): pytest.skip('build_mixed_pgo.sh required')
    return native_provenance(SOURCE)


def invoke(tmp_path, source_shift=None, planar=False, reverse=False):
    truth = T(.4, -.3, .2, .13); gauge = T(20, -3, 1, .7)
    target = scene()
    if planar: target[:, 2] = 0
    source = transform(inv(truth), target)
    if source_shift is not None: source += source_shift
    np.asarray(target, dtype='<f8').tofile(tmp_path/'target.bin')
    np.asarray(source, dtype='<f8').tofile(tmp_path/'source.bin')
    measured = truth@T(.10, -.08, .06, .01)
    seed = gauge@measured
    nodes = [dict(key=0, T_world_body=gauge.tolist()), dict(key=1, T_world_body=seed.tolist()),
             dict(key=2, T_world_body=T(100, 30, 4, 1).tolist())]
    pair = dict(i=0, j=1, information=(np.eye(6)*10).tolist(), endpoint_i=['Alpha', 0], endpoint_j=['Bob', 0])
    if reverse: pair.update(i=1, j=0, endpoint_i=['Bob', 0], endpoint_j=['Alpha', 0])
    factors = [dict(kind='anchor', i=0, measurement=gauge.tolist(), information=(np.eye(6)*1e12).tolist()),
        dict(kind='loop', i=0, j=1, measurement=measured.tolist(), information=(np.eye(6)*10).tolist()),
        dict(kind='anchor', i=2, measurement=nodes[2]['T_world_body'], information=(np.eye(6)*1e12).tolist())]
    spec = dict(schema_version=1, settings=settings(dict(enabled=True, max_information_ratio=100., num_threads=2)),
        nodes=nodes, pose_factors=factors, registrations=[pair], clouds=[
            dict(key=k, path=str(tmp_path/name), points=len(target)) for k, name in enumerate(['target.bin', 'source.bin'])])
    write_json(tmp_path/'input.json', spec)
    subprocess.run([str(BINARY), str(tmp_path/'input.json'), str(tmp_path/'output.json')], check=True, timeout=30)
    return read_json(tmp_path/'output.json'), truth, nodes


@pytest.mark.parametrize('reverse', [False, True])
def test_live_gicp_known_transform_and_separate_component(native, tmp_path, reverse):
    result, truth, nodes = invoke(tmp_path, reverse=reverse)
    final = {r['key']: np.array(r['T_world_body']) for r in result['poses']}
    baseline = {r['key']: np.array(r['T_world_body']) for r in result['baseline_poses']}
    before = np.linalg.norm((inv(baseline[0])@baseline[1]) - truth)
    after = np.linalg.norm((inv(final[0])@final[1]) - truth)
    assert after < .15*before, (before, after)
    assert np.allclose(final[2], nodes[2]['T_world_body'], atol=1e-8)
    assert result['pose_factor_count'] == 3 and result['registration_factor_count'] == 1
    assert result['final_mixed_error'] < result['initial_mixed_error']
    diag = result['registrations'][0]
    assert diag['linearizations'] > 2 and diag['information_scale'] > 0
    assert diag['initial_max_information_ratio'] == 100.


@pytest.mark.parametrize('planar,shift,reason', [(True, None, 'unobservable'), (False, [100, 0, 0], 'insufficient_inliers')])
def test_geometry_rejection_keeps_pose_graph(native, tmp_path, planar, shift, reason):
    result, _, _ = invoke(tmp_path, planar=planar, source_shift=shift)
    assert result['registration_factor_count'] == 0
    assert result['registrations'][0]['reason'] == reason
    assert result['poses'] == result['baseline_poses']


def test_native_rejects_reciprocal_duplicates(native, tmp_path):
    invoke(tmp_path)
    spec = read_json(tmp_path/'input.json')
    spec['registrations'].append(dict(spec['registrations'][0], i=1, j=0))
    write_json(tmp_path/'input.json', spec)
    result = subprocess.run([str(BINARY), str(tmp_path/'input.json'), str(tmp_path/'invalid.json')], capture_output=True, text=True)
    assert result.returncode != 0 and 'duplicate' in result.stderr
    assert not (tmp_path/'invalid.json').exists()


def test_gnc_selection_and_immutable_clouds(native, tmp_path):
    rows, loops = graph_fixture(True)
    graph = optimize(rows, loops, CFG['pgo'])
    stores = {}; hashes = {}
    for robot in ('Alpha', 'Bob', 'Carol'):
        root = tmp_path/robot; root.mkdir(); stores[robot] = root; metadata = []
        for row in [r for r in rows if r['robot_id'] == robot]:
            k = row['keyframe_id']
            metadata.append(dict(row, cloud_frame=f'{robot}/imu', body_frame=f'{robot}/imu',
                submap_end_ns=row['stamp_ns'], geometry_preprocessing='causal trailing submap in keyframe IMU frame'))
            # Bob has a different odometry gauge; the physical scan pose is shared.
            np.savez_compressed(root/f'{k:06d}.npz', cloud=transform(inv(T(k, k%2, 0, .05*k)), scene()))
        write_jsonl(root/'keyframes.jsonl', metadata)
        hashes.update({p:file_hash(p) for p in root.iterdir()})
    output = tmp_path/'out'; output.mkdir()
    result = refine_graph(graph, stores, dict(enabled=True, max_points=4000, min_overlap=.2), SOURCE, output, native)
    assert result['components'] == graph['components']
    assert len([f for f in result['factors'] if f['kind'] == 'registration']) == 3
    serialized = read_json(output/'native-input.json')
    assert len(serialized['registrations']) == 3
    assert all(p['endpoint_i'][1] != 5 for p in serialized['registrations'])
    assert all(c['endpoint'][0] != 'Carol' for c in result['registration']['clouds'])
    assert all(file_hash(p) == h for p, h in hashes.items())
    assert not list(output.glob('registration-clouds-*'))
    for row in result['poses']:
        assert 'T_pose_only_body' in row


def test_configuration_and_disabled_path(tmp_path):
    for cfg in ({'voxel_m': float('nan')}, {'enabled': 1}, {'num_threads': 0}, {'min_overlap': 2},
                {'max_information_ratio': 0}, {'factor': 'invented'}, {'unknown': 1}):
        with pytest.raises(ValueError): settings(cfg)
    graph = {'sentinel': 'unchanged'}
    assert refine_graph(graph, {}, {}, SOURCE, tmp_path) is graph
    assert not list(tmp_path.iterdir())


def test_native_input_cannot_restore_rejected_loops():
    rows, loops = graph_fixture(True)
    graph = optimize(rows, loops, CFG['pgo'])
    spec = native_input(graph, {}, settings({'enabled': True}))
    assert len(spec['registrations']) == 3
    assert all(p['endpoint_i'][1] != 5 for p in spec['registrations'])
