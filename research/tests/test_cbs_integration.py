"""Explicit opt-in native DDS test; requires the project CBS overlay."""
import os
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import write_jsonl, read_jsonl, read_json
from s3e_pipeline.geometry import constraint, inv, pose
from s3e_pipeline.dpgo import run


@pytest.mark.skipif(os.environ.get('S3E_TEST_DDS') != '1', reason='Set S3E_TEST_DDS=1 for native DDS tests')
@pytest.mark.parametrize('disconnected', [False, True, 'alpha'])
@pytest.mark.parametrize('pcm_enabled', [False, True])
def test_native_unknown_frames_and_components(tmp_path, disconnected, pcm_enabled):
    source = Path(__file__).resolve().parents[3]
    robots = ['Alpha','Bob','Carol']; artifacts = {}; truth = {}; all_loops = []
    for index, robot in enumerate(robots):
        store = tmp_path/robot/'store'; store.mkdir(parents=True)
        gauge = np.eye(4); gauge[:3,:3] = Rotation.from_rotvec([.2*index,-.1*index,.7*index]).as_matrix()
        gauge[:3,3] = [7*index,-12*index,3*index]
        rows = []
        for k in range(8):
            T = np.eye(4); T[:3,:3] = Rotation.from_rotvec([.03*k,-.01*k,.07*k]).as_matrix()
            T[:3,3] = [k,2*index+np.sin(k),.2*index]
            truth[robot,k] = T
            rows.append(dict(robot_id=robot,keyframe_id=k,stamp_ns=1700000000000000001+k*10**9,
                T_world_body=(inv(gauge)@T).tolist()))
        write_jsonl(store/'keyframes.jsonl',rows)
        artifacts[f'keyframes.livo.{robot}'] = str(store.parent)
        artifacts[f'descriptors.livo.megaloc_mapclosures.{robot}'] = str(store)
    pairs = [('Bob','Carol')] if disconnected == 'alpha' else ([('Alpha','Bob')] if disconnected else [('Alpha','Bob'),('Bob','Carol'),('Alpha','Carol')])
    for a,b in pairs:
        for k in (0,3,7):
            all_loops.append(constraint([a,k],[b,k],inv(truth[a,k])@truth[b,k],np.eye(6)*100,
                                       kind='loop',accepted=True))
    expected_loops = len(all_loops)
    if pcm_enabled:
        # Gross translation + rotation errors, with correct endpoint IDs.
        error = np.eye(4); error[:3,3] = [35.,-20.,8.]
        error[:3,:3] = Rotation.from_rotvec([.4,-.5,1.]).as_matrix()
        for a,b in pairs:
            for k in (1,5):
                all_loops.append(constraint([a,k],[b,k],inv(truth[a,k])@truth[b,k]@error,
                                             np.eye(6)*100,kind='loop',accepted=True))
        if disconnected:
            # A single unsupported bridge must not initialize a remote anchor.
            a,b = ('Alpha','Bob') if disconnected == 'alpha' else ('Bob','Carol')
            all_loops.append(constraint([a,2],[b,2],inv(truth[a,2])@truth[b,2]@error,
                                         np.eye(6)*100,kind='loop',accepted=True))
    loop = tmp_path/'loops'; loop.mkdir(); write_jsonl(loop/'constraints.jsonl',all_loops)
    artifacts['loops.livo.megaloc_mapclosures'] = str(loop)
    cfg = dict(robots=robots,backend=dict(name='megaloc_mapclosures'),loops={},
        pgo=dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15),
        dpgo=dict(mode='frozen',settle_iterations=60,loop_rate_hz=10.,timeout_s=60,
                  pcm=dict(enabled=pcm_enabled,probability=.99,minimum_clique_size=2,timeout_s=20.),
                  ros_domain_id=78 if disconnected == 'alpha' else (77 if disconnected else 76)))
    out = tmp_path/'result'; out.mkdir(); run(cfg,artifacts,source,out)
    assert len(read_jsonl(out/'constraints.jsonl')) == expected_loops
    if pcm_enabled:
        import csv
        pcm = read_json(out/'pcm.json'); summary = read_json(out/'summary.json')
        assert pcm['excluded_loops'] == len(all_loops)-expected_loops
        assert sum(v['retained'] for v in pcm['verdicts']) == expected_loops
        wire = [r for path in out.glob('*/native/*/cbs_online/pcm_wire.csv')
                for r in csv.DictReader(path.open())]
        assert len(wire) == (len(pairs) + bool(disconnected))*4
        assert sum(int(r['serialized_bytes']) for r in wire) == summary['pcm_network_cdr_bytes'] > 0
        for robot in robots:
            receipt = read_json(out/robot/'pcm.json')
            assert receipt['state'] == 'ready' and receipt['own_nodes'] == 8
    for row in read_jsonl(out/'poses.jsonl'):
        robot,key = row['robot_id'],row['keyframe_id']
        reference = ('Bob' if robot in ('Bob','Carol') else 'Alpha') if disconnected == 'alpha' else ('Carol' if disconnected and robot=='Carol' else 'Alpha')
        expected = inv(truth[reference,0])@truth[robot,key]
        error = inv(expected)@pose(row['T_world_body'])
        assert np.linalg.norm(error[:3,3]) < 1e-3
        assert Rotation.from_matrix(error[:3,:3]).magnitude() < 1e-4
        assert row['reference_available'] == (reference == 'Alpha')
        assert row['component'] == reference
