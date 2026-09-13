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
def test_native_unknown_frames_and_components(tmp_path, disconnected):
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
    loop = tmp_path/'loops'; loop.mkdir(); write_jsonl(loop/'constraints.jsonl',all_loops)
    artifacts['loops.livo.megaloc_mapclosures'] = str(loop)
    cfg = dict(robots=robots,backend=dict(name='megaloc_mapclosures'),loops={},
        pgo=dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15),
        dpgo=dict(mode='frozen',settle_iterations=60,loop_rate_hz=10.,timeout_s=60,
                  ros_domain_id=78 if disconnected == 'alpha' else (77 if disconnected else 76)))
    out = tmp_path/'result'; out.mkdir(); run(cfg,artifacts,source,out)
    for row in read_jsonl(out/'poses.jsonl'):
        robot,key = row['robot_id'],row['keyframe_id']
        reference = ('Bob' if robot in ('Bob','Carol') else 'Alpha') if disconnected == 'alpha' else ('Carol' if disconnected and robot=='Carol' else 'Alpha')
        expected = inv(truth[reference,0])@truth[robot,key]
        error = inv(expected)@pose(row['T_world_body'])
        assert np.linalg.norm(error[:3,3]) < 1e-3
        assert Rotation.from_matrix(error[:3,:3]).magnitude() < 1e-4
        assert row['reference_available'] == (reference == 'Alpha')
        assert row['component'] == reference
