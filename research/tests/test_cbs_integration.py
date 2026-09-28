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
    exercise_native(tmp_path, disconnected, pcm_enabled)


@pytest.mark.skipif(os.environ.get('S3E_TEST_DDS') != '1', reason='Set S3E_TEST_DDS=1 for native DDS tests')
def test_native_distributed_registration(tmp_path):
    exercise_native(tmp_path, False, True, registration=True)


@pytest.mark.skipif(os.environ.get('S3E_TEST_DDS') != '1', reason='Set S3E_TEST_DDS=1 for native DDS tests')
def test_native_four_robot_pcm_registration(tmp_path):
    exercise_native(tmp_path, False, True, registration=True,
                    robots=['robot1', 'robot2', 'robot3', 'robot4'])


@pytest.mark.skipif(os.environ.get('S3E_TEST_DDS') != '1', reason='Requires isolated CBS DDS overlay')
@pytest.mark.parametrize('order',[['Alpha','Bob','Carol'],['Carol','Alpha','Bob']])
def test_native_distributed_ellipsoid_registration(tmp_path,order):
    exercise_native(tmp_path,False,True,registration='ellipsoid',robots=order)


@pytest.mark.skipif(os.environ.get('S3E_TEST_GPU_DDS') != '1', reason='Requires isolated CUDA CBS overlay')
@pytest.mark.parametrize('order',[['Alpha','Bob','Carol'],['Carol','Alpha','Bob']])
def test_native_distributed_gpu_registration(tmp_path,order):
    exercise_native(tmp_path,False,True,registration='vgicp_gpu',robots=order)


def exercise_native(tmp_path, disconnected, pcm_enabled, registration=False, robots=None):
    source = Path(__file__).resolve().parents[3]
    robots = robots or ['Alpha','Bob','Carol']; artifacts = {}; truth = {}; all_loops = []
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
            if registration:
                from test_pipeline import scene
                from s3e_pipeline.geometry import transform
                rows[-1].update(cloud_frame=robot+'/imu', body_frame=robot+'/imu',
                    submap_end_ns=rows[-1]['stamp_ns'],
                    geometry_preprocessing='causal trailing submap in keyframe IMU frame')
                if registration=='ellipsoid':
                    import hashlib
                    from s3e_pipeline.ellipsoid_registration import bounded_ellipsoids
                    centers=np.array(np.meshgrid(np.arange(-5,6),np.arange(-5,6),np.arange(-3,4))).reshape(3,-1).T*1.1
                    local=transform(inv(T),centers)
                    basis=np.broadcast_to(T[:3,:3].T,(len(local),3,3)).reshape(-1,9)
                    evidence=bounded_ellipsoids(np.c_[local,np.full((len(local),3),.2),basis])
                    rows[-1].update(ellipsoid_evidence_policy=dict(voxel_m=.4,max_count=50000),
                        ellipsoid_evidence_sha256=hashlib.sha256(evidence.astype('<f8').tobytes()).hexdigest())
                    np.savez_compressed(store/f'{k:06d}.npz',ellipsoids=evidence)
                else:np.savez_compressed(store/f'{k:06d}.npz', cloud=transform(inv(T), scene()))
        write_jsonl(store/'keyframes.jsonl',rows)
        artifacts[f'keyframes.livo.{robot}'] = str(store.parent)
        artifacts[f'descriptors.livo.megaloc_mapclosures.{robot}'] = str(store)
    pairs = [('Bob','Carol')] if disconnected == 'alpha' else ([('Alpha','Bob')] if disconnected else [('Alpha','Bob'),('Bob','Carol'),('Alpha','Carol')])
    if not disconnected:
        from itertools import combinations
        pairs = list(combinations(robots, 2))
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
    if registration=='ellipsoid':
        for edge in all_loops:
            metadata=[]
            for robot,key in (edge['i'],edge['j']):
                row=read_jsonl(Path(artifacts[f'keyframes.livo.{robot}'])/'store/keyframes.jsonl')[key]
                metadata.append(dict(robot=robot,key=key,stamp_ns=row['stamp_ns'],cloud_frame=row['cloud_frame'],
                                     payload_sha256=row['ellipsoid_evidence_sha256']))
            edge['diagnostics']=dict(ellipsoid_endpoints=metadata)
    loop = tmp_path/'loops'; loop.mkdir(); write_jsonl(loop/'constraints.jsonl',all_loops)
    artifacts['loops.livo.megaloc_mapclosures'] = str(loop)
    cfg = dict(robots=robots,backend=dict(name='megaloc_mapclosures'),loops={},
        pgo=dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15),
        dpgo=dict(mode='frozen',settle_iterations=60,loop_rate_hz=10.,timeout_s=60,
                  pcm=dict(enabled=pcm_enabled,probability=.99,minimum_clique_size=2,timeout_s=20.),
                  ros_domain_id=78 if disconnected == 'alpha' else (77 if disconnected else 76)))
    if registration:
        cfg['dpgo']['registration_factors'] = dict(enabled=True, num_threads=1, voxel_m=.01, max_points=4000)
    if registration=='ellipsoid':
        cfg['backend'].update(ellipsoid_only=True,registration=dict(method='ellipsoid'))
        cfg['dpgo']['registration_factors'].update(factor='ellipsoid')
    if registration=='vgicp_gpu':
        cfg['dpgo']['registration_factors'].update(factor='vgicp_gpu')
    out = tmp_path/'result'; out.mkdir(); run(cfg,artifacts,source,out)
    assert len(read_jsonl(out/'constraints.jsonl')) == expected_loops
    if registration:
        from s3e_pipeline.artifacts import file_hash
        result = read_json(out/'registration.json'); summary = read_json(out/'summary.json')
        assert result['registration_factor_count'] == expected_loops
        assert [result['robots'][r]['registration_factor_count'] for r in robots] == [3*(len(robots)-i-1) for i in range(len(robots))]
        assert all(p['linearizations'] > 20 and p['final_quality_reason'] == 'accepted'
            for r in result['robots'].values() for p in r['pairs'])
        if registration=='vgicp_gpu':
            assert all(p['gpu_linearization_batches']>0 and p['gpu_levels']==2 and not p['synchronous_factor_fallback']
                for r in result['robots'].values() for p in r['pairs'])
        wire = [e for p in out.glob('*/wire.jsonl') for e in read_jsonl(p)
                if e['kind'].startswith('registration_')]
        assert sum(e['network_bytes'] for e in wire) == summary['registration_network_cdr_bytes'] > 0
        assert not list(out.glob('*/registration/*.bin'))
        for p in out.glob('*/registration/transport.json'):
            assert all(file_hash(s['source_npz']) == s['source_sha256'] for s in read_json(p)['sources'])
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
        if not disconnected:reference = robots[0]
        expected = inv(truth[reference,0])@truth[robot,key]
        error = inv(expected)@pose(row['T_world_body'])
        assert np.linalg.norm(error[:3,3]) < (2e-3 if registration else 1e-3)
        assert Rotation.from_matrix(error[:3,:3]).magnitude() < (1e-3 if registration else 1e-4)
        assert row['reference_available'] == (reference == robots[0])
        assert row['component'] == reference
