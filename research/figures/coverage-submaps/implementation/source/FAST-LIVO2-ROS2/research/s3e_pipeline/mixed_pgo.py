"""Optional native GICP factors after the existing robust pose-graph selection.

Only retained loop endpoints are loaded. Clouds are immutable trailing submaps
in the keyframe IMU frame. The native ABI is isolated from Python GTSAM 4.2.
"""
from pathlib import Path
import copy
import math
import subprocess
import tempfile
import time

import numpy as np

from .artifacts import file_hash, read_json, read_jsonl, write_json
from .geometry import pose, inv, TANGENT_ORDER
from .registration import bounded_cloud

DEFAULTS = dict(enabled=False, factor='gicp', voxel_m=.5, max_points=12000,
    max_range_m=80., covariance_neighbors=20, correspondence_m=1.5,
    min_inliers=100, min_overlap=.3, min_observability=1e-4, max_condition=1e6,
    max_information_ratio=1., max_iterations=30, num_threads=4, timeout_s=180.)


def settings(cfg):
    if not isinstance(cfg, dict) or set(cfg) - set(DEFAULTS):
        raise ValueError('Unknown registration_factors settings')
    value = dict(DEFAULTS, **cfg)
    if type(value['enabled']) is not bool or value['factor'] != 'gicp':
        raise ValueError('Expected boolean enabled and factor: gicp')
    for key in ('max_points', 'covariance_neighbors', 'min_inliers', 'max_iterations', 'num_threads'):
        if type(value[key]) is not int or value[key] < 1:
            raise ValueError(f'Expected positive integer {key}')
    for key in ('voxel_m', 'max_range_m', 'correspondence_m', 'min_overlap',
                'min_observability', 'max_condition', 'max_information_ratio', 'timeout_s'):
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]) or value[key] <= 0:
            raise ValueError(f'Expected finite positive {key}')
    if (value['min_overlap'] > 1 or value['num_threads'] > 64 or
            not 3 <= value['covariance_neighbors'] <= value['min_inliers'] <= value['max_points'] <= 1000000):
        raise ValueError('Invalid registration point or thread bounds')
    return value


def native_provenance(source):
    binary = Path(source)/'.ros2/mixed-pgo-install/bin/s3e_mixed_pgo'
    if not binary.is_file() or not (binary.parent/'build.json').is_file():
        raise FileNotFoundError('Build registration factors with bash FAST-LIVO2-ROS2/scripts/build_mixed_pgo.sh')
    manifest = read_json(binary.parent/'build.json')
    sources = Path(__file__).resolve().parents[1]/'adapters/gtsam_points'
    if {str(p.relative_to(sources)): file_hash(p) for p in sources.rglob('*') if p.is_file()} != manifest['adapter_sources']:
        raise ValueError('Mixed PGO adapter changed; rebuild native executable')
    if file_hash(binary) != manifest['binary_sha256']:
        raise ValueError('Mixed PGO binary differs from build manifest')
    loaded = {}
    for line in subprocess.check_output(['ldd', str(binary)], text=True).splitlines():
        if 'gtsam' in line and '=>' in line:
            p = Path(line.split('=>')[1].split()[0]).resolve(); loaded[str(p)] = file_hash(p)
    if loaded != manifest['libraries']:
        raise ValueError('Mixed PGO loaded libraries differ from build manifest')
    return dict(manifest, binary=str(binary))


def native_input(graph, clouds, cfg):
    """Serialize an already selected graph; rejected loops cannot reenter it."""
    rows = graph['poses']; ids = {(r['robot_id'], r['keyframe_id']): k for k, r in enumerate(rows)}
    nodes = [dict(key=k, T_world_body=r['T_world_body']) for k, r in enumerate(rows)]
    factors = []; pairs = []
    for f in graph['factors']:
        if f['solver_weight'] == 0: continue
        i = ids[tuple(f['i'])]
        item = dict(kind=f['kind'], i=i)
        if f['kind'] == 'anchor':
            item.update(measurement=rows[i]['T_world_body'], information=(np.eye(6)*1e12).tolist())
        else:
            j = ids[tuple(f['j'])]
            info = np.diag(1/np.square(f['sigmas'])) if f['kind'] == 'odometry' else np.asarray(f['information'])
            item.update(j=j, measurement=f['T_i_j'], information=info.tolist())
            if f['kind'] == 'loop':
                if rows[i]['component'] != rows[j]['component']:
                    raise ValueError('Registration cannot join rejected/disconnected components')
                pairs.append(dict(i=i, j=j, information=info.tolist(), endpoint_i=f['i'], endpoint_j=f['j']))
        factors.append(item)
    return dict(schema_version=1, settings=cfg, nodes=nodes, pose_factors=factors,
        registrations=pairs, clouds=[dict(key=ids[e], **c) for e, c in sorted(clouds.items())])


def refine_graph(graph, stores, cfg, source, output, provenance=None):
    """Retain all selected pose factors and add live binary GICP costs."""
    import gtsam
    start = time.monotonic(); cfg = settings(cfg); output = Path(output)
    if not cfg['enabled']: return graph
    provenance = provenance or native_provenance(source)
    selected = [f for f in graph['factors'] if f['kind'] == 'loop' and f['solver_weight'] > 0]
    endpoints = sorted({tuple(f[e]) for f in selected for e in ('i', 'j')})
    write_json(output/'pose-only-graph.json', graph)
    clouds = {}; cloud_report = []
    metadata = {(r['robot_id'], r['keyframe_id']): r for robot, root in stores.items()
                for r in read_jsonl(Path(root)/'keyframes.jsonl') if r['robot_id'] == robot}
    # Binary payloads only bridge two local processes; saved inputs name and
    # hash the original NPZs and deterministic preprocessing instead of retaining
    # another copy of every cloud.
    with tempfile.TemporaryDirectory(prefix='registration-clouds-', dir=output) as temp:
        for index, endpoint in enumerate(endpoints):
            robot, key = endpoint
            path = Path(stores[robot])/f'{key:06d}.npz'
            row = metadata[endpoint]
            if (row.get('cloud_frame') != row.get('body_frame') or not row.get('body_frame') or
                    row.get('submap_end_ns') != row['stamp_ns'] or
                    row.get('geometry_preprocessing') != 'causal trailing submap in keyframe IMU frame'):
                raise ValueError(f'Unverified keyframe cloud coordinate frame: {endpoint}')
            with np.load(path, allow_pickle=False) as f:
                points, voxel = bounded_cloud(f['cloud'], cfg['voxel_m'], cfg['max_points'], cfg['max_range_m'])
            if len(points) < cfg['covariance_neighbors']:
                raise ValueError(f'Insufficient geometry at selected endpoint {endpoint}')
            payload = Path(temp)/f'{index:06d}.bin'
            np.asarray(points, dtype='<f8').tofile(payload)
            clouds[endpoint] = dict(path=str(payload), points=len(points))
            cloud_report.append(dict(endpoint=list(endpoint), source_npz=str(path), source_sha256=file_hash(path),
                payload_sha256=file_hash(payload), points=len(points), effective_voxel_m=voxel,
                coordinate_frame='keyframe IMU/body; T_i_j maps source j into target i'))
        spec = native_input(graph, clouds, cfg)
        write_json(output/'native-input.json', spec)
        with (output/'native.log').open('w') as log:
            subprocess.run([provenance['binary'], str(output/'native-input.json'), str(output/'native-result.json')],
                check=True, timeout=cfg['timeout_s'], stdout=log, stderr=subprocess.STDOUT)
    native = read_json(output/'native-result.json')
    if not native['success']: raise RuntimeError('Mixed PGO failed final geometry checks')
    result = copy.deepcopy(graph)
    final = {r['key']: pose(r['T_world_body']) for r in native['poses']}
    baseline = {r['key']: pose(r['T_world_body']) for r in native['baseline_poses']}
    if set(final) != set(range(len(result['poses']))) or set(baseline) != set(final):
        raise ValueError('Incomplete native pose output')
    ids = {(r['robot_id'], r['keyframe_id']): k for k, r in enumerate(result['poses'])}
    for k, row in enumerate(result['poses']):
        row['T_pose_only_body'] = baseline[k].tolist(); row['T_world_body'] = final[k].tolist()
    for f in result['factors']:
        f['pose_only_squared_whitened_residual'] = f['squared_whitened_residual']
        i = ids[tuple(f['i'])]
        if f['kind'] == 'anchor':
            residual = gtsam.Pose3.Logmap(gtsam.Pose3(baseline[i]).between(gtsam.Pose3(final[i])))
            info = np.eye(6)*1e12
        else:
            j = ids[tuple(f['j'])]
            residual = gtsam.Pose3.Logmap(gtsam.Pose3(pose(f['T_i_j'])).between(gtsam.Pose3(inv(final[i])@final[j])))
            info = np.diag(1/np.square(f['sigmas'])) if f['kind'] == 'odometry' else np.asarray(f['information'])
            f['residual_rotation_translation'] = residual.tolist()
        f['squared_whitened_residual'] = float(residual@info@residual)
    for diagnostic in native['registrations']:
        if not diagnostic['accepted']: continue
        result['factors'].append(dict(kind='registration', factor_type='gtsam_points::IntegratedGICPFactor',
            i=diagnostic['endpoint_i'], j=diagnostic['endpoint_j'], factor_index=len(result['factors']),
            solver_weight=1., robust_weight=1., known_inlier=False,
            squared_whitened_residual=2*diagnostic['final_error_fixed_last_correspondences'],
            residual_units='twice the scaled native matching objective; not a calibrated chi-square',
            diagnostics=diagnostic))
    result.update(optimization_method='pose GNC-TLS selection, then joint LM with selected pose factors and live binary GICP factors',
        pose_only_initial_error=graph['initial_error'], initial_error=native['initial_mixed_error'],
        pose_only_final_solver_error=graph['final_solver_error'], final_solver_error=native['final_mixed_error'],
        final_unrobust_error=None, gtsam_version=native['gtsam_version'],
        registration=dict(settings=cfg, native=provenance, clouds=cloud_report,
            **{k:v for k,v in native.items() if k not in ('poses', 'baseline_poses')},
            total_refinement_wall_s=time.monotonic()-start,
            weighting='fixed initial generalized-eigenvalue curvature cap against the verified loop information',
            uncertainty='pose and registration costs reuse LiDAR evidence; heuristic regularization, not independent measurements',
            topology='GNC selected topology is fixed; no rejected loop is restored'))
    write_json(output/'registration.json', result['registration'])
    return result
