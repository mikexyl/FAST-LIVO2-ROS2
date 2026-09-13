"""Trajectory association, SE(3) alignment and translation APE delegated to evo."""
from collections import defaultdict
import copy
from importlib.metadata import version
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.core.geometry import GeometryException
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.main_ape import ape
from evo.tools import file_interface

from .artifacts import write_json

EVO_VERSION = '1.36.5'


def associate(rows, truth, max_diff_s=.05):
    """Use evo's standard nearest timestamp association, without interpolation."""
    if not rows or not len(truth[0]):
        raise sync.SyncException('Empty estimate or position ground truth')
    reference = PoseTrajectory3D(positions_xyz=truth[1],
        orientations_quat_wxyz=np.tile([1., 0., 0., 0.], (len(truth[0]), 1)),
        timestamps=np.asarray(truth[0], dtype=np.float64)/1e9)
    estimate = PoseTrajectory3D(poses_se3=[np.asarray(r['T_world_body']) for r in rows],
        timestamps=np.asarray([r['stamp_ns'] for r in rows], dtype=np.float64)/1e9)
    return sync.associate_trajectories(reference, estimate, max_diff=max_diff_s,
        first_name='position ground truth', snd_name='estimate')


def evaluate(graph, truth, cfg, output=None):
    if version('evo') != EVO_VERSION:
        raise ValueError(f'evo {EVO_VERSION} required')
    maximum_diff = float(cfg.get('evo_max_diff_s', .05))
    if maximum_diff <= 0:
        raise ValueError('evo_max_diff_s must be positive')
    groups = defaultdict(lambda: defaultdict(list))
    for row in graph['poses']:
        groups[row['component']][row['robot_id']].append(row)
    results = {}; status = {}
    output = Path(output) if output is not None else None
    if output is not None:
        output.mkdir(parents=True, exist_ok=True)
    for component, robots in sorted(groups.items()):
        pairs = {}; availability = {}
        for robot, rows in sorted(robots.items()):
            try:
                reference, estimate = associate(rows, truth[robot], maximum_diff)
                pairs[robot] = (reference, estimate)
                availability[robot] = dict(matched_samples=estimate.num_poses,
                    estimate_samples=len(rows), reference_samples=len(truth[robot][0]))
            except sync.SyncException as exc:
                availability[robot] = dict(matched_samples=0, estimate_samples=len(rows),
                    reference_samples=len(truth[robot][0]), reason=str(exc))
        status[component] = dict(robots=availability, available=False)
        count = sum(est.num_poses for _, est in pairs.values())
        if count < 3:
            status[component]['reason'] = 'fewer than three evo-associated positions'
            continue
        # Associate within each robot first. Pool corresponding poses only for
        # one component-wide evo alignment; timestamps never cross robot IDs.
        reference = PosePath3D(poses_se3=np.concatenate([r.poses_se3 for r, _ in pairs.values()]))
        estimate = PosePath3D(poses_se3=np.concatenate([e.poses_se3 for _, e in pairs.values()]))
        unaligned = copy.deepcopy(estimate)
        try:
            result = ape(reference, estimate, metrics.PoseRelation.translation_part,
                align=True, correct_scale=False, ref_name='reference', est_name='estimate')
        except GeometryException as exc:
            status[component]['reason'] = f'evo alignment failed: {exc}'
            continue
        alignment = result.np_arrays['alignment_transformation_sim3']
        result.info.update(engine=f'evo {EVO_VERSION}', component=component,
            robots=list(pairs), alignment_scope='one shared SE(3) alignment per connected component; scale fixed to 1',
            association_max_diff_s=maximum_diff, association='evo.sync.associate_trajectories; no interpolation',
            orientation_ground_truth_used=False)
        result.add_np_array('robot_index', np.concatenate([np.full(e.num_poses, i, dtype=np.int32) for i, (_, e) in enumerate(pairs.values())]))
        result.add_np_array('estimate_timestamps', np.concatenate([e.timestamps for _, e in pairs.values()]))
        result.add_np_array('reference_timestamps', np.concatenate([r.timestamps for r, _ in pairs.values()]))
        robot_results = {}
        for robot, (ref, est) in pairs.items():
            est.transform(alignment)
            individual = ape(ref, est, metrics.PoseRelation.translation_part, align=False,
                ref_name=f'{robot}-reference', est_name=f'{robot}-estimate')
            individual.info.update(engine=f'evo {EVO_VERSION}',
                alignment_scope='saved shared component SE(3) alignment; no additional robot alignment',
                component=component, association_max_diff_s=maximum_diff, orientation_ground_truth_used=False)
            robot_results[robot] = dict(samples=est.num_poses, statistics=individual.stats)
            if output is not None:
                file_interface.save_res_file(output/f'{robot}-ape.zip', individual)
                file_interface.write_tum_trajectory_file(output/f'{robot}-reference-matched.tum', ref)
                file_interface.write_tum_trajectory_file(output/f'{robot}-estimate-aligned.tum', est)
        if output is not None:
            file_interface.save_res_file(output/f'{component}-component-ape.zip', result)
            file_interface.write_kitti_poses_file(output/f'{component}-reference.kitti', reference)
            file_interface.write_kitti_poses_file(output/f'{component}-estimate.kitti', unaligned)
        results[component] = dict(samples=count, robots=list(pairs), rmse_m=result.stats['rmse'],
            median_m=result.stats['median'], statistics=result.stats, per_robot=robot_results,
            alignment_SE3=alignment.tolist(), scale=1., alignment_scope=result.info['alignment_scope'],
            engine=f'evo {EVO_VERSION}', association_max_diff_s=maximum_diff,
            limitation='translation APE only; supplied GT orientations unused; antenna lever arm uncorrected')
        status[component]['available'] = True
    if output is not None:
        write_json(output/'evaluation.json', dict(engine=f'evo {EVO_VERSION}',
            association_max_diff_s=maximum_diff, time_offset_s=0, interpolation=False,
            scale_fitting=False, components=status, metrics=results,
            orientation_ground_truth_used=False))
        (output/'README.md').write_text(
            '# evo trajectory evaluation\n\n'
            f'evo {EVO_VERSION}; nearest timestamp association within {maximum_diff:g} s, no interpolation or time offset. '
            'Translation APE uses one shared SE(3) alignment per connected component with scale fixed to 1. '
            'Association is performed separately for each robot. The pooled KITTI files contain matched poses in robot order. '
            'They are for component APE, not RPE or temporal analysis.\n\n'
            'Reproduce a component result (replace `Alpha` with its component ID):\n\n'
            '```bash\nevo_ape kitti Alpha-reference.kitti Alpha-estimate.kitti -a -r trans_part --save_results check.zip\n'
            'evo_res Alpha-component-ape.zip --save_plot ape.pdf --save_table statistics.csv\n```\n\n'
            'Each robot ZIP uses the shared component alignment without an additional fit. '
            'Unavailable results are recorded in `evaluation.json`; no numeric accuracy is invented. '
            'GT orientations are not used.\n')
    return results
