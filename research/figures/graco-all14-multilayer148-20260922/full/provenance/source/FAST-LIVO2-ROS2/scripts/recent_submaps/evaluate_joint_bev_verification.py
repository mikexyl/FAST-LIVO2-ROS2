#!/usr/bin/env python3
"""Audit frozen GICP residuals, then evaluate poses using GT in a separate stage."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from s3e_pipeline.geometry import transform
from s3e_pipeline.registration import bounded_cloud


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose_error(estimate, truth, query_ground, candidate_ground):
    estimate = np.asarray(estimate)
    level = query_ground@estimate@np.linalg.inv(candidate_ground)
    reference = query_ground@truth@np.linalg.inv(candidate_ground)
    delta = level[:3, 3]-reference[:3, 3]
    angle = np.arctan2(level[1, 0], level[0, 0])-np.arctan2(reference[1, 0], reference[0, 0])
    return dict(translation_m=float(np.linalg.norm(estimate[:3, 3]-truth[:3, 3])),
        rotation_deg=float(np.degrees(Rotation.from_matrix(truth[:3, :3].T@estimate[:3, :3]).magnitude())),
        horizontal_translation_m=float(np.linalg.norm(delta[:2])), vertical_translation_error_m=float(delta[2]),
        yaw_error_deg=float(abs(np.degrees(np.arctan2(np.sin(angle), np.cos(angle))))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args(); out = args.output.resolve(); reference = args.reference.resolve()
    summary_path = out/'summary.json'; before = sha(summary_path)
    data = json.loads(summary_path.read_text()); assert data['complete'] and not data['ground_truth_used']
    assert not (out/'evaluation.json').exists()
    maps = {(m['robot'], m['key']): m for m in data['maps']}
    clouds, grounds, kdtrees = {}, {}, {}
    for key, m in maps.items():
        assert sha(out/m['geometry']) == m['sha256']
        with np.load(out/m['geometry']) as z:
            cloud, _ = bounded_cloud(z['cloud'], data['design']['registration']['voxel_m'], None, None)
            grounds[key] = z['ground'].copy()
        clouds[key] = cloud; kdtrees[key] = cKDTree(cloud)
    geometric_audit = []
    for row in data['results']:
        q, c = tuple(row['query']), tuple(row['candidate']); v = row['verification']
        T = np.asarray(v.get('T_i_j', row['height_seed_T_i_j']))
        target, source = clouds[q], clouds[c]
        assert (len(target), len(source)) == (v['target_points'], v['source_points'])
        aligned = transform(T, source)
        forward = kdtrees[q].query(aligned, workers=1)[0]
        reverse = cKDTree(aligned).query(target, workers=1)[0]
        mask = forward < data['design']['registration']['inlier_m']
        source_overlap = float(mask.mean()); target_overlap = float((reverse < .6).mean())
        rmse = float(np.sqrt(np.mean(forward[mask]**2))) if mask.any() else None
        initial = kdtrees[q].query(transform(np.asarray(row['height_seed_T_i_j']), source), workers=1)[0]
        zero = kdtrees[q].query(transform(np.asarray(row['bev_initial_T_i_j']), source), workers=1)[0]
        assert np.isclose((initial < 1.5).mean(), v['initial_overlap'], atol=1e-12)
        if 'T_i_j' in v:
            assert int(mask.sum()) == v['inliers']
            assert np.isclose(min(source_overlap, target_overlap), v['overlap'], atol=1e-12)
            assert np.isclose(rmse, v['rmse_m'], atol=1e-12)
        geometric_audit.append(dict(id=row['id'], accepted=v['accepted'], reason=v['reason'],
            ground_to_aerial_overlap=source_overlap, aerial_to_ground_overlap=target_overlap,
            symmetric_overlap=min(source_overlap, target_overlap), rmse_m=rmse,
            zero_height_initial_overlap=float((zero < 1.5).mean()),
            with_height_initial_overlap=float((initial < 1.5).mean()), residuals_exactly_reproduced=True))
    # GT access begins only after frozen geometric verification has been audited.
    evaluator_path = Path(__file__).with_name('evaluate_joint_bev_matches.py')
    spec = importlib.util.spec_from_file_location('bev_gt_evaluation', evaluator_path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    paths = {r: reference/f'{r}_gt.txt' for r in {key[0] for key in maps}}
    tracks = {r: module.track(path) for r, path in paths.items()}
    poses = {key: tracks[key[0]](m['stamp_ns']) for key, m in maps.items()}
    rows = []
    for row, audit in zip(data['results'], geometric_audit):
        q, c = tuple(row['query']), tuple(row['candidate']); truth = np.linalg.inv(poses[q])@poses[c]
        stages = dict(bev=row['bev_initial_T_i_j'], height_seed=row['height_seed_T_i_j'])
        if 'T_i_j' in row['verification']:
            stages['gicp'] = row['verification']['T_i_j']
        errors = {name: pose_error(T, truth, grounds[q], grounds[c]) for name, T in stages.items()}
        rows.append(dict(id=row['id'], accepted=row['verification']['accepted'], errors=errors,
                         geometric_audit=audit))
    accepted = [r for r in rows if r['accepted']]
    def summarize(key):
        values = [r['errors']['gicp'][key] for r in accepted]
        return dict(minimum=min(values), median=float(np.median(values)), maximum=max(values)) if values else None
    result = dict(ground_truth_used_only_for_evaluation=True, ground_truth_used_for_registration=False,
        thresholds_and_acceptance_unchanged=True, summary_sha256=before, ground_truth_sha256={str(p): sha(p) for p in paths.values()},
        evaluator_sha256=sha(Path(__file__)), interpolation_source_sha256=sha(evaluator_path),
        gt_interpolation_max_bracket_s=.05, pairs=rows, accepted=len(accepted),
        accepted_translation_error_m=summarize('translation_m'), accepted_rotation_error_deg=summarize('rotation_deg'),
        error_metric='relative transform error in original RTK/INS IMU frames; no trajectory alignment, not ATE',
        residuals_exactly_reproduced=True, pcm_run=False, cbs_run=False)
    assert sha(summary_path) == before
    (out/'evaluation.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    shutil.copy2(Path(__file__), out/'source'/Path(__file__).name)
    shutil.copy2(evaluator_path, out/'source'/evaluator_path.name)
    print(json.dumps({k: v for k, v in result.items() if k != 'pairs'}, indent=2))
    for row in rows:
        print(row['id'], row['accepted'], row['errors'].get('gicp'), row['geometric_audit'], flush=True)


if __name__ == '__main__':
    main()
