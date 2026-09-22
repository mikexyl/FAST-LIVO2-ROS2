#!/usr/bin/env python3
"""Evaluate already-frozen joint-BEV matches. GT is accessed only here.

This does not change matching results, thresholds, descriptors or selected pairs.
The metrics are relative planar-pose diagnostics, not trajectory ATE.
"""
import argparse
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def track(path):
    rows = [line.split() for line in path.read_text().splitlines() if line.strip() and not line.startswith('#')]
    ns = np.array([int(Decimal(row[0])*10**9) for row in rows], dtype=np.int64)
    xyz = np.array([[float(v) for v in row[1:4]] for row in rows])
    quat = np.array([[float(v) for v in row[4:8]] for row in rows])
    assert np.all(np.diff(ns) > 0) and np.isfinite(xyz).all() and np.isfinite(quat).all()
    slerp = Slerp((ns-ns[0])/1e9, Rotation.from_quat(quat))
    def pose(stamp):
        j = int(np.searchsorted(ns, stamp))
        if j < len(ns) and ns[j] == stamp:
            position = xyz[j]
        else:
            if j == 0 or j == len(ns) or ns[j]-ns[j-1] > 50_000_000:
                raise ValueError('Missing GT within a 50 ms interpolation bracket')
            f = (stamp-ns[j-1])/(ns[j]-ns[j-1]); position = xyz[j-1]*(1-f)+xyz[j]*f
        T = np.eye(4); T[:3, 3] = position
        T[:3, :3] = slerp([(stamp-ns[0])/1e9]).as_matrix()[0]
        return T
    return pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args(); out = args.preview.resolve()
    match_path = out/'matching.json'; original_hash = sha(match_path)
    data = json.loads(match_path.read_text()); assert data['complete'] and not data['ground_truth_used']
    assert not (out/'evaluation.json').exists(), 'Preserve the previous evaluation'
    tracks = {r: track(args.reference/f'{r}_gt.txt') for r in data['design']['endpoint_ids']}
    maps = {(m['robot'], m['key']): m for m in data['maps']}; poses, gravity = {}, {}
    for key, m in maps.items():
        poses[key] = tracks[key[0]](m['stamp_ns'])
        with np.load(out/'features'/f'{key[0]}-{key[1]:06d}-full.npz') as f:
            gravity[key] = f['ground'].copy()
    rows = []
    for pair in data['pairs']:
        q, c = tuple(pair['query']), tuple(pair['candidate'])
        truth = gravity[q]@np.linalg.inv(poses[q])@poses[c]@np.linalg.inv(gravity[c])
        yaw = np.arctan2(truth[1, 0], truth[0, 0])
        distance = float(np.linalg.norm(poses[q][:2, 3]-poses[c][:2, 3]))
        radius = maps[q]['area_radius_m']+maps[c]['area_radius_m']
        row = dict(id=pair['id'], horizontal_center_separation_m=distance,
                   conservative_nonoverlap=distance > radius+20., area_radii_sum_m=radius,
                   negative_separation_margin_m=distance-radius, methods={})
        for method, fit in pair['methods'].items():
            result = dict(passes_2d_gate=fit['passes_2d_gate'], inliers=fit['inliers'])
            if fit['valid_pose']:
                T = np.asarray(fit['T_query_candidate_level_2d'])
                angle = np.arctan2(T[1, 0], T[0, 0])-yaw
                xy = float(np.linalg.norm(T[:2, 2]-truth[:2, 3]))
                degrees = float(abs(np.degrees(np.arctan2(np.sin(angle), np.cos(angle)))))
                result.update(xy_translation_error_m=xy, yaw_error_deg=degrees,
                              within_diagnostic_5m_5deg=xy <= 5. and degrees <= 5.)
            row['methods'][method] = result
        rows.append(row)
    negatives = [r for r in rows if r['conservative_nonoverlap']]
    assert negatives, 'The fixed pair set contains no conservative negative controls'
    aggregate = {}
    for method in data['design']['comparisons']:
        passed = [r['methods'][method] for r in rows if r['methods'][method]['passes_2d_gate']]
        negative_passes = sum(r['methods'][method]['passes_2d_gate'] for r in negatives)
        inaccurate = sum(not r.get('within_diagnostic_5m_5deg', False) for r in passed)
        aggregate[method] = dict(pairs=len(rows), passes_2d_gate=len(passed),
            conservative_negative_pairs=len(negatives), negative_pairs_passing_2d_gate=negative_passes,
            negative_gate_pass_fraction=negative_passes/len(negatives),
            passing_poses_outside_5m_or_5deg=inaccurate,
            passing_poses_within_5m_and_5deg=len(passed)-inaccurate)
    evaluation = dict(ground_truth_used_only_for_evaluation=True, matching_sha256=original_hash,
        ground_truth_sha256={str(args.reference/f'{r}_gt.txt'): sha(args.reference/f'{r}_gt.txt') for r in tracks},
        script_sha256=sha(Path(__file__)), selected_pairs_unchanged=True, thresholds_unchanged=True,
        negative_definition='GT horizontal anchor separation > sum of 80 m area radii + 20 m safety margin',
        negative_label_limitation='Conservative area-footprint label allowing for odometry and gravity error; possible-overlap pairs are not automatically positive',
        pose_error_limitation='Planar seed errors only; 5 m / 5 degree bands summarize accuracy and do not accept or reject pipeline loops',
        gt_interpolation_max_bracket_s=.05, aggregate=aggregate, pairs=rows,
        not_a_retrieval_benchmark=True, gt_used_for_matching=False, registration_run=False, pcm_run=False, cbs_run=False)
    assert sha(match_path) == original_hash
    (out/'evaluation.json').write_text(json.dumps(evaluation, indent=2)+'\n')
    lines = ['# Evaluation of frozen joint multilayer BEV matches', '',
        f'All {len(rows)} pairs were fixed before matching. GT was loaded afterward in this separate evaluation process. {len(negatives)} pairs have horizontal anchor separation greater than the sum of their area radii plus 20 m and serve as conservative non-overlap controls.', '',
        'Other pairs may have intersecting footprints; that does not establish common visible surfaces. The 5 m / 5 degree diagnostic bands describe planar-seed error, not pipeline acceptance. No thresholds were tuned, no 3D registration was run, and no loops were added.', '',
        '| Method | Pass >5 gate | Non-overlap pairs passing | Passing poses within 5 m / 5° | Passing poses outside bands |',
        '|---|---:|---:|---:|---:|']
    for name, a in aggregate.items():
        lines.append(f'| {name} | {a["passes_2d_gate"]}/{len(rows)} | {a["negative_pairs_passing_2d_gate"]}/{len(negatives)} | {a["passing_poses_within_5m_and_5deg"]} | {a["passing_poses_outside_5m_or_5deg"]} |')
    lines += ['', '## Previously illustrated pairs', '', '| Pair | Method | Unique inliers | XY error [m] | Yaw error [deg] |', '|---|---|---:|---:|---:|']
    for row in rows:
        if row['id'] not in ['aerial06-27__ground06-17', 'aerial06-17__ground06-7']:
            continue
        for name in ['full', 'low', 'middle', 'pooled']:
            m = row['methods'][name]
            lines.append(f'| {row["id"]} | {name} | {m["inliers"]} | {m.get("xy_translation_error_m", float("nan")):.3f} | {m.get("yaw_error_deg", float("nan")):.3f} |')
    lines += ['', '## Conservative negative pairs', '', '| Pair | Separation [m] | Full inliers | Pooled inliers | Pooled passes |', '|---|---:|---:|---:|---|']
    for row in negatives:
        lines.append(f'| {row["id"]} | {row["horizontal_center_separation_m"]:.1f} | {row["methods"]["full"]["inliers"]} | {row["methods"]["pooled"]["inliers"]} | {row["methods"]["pooled"]["passes_2d_gate"]} |')
    lines += ['', '[Gallery](index.html) · [Matching method](README.md) · [Complete evaluation](evaluation.json)', '']
    (out/'evaluation-report.md').write_text('\n'.join(lines))
    shutil.copy2(Path(__file__), out/Path(__file__).name)
    print(json.dumps(aggregate, indent=2))


if __name__ == '__main__':
    main()
