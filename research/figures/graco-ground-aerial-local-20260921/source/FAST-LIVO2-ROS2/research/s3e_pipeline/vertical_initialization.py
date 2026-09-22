"""Estimate the vertical translation missing from a gravity-horizontal BEV pose.

Only the two candidate payloads are used. This estimates a seed, not a loop
acceptance decision; the normal geometric verifier and PCM remain responsible
for accepting measurements.
"""
import time
import numpy as np
from scipy.spatial import cKDTree
from .geometry import pose, transform
from .registration import bounded_cloud


DEFAULTS = dict(voxel_m=.8, xy_neighbors=8, xy_radius_m=1., histogram_bin_m=.5,
                peak_separation_m=2., modal_peaks=5, coarse_inlier_m=1.5)


def initialize_vertical(target, source, initial, target_ground, source_ground, options=None):
    """Return T_target_source and diagnostics without modifying input geometry.

    The returned transform preserves rotation and leveled XY translation. A
    zero correction is always considered, including when the input seed already
    has a vertical component. Empty/nonfinite-only geometry and no horizontal
    correspondences leave the seed unchanged for the normal verifier to reject.
    """
    started = time.monotonic()
    cfg = dict(DEFAULTS)
    cfg.update({k:v for k,v in (options or {}).items() if k != 'enabled'})
    if set(cfg) != set(DEFAULTS):
        raise ValueError('Unknown vertical-initialization option')
    for key in ('voxel_m', 'xy_radius_m', 'histogram_bin_m', 'peak_separation_m', 'coarse_inlier_m'):
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'Invalid vertical-initialization {key}')
    for key in ('xy_neighbors', 'modal_peaks'):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f'Invalid vertical-initialization {key}')
    initial = pose(initial)
    Gq, Gc = pose(target_ground), pose(source_ground)
    level = Gq @ initial @ np.linalg.inv(Gc)
    if not np.allclose(level[2, :3], [0, 0, 1], atol=1e-5):
        raise ValueError('Vertical initialization requires a gravity-planar pose')
    diagnostic = dict(method='XY-neighbor height voting and symmetric coarse 3D overlap',
                      settings=cfg, ground_truth_used=False, input_level_dz_m=float(level[2, 3]))

    def finish(correction, status, **values):
        adjusted = initial.copy()
        adjusted[:3, 3] += Gq[:3, :3].T @ np.array([0., 0., correction])
        diagnostic.update(status=status, correction_m=float(correction),
                          output_level_dz_m=float(level[2, 3]+correction),
                          runtime_s=time.monotonic()-started, **values)
        return adjusted, diagnostic

    def prepare(points, ground):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('Vertical initialization requires Nx3 geometry')
        points = points[np.isfinite(points).all(axis=1)]
        return bounded_cloud(transform(ground, points), cfg['voxel_m'],
                             max_points=None, max_range=None)[0]

    t, s = prepare(target, Gq), prepare(source, Gc)
    diagnostic.update(target_points=len(t), source_points=len(s))
    if min(len(t), len(s)) == 0:
        return finish(0., 'insufficient_geometry', height_vote_pairs=0, hypotheses=[])
    s = transform(level, s)
    k = min(cfg['xy_neighbors'], len(t))
    distance, index = cKDTree(t[:, :2]).query(s[:, :2], k=k, workers=1)
    distance, index = distance.reshape(len(s), k), index.reshape(len(s), k)
    valid = distance < cfg['xy_radius_m']
    count = int(valid.sum())
    if not count:
        return finish(0., 'no_horizontal_support', height_vote_pairs=0, hypotheses=[])
    offsets = (t[index, 2]-s[:, 2, None])[valid]
    centers, counts = np.unique(np.rint(offsets/cfg['histogram_bin_m']).astype(np.int64), return_counts=True)
    peaks = []
    for j in np.argsort(-counts, kind='stable'):
        z = float(centers[j]*cfg['histogram_bin_m'])
        if all(abs(z-x) > cfg['peak_separation_m'] for x in peaks):
            peaks.append(z)
        if len(peaks) == cfg['modal_peaks']:
            break
    tree_t, tree_s = cKDTree(t), cKDTree(s)
    hypotheses = []
    for z in sorted(set([0., *peaks])):
        shift = np.array([0., 0., z])
        forward = float((tree_t.query(s+shift, workers=1)[0] < cfg['coarse_inlier_m']).mean())
        reverse = float((tree_s.query(t-shift, workers=1)[0] < cfg['coarse_inlier_m']).mean())
        hypotheses.append(dict(correction_m=z, coarse_symmetric_overlap=min(forward, reverse)))
    best = max(hypotheses, key=lambda x:(x['coarse_symmetric_overlap'], -abs(x['correction_m'])))
    return finish(best['correction_m'], 'estimated', height_vote_pairs=count,
                  hypotheses=hypotheses, coarse_symmetric_overlap=best['coarse_symmetric_overlap'])
