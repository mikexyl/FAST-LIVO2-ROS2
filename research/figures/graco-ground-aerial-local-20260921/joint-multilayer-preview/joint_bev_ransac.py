"""Offline rigid SE(2) consensus for height-labelled BEV correspondences.

Inputs are metric, gravity-level map coordinates, not cropped image pixels.
The full-height control must be fitted separately from the disjoint slices.
No poses, terrain models, ground truth or registration priors enter this fit.
"""
import numpy as np


SETTINGS = dict(residual_m=1.5, duplicate_m=.5, minimum_baseline_m=.5,
                trials=688, seed=0, inliers_threshold=5, refinement_iterations=10)


def _points(value):
    p = np.asarray(value, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or not np.isfinite(p).all():
        raise ValueError('Correspondences must be finite Nx2 metric coordinates')
    return p


def pool_correspondences(packets, resolution_m=.5, duplicate_m=.5):
    """Keep the lowest-Hamming representative when BOTH endpoints repeat.

    Native xy already includes the density-map crop origin, in pixel units.
    An endpoint within one pixel of both endpoints of a retained match is
    considered redundant. Discarded memberships remain in duplicate_groups.
    """
    if 'full' in packets and len(packets) != 1:
        raise ValueError('Full-height control cannot be pooled with its slices')
    if resolution_m <= 0 or duplicate_m < 0:
        raise ValueError('Invalid coordinate resolution or duplicate tolerance')
    q, c, labels, distance, source = [], [], [], [], []
    for label, packet in packets.items():
        query = _points(packet['query_xy']) * resolution_m
        candidate = _points(packet['candidate_xy']) * resolution_m
        hamming = np.asarray(packet['hamming'], dtype=float)
        if query.shape != candidate.shape or hamming.shape != (len(query),):
            raise ValueError('Correspondence arrays have different lengths')
        if not np.isfinite(hamming).all():
            raise ValueError('Invalid descriptor distance')
        q.extend(query); c.extend(candidate)
        labels.extend([label] * len(query)); distance.extend(hamming)
        source.extend((label, i) for i in range(len(query)))
    q = np.asarray(q, dtype=float).reshape(-1, 2)
    c = np.asarray(c, dtype=float).reshape(-1, 2)
    hamming = np.asarray(distance, dtype=float)
    order = sorted(range(len(q)), key=lambda i: (hamming[i], labels[i], *q[i], *c[i]))
    kept, groups = [], []
    for i in order:
        duplicate = [j for j, k in enumerate(kept)
                     if np.linalg.norm(q[i]-q[k]) <= duplicate_m
                     and np.linalg.norm(c[i]-c[k]) <= duplicate_m]
        if duplicate:
            groups[duplicate[0]].append(source[i])
        else:
            kept.append(i); groups.append([source[i]])
    return dict(query_xy_m=q[kept], candidate_xy_m=c[kept],
                layer=np.asarray(labels, dtype=str)[kept], hamming=hamming[kept],
                duplicate_groups=groups, input_matches=len(q), duplicates_removed=len(q)-len(kept))


def rigid_fit(candidate, query):
    """Least-squares proper 2D rotation and translation, with no scale fit."""
    c, q = _points(candidate), _points(query)
    if len(c) < 2 or c.shape != q.shape:
        raise ValueError('A rigid fit requires at least two paired points')
    cm, qm = c.mean(axis=0), q.mean(axis=0)
    a, b = c-cm, q-qm
    cosine = np.sum(a*b)
    sine = np.sum(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])
    if np.hypot(cosine, sine) < 1e-12:
        raise ValueError('Degenerate rigid fit')
    angle = np.arctan2(sine, cosine)
    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    T = np.eye(3); T[:2, :2] = R; T[:2, 2] = qm-R@cm
    return T


def residuals(T, candidate, query):
    return np.linalg.norm(candidate@T[:2, :2].T+T[:2, 2]-query, axis=1)


def unique_support(pool, errors, threshold_m, duplicate_m):
    """Prevent many-to-one endpoint matches from inflating consensus support."""
    ids = np.flatnonzero(errors < threshold_m)
    ids = ids[np.lexsort((ids, pool['hamming'][ids], errors[ids]))]
    q, c = pool['query_xy_m'], pool['candidate_xy_m']
    kept = []
    for i in ids:
        if kept and (np.any(np.linalg.norm(q[kept]-q[i], axis=1) <= duplicate_m)
                     or np.any(np.linalg.norm(c[kept]-c[i], axis=1) <= duplicate_m)):
            continue
        kept.append(int(i))
    return np.asarray(sorted(kept), dtype=int)


def fit_consensus(pool, settings=None):
    cfg = dict(SETTINGS)
    if settings:
        cfg.update(settings)
    if cfg['residual_m'] <= 0 or cfg['duplicate_m'] < 0 or cfg['trials'] < 1:
        raise ValueError('Invalid consensus settings')
    q, c = _points(pool['query_xy_m']), _points(pool['candidate_xy_m'])
    labels = np.asarray(pool['layer']); n = len(q)
    if c.shape != q.shape or labels.shape != (n,) or np.asarray(pool['hamming']).shape != (n,):
        raise ValueError('Invalid pool dimensions')
    result = dict(valid_pose=False, matches=n, input_matches=pool['input_matches'],
                  duplicates_removed=pool['duplicates_removed'], inliers=0,
                  inlier_indices=[], passes_2d_gate=False, settings=cfg,
                  T_query_candidate_level_2d=None, layer_support={}, residual_rmse_m=None)
    if n < 3:
        return result
    bins = [np.flatnonzero(labels == label) for label in sorted(set(labels))]
    rng = np.random.default_rng(cfg['seed'])
    best_ids = np.empty(0, dtype=int); best_score = (-1, -np.inf); best_T = None
    for _ in range(cfg['trials']):
        # Each draw selects a layer uniformly, then a match within that layer.
        i = int(rng.choice(bins[int(rng.integers(len(bins)))]))
        j = int(rng.choice(bins[int(rng.integers(len(bins)))]))
        if i == j or min(np.linalg.norm(c[i]-c[j]), np.linalg.norm(q[i]-q[j])) < cfg['minimum_baseline_m']:
            continue
        try:
            T = rigid_fit(c[[i, j]], q[[i, j]])
        except ValueError:
            continue
        error = residuals(T, c, q)
        # An inexpensive upper bound avoids packing clearly inferior hypotheses.
        if np.count_nonzero(error < cfg['residual_m']) < len(best_ids):
            continue
        ids = unique_support(pool, error, cfg['residual_m'], cfg['duplicate_m'])
        score = (len(ids), -float(np.mean(error[ids]**2)) if len(ids) else -np.inf)
        if score > best_score:
            best_ids, best_score, best_T = ids, score, T
    if len(best_ids) < 3:
        return result
    T, ids = best_T, best_ids
    for _ in range(cfg['refinement_iterations']):
        try:
            refined = rigid_fit(c[ids], q[ids])
        except ValueError:
            break
        refined_ids = unique_support(pool, residuals(refined, c, q), cfg['residual_m'], cfg['duplicate_m'])
        if len(refined_ids) < 3:
            break
        stable = np.array_equal(ids, refined_ids)
        T, ids = refined, refined_ids
        if stable:
            break
    error = residuals(T, c, q)
    assert np.all(error[ids] < cfg['residual_m'])
    assert np.allclose(T[:2, :2].T@T[:2, :2], np.eye(2)) and np.isclose(np.linalg.det(T[:2, :2]), 1.)
    spread = {}
    for name, points in [('query', q[ids]), ('candidate', c[ids])]:
        spread[name+'_minor_major_std_m'] = np.sqrt(np.maximum(0., np.linalg.eigvalsh(np.cov(points.T)))).tolist()
        spread[name+'_occupied_5m_cells'] = len(np.unique(np.floor(points/5), axis=0))
    result.update(valid_pose=True, inliers=len(ids), inlier_indices=ids.tolist(),
                  passes_2d_gate=len(ids) > cfg['inliers_threshold'],
                  T_query_candidate_level_2d=T.tolist(),
                  layer_support={label: int(np.sum(labels[ids] == label)) for label in sorted(set(labels))},
                  residual_rmse_m=float(np.sqrt(np.mean(error[ids]**2))),
                  residual_max_m=float(error[ids].max()), spatial_support=spread)
    return result
