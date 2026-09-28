"""Rigid submap registration against native EllipseLIO tensor ellipsoids.

The native odometry projects a scan point onto a weighted plane, line and
point at the nearest fitted map representative. Here both maps supply native
fitted ellipsoids; their centers are the measurements in each direction.
The exported axes encode the relative tensor eigenvalues, so they recover
the same three projection weights without fitting new geometry from points.
"""
import numpy as np
from scipy.spatial import cKDTree
from .geometry import pose, transform


def validate(ellipsoids):
    e = np.asarray(ellipsoids, dtype=np.float64)
    if e.ndim != 2 or e.shape[1] != 15 or not np.isfinite(e).all():
        raise ValueError('Expected finite native ellipsoids with 15 columns')
    if len(e) and (np.any(e[:, 3:6] <= 0) or
            np.max(np.abs(e[:, 6:].reshape(-1, 3, 3).transpose(0, 2, 1) @
                          e[:, 6:].reshape(-1, 3, 3) - np.eye(3))) > 1e-3):
        raise ValueError('Invalid native ellipsoid axes or basis')
    return e


def bounded_ellipsoids(ellipsoids, voxel_m=.4, max_count=50000):
    """Keep original ellipsoids, one per occupied center voxel, with a cap."""
    e = validate(ellipsoids)
    if not np.isfinite(voxel_m) or voxel_m <= 0 or type(max_count) is not int or max_count < 3:
        raise ValueError('Invalid ellipsoid evidence budget')
    if not len(e): return e.copy()
    cells = np.floor(e[:, :3] / voxel_m).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    indices.sort()
    if len(indices) > max_count:
        indices = indices[np.floor(np.arange(max_count)*len(indices)/max_count).astype(int)]
    return np.ascontiguousarray(e[indices])


def projectors(e):
    """Reconstruct EllipseLIO's plane/line/ball weights from inverse axes."""
    axes = e[:, 3:6]
    inv = 1. / axes
    if len(e) and np.any(np.diff(inv, axis=1) < -1e-5*np.max(inv, axis=1, keepdims=True)):
        raise ValueError('Native ellipsoid axes have inconsistent tensor order')
    saliency = np.maximum(np.column_stack((inv[:, 2]-inv[:, 1], inv[:, 1]-inv[:, 0], inv[:, 0])), 0.)
    saliency /= saliency.sum(axis=1, keepdims=True)
    basis = e[:, 6:].reshape(-1, 3, 3)
    line = basis[:, :, 0]
    normal = basis[:, :, 2]
    eye = np.eye(3)
    return (saliency[:, 0, None, None]*normal[:, :, None]*normal[:, None, :] +
            saliency[:, 1, None, None]*(eye-line[:, :, None]*line[:, None, :]) +
            saliency[:, 2, None, None]*eye)


def correspondences(target, tree, P, source_centers, T, radius):
    moved = transform(T, source_centers)
    distance, index = tree.query(moved, workers=1)
    mask = distance < radius
    ids = np.flatnonzero(mask)
    if not len(ids): return ids, np.empty((0, 3)), np.empty((0, 3, 3)), moved
    Q = P[index[ids]]
    residual = np.einsum('nij,nj->ni', Q, moved[ids]-target[index[ids]])
    return ids, residual, Q, moved


def right_jacobian(source_centers, rotation, projector):
    """Primitive residual Jacobian in source-frame [rotation, translation]."""
    n = len(source_centers)
    cross = np.cross(np.broadcast_to(np.eye(3), (n, 3, 3)),
                     source_centers[:, None, :]).transpose(0, 2, 1)
    d_rotation = rotation @ cross
    d_translation = np.broadcast_to(rotation, d_rotation.shape)
    return np.concatenate((projector @ d_rotation, projector @ d_translation), axis=2)


def refine_ellipsoids(target, source, initial, cfg):
    """Return a source-to-target loop measurement or a reason for rejection."""
    if initial is None: return dict(accepted=False, reason='no_pose_initialization')
    T = pose(initial).copy()
    target = validate(target); source = validate(source)
    sizes = dict(target_ellipsoids=len(target), source_ellipsoids=len(source),
                 registration_method='native_ellipsoid_primitives')
    if min(len(target), len(source)) < cfg['min_inliers']:
        return dict(accepted=False, reason='insufficient_ellipsoids', **sizes)
    tc, sc = target[:, :3], source[:, :3]
    ttree = cKDTree(tc); stree = cKDTree(sc)
    Pt, Ps = projectors(target), projectors(source)
    radius = cfg['correspondence_m']
    ids, residual, Q, moved = correspondences(tc, ttree, Pt, sc, T, radius)
    initial_overlap = len(ids)/len(sc)
    if initial_overlap < cfg.get('min_initial_overlap', .1):
        return dict(accepted=False, reason='initial_low_overlap',
                    initial_overlap=float(initial_overlap), **sizes)

    def linearize(transform_now):
        selected, residual_now, projector, moved_now = correspondences(
            tc, ttree, Pt, sc, transform_now, radius)
        if not len(selected): return None
        # Right perturbation of T_target_source, in source-frame GTSAM
        # rotation/translation tangent order: R[-skew(p), I].
        J = right_jacobian(sc[selected], transform_now[:3, :3], projector)
        norm = np.linalg.norm(residual_now, axis=1)
        weight = np.minimum(1., cfg['inlier_m']/np.maximum(norm, 1e-12))
        return selected, residual_now, J, weight

    converged = False; iterations = 0
    for iteration in range(cfg.get('max_iterations', 40)):
        linear = linearize(T)
        if linear is None or len(linear[0]) < 6: break
        _, r, J, weight = linear
        H = np.einsum('nki,nkj,n->ij', J, J, weight)
        g = np.einsum('nki,nk,n->i', J, r, weight)
        if not np.isfinite(H).all() or not np.isfinite(g).all(): break
        step = -np.linalg.solve(H + np.eye(6)*max(np.trace(H)*1e-6, 1e-6), g)
        if not np.isfinite(step).all(): break
        scale = min(1., .2/max(np.linalg.norm(step[:3]), 1e-12),
                    1./max(np.linalg.norm(step[3:]), 1e-12))
        step *= scale
        from scipy.spatial.transform import Rotation
        old_cost = np.sum(weight*np.sum(r*r, axis=1))/len(r)
        matched = ttree.query(transform(T, sc[linear[0]]), workers=1)[1]
        fixed_centers, fixed_projectors = tc[matched], Pt[matched]
        accepted_step = None
        for fraction in (1., .5, .25, .125, .0625, .03125):
            scaled = fraction*step
            candidate = T.copy()
            candidate[:3, :3] = T[:3, :3] @ Rotation.from_rotvec(scaled[:3]).as_matrix()
            candidate[:3, 3] += T[:3, :3] @ scaled[3:]
            # Recompute correspondences; a fixed-pair descent may lose the
            # overlapping region after the nearest primitives are refreshed.
            trial = linearize(candidate)
            if trial is None or len(trial[0]) < .8*len(linear[0]): continue
            fixed_residual = np.einsum('nij,nj->ni', fixed_projectors,
                transform(candidate, sc[linear[0]])-fixed_centers)
            new_cost = np.sum(weight*np.sum(fixed_residual*fixed_residual, axis=1))/len(r)
            if new_cost <= old_cost + 1e-9:
                accepted_step = candidate, scaled, new_cost
                break
        if accepted_step is None:
            if np.linalg.norm(step) < 1e-3: converged = True
            break
        T, applied, updated_cost = accepted_step; iterations = iteration+1
        if np.linalg.norm(applied) < 1e-4 or (old_cost-updated_cost)/max(old_cost, 1e-12) < 1e-6:
            converged = True; break
    else:
        converged = True

    forward = linearize(T)
    if forward is None:
        return dict(accepted=False, reason='no_final_correspondences', initial_overlap=float(initial_overlap), **sizes)
    _, f_residual, J, _ = forward
    # The reverse direction uses source primitives, preventing a dense map from
    # hiding a small common area on the other map.
    inverse = np.linalg.inv(T)
    _, b_residual, _, _ = correspondences(sc, stree, Ps, tc, inverse, radius)
    f_norm = np.linalg.norm(f_residual, axis=1)
    b_norm = np.linalg.norm(b_residual, axis=1)
    f_good = f_norm < cfg['inlier_m']; b_good = b_norm < cfg['inlier_m']
    count = int(f_good.sum()); overlap = min(float(count/len(sc)), float(b_good.sum()/len(tc)))
    rmse = float(np.sqrt(np.mean(np.r_[f_norm[f_good]**2, b_norm[b_good]**2]))) if (count or b_good.any()) else None
    H = np.einsum('nki,nkj->ij', J[f_good], J[f_good])/max(count, 1)
    eigen = np.linalg.eigvalsh(H)
    condition = float(eigen[-1]/max(eigen[0], 1e-15))
    reason = 'accepted'
    if not converged: reason = 'not_converged'
    if count < cfg['min_inliers']: reason = 'insufficient_inliers'
    elif overlap < cfg['min_overlap']: reason = 'low_overlap'
    elif rmse is None or rmse > cfg['max_rmse_m']: reason = 'high_rmse'
    elif eigen[0] < cfg['min_observability'] or condition > cfg['max_condition']: reason = 'unobservable'
    if not np.isfinite(T).all() or not np.isfinite(H).all() or not np.isfinite(rmse or 0):
        return dict(accepted=False, reason='nonfinite_solution', **sizes)
    noise = max(rmse or cfg['translation_sigma_floor_m'], cfg['translation_sigma_floor_m'])
    covariance = np.linalg.inv(H/noise**2 + np.eye(6)*1e-12)
    covariance += np.diag([np.deg2rad(cfg['rotation_sigma_floor_deg'])**2]*3 +
                          [cfg['translation_sigma_floor_m']**2]*3)
    information = np.linalg.inv(covariance)
    return dict(accepted=reason=='accepted', reason=reason, T_i_j=T.tolist(),
                information=information.tolist(), initial_overlap=float(initial_overlap),
                converged=converged, iterations=iterations, inliers=count,
                reverse_inliers=int(b_good.sum()), overlap=overlap, rmse_m=rmse,
                observability_eigenvalues=eigen.tolist(), condition=condition,
                uncertainty='right tangent native primitive projection Hessian, conservative covariance floors',
                **sizes)
