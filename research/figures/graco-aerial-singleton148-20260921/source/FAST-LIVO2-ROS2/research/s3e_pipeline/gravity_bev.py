"""Gravity-horizontal BEVs; registration geometry remains in anchor IMU coordinates."""
import numpy as np


def gravity_ground(gravity_imu):
    """Return T_level_imu with +Z opposite gravity and horizontal projected body X.

    This is an orthographic projection frame, not a ground-plane height estimate.
    Translation stays zero: IMU gravity alone cannot determine terrain elevation.
    """
    gravity = np.asarray(gravity_imu, dtype=float)
    if gravity.shape != (3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity) < 1e-6:
        raise ValueError('Invalid anchor IMU gravity')
    up = -gravity / np.linalg.norm(gravity)
    heading = np.array([1., 0., 0.])
    if abs(up @ heading) > .99:
        heading = np.array([0., 1., 0.])
    x = heading - up * (up @ heading)
    x /= np.linalg.norm(x)
    result = np.eye(4)
    result[:3, :3] = np.vstack([x, np.cross(up, x), up])
    return result


def submap_gravity(row, sidecar=None):
    """Require native anchor gravity or an explicitly identified historical estimate."""
    if 'gravity_imu_m_s2' in row:
        gravity = row['gravity_imu_m_s2']
        provenance = dict(source=row['gravity_source'], stamp_ns=row['stamp_ns'], reconstructed=False)
        if sidecar is not None:
            raise ValueError('Cannot override native anchor gravity with a reconstruction')
        if 'gravity_world_m_s2' in row:
            rotation = np.asarray(row['T_world_imu']).reshape(4, 4)[:3, :3]
            if not np.allclose(rotation @ gravity, row['gravity_world_m_s2'], atol=1e-7):
                raise ValueError('Inconsistent world/IMU gravity')
    elif sidecar is not None:
        if sidecar['robot_id'] != row['robot_id'] or sidecar['ground_truth_used'] is not False:
            raise ValueError('Invalid gravity reconstruction provenance')
        entry = sidecar['submaps'][str(row['submap_id'])]
        if entry['stamp_ns'] != row['stamp_ns'] or entry['payload_sha256'] != row['sha256']:
            raise ValueError('Gravity reconstruction does not match submap payload/anchor')
        if sidecar['available_ns'] > row['begin_ns']:
            raise ValueError('Noncausal startup gravity estimate')
        gravity = entry['gravity_imu_m_s2']
        provenance = dict(source=sidecar['method'], reconstructed=True,
                          available_ns=sidecar['available_ns'], stamp_ns=row['stamp_ns'])
    else:
        raise ValueError('Submap has no anchor gravity; supply an explicit reconstruction '
                         'or select projection_alignment: local_ground for legacy behavior')
    transform = gravity_ground(gravity)
    provenance.update(gravity_imu_m_s2=list(gravity), projection='orthographic gravity-horizontal',
                      vertical_translation='zero; terrain height not estimated')
    return transform, provenance
