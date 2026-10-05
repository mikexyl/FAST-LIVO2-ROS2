"""Separate dense odometry motion from motion introduced by graph correction."""
import numpy as np
from scipy.spatial.transform import Rotation


def motion_series(raw, corrected):
    if len(raw) != len(corrected) or len(raw) < 2:
        raise ValueError('Continuity audit needs equal trajectories with at least two poses')
    if any((a['robot_id'], a['stamp_ns']) != (b['robot_id'], b['stamp_ns'])
           for a, b in zip(raw, corrected)):
        raise ValueError('Raw/corrected robot identities and timestamps must match exactly')
    stamps = np.asarray([r['stamp_ns'] for r in raw], dtype=np.int64)
    dt = np.diff(stamps).astype(float) * 1e-9
    if np.any(dt <= 0):
        raise ValueError('Continuity audit requires strictly chronological poses')
    a = np.asarray([r['T_world_body'] for r in raw], dtype=float)
    b = np.asarray([r['T_world_body'] for r in corrected], dtype=float)
    if a.shape != (len(raw), 4, 4) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Continuity audit requires finite SE(3) poses')
    correction_rotations = b[:, :3, :3] @ a[:, :3, :3].transpose(0, 2, 1)
    raw_delta = np.diff(a[:, :3, 3], axis=0)
    corrected_delta = np.diff(b[:, :3, 3], axis=0)
    # Advance the next raw position with the *previous* correction. Subtracting
    # that expected motion isolates the contribution of changing correction C.
    expected_delta = np.einsum('nij,nj->ni', correction_rotations[:-1], raw_delta)
    added = np.linalg.norm(corrected_delta - expected_delta, axis=1)
    angular = Rotation.from_matrix(correction_rotations[:-1].transpose(0, 2, 1) @
                                   correction_rotations[1:]).magnitude()
    return dict(stamp_ns=stamps[:-1], elapsed_s=(stamps[:-1]-stamps[0]).astype(float)*1e-9,
                dt_s=dt, raw_step_m=np.linalg.norm(raw_delta, axis=1),
                corrected_step_m=np.linalg.norm(corrected_delta, axis=1),
                correction_added_step_m=added,
                correction_angular_step_deg=np.rad2deg(angular))


def continuity_summary(raw, corrected):
    s = motion_series(raw, corrected)
    i = int(np.argmax(s['correction_added_step_m']))
    return dict(samples=len(raw), identical_timestamps=True,
        raw_max_step_m=float(max(s['raw_step_m'])),
        corrected_max_step_m=float(max(s['corrected_step_m'])),
        raw_max_speed_m_s=float(max(s['raw_step_m']/s['dt_s'])),
        corrected_max_speed_m_s=float(max(s['corrected_step_m']/s['dt_s'])),
        max_correction_added_step_m=float(s['correction_added_step_m'][i]),
        p99_correction_added_step_m=float(np.quantile(s['correction_added_step_m'], .99)),
        max_correction_angular_step_deg=float(max(s['correction_angular_step_deg'])),
        largest_added_step_stamp_ns=int(s['stamp_ns'][i]),
        largest_added_step_elapsed_s=float(s['elapsed_s'][i]),
        max_sample_gap_s=float(max(s['dt_s'])),
        definition='norm of corrected displacement minus raw displacement rotated by the previous correction; detects motion introduced by changing graph correction')
