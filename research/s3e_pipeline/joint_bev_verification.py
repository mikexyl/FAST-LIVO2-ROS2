"""Frame conversion for offline verification of gravity-level joint BEV fits."""
import numpy as np
from .geometry import pose


def lift_planar_pose(planar, query_ground, candidate_ground):
    """Lift T_query-level_candidate-level to T_query-IMU_candidate-IMU.

    BEV supplies yaw and horizontal translation only. Level-frame vertical
    translation starts at zero and is estimated by the existing geometry stage.
    Ground transforms map each endpoint's IMU coordinates into its own level
    coordinates; neither an odometry world pose nor ground truth is needed.
    """
    planar = np.asarray(planar, dtype=float)
    if planar.shape != (3, 3) or not np.isfinite(planar).all():
        raise ValueError('Planar seed must be a finite 3x3 transform')
    R = planar[:2, :2]
    if (not np.allclose(planar[2], [0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(R.T@R, np.eye(2), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(R), 1., atol=1e-8, rtol=0)):
        raise ValueError('Planar seed must be a proper rigid SE(2) transform')
    Gq, Gc = pose(query_ground), pose(candidate_ground)
    level = np.eye(4); level[:2, :2] = R; level[:2, 3] = planar[:2, 2]
    return pose(np.linalg.inv(Gq)@level@Gc)
