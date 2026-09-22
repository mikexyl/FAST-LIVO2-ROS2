import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.joint_bev_verification import lift_planar_pose


def test_lift_preserves_metric_level_pose_and_reverse_endpoints():
    Gq, Gc = np.eye(4), np.eye(4)
    Gq[:3, :3] = Rotation.from_euler('xyz', [.8, -.2, .4]).as_matrix()
    Gc[:3, :3] = Rotation.from_euler('xyz', [-.1, .4, -.6]).as_matrix()
    Gq[:3, 3], Gc[:3, 3] = [1, -2, 3], [-4, 5, -6]
    planar = np.eye(3); planar[:2, :2] = Rotation.from_euler('z', .9).as_matrix()[:2, :2]
    planar[:2, 2] = [17, -9]
    original = [a.copy() for a in [planar, Gq, Gc]]
    actual = lift_planar_pose(planar, Gq, Gc)
    level = Gq@actual@np.linalg.inv(Gc)
    np.testing.assert_allclose(level[:2, :2], planar[:2, :2], atol=1e-12)
    np.testing.assert_allclose(level[:3, 3], [17, -9, 0], atol=1e-12)
    reverse = lift_planar_pose(np.linalg.inv(planar), Gc, Gq)
    np.testing.assert_allclose(reverse, np.linalg.inv(actual), atol=1e-12)
    for a, b in zip([planar, Gq, Gc], original):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize('bad', [np.eye(4), np.full((3, 3), np.nan),
    np.diag([-1., 1., 1.]), np.diag([2., 2., 1.]), np.diag([1., 1., 2.])])
def test_invalid_planar_seed_is_rejected(bad):
    with pytest.raises(ValueError):
        lift_planar_pose(bad, np.eye(4), np.eye(4))
