"""Loop measurements must obey the GTSAM endpoint coordinate convention."""
import numpy as np
from scipy.spatial.transform import Rotation
from cslam.lidar_pr import icp_utils

def matrix(msg):
    T=np.eye(4)
    T[:3,:3]=Rotation.from_quat([msg.rotation.x,msg.rotation.y,msg.rotation.z,msg.rotation.w]).as_matrix()
    T[:3,3]=[msg.translation.x,msg.translation.y,msg.translation.z]
    return T

def test_pose_factor_direction_with_rotation(monkeypatch):
    expected=np.eye(4)
    expected[:3,:3]=Rotation.from_euler('xyz',[.1,-.2,.3]).as_matrix()
    expected[:3,3]=[1.,2.,.3]
    cloud_alignment=np.linalg.inv(expected)
    monkeypatch.setattr(icp_utils,'solve_teaser',lambda *args:(True,cloud_alignment[:3,3],cloud_alignment[:3,:3]))
    msg,success=icp_utils.compute_transform(None,None,.1,60)
    assert success
    np.testing.assert_allclose(matrix(msg),expected,atol=1e-12)

def test_real_teaser_and_reversed_endpoints():
    rng=np.random.default_rng(6)
    a=rng.uniform([-6,-4,-2],[6,4,2],(500,3));translation=np.array([1.,2.,.3])
    b=a-translation
    pc0,pc1=icp_utils.downsample(a,.1),icp_utils.downsample(b,.1)
    forward,ok0=icp_utils.compute_transform(pc0,pc1,.1,60)
    reverse,ok1=icp_utils.compute_transform(pc1,pc0,.1,60)
    assert ok0 and ok1
    expected=np.eye(4);expected[:3,3]=translation
    np.testing.assert_allclose(matrix(forward),expected,atol=1e-5)
    np.testing.assert_allclose(matrix(reverse)@matrix(forward),np.eye(4),atol=1e-5)
