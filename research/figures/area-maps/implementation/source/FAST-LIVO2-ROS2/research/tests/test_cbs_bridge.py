from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.cbs_bridge import local_graph, loop_for_robot
from s3e_pipeline.geometry import constraint, reverse_constraint, inv, pose


def test_local_gauge_and_integer_timestamps():
    origin = np.eye(4); origin[:3, :3] = Rotation.from_rotvec([.3, -.7, .4]).as_matrix()
    origin[:3, 3] = [50., -12., 8.]
    second = np.eye(4); second[:3, 3] = [2., 1., -.4]
    rows = [dict(robot_id='Alpha', keyframe_id=i, stamp_ns=1700000000000000001+i,
                 T_world_body=(origin @ x).tolist()) for i, x in enumerate((np.eye(4), second))]
    nodes, edges = local_graph(rows, 'Alpha', dict(odometry_rotation_sigma_deg=1., odometry_translation_sigma_m=.15))
    assert np.allclose(nodes[0]['T_world_body'], np.eye(4))
    assert np.allclose(nodes[1]['T_world_body'], second)
    assert nodes[1]['stamp_ns'] == 1700000000000000002
    assert np.allclose(edges[0]['T_i_j'], second)
    with pytest.raises(ValueError): local_graph(rows, 'Bob', {})
    rows[0]['stamp_ns'] = float(rows[0]['stamp_ns'])
    with pytest.raises(ValueError): local_graph(rows, 'Alpha', {})


def test_reciprocal_covariance_and_incident_only():
    T = np.eye(4); T[:3, :3] = Rotation.from_rotvec([.5, -.1, .3]).as_matrix(); T[:3, 3] = [3, 2, 1]
    A = np.arange(36).reshape(6,6)/50; information = A.T@A+np.eye(6)
    edge = constraint(['Alpha', 4], ['Bob', 7], T, information, kind='loop', accepted=True)
    forward = loop_for_robot(edge, 'Alpha'); reverse = loop_for_robot(reverse_constraint(edge), 'Bob')
    assert np.allclose(forward['T_i_j'], reverse['T_i_j'])
    assert np.allclose(forward['information'], reverse['information'])
    assert np.linalg.norm(np.linalg.inv(information)-np.diag(np.diag(np.linalg.inv(information)))) > .1
    with pytest.raises(ValueError): loop_for_robot(edge, 'Carol')


def test_ros_serialization_contract():
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message, deserialize_message
    from pose_graph_tools_msgs.msg import PoseGraph
    from s3e_pipeline.cbs_bridge import ros_graph, matrix_pose
    T = np.eye(4); T[:3,3] = [1,2,3]
    info = np.diag([2.,3.,4.,5.,6.,7.]); info[0,4] = info[4,0] = .4
    edge = constraint(['Alpha',0],['Bob',1],T,info,kind='loop',accepted=True)
    node = dict(robot_id='Alpha',keyframe_id=0,stamp_ns=1700000000000000001,T_world_body=T.tolist())
    message = deserialize_message(serialize_message(ros_graph([node],[edge],['Alpha','Bob'])),PoseGraph)
    assert message.nodes[0].header.stamp.nanosec == 1
    assert np.allclose(matrix_pose(message.edges[0].pose),T)
    assert np.allclose(np.asarray(message.edges[0].covariance).reshape(6,6),np.linalg.inv(info))
