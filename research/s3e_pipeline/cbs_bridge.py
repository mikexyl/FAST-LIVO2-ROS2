"""FAST-LIVO2 -> CBS SE(3) contract. No ground truth or robot alignment."""
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import pose, inv, constraint, canonical_edge, TANGENT_ORDER


def named_pcm_verdicts(pcm, robots):
    """Match native numeric endpoint order to name-canonical Python edges."""
    result = {}
    for verdict in pcm['verdicts']:
        endpoints = ((robots[verdict['robot_from']], verdict['key_from']),
                     (robots[verdict['robot_to']], verdict['key_to']))
        key = tuple(sorted(endpoints))
        if key in result:
            raise ValueError('Duplicate PCM endpoint verdict')
        result[key] = verdict
    return result


def native_pair(endpoints, robots):
    """Order named endpoints by the IDs used for native factor ownership."""
    return tuple(sorted(endpoints, key=lambda e: (robots.index(e[0]), e[1])))


def local_graph(rows, robot, cfg, store=None, spatial_graph=None):
    if not rows or any(r['robot_id'] != robot for r in rows):
        raise ValueError('A publisher may load only its own nonempty trajectory')
    if [r['keyframe_id'] for r in rows] != list(range(len(rows))):
        raise ValueError('CBS requires consecutive local keys starting at zero')
    if any(type(r['stamp_ns']) is not int for r in rows):
        raise ValueError('Timestamps must be integer nanoseconds')
    if any(r.get('strategy') == 'fixed_spatial' for r in rows):
        from .circle_submaps import validate_row
        for row in rows: validate_row(row)
        if any(b['stamp_ns'] < a['stamp_ns'] for a,b in zip(rows,rows[1:])):
            raise ValueError('Spatial anchors must follow creation order')
        # Equal creation times are valid: identity is (robot, submap ID),
        # never the timestamp. Geometry remains in each creation-anchor frame.
    origin = inv(pose(rows[0]['T_world_body']))
    nodes = [dict(r, T_world_body=(origin @ pose(r['T_world_body'])).tolist()) for r in rows]
    sigmas = np.array([np.deg2rad(cfg['odometry_rotation_sigma_deg'])] * 3 +
                      [cfg['odometry_translation_sigma_m']] * 3)
    information = np.diag(1 / sigmas**2)
    edges = [constraint([robot, a['keyframe_id']], [robot, b['keyframe_id']],
                        inv(pose(a['T_world_body'])) @ pose(b['T_world_body']),
                        information, kind='odometry', accepted=True)
             for a, b in zip(rows, rows[1:])]
    if cfg.get('spatial_neighbors', {}).get('enabled', False):
        from .spatial_graph import SpatialGraph
        if spatial_graph is None:
            if store is None:
                raise ValueError('Spatial graph requires the owned sealed geometry store')
            spatial_graph = SpatialGraph(robot, store, cfg)
        spatial = spatial_graph.update(rows)
        edges = edges + spatial if spatial_graph.cfg.topology == 'creation_backbone' else spatial
    return nodes, edges


def loop_for_robot(edge, robot):
    if not edge.get('accepted') or edge.get('tangent_order') != TANGENT_ORDER:
        raise ValueError('Expected a verified constraint in GTSAM tangent order')
    edge = canonical_edge(constraint(edge['i'], edge['j'], edge['T_i_j'], edge['information'],
        **{k: v for k, v in edge.items() if k not in
           ('schema_version', 'i', 'j', 'T_i_j', 'information', 'tangent_order', 'perturbation')}))
    if robot not in (edge['i'][0], edge['j'][0]):
        raise ValueError('A robot cannot ingest an unrelated loop')
    return edge


def ros_pose(matrix):
    from geometry_msgs.msg import Pose
    T = pose(matrix); q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, T[:3, 3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
    return msg


def matrix_pose(msg):
    p, q = msg.position, msg.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return pose(T)


def ros_graph(nodes, edges, robots):
    from pose_graph_tools_msgs.msg import PoseGraph, PoseGraphNode, PoseGraphEdge
    msg = PoseGraph()
    for row in nodes:
        n = PoseGraphNode(robot_id=robots.index(row['robot_id']), key=row['keyframe_id'])
        n.header.stamp.sec, n.header.stamp.nanosec = divmod(row['stamp_ns'], 10**9)
        n.header.frame_id = row['robot_id']; n.pose = ros_pose(row['T_world_body'])
        msg.nodes.append(n)
    for edge in edges:
        e = PoseGraphEdge(robot_from=robots.index(edge['i'][0]), robot_to=robots.index(edge['j'][0]),
                          key_from=edge['i'][1], key_to=edge['j'][1])
        from .spatial_graph import MAP_CONSISTENCY_EDGE_TYPE
        kinds = {'odometry': e.ODOM, 'loop': e.LOOPCLOSE,
                 'map_consistency': MAP_CONSISTENCY_EDGE_TYPE}
        if edge['kind'] not in kinds:
            raise ValueError('Unknown CBS edge kind')
        if edge['kind'] == 'map_consistency' and edge['i'][0] != edge['j'][0]:
            raise ValueError('Map consistency must remain robot-local')
        e.type = kinds[edge['kind']]
        e.pose = ros_pose(edge['T_i_j'])
        # CBS's PoseGraphEdge uses [rotation, translation], NOT ROS
        # PoseWithCovariance's [translation, rotation]. Keep cross terms.
        e.covariance = np.linalg.inv(np.asarray(edge['information'])).reshape(-1).tolist()
        e.has_scale = False; e.scale = 1.; e.scale_sigma = -1.
        msg.edges.append(e)
    return msg
