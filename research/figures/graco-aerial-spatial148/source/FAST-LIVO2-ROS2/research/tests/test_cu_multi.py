"""Calibration, clock precision, sensor isolation and four-robot evaluation."""
from pathlib import Path
import sqlite3
import struct
import sys
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.cu_multi import (urdf_transform, position_ground_truth, header_stamp,
                                  sensor_messages, validate_cloud, extract_sensor_archive)
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth, comparison_plot
from s3e_pipeline.evo_evaluation import evaluate


def test_urdf_inverse_and_rotated_translation(tmp_path):
    path = tmp_path/'robot.urdf'
    path.write_text('''<robot><joint type="fixed"><parent link="sensor"/><child link="mount"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
        <joint type="fixed"><parent link="mount"/><child link="imu"/>
        <origin xyz="2 0 0"/></joint></robot>''')
    T = urdf_transform(path, 'sensor', 'imu')
    assert T[:3, 3] == pytest.approx([1, 4, 3])
    assert urdf_transform(path, 'imu', 'sensor') @ T == pytest.approx(np.eye(4))
    with pytest.raises(ValueError, match='No fixed calibration chain'):
        urdf_transform(path, 'unknown', 'imu')


def test_cu_multi_utm_lever_arm_and_shared_four_robot_alignment(tmp_path):
    robots = [f'robot{i}' for i in range(1, 5)]
    positions = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 1]], dtype=float)
    R = Rotation.from_euler('z', 90, degrees=True); q = R.as_quat()
    offset = np.eye(4); offset[:3, 3] = [1, 0, 0]
    epoch = 1722894200000000017
    rows = []
    cfg = dict(ground_truth_format='cu_multi_utm', T_reference_body={r: offset.tolist() for r in robots},
               gt_orientation_used_for_lever_arm=True)
    for i, robot in enumerate(robots):
        path = tmp_path/robot/f'{robot}_{tmp_path.name}_gt_utm_poses.csv'; path.parent.mkdir()
        text = '# timestamp,x,y,z,qx,qy,qz,qw\n'
        for k, xyz in enumerate(positions + [477144+i*4, 4428490, 1630]):
            stamp = epoch+k*10**9
            text += f'{stamp//10**9}.{stamp%10**9:09d},'+','.join(map(str, [*xyz, *q]))+'\n'
            T = np.eye(4); T[:3, 3] = xyz + R.apply(offset[:3, 3]) + [10, 20, 30]
            rows.append(dict(robot_id=robot, stamp_ns=stamp, component='robot1', T_world_body=T.tolist()))
        path.write_text(text)
        stamps, xyz = position_ground_truth(path, offset)
        assert stamps[0] == epoch
        assert xyz[0] == pytest.approx([477144+i*4, 4428491, 1630])
    truth, status = evaluation_ground_truth(robots, tmp_path, rows, cfg)
    assert all(s['available'] and s['orientation_used_for_lever_arm'] for s in status.values())
    metrics = evaluate(dict(poses=rows), truth, cfg, tmp_path/'evo')
    assert metrics['robot1']['rmse_m'] < 1e-8
    assert metrics['robot1']['samples'] == 16 and metrics['robot1']['scale'] == 1
    report = dict(trajectory=metrics, centralized_reference=None, components={r: 'robot1' for r in robots}, ground_truth=status)
    comparison_plot(dict(robots=robots, dataset=str(tmp_path), evaluation=dict(gt_max_gap_s=2)), rows, [], truth, report, tmp_path)
    assert (tmp_path/'trajectories.png').stat().st_size > 1000


def test_sensor_topic_allowlist_and_exact_header_nanoseconds(tmp_path):
    path = tmp_path/'input.db3'
    stamp = 1722894200000000017
    def payload(ns):return b'\x00\x01\x00\x00'+struct.pack('<iI', ns//10**9, ns%10**9)
    with sqlite3.connect(path) as db:
        db.executescript('CREATE TABLE topics(id INTEGER, name TEXT, type TEXT); CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        db.executemany('INSERT INTO topics VALUES(?,?,?)', [(1,'robot1/imu/data','sensor_msgs/msg/Imu'), (2,'robot1/ground_truth/odometry','nav_msgs/msg/Odometry')])
        db.executemany('INSERT INTO messages VALUES(?,?,?,?)', [(1,1,stamp+100,payload(stamp)), (2,2,stamp+200,b'not a sensor'), (3,1,stamp+10**7,payload(stamp+10**7))])
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(relative_file_paths=['input.db3']))))
    messages = list(sensor_messages(tmp_path, '/robot1/imu/data', 'sensor_msgs/msg/Imu'))
    assert [m[0] for m in messages] == [stamp, stamp+10**7]
    assert messages[0][3] == payload(stamp)
    assert header_stamp(payload(stamp)) == stamp
    with sqlite3.connect(path) as db:db.execute('INSERT INTO messages VALUES(4,1,?,?)',(stamp+2*10**7,payload(stamp)))
    with pytest.raises(ValueError, match='Nonmonotonic'):
        list(sensor_messages(tmp_path, '/robot1/imu/data', 'sensor_msgs/msg/Imu'))


def test_cloud_requires_native_point_timing():
    names = [('x',7),('y',7),('z',7),('intensity',7),('t',6),('reflectivity',4),('range',6)]
    fields = [SimpleNamespace(name=n, datatype=t, count=1, offset=i*4) for i,(n,t) in enumerate(names)]
    raw = bytearray(1000*32)
    for i in range(1000):struct.pack_into('<I', raw, i*32+16, i*50000)
    msg = SimpleNamespace(fields=fields, width=1000, height=1, point_step=32, row_step=32000,
                          data=raw, is_bigendian=False, header=SimpleNamespace(frame_id='robot1_os_sensor'))
    assert validate_cloud(msg)['point_time_max_ns'] == 49950000
    msg.fields = [f for f in fields if f.name != 't']
    with pytest.raises(ValueError, match='no synthetic point timing'):
        validate_cloud(msg)


def test_archive_rejects_labels_and_incomplete_cache(tmp_path):
    archive = tmp_path/'labels.zip'
    with zipfile.ZipFile(archive,'w') as z:z.writestr('0000000.bin', b'labels only')
    with pytest.raises(ValueError, match='lacks a ROS2 sensor bag'):
        extract_sensor_archive(archive, tmp_path/'out', min_free_gib=0)
    archive = tmp_path/'sensors.zip'
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr('bag/metadata.yaml','metadata');z.writestr('bag/input.db3',b'data')
    output = extract_sensor_archive(archive, tmp_path/'out', min_free_gib=0)
    assert extract_sensor_archive(archive, output, min_free_gib=0) == output
    (output/'input.db3').write_bytes(b'')
    with pytest.raises(ValueError, match='Incomplete extracted sensor bag'):
        extract_sensor_archive(archive, output, min_free_gib=0)


def test_native_ros2_staging_preserves_payloads_and_excludes_ground_truth(tmp_path):
    rosbag2 = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import String
    from s3e_pipeline.cu_multi import stage_sensors
    epoch = 1722894200000000017
    lidar = tmp_path/'lidar'; imu_bag = tmp_path/'imu'
    def writer(path, topics):
        w = rosbag2.SequentialWriter()
        w.open(rosbag2.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2.ConverterOptions('', ''))
        for name, kind in topics:
            w.create_topic(rosbag2.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
        return w
    w = writer(imu_bag, [('robot1/imu/data','sensor_msgs/msg/Imu'), ('robot1/ground_truth','std_msgs/msg/String')])
    for i in range(1000):
        msg = Imu(); stamp = epoch+i*2_000_000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = 'robot1_imu_link'; msg.linear_acceleration.z = 9.81
        w.write('robot1/imu/data', serialize_message(msg), stamp+1_000_000)
    w.write('robot1/ground_truth', serialize_message(String(data='must never reach the frontend')), epoch)
    del w
    w = writer(lidar, [('robot1/ouster/points','sensor_msgs/msg/PointCloud2')])
    for i in range(35):
        msg = PointCloud2(); stamp = epoch+100_000_000+i*50_000_000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = 'robot1_os_sensor'; msg.width = 1000; msg.height = 1
        msg.point_step = 32; msg.row_step = 32000
        raw = bytearray(msg.row_step)
        for k in range(1000):struct.pack_into('<I',raw,k*32+16,k*50000)
        msg.data = bytes(raw)
        msg.fields = [PointField(name=n, datatype=t, offset=j*4, count=1) for j,(n,t) in enumerate(
            [('x',7),('y',7),('z',7),('intensity',7),('t',6),('reflectivity',4),('range',6)])]
        w.write('robot1/ouster/points', serialize_message(msg), stamp+2_000_000)
    del w
    output = tmp_path/'staged'
    info = stage_sensors(lidar, imu_bag, output, 'robot1', 1.5)
    assert info['counts'] == {'/robot1/ouster/points': 28, '/robot1/imu/data': 750}
    original = list(sensor_messages(lidar, 'robot1/ouster/points','sensor_msgs/msg/PointCloud2'))[:28]
    staged = list(sensor_messages(output, '/robot1/ouster/points','sensor_msgs/msg/PointCloud2'))
    assert [(s[0],s[3]) for s in staged] == [(s[0],s[3]) for s in original]
    with sqlite3.connect(next(output.glob('*.db3'))) as db:
        assert {r[0] for r in db.execute('select name from topics')} == {'/robot1/ouster/points','/robot1/imu/data'}
    assert (output/'COMPLETE.json').exists()
