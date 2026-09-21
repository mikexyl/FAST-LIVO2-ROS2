"""CU-Multi sensor and calibration adapter. GT is read only by evaluation."""
from decimal import Decimal
import heapq
from pathlib import Path
import shutil
import sqlite3
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .artifacts import file_hash, read_json, write_json
from .geometry import inv

ROBOTS = ('robot1', 'robot2', 'robot3', 'robot4')


def urdf_transform(path, target, source):
    """T_target_source maps source coordinates into target, using fixed joints."""
    graph = {}
    root = ET.parse(path).getroot()
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent = joint.find('parent').get('link'); child = joint.find('child').get('link')
        origin = joint.find('origin')
        attrs = {} if origin is None else origin.attrib
        T = np.eye(4)
        T[:3, 3] = [float(x) for x in attrs.get('xyz', '0 0 0').split()]
        T[:3, :3] = Rotation.from_euler('xyz', [float(x) for x in attrs.get('rpy', '0 0 0').split()]).as_matrix()
        graph.setdefault(parent, []).append((child, T))
        graph.setdefault(child, []).append((parent, inv(T)))
    pending = [(target, np.eye(4))]; seen = set()
    while pending:
        frame, T = pending.pop()
        if frame == source:
            return T
        if frame in seen:
            continue
        seen.add(frame)
        pending.extend((child, T @ edge) for child, edge in graph.get(frame, []) if child not in seen)
    raise ValueError(f'No fixed calibration chain from {target} to {source}')


def position_ground_truth(path, T_reference_body):
    """CSV LiDAR reference poses -> IMU positions; retain common UTM frame/time."""
    T = np.asarray(T_reference_body, dtype=float).reshape(4, 4)
    if not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError('Invalid reference-to-body calibration')
    stamps = []; positions = []
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        cells = line.split(',')
        if len(cells) != 8:
            raise ValueError('CU-Multi GT requires timestamp,x,y,z,qx,qy,qz,qw')
        ns = Decimal(cells[0].strip()) * 10**9
        if ns != ns.to_integral_value():
            raise ValueError('GT timestamp exceeds nanosecond precision')
        values = np.asarray(cells[1:], dtype=float)
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 1e-3:
            raise ValueError('Invalid CU-Multi reference pose')
        stamps.append(int(ns))
        positions.append(values[:3] + Rotation.from_quat(values[3:]).apply(T[:3, 3]))
    stamps = np.asarray(stamps, dtype=np.int64)
    if len(stamps) < 3 or np.any(np.diff(stamps) <= 0):
        raise ValueError('CU-Multi reference timestamps must be strictly increasing')
    return stamps, np.asarray(positions)


def extract_sensor_archive(archive, output, min_free_gib=100):
    """Extract just a ROS2 sensor bag, with CRC checks and an atomic marker."""
    archive = Path(archive); output = Path(output)
    marker = output/'EXTRACTED.json'
    if marker.exists():
        saved = read_json(marker)
        if saved['archive_bytes'] != archive.stat().st_size or saved['archive_mtime_ns'] != archive.stat().st_mtime_ns:
            raise ValueError('Sensor archive changed after extraction')
        if any(not (output/n).is_file() or (output/n).stat().st_size != size for n, size in saved['files'].items()):
            raise ValueError('Incomplete extracted sensor bag')
        return output
    if output.exists():
        raise ValueError(f'Incomplete extraction exists: {output}; choose a fresh input directory')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name+'.incomplete')
    if temporary.exists():
        raise ValueError(f'Incomplete extraction exists: {temporary}')
    with zipfile.ZipFile(archive) as bundle:
        files = [i for i in bundle.infolist() if i.filename.endswith(('.db3', 'metadata.yaml'))]
        if not any(i.filename.endswith('metadata.yaml') for i in files) or not any(i.filename.endswith('.db3') for i in files):
            raise ValueError(f'Archive lacks a ROS2 sensor bag: {archive}')
        if len({Path(i.filename).name for i in files}) != len(files):
            raise ValueError('Ambiguous sensor bag archive')
        required = sum(i.file_size for i in files) + min_free_gib * 2**30
        if shutil.disk_usage(output.parent).free < required:
            raise ValueError('Insufficient free space for sensor extraction and reserve')
        temporary.mkdir()
        for member in files:
            if Path(member.filename).is_absolute() or '..' in Path(member.filename).parts:
                raise ValueError('Unsafe archive entry')
            with bundle.open(member) as source, (temporary/Path(member.filename).name).open('wb') as dest:
                shutil.copyfileobj(source, dest, length=4*1024*1024)
        info = dict(archive=str(archive), archive_bytes=archive.stat().st_size,
                    archive_mtime_ns=archive.stat().st_mtime_ns,
                    integrity='ZIP member CRC verified during extraction; transfer verification is recorded separately',
                    files={Path(i.filename).name: i.file_size for i in files})
        write_json(temporary/'EXTRACTED.json', info)
    temporary.rename(output)
    return output


def header_stamp(data):
    if bytes(data[:4]) not in (b'\x00\x01\x00\x00', b'\x00\x00\x00\x00'):
        raise ValueError('Unsupported ROS2 CDR encapsulation')
    sec, nsec = struct.unpack_from('<iI' if data[1] else '>iI', data, 4)
    if sec < 0 or nsec >= 10**9:
        raise ValueError('Invalid ROS2 header timestamp')
    return sec * 10**9 + nsec


def sensor_messages(bag, topic, msgtype):
    """Read only an allowlisted sensor topic; preserve each serialized payload."""
    bag = Path(bag)
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    previous = None
    for name in metadata['relative_file_paths']:
        path = (bag/name).resolve()
        if not path.is_relative_to(bag.resolve()):
            raise ValueError('Unsafe bag metadata path')
        with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as db:
            row = db.execute('SELECT id,type FROM topics WHERE ltrim(name, ?)=?', ('/', topic.lstrip('/'))).fetchall()
            if len(row) != 1 or row[0][1] != msgtype:
                raise ValueError(f'Missing or ambiguous sensor topic {topic}')
            for record_ns, payload in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (row[0][0],)):
                stamp = header_stamp(payload)
                if previous is not None and stamp <= previous:
                    raise ValueError(f'Nonmonotonic sensor header stamps: {topic}')
                if abs(stamp-record_ns) > 2*10**9:
                    raise ValueError(f'Header and bag clocks disagree: {topic}')
                previous = stamp
                yield stamp, topic, msgtype, payload


def validate_cloud(msg):
    """Reject XYZ-only/semantic exports: native Ouster deskew needs real point time."""
    required = dict(x=7, y=7, z=7, intensity=7, t=6, reflectivity=4, range=6)
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != dtype or fields[n].count != 1 for n, dtype in required.items()):
        raise ValueError('Cloud is not native Ouster XYZ/intensity/t/reflectivity/range; no synthetic point timing is allowed')
    if msg.width * msg.height < 1000 or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated or unexpectedly small Ouster cloud')
    f = fields['t']
    t = np.ndarray((msg.height, msg.width), dtype='>u4' if msg.is_bigendian else '<u4',
                   buffer=msg.data, offset=f.offset, strides=(msg.row_step, msg.point_step))
    if int(t.max()) == 0 or int(t.max()) > 100_000_000:
        raise ValueError('Ouster point times must be nonzero relative nanoseconds within a scan')
    return dict(frame=msg.header.frame_id, points=int(msg.width*msg.height),
                point_time_min_ns=int(t.min()), point_time_max_ns=int(t.max()),
                fields={n: dict(offset=f.offset, datatype=f.datatype, count=f.count) for n, f in fields.items()})


def stage_sensors(lidar_bag, imu_bag, output, robot, duration_s, start_offset_s=0.):
    """Merge raw LiDAR and LORD IMU, excluding GNSS, EKF, GT and labels."""
    import rosbag2_py
    from rosbags.typesys import Stores, get_typestore
    types = get_typestore(Stores.ROS2_HUMBLE)
    streams = [(lidar_bag, f'/{robot}/ouster/points', 'sensor_msgs/msg/PointCloud2'),
               (imu_bag, f'/{robot}/imu/data', 'sensor_msgs/msg/Imu')]
    output = Path(output)
    if output.exists():
        raise ValueError(f'Refusing to overwrite staged bag {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for _, topic, msgtype in streams:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=msgtype, serialization_format='cdr'))
    counts = {topic: 0 for _, topic, _ in streams}; first = {}; last = {}; max_gap = {}; diagnostics = {}
    first_ns = None; begin_ns = None; end_ns = None
    try:
        for stamp, topic, msgtype, payload in heapq.merge(*(sensor_messages(*s) for s in streams), key=lambda x: (x[0], x[1])):
            if first_ns is None:
                first_ns = stamp; begin_ns = stamp + round(start_offset_s*1e9)
                end_ns = begin_ns + round(duration_s*1e9) if duration_s else None
            if stamp < begin_ns:
                continue
            if end_ns is not None and stamp >= end_ns:
                break
            if topic.endswith('/points'):
                cloud = types.deserialize_cdr(payload, msgtype)
                detail = validate_cloud(cloud)
                if 'cloud' in diagnostics and diagnostics['cloud']['frame'] != detail['frame']:
                    raise ValueError('Cloud frame changed during replay')
                diagnostics['cloud'] = detail
            elif counts[topic] < 500:
                imu = types.deserialize_cdr(payload, msgtype)
                acc = imu.linear_acceleration; gyro = imu.angular_velocity
                if not np.isfinite([acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z]).all():
                    raise ValueError('Nonfinite raw IMU')
                if 'imu_frame' in diagnostics and diagnostics['imu_frame'] != imu.header.frame_id:
                    raise ValueError('IMU frame changed')
                diagnostics['imu_frame'] = imu.header.frame_id
            if topic in last:
                max_gap[topic] = max(max_gap.get(topic, 0), stamp-last[topic])
            first.setdefault(topic, stamp); last[topic] = stamp; counts[topic] += 1
            writer.write(topic, payload, stamp)
    finally:
        del writer
    if min(counts.values()) < 10:
        raise ValueError('Insufficient overlapping LiDAR/IMU measurements')
    imu_topic = streams[1][1]; lidar_topic = streams[0][1]
    if first[imu_topic] > first[lidar_topic] or last[imu_topic] < last[lidar_topic]:
        raise ValueError('Raw IMU does not cover the LiDAR replay interval')
    if max_gap[imu_topic] > 200_000_000:
        raise ValueError('Raw IMU gap exceeds 200 ms')
    info = dict(robot=robot, duration_s=duration_s, start_offset_s=start_offset_s,
                counts=counts, first_stamp_ns=first, last_stamp_ns=last, max_gap_ns=max_gap,
                diagnostics=diagnostics, ground_truth_used=False,
                source_bags=[str(s[0]) for s in streams],
                preprocessing='Unchanged serialized sensor messages; leading slash added to topics; bag record time set to original header time; no timestamp rebasing')
    write_json(output/'SENSORS.json', info)
    write_json(output/'COMPLETE.json', dict(files={p.name: file_hash(p) for p in output.iterdir() if p.is_file()}))
    return info


def mapping_config(source, urdf, robot, sensors, keyframes):
    cloud_frame = sensors['diagnostics']['cloud']['frame'].lstrip('/')
    imu_frame = sensors['diagnostics']['imu_frame'].lstrip('/')
    if imu_frame != robot+'_imu_link':
        raise ValueError('Unexpected LORD IMU frame; calibration must be verified')
    # The source URDF's root is the Ouster sensor. Do not silently interpret a
    # missing/renamed cloud frame as an identity extrinsic.
    T = urdf_transform(urdf, imu_frame, cloud_frame)
    cfg = yaml.safe_load((Path(source)/'ellipselio/config/os64_ncd.yaml').read_text())
    p = cfg['/**']['ros__parameters']
    p['mapping']['namespace'] = robot
    p['publish'] = dict(map=False, scan=False, markers=False, odometry=True, analytics=False, tf=False)
    p['lidar'].update(type=3, rate=20, scan_lines=64, vertical_fov=45., min_range=1., max_range=100.,
                      topic=f'/{robot}/ouster/points', t_imu_lidar=T[:3, 3].tolist(), r_imu_lidar=T[:3, :3].ravel().tolist())
    # Declared initial noise model, not a calibrated CU-Multi noise estimate.
    p['imu'] = dict(rate=500, acc_noise=.1, gyr_noise=.01, acc_bias=.0001, gyr_bias=.0001, topic=f'/{robot}/imu/data')
    p['input'] = dict(reliable=True); p['cameras'] = dict(num_cams=0)
    p['research'] = dict(ellipsoid_keyframes=True, ellipsoid_range_m=80., ellipsoid_world_frame=True,
                         ellipsoid_delta_export=True, keyframe_translation_m=keyframes['translation_m'],
                         keyframe_rotation_deg=keyframes['rotation_deg'], keyframe_interval_s=keyframes['max_interval_s'])
    return cfg, urdf_transform(urdf, robot+'_os_sensor', imu_frame)
