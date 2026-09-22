"""GRACO ground-01..06 sensor adapter; reference poses are evaluation-only."""
import hashlib
import sqlite3
from pathlib import Path

import numpy as np
import yaml

from .artifacts import file_hash, read_json, write_json
from .cu_multi import header_stamp

ROBOTS = tuple(f'robot{i}' for i in range(1, 7))
TOPICS = {'/velodyne/points': 'sensor_msgs/msg/PointCloud2',
          '/gnss/imu': 'sensor_msgs/msg/Imu'}


def validate_cloud(msg):
    fields = {f.name: f for f in msg.fields}
    required = dict(x=7, y=7, z=7, intensity=7, ring=4, time=7)
    if any(n not in fields or fields[n].datatype != t or fields[n].count != 1
           for n, t in required.items()):
        raise ValueError('GRACO requires native Velodyne XYZ/intensity/ring/time fields')
    if msg.width * msg.height < 1000 or len(msg.data) != msg.row_step * msg.height:
        raise ValueError('Truncated or unexpectedly small Velodyne cloud')
    def field(name, dtype):
        return np.ndarray((msg.height, msg.width), dtype=('>' if msg.is_bigendian else '<')+dtype,
                          buffer=msg.data, offset=fields[name].offset,
                          strides=(msg.row_step, msg.point_step))
    times = field('time', 'f4'); rings = field('ring', 'u2')
    # Native GRACO scans can start a fraction of a millisecond before their header.
    # Ground-05 contains two native double-duration scans (~0.2 s, 57,870
    # points). Preserve them, and record them instead of clipping their timing.
    if not np.isfinite(times).all() or times.min() < -.002 or times.max() > .22 or np.ptp(times) < .05:
        raise ValueError('Invalid Velodyne relative point time in seconds')
    if rings.max() >= 16:
        raise ValueError('Expected a 16-ring VLP-16 scan')
    if msg.header.frame_id.lstrip('/') != 'velodyne':
        raise ValueError('Unexpected LiDAR frame; verify calibration')
    return dict(frame=msg.header.frame_id, point_time_min_s=float(times.min()),
                point_time_max_s=float(times.max()), points=msg.width*msg.height,
                fields={n: dict(offset=f.offset, datatype=f.datatype) for n, f in fields.items()})


def stage_sensors(bag, output):
    """Copy only LiDAR/IMU CDR bytes and original record times into a ROS2 bag."""
    from rosbags.typesys import Stores, get_typestore
    types = get_typestore(Stores.ROS2_HUMBLE)
    bag = Path(bag); output = Path(output)
    if output.exists():
        marker = read_json(output/'COMPLETE.json')
        if any(file_hash(output/n) != h for n, h in marker['files'].items()):
            raise ValueError('Staged sensor bag failed hash verification')
        return read_json(output/'SENSORS.json')
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    temporary = output.with_name(output.name+'.incomplete'); temporary.mkdir(parents=True, exist_ok=False)
    ids = {name: i+1 for i, name in enumerate(TOPICS)}
    counts = dict.fromkeys(TOPICS, 0); first = {}; last = {}; max_gap = dict.fromkeys(TOPICS, 0)
    hashes = {name: hashlib.sha256() for name in TOPICS}; diagnostics = {}; max_header_offset = 0
    record_first = None; record_last = None; sources = []
    with sqlite3.connect(temporary/'sensors.db3') as dest:
        dest.executescript('''CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL,
            type TEXT NOT NULL, serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
            CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL, data BLOB NOT NULL);
            CREATE INDEX timestamp_idx ON messages(timestamp ASC);''')
        for name, typ in TOPICS.items():
            dest.execute('INSERT INTO topics VALUES (?,?,?,?,?)', (ids[name], name, typ, 'cdr', ''))
        for filename in metadata['relative_file_paths']:
            source = (bag/filename).resolve()
            if not source.is_relative_to(bag.resolve()):
                raise ValueError('Unsafe source bag path')
            sources.append(dict(path=str(source), bytes=source.stat().st_size, mtime_ns=source.stat().st_mtime_ns))
            with sqlite3.connect(f'file:{source}?mode=ro', uri=True) as db:
                selected = {}
                for name, typ in TOPICS.items():
                    found = db.execute('SELECT id,type FROM topics WHERE name=?', (name,)).fetchall()
                    if len(found) != 1 or found[0][1] != typ:
                        raise ValueError(f'Missing or ambiguous sensor topic: {name}')
                    selected[found[0][0]] = name
                query = 'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp,id'
                for tid, record_ns, payload in db.execute(query, tuple(selected)):
                    name = selected[tid]; stamp = header_stamp(payload)
                    if name in last and stamp <= last[name]:
                        raise ValueError(f'Nonmonotonic sensor header: {name}')
                    if record_last is not None and record_ns < record_last:
                        raise ValueError('Nonmonotonic source bag record timestamps')
                    offset = abs(record_ns-stamp); max_header_offset = max(max_header_offset, offset)
                    if offset > 1_000_000:
                        raise ValueError('GRACO sensor header/record clocks differ by more than 1 ms')
                    msg = types.deserialize_cdr(payload, TOPICS[name])
                    if name == '/velodyne/points':
                        detail = validate_cloud(msg)
                        diagnostics.setdefault('cloud', detail)
                        if detail['point_time_max_s'] > .12:
                            diagnostics.setdefault('extended_scans', []).append(dict(stamp_ns=stamp, **detail))
                    else:
                        if msg.header.frame_id.lstrip('/') != 'gnss':
                            raise ValueError('Unexpected IMU frame; verify calibration')
                        a = msg.linear_acceleration; w = msg.angular_velocity
                        if not np.isfinite([a.x,a.y,a.z,w.x,w.y,w.z]).all():
                            raise ValueError('Nonfinite native IMU measurement')
                        diagnostics.setdefault('imu_frame', msg.header.frame_id)
                    if name in last:
                        max_gap[name] = max(max_gap[name], stamp-last[name])
                    first.setdefault(name, stamp); last[name] = stamp; counts[name] += 1
                    hashes[name].update(record_ns.to_bytes(8, 'little')); hashes[name].update(payload)
                    dest.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES (?,?,?)',
                                 (ids[name], record_ns, payload))
                    if record_first is None: record_first = record_ns
                    record_last = record_ns
            dest.commit()
    expected = {t['topic_metadata']['name']: t['message_count'] for t in metadata['topics_with_message_count']}
    if any(counts[n] != expected[n] or counts[n] < 10 for n in TOPICS):
        raise ValueError('Incomplete sensor copy')
    if max_gap['/gnss/imu'] > 200_000_000:
        raise ValueError('IMU gap exceeds the 200 ms validity guard')
    boundary = dict(leading_lidar_without_imu_ns=max(0,first['/gnss/imu']-first['/velodyne/points']),
                    trailing_lidar_after_imu_ns=max(0,last['/velodyne/points']-last['/gnss/imu']))
    # The supplied ground-01 bag starts its IMU 67 ms into the first scan.
    # Preserve that scan and let native synchronization discard its uncovered
    # prefix; fail larger boundary mismatches instead of fabricating IMU data.
    if max(boundary.values()) > 120_000_000:
        raise ValueError('IMU coverage boundary differs from LiDAR by more than one scan')
    diagnostics['imu_lidar_boundary'] = boundary
    info = dict(source_bag=str(bag), source_files=sources, source_metadata_sha256=file_hash(bag/'metadata.yaml'),
                counts=counts, first_stamp_ns=first, last_stamp_ns=last, max_gap_ns=max_gap,
                max_header_record_offset_ns=max_header_offset, diagnostics=diagnostics,
                sensor_stream_sha256={n:h.hexdigest() for n,h in hashes.items()}, ground_truth_used=False,
                preprocessing='Unchanged native CDR payloads and record/header/point timestamps; only LiDAR and IMU topics; no time rebasing')
    write_json(temporary/'SENSORS.json', info)
    meta = dict(version=5, storage_identifier='sqlite3', relative_file_paths=['sensors.db3'],
                duration=dict(nanoseconds=record_last-record_first), starting_time=dict(nanoseconds_since_epoch=record_first),
                message_count=sum(counts.values()), compression_format='', compression_mode='',
                files=[dict(path='sensors.db3',duration=dict(nanoseconds=record_last-record_first),
                            starting_time=dict(nanoseconds_since_epoch=record_first),message_count=sum(counts.values()))],
                topics_with_message_count=[dict(topic_metadata=dict(name=n,type=t,serialization_format='cdr',offered_qos_profiles=''),
                                               message_count=counts[n]) for n,t in TOPICS.items()])
    (temporary/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=meta), sort_keys=False))
    write_json(temporary/'COMPLETE.json', dict(files={p.name:file_hash(p) for p in temporary.iterdir() if p.is_file()}))
    temporary.rename(output)
    return info


def mapping_config(source, calibration, robot, keyframes):
    """Apply the dataset's T_Imu_Lidar and measured IMU noise to the upstream preset."""
    calibration = Path(calibration)
    transform = yaml.safe_load((calibration/'imu-lidar.yaml').read_text())['T_Imu_Lidar']
    T = np.asarray(transform['data'], dtype=float).reshape(4,4)
    if not np.isfinite(T).all() or not np.allclose(T[3], [0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3], np.eye(3), atol=1e-6) or np.linalg.det(T[:3,:3]) < 0:
        raise ValueError('Invalid GRACO T_Imu_Lidar calibration')
    imu = yaml.safe_load((calibration/'imu.yaml').read_text())
    cfg = yaml.safe_load((Path(source)/'ellipselio/config/vlp16_graco.yaml').read_text())
    p = cfg['/**']['ros__parameters']; p['mapping']['namespace'] = robot
    p['publish'] = dict(map=False, scan=False, markers=False, odometry=True, analytics=True, tf=False)
    p['lidar'].update(t_imu_lidar=T[:3,3].tolist(), r_imu_lidar=T[:3,:3].ravel().tolist())
    p['imu'].update(acc_noise=imu['accelerometer_noise_density'], gyr_noise=imu['gyroscope_noise_density'],
                    acc_bias=imu['accelerometer_random_walk'], gyr_bias=imu['gyroscope_random_walk'])
    p['input'] = dict(reliable=True); p['cameras'] = dict(num_cams=0)
    p['research'] = dict(ellipsoid_keyframes=True, ellipsoid_range_m=80., ellipsoid_world_frame=True,
                         ellipsoid_delta_export=True, keyframe_translation_m=keyframes['translation_m'],
                         keyframe_rotation_deg=keyframes['rotation_deg'], keyframe_interval_s=keyframes['max_interval_s'])
    return cfg


def connectivity(robots, loops, poses):
    """Check measured connectivity and common output frames without imposing edges."""
    adjacency = {r:set() for r in robots}; pairs = {}
    for edge in loops:
        a,b = edge['i'][0],edge['j'][0]
        if a not in adjacency or b not in adjacency:
            raise ValueError('Constraint endpoint outside the requested robot group')
        if a != b:
            adjacency[a].add(b); adjacency[b].add(a)
            key = '--'.join(sorted([a,b])); pairs[key] = pairs.get(key,0)+1
    groups = []; remaining = set(robots)
    while remaining:
        group = set(); pending = [min(remaining)]
        while pending:
            r = pending.pop()
            if r in group: continue
            group.add(r); pending.extend(adjacency[r]-group)
        remaining -= group; groups.append(sorted(group))
    frames = {r:{p['component'] for p in poses if p['robot_id']==r} for r in robots}
    valid_frames = all(len(v)==1 for v in frames.values())
    if valid_frames:
        valid_frames = all(len(set().union(*(frames[r] for r in g)))==1 for g in groups)
    return dict(expected_robots=list(robots), expected_components=1, measured_components=groups,
                inter_robot_loop_counts=pairs, output_frames={r:sorted(v) for r,v in frames.items()},
                common_frame_contract=valid_frames, all_six_connected=len(groups)==1 and valid_frames,
                ground_truth_used=False)
