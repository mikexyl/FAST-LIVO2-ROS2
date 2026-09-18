#!/usr/bin/env python3
"""Read bounded direct CDR packets from the mapper; write standard ROS2 MCAP."""
import argparse
import json
from pathlib import Path
import struct
import sys
import time

from mcap.writer import Writer, CompressionType
from rosbags.typesys import Stores, get_typestore

# Also supports execution as an absolute script by the C++ mapper.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import canonical, file_hash, write_json


def exact(stream, n):
    chunks = bytearray()
    while len(chunks) < n:
        block = stream.read(n - len(chunks))
        if not block:
            raise EOFError('Mapper export ended without an explicit end record')
        chunks.extend(block)
    return bytes(chunks)


def consume(stream, output, message_filter=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    types = get_typestore(Stores.ROS2_HUMBLE)
    channels = {}
    count, previous, robot = 0, -1, None
    source_metadata = {}
    start = time.monotonic()
    with (output / 'sensors.mcap').open('xb') as f, (output / 'frames.jsonl').open('xb') as index:
        writer = Writer(f, compression=CompressionType.ZSTD, chunk_size=4 * 1024 * 1024)
        writer.start(profile='ros2', library='S3E direct synchronized odometry export v1')
        while True:
            n = struct.unpack('<I', exact(stream, 4))[0]
            if not n:
                break
            if n > 1024 * 1024:
                raise ValueError('Invalid export packet header size')
            row = json.loads(exact(stream, n))
            stamp = row['stamp_ns']
            if type(stamp) is not int or stamp <= previous or row['frame_id'] != count:
                raise ValueError('Nonmonotonic export stamp or frame ID')
            if robot is not None and robot != row['robot_id']:
                raise ValueError('Mixed robot export')
            robot, previous = row['robot_id'], stamp
            current = {k:row[k] for k in ('frontend','cloud_source','pose_source') if k in row}
            if count and current != source_metadata: raise ValueError('Changing export provenance')
            source_metadata = current
            retained=[]
            for entry in row['messages']:
                topic, typename = entry['topic'], entry['type']
                if not topic.startswith(f'/{robot}/research/'):
                    raise ValueError('Cross-robot channel')
                if not 0 < entry['size'] <= 128 * 1024 * 1024:
                    raise ValueError('Invalid serialized message length')
                data = exact(stream, entry['size'])
                if topic not in channels:
                    schema = writer.register_schema(typename, 'ros2msg',
                        types.generate_msgdef(typename, ros_version=2)[0].encode())
                    channels[topic] = writer.register_channel(topic, 'cdr', schema)
                msg = types.deserialize_cdr(data, typename)
                if msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec != stamp:
                    raise ValueError('Sensor/state timestamp association failed')
                if message_filter is not None and not message_filter(row,entry,msg):
                    continue
                writer.add_message(channels[topic], stamp, data, publish_time=stamp, sequence=count)
                retained.append(entry)
            row['messages']=retained
            index.write(canonical(row) + b'\n')
            count += 1
        writer.finish()
    if not count:
        raise ValueError('Export contained no frames')
    extra_files=message_filter.files if message_filter is not None else []
    write_json(output / 'manifest.json', dict(schema_version=1, complete=True, robot_id=robot,
        frames=count, last_stamp_ns=previous, elapsed_s=time.monotonic()-start,
        files={name:file_hash(output/name) for name in ('sensors.mcap','frames.jsonl',*extra_files)},
        covariance='filter marginals are diagnostics only',
        **dict(dict(cloud_source='complete preprocessed deskewed LiDAR in lidar frame, before mapping downsample',
            pose_source='IMU/body state after visual update, at exact input image timestamp'), **source_metadata)))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    consume(sys.stdin.buffer, p.parse_args().output)
