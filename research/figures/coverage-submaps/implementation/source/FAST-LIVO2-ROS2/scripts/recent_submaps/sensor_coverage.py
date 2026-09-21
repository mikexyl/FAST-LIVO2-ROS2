#!/usr/bin/env python3
"""Check the native trajectory tail against the selected source sensor topics."""
import argparse
import json
from pathlib import Path
import sqlite3
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu


def audit(trial):
    summary = json.loads((trial / 'frontend/summary.json').read_text())
    params = yaml.safe_load((trial / 'frontend/runtime.yaml').read_text())['/**']['ros__parameters']
    bag = Path(summary['bag'])
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    rows = [json.loads(line) for line in (trial / 'frontend/native_updates.jsonl').read_text().splitlines()]
    result = {}
    for label, msg_type in [('lidar', PointCloud2), ('imu', Imu)]:
        topic = params[label]['topic']; candidates = []
        for filename in metadata['relative_file_paths']:
            with sqlite3.connect(f'file:{bag / filename}?mode=ro', uri=True) as connection:
                value = connection.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id '
                                           'WHERE t.name=? ORDER BY m.timestamp DESC LIMIT 1', (topic,)).fetchone()
                if value:
                    message = deserialize_message(value[1], msg_type)
                    stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                    candidates.append((value[0], stamp))
        last_record, last_header = max(candidates)
        count = sum(x['message_count'] for x in metadata['topics_with_message_count'] if x['topic_metadata']['name'] == topic)
        result[label] = dict(topic=topic, source_messages=count, last_record_ns=last_record, last_header_ns=last_header)
    native_end = rows[-1]['sensor_stamp_ns']
    # Last clouds may be discarded if the IMU stream ends before their end time.
    processable_tail = min(result['lidar']['last_header_ns'], result['imu']['last_header_ns'])
    result.update(native_last_scan_end_ns=native_end, tail_shortfall_s=max(0, processable_tail-native_end)/1e9,
                  full_selected_sensor_tail_reached=(processable_tail-native_end)<=200000000,
                  tolerance_s=.2, native_pose_count=len(rows))
    (trial / 'sensor-coverage.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path)
    print(json.dumps(audit(parser.parse_args().trial), indent=2))
