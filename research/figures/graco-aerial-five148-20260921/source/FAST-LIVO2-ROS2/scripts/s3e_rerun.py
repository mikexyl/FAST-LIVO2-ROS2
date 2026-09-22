#!/usr/bin/env python3
"""Forward ROS 2 CDR messages through Rerun 0.37.1's official MCAP importer.

Point clouds, images, CameraInfo, TF, IMU and GNSS use Rerun's native decoders.
Only the odometry breadcrumb trail is logged separately, following the official
ROS node example. The original messages are also retained in visualization.mcap.
"""
import argparse
from collections import Counter
import io
import json
from pathlib import Path
import queue
import threading

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from mcap.writer import Writer, CompressionType
from rosbags.typesys import get_typestore, Stores


TIMELINE = 'ros2_timestamp'


def pcl_colors(msg):
    """PCL packs RGB bits in a float field; only adapt this missing semantic."""
    field = next((f for f in msg.fields if f.name == 'rgb'), None)
    if field is None:
        return None
    packed = np.ndarray((msg.height, msg.width), dtype='>u4' if msg.is_bigendian else '<u4',
                        buffer=msg.data, offset=field.offset,
                        strides=(msg.row_step, msg.point_step)).ravel().astype(np.uint32)
    return ((packed & 0x00FFFFFF) << 8) | 255


def blueprint(robot):
    history = rrb.VisibleTimeRange(TIMELINE, start=rrb.TimeRangeBoundary.infinite(),
                                  end=rrb.TimeRangeBoundary.cursor_relative())
    world = rrb.SpatialInformation(target_frame='camera_init')
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Tabs(
                rrb.Spatial3DView(name='Accumulated color map',
                    contents=['/cloud_registered', '/trajectory'],
                    spatial_information=world, time_ranges=history,
                    overrides={'/cloud_registered': rr.Points3D.from_fields(radii=0.025).visualizer()}),
                rrb.Spatial3DView(name='Live camera and TF',
                    contents=['/cloud_registered', '/tf', '/tf_static', '/s3e/camera/**'],
                    spatial_information=world,
                    overrides={'/tf': rr.TransformAxes3D(0.5, show_frame=True).visualizer(),
                               '/tf_static': rr.TransformAxes3D(0.3, show_frame=True).visualizer(),
                               '/s3e/camera/camera_info': rr.Pinhole.from_fields(image_plane_distance=1.0).visualizer()}),
            ),
            rrb.Vertical(
                rrb.Spatial2DView(name='Rectified camera / tracked features',
                                 origin='/s3e/camera/image_rect/compressed'),
                rrb.Tabs(
                    rrb.TimeSeriesView(name='IMU', contents=[f'/{robot}/imu/data']),
                    rrb.MapView(name='GNSS', contents=[f'/{robot}/fix']),
                ), row_shares=[0.65, 0.35]),
            column_shares=[0.7, 0.3]),
        rrb.TimePanel(timeline=TIMELINE, play_state='Following', expanded=True),
        auto_layout=False, auto_views=False,
    )


class McapStream:
    """Preserve serialized messages; delegate all ROS schema decoding to Rerun."""
    def __init__(self, output):
        self.types = get_typestore(Stores.ROS2_HUMBLE)
        self.definitions = {}
        self.file = (output / 'visualization.mcap').open('wb')
        self.writer = Writer(self.file, compression=CompressionType.ZSTD)
        self.writer.start(profile='ros2', library='FAST-LIVO2 S3E ROS 2 forwarding')
        self.channels = {}
        self.records = []
        self.counts = Counter()
        self.pending = queue.Queue(maxsize=8)
        self.error = None
        self.viewer_disconnect = None
        self.batches = 0
        self.worker = threading.Thread(target=self.import_batches, daemon=True)
        self.worker.start()

    def channel(self, writer, channels, topic, typename):
        if topic not in channels:
            if typename not in self.definitions:
                self.definitions[typename] = self.types.generate_msgdef(typename, ros_version=2)[0].encode()
            schema = writer.register_schema(typename, 'ros2msg', self.definitions[typename])
            channels[topic] = writer.register_channel(topic, 'cdr', schema)
        return channels[topic]

    def add(self, topic, typename, stamp, data):
        if self.error:
            raise RuntimeError(f'Rerun importer failed: {self.error}')
        channel = self.channel(self.writer, self.channels, topic, typename)
        self.writer.add_message(channel, stamp, data, stamp)
        self.records.append((topic, typename, stamp, data))
        self.counts[topic] += 1

    def flush_batch(self):
        if self.error:
            raise RuntimeError(f'Rerun importer failed: {self.error}')
        if self.records:
            self.pending.put(self.records, timeout=10)
            self.records = []

    def import_batches(self):
        try:
            while True:
                records = self.pending.get()
                if records is None:
                    return
                buffer = io.BytesIO()
                writer = Writer(buffer, compression=CompressionType.NONE)
                writer.start(profile='ros2')
                channels = {}
                for topic, typename, stamp, data in records:
                    channel = self.channel(writer, channels, topic, typename)
                    writer.add_message(channel, stamp, data, stamp)
                writer.finish()
                rr.log_file_from_contents('s3e_live.mcap', buffer.getvalue())
                self.batches += 1
        except Exception as exc:
            self.error = repr(exc)

    def close(self):
        self.flush_batch()
        self.pending.put(None, timeout=10)
        self.worker.join(timeout=30)
        self.writer.finish()
        self.file.close()
        if self.worker.is_alive() or self.error:
            raise RuntimeError(self.error or 'Rerun importer did not drain')
        try:
            rr.get_data_recording().flush()
        except RuntimeError as exc:
            # Closing the viewer severs only the gRPC sink. The independent
            # MCAP writer is finished above and the FileSink still saves RRD.
            if 'gRPC connection severed' not in str(exc):
                raise
            self.viewer_disconnect = str(exc)


class Bridge(Node):
    def __init__(self, robot, output):
        super().__init__('s3e_rerun_bridge')
        self.stream = McapStream(output)
        self.output = output
        self.last_stamp = None
        topics = {
            '/cloud_registered': 'sensor_msgs/msg/PointCloud2',
            '/aft_mapped_to_init': 'nav_msgs/msg/Odometry',
            '/tf': 'tf2_msgs/msg/TFMessage',
            '/tf_static': 'tf2_msgs/msg/TFMessage',
            '/s3e/camera/image_rect/compressed': 'sensor_msgs/msg/CompressedImage',
            '/s3e/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
            f'/{robot}/imu/data': 'sensor_msgs/msg/Imu',
            f'/{robot}/fix': 'sensor_msgs/msg/NavSatFix',
        }
        for topic, typename in topics.items():
            qos = QoSProfile(depth=100)
            if topic == '/tf_static':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(get_message(typename), topic,
                                     self.callback(topic, typename), qos)
        self.create_timer(1.0, self.stream.flush_batch, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(5.0, self.status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        (output / 'rerun_ready').touch()
        self.get_logger().info('Ready: ROS 2 CDR -> official Rerun 0.37.1 MCAP importer')

    def callback(self, topic, typename):
        def receive(msg):
            if topic == '/cloud_registered' and not msg.width * msg.height:
                return  # The upstream LIVO node emits empty clouds between colored updates.
            header = msg.header if hasattr(msg, 'header') else (msg.transforms[0].header if msg.transforms else None)
            if header is None:
                return
            stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            self.last_stamp = max(self.last_stamp or 0, stamp)
            self.stream.add(topic, typename, stamp, serialize_message(msg))
            if topic == '/cloud_registered':
                # The native importer preserves PCL rgb as an extra field but
                # does not assign it to Points3D.colors in 0.37.1. Positions,
                # field decoding, frames and times still use its native parser.
                colors = pcl_colors(msg)
                if colors is not None:
                    rr.set_time(TIMELINE, timestamp=np.datetime64(stamp, 'ns'))
                    rr.log(topic, rr.Points3D.from_fields(colors=colors))
            if topic == '/aft_mapped_to_init':
                p = msg.pose.pose.position
                rr.set_time(TIMELINE, timestamp=np.datetime64(stamp, 'ns'))
                rr.log('/trajectory', rr.Points3D([[p.x, p.y, p.z]], colors=[255, 170, 40], radii=0.08),
                       rr.CoordinateFrame(msg.header.frame_id))
        return receive

    def status(self, finished=False):
        status = dict(rerun_version=rr.__version__, importer='official native MCAP',
                      counts=dict(self.stream.counts), batches=self.stream.batches,
                      queued_batches=self.stream.pending.qsize(),
                      last_sensor_stamp_ns=self.last_stamp, error=self.stream.error,
                      viewer_disconnect=self.stream.viewer_disconnect, finished=finished)
        (self.output / 'rerun_summary.json').write_text(json.dumps(status, indent=2) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', choices=['Alpha', 'Bob', 'Carol'], default='Alpha')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--url', help='Connect to a viewer; omitted for a recording-only test')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rr.init('FAST-LIVO2 S3E', recording_id=args.output.name, strict=True)
    sinks = [rr.FileSink(args.output / 'visualization.rrd')]
    if args.url:
        sinks.append(rr.GrpcSink(args.url))
    rr.set_sinks(*sinks)
    rr.send_blueprint(blueprint(args.robot))
    rr.log('/world', rr.CoordinateFrame('camera_init'), rr.ViewCoordinates.FLU, static=True)
    rclpy.init()
    node = Bridge(args.robot, args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stream.close()
        node.status(finished=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        rr.disconnect()


if __name__ == '__main__':
    main()
