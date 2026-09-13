#!/usr/bin/env python3
"""Decode S3E JPEGs and rebase LiDAR scans while preserving acquisition times."""
import copy
from array import array
import cv2
import numpy as np
import rclpy
import yaml
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def rebase_scan(msg):
    """Keep header + point.time invariant; support organized clouds/row padding."""
    field = next((f for f in msg.fields if f.name == 'time'), None)
    if field is None or field.datatype != PointField.FLOAT32 or field.count != 1:
        raise ValueError('S3E cloud requires one float32 time field in seconds')
    if not msg.width or not msg.height:
        raise ValueError('Empty S3E cloud')
    out = copy.deepcopy(msg)
    times = np.ndarray((out.height, out.width),
                       dtype='>f4' if out.is_bigendian else '<f4',
                       buffer=out.data, offset=field.offset,
                       strides=(out.row_step, out.point_step))
    if not np.isfinite(times).all():
        raise ValueError('Non-finite point timestamps')
    start = float(times.min())
    span = float(times.max()) - start
    if not 0.05 < span < 0.15:
        raise ValueError(f'Unexpected S3E scan duration: {span:.6f}s')
    stamp = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
    out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp + round(start * 1e9), 1_000_000_000)
    times -= start
    # Packet/ring ordering is not always acquisition ordering. FAST-LIVO2 uses
    # the last point as the scan end and splits scans at image timestamps.
    order = np.argsort(times.ravel(), kind='stable')
    records = np.ndarray((out.height, out.width), dtype=f'V{out.point_step}',
                         buffer=out.data, strides=(out.row_step, out.point_step))
    out.data = array('B', records.ravel()[order].tobytes())
    out.width *= out.height
    out.height = 1
    out.row_step = out.width * out.point_step
    return out


class Adapter(Node):
    def __init__(self):
        super().__init__('s3e_adapter')
        robot = self.declare_parameter('robot', 'Alpha').value
        if robot not in ('Alpha', 'Bob', 'Carol'):
            raise ValueError('robot must be Alpha, Bob, or Carol')
        self.cloud_pub = self.create_publisher(PointCloud2, f'/{robot}/velodyne_points_start', 100)
        self.image_pub = self.create_publisher(Image, f'/{robot}/left_camera/image', 30)
        # rosbag2 publishes reliably. Request reliable delivery for the large
        # fragmented clouds/images so missing packets cannot create scan gaps.
        incoming = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, f'/{robot}/velodyne_points', self.cloud, incoming)
        self.create_subscription(CompressedImage, f'/{robot}/left_camera/compressed', self.image, incoming)
        cv2.setNumThreads(1)
        self.first_cloud = True
        self.cloud_count = 0
        self.image_count = 0
        if self.declare_parameter('visualization', False).value:
            self.setup_visualization(robot)

    def setup_visualization(self, robot):
        camera_path = self.declare_parameter('camera_config', '').value
        mapping_path = self.declare_parameter('mapping_config', '').value
        self.camera = yaml.safe_load(Path(camera_path).read_text())['/**']['ros__parameters']
        mapping = yaml.safe_load(Path(mapping_path).read_text())['/**']['ros__parameters']
        extrinsic = mapping['extrin_calib']
        imu_from_lidar = np.array(extrinsic['extrinsic_R']).reshape(3, 3)
        camera_from_lidar = np.array(extrinsic['Rcl']).reshape(3, 3)
        imu_from_camera = imu_from_lidar @ camera_from_lidar.T
        translation = np.array(extrinsic['extrinsic_T']) - imu_from_camera @ np.array(extrinsic['Pcl'])
        axis_angle = cv2.Rodrigues(imu_from_camera)[0].reshape(3)
        angle = np.linalg.norm(axis_angle)
        quaternion = np.r_[axis_angle * (np.sin(angle / 2) / angle if angle else 0), np.cos(angle / 2)]
        self.camera_frame = f's3e_{robot.lower()}_camera_optical'
        transform = TransformStamped()
        transform.header.frame_id = 'aft_mapped'
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, translation)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, quaternion)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(transform)
        self.rect_pub = self.create_publisher(CompressedImage, '/s3e/camera/image_rect/compressed', 30)
        self.info_pub = self.create_publisher(CameraInfo, '/s3e/camera/camera_info', 30)
        self.rect_maps = None
        self.rect_size = None
        self.create_subscription(Image, '/rgb_img', self.visual_image, 30)

    def visual_image(self, msg):
        # FAST-LIVO2's debug image is scaled but still distorted. Rectify only
        # this visualization copy, making the native CameraInfo pinhole exact.
        if msg.encoding != 'bgr8':
            raise ValueError(f'Unexpected mapper image encoding: {msg.encoding}')
        if self.rect_size != (msg.width, msg.height):
            self.rect_size = (msg.width, msg.height)
            c = self.camera
            sx, sy = msg.width / c['cam_width'], msg.height / c['cam_height']
            self.rect_k = np.array([[c['cam_fx'] * sx, 0, c['cam_cx'] * sx],
                                    [0, c['cam_fy'] * sy, c['cam_cy'] * sy], [0, 0, 1]])
            distortion = np.array([c[f'cam_d{i}'] for i in range(4)] + [0.0])
            self.rect_maps = cv2.initUndistortRectifyMap(self.rect_k, distortion, np.eye(3),
                                                       self.rect_k, self.rect_size, cv2.CV_32FC1)
        pixels = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                            buffer=msg.data, strides=(msg.step, 3, 1))
        rectified = cv2.remap(pixels, *self.rect_maps, cv2.INTER_LINEAR)
        success, jpeg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise ValueError('Could not encode rectified visualization image')
        out = CompressedImage()
        out.header = copy.deepcopy(msg.header)
        out.header.frame_id = self.camera_frame
        out.format = 'jpeg'
        out.data = array('B', jpeg.tobytes())
        info = CameraInfo()
        info.header = out.header
        info.width, info.height = msg.width, msg.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = self.rect_k.ravel().tolist()
        info.r = np.eye(3).ravel().tolist()
        info.p = np.column_stack([self.rect_k, np.zeros(3)]).ravel().tolist()
        self.info_pub.publish(info)
        self.rect_pub.publish(out)

    def cloud(self, msg):
        try:
            adapted = rebase_scan(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise
        self.cloud_pub.publish(adapted)
        self.cloud_count += 1
        if self.first_cloud:
            self.get_logger().info('LiDAR scan rebased to first point; absolute point times preserved')
            self.first_cloud = False

    def image(self, msg):
        decoded = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError('Could not decode S3E camera JPEG')
        out = Image()
        out.header = msg.header
        out.height, out.width = decoded.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = array('B', decoded.tobytes())
        self.image_pub.publish(out)
        self.image_count += 1


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f'S3E adapter received {node.cloud_count} clouds and {node.image_count} images', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
