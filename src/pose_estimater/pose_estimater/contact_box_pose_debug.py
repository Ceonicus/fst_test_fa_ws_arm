#!/usr/bin/env python3

import math
from threading import Lock

import numpy as np
import rclpy
import rclpy.time
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


TF_FAILURE_LOG_THROTTLE_SEC = 2.0


class ContactBoxPoseDebug(Node):
    """Debug-only contact-box position estimate from YOLO pixel + aligned depth."""

    def __init__(self):
        super().__init__('contact_box_pose_debug')

        self.declare_parameter('pixel_topic', '/pixel_coordinates')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('window_px', 7)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('publish_topic', '/contact_box_pose_debug')
        self.declare_parameter('use_latest_tf', True)
        self.declare_parameter('tf_timeout_sec', 0.2)

        self.pixel_topic = self.get_parameter('pixel_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.window_px = max(1, int(self.get_parameter('window_px').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.publish_topic = self.get_parameter('publish_topic').value
        self.use_latest_tf = bool(self.get_parameter('use_latest_tf').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        if self.tf_timeout_sec < 0.0:
            self.get_logger().warn(
                f'tf_timeout_sec must be >= 0.0; received {self.tf_timeout_sec}. '
                'Using 0.2 instead.'
            )
            self.tf_timeout_sec = 0.2
        self.tf_mode = 'latest' if self.use_latest_tf else 'message_stamp'

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = Lock()
        self.latest_depth_msg = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.graph_warning_printed = False
        self.last_throttled_warn_time = {}

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.pixel_sub = self.create_subscription(
            Point,
            self.pixel_topic,
            self.pixel_callback,
            10,
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.publish_topic, 10)

        self.create_timer(5.0, self.warn_if_motion_graph_detected)

        self.get_logger().warn(
            'DEBUG ONLY: contact_box_pose_debug publishes only '
            f'{self.publish_topic}; it does not publish /object_pose and does '
            'not command MoveIt, the robot, gripper, or suction.'
        )
        self.get_logger().info(
            'Configured topics: '
            f'pixel_topic={self.pixel_topic}, depth_topic={self.depth_topic}, '
            f'camera_info_topic={self.camera_info_topic}, publish_topic={self.publish_topic}'
        )
        self.get_logger().info(
            'Configured frames/depth: '
            f'camera_frame={self.camera_frame}, target_frame={self.target_frame}, '
            f'window_px={self.window_px}, min_depth_m={self.min_depth_m:.3f}, '
            f'max_depth_m={self.max_depth_m:.3f}'
        )
        if self.use_latest_tf:
            self.get_logger().info('TF mode: latest available transform')
        else:
            self.get_logger().info('TF mode: message timestamp transform')
        self.get_logger().info(f'TF timeout: {self.tf_timeout_sec:.3f} sec')

    def depth_callback(self, msg):
        if msg.encoding not in ('16UC1', '32FC1'):
            self.get_logger().warn(
                f'Rejecting depth image: unsupported encoding {msg.encoding}; '
                'expected 16UC1 millimeters or 32FC1 meters.'
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Rejecting depth image: cv_bridge failed: {exc}')
            return

        if depth_image.ndim == 3 and depth_image.shape[2] == 1:
            depth_image = depth_image[:, :, 0]
        if depth_image.ndim != 2:
            self.get_logger().warn(
                f'Rejecting depth image: expected single-channel image, got shape {depth_image.shape}.'
            )
            return

        with self.lock:
            self.latest_depth_msg = msg
            self.latest_depth_image = np.array(depth_image, copy=True)

    def camera_info_callback(self, msg):
        with self.lock:
            self.latest_camera_info = msg

    def pixel_callback(self, pixel_msg):
        if not (math.isfinite(pixel_msg.x) and math.isfinite(pixel_msg.y)):
            self.get_logger().warn(
                f'Rejecting pixel: non-finite values x={pixel_msg.x}, y={pixel_msg.y}.'
            )
            return

        u = int(round(pixel_msg.x))
        v = int(round(pixel_msg.y))

        with self.lock:
            depth_msg = self.latest_depth_msg
            depth_image = None if self.latest_depth_image is None else self.latest_depth_image.copy()
            camera_info = self.latest_camera_info

        if depth_msg is None or depth_image is None:
            self.get_logger().warn(
                f'Rejecting pixel ({u}, {v}): no aligned depth image received yet.'
            )
            return
        if camera_info is None:
            self.get_logger().warn(
                f'Rejecting pixel ({u}, {v}): no color camera_info received yet.'
            )
            return

        height, width = depth_image.shape
        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn(
                f'Rejecting pixel ({u}, {v}): outside depth image bounds {width}x{height}.'
            )
            return

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn(
                f'Rejecting pixel ({u}, {v}): invalid intrinsics fx={fx}, fy={fy}.'
            )
            return

        depth_result = self.get_median_depth(depth_image, depth_msg.encoding, u, v)
        if depth_result is None:
            return

        depth_m, center_raw, median_raw, valid_count, total_count = depth_result
        camera_x = (float(u) - cx) * depth_m / fx
        camera_y = (float(v) - cy) * depth_m / fy
        camera_z = depth_m

        point_stamped = PointStamped()
        point_stamped.header.stamp = depth_msg.header.stamp
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.point.x = camera_x
        point_stamped.point.y = camera_y
        point_stamped.point.z = camera_z

        if depth_msg.header.frame_id and depth_msg.header.frame_id != self.camera_frame:
            self.get_logger().warn(
                f'Depth header frame is {depth_msg.header.frame_id}, but projection is '
                f'published in configured camera_frame {self.camera_frame}.'
            )

        transform_time = self.get_transform_time(depth_msg)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                transform_time,
                Duration(seconds=self.tf_timeout_sec),
            )
            transformed_point = do_transform_point(point_stamped, transform)
        except TransformException as exc:
            stamp = depth_msg.header.stamp
            requested_time = 'latest available' if self.use_latest_tf else (
                f'{stamp.sec}.{stamp.nanosec:09d}'
            )
            self.throttled_warn(
                'tf_failure',
                f'Rejecting pixel ({u}, {v}): transform unavailable '
                f'{self.camera_frame}->{self.target_frame}; tf_mode={self.tf_mode}, '
                f'requested_tf_time={requested_time}, timeout_sec={self.tf_timeout_sec:.3f}; '
                f'reason={exc}. '
                f'camera_xyz=({camera_x:.4f}, {camera_y:.4f}, {camera_z:.4f}), '
                f'raw_depth={self.format_raw(center_raw)}, '
                f'median_raw_depth={self.format_raw(median_raw)}, '
                f'valid_depth_pixels={valid_count}/{total_count}.',
                TF_FAILURE_LOG_THROTTLE_SEC,
            )
            return

        pose = PoseStamped()
        pose.header.stamp = depth_msg.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position = transformed_point.point
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        tf_stamp = transform.header.stamp
        self.get_logger().info(
            f'contact_box_pose_debug pixel=({u}, {v}), '
            f'raw_depth={self.format_raw(center_raw)}, '
            f'median_raw_depth={self.format_raw(median_raw)}, '
            f'depth_m={depth_m:.4f}, valid_depth_pixels={valid_count}/{total_count}, '
            f'camera_xyz=({camera_x:.4f}, {camera_y:.4f}, {camera_z:.4f}), '
            f'base_xyz=({pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, '
            f'{pose.pose.position.z:.4f}), tf_status=OK, '
            f'tf_mode={self.tf_mode}, '
            f'tf_stamp={tf_stamp.sec}.{tf_stamp.nanosec:09d}'
        )

    def get_transform_time(self, depth_msg):
        if self.use_latest_tf:
            return rclpy.time.Time()
        return rclpy.time.Time.from_msg(depth_msg.header.stamp)

    def get_median_depth(self, depth_image, encoding, u, v):
        radius = self.window_px // 2
        height, width = depth_image.shape
        u_min = max(0, u - radius)
        u_max = min(width - 1, u + radius)
        v_min = max(0, v - radius)
        v_max = min(height - 1, v + radius)

        window = depth_image[v_min:v_max + 1, u_min:u_max + 1].astype(np.float64)
        center_raw = depth_image[v, u].item()
        total_count = int(window.size)

        if encoding == '16UC1':
            depth_m_values = window / 1000.0
        else:
            depth_m_values = window

        valid_mask = (
            np.isfinite(depth_m_values)
            & (depth_m_values > 0.0)
            & (depth_m_values >= self.min_depth_m)
            & (depth_m_values <= self.max_depth_m)
        )
        valid_count = int(np.count_nonzero(valid_mask))

        if valid_count == 0:
            self.get_logger().warn(
                f'Rejecting pixel ({u}, {v}): no valid depth pixels in '
                f'{self.window_px}px window; center_raw_depth={self.format_raw(center_raw)}, '
                f'valid_depth_pixels=0/{total_count}, min_depth_m={self.min_depth_m:.3f}, '
                f'max_depth_m={self.max_depth_m:.3f}.'
            )
            return None

        valid_depth_m = depth_m_values[valid_mask]
        depth_m = float(np.median(valid_depth_m))
        median_raw = float(np.median(window[valid_mask]))

        return depth_m, center_raw, median_raw, valid_count, total_count

    def warn_if_motion_graph_detected(self):
        if self.graph_warning_printed:
            return

        warnings = []
        try:
            object_pose_publishers = self.get_publishers_info_by_topic('/object_pose')
            if object_pose_publishers:
                names = sorted(
                    {
                        f'{info.node_namespace.rstrip("/")}/{info.node_name}'.replace('//', '/')
                        for info in object_pose_publishers
                    }
                )
                warnings.append(f'/object_pose publisher(s) detected: {names}')

            node_names = set(self.get_node_names())
            risky_nodes = sorted(
                name
                for name in node_names
                if name in ('moveit_move', 'moveit_pose_subscriber')
            )
            if risky_nodes:
                warnings.append(f'moveit_move-related node(s) detected: {risky_nodes}')
        except Exception as exc:
            self.get_logger().warn(f'Could not inspect ROS graph for motion risks: {exc}')
            self.graph_warning_printed = True
            return

        if warnings:
            self.get_logger().warn(
                'Safety check: this debug node is isolated, but motion-capable graph activity '
                'may be present. ' + '; '.join(warnings)
            )
            self.graph_warning_printed = True

    def throttled_warn(self, key, message, throttle_sec):
        now = self.get_clock().now().nanoseconds / 1e9
        last = self.last_throttled_warn_time.get(key)
        if last is None or now - last >= throttle_sec:
            self.get_logger().warn(message)
            self.last_throttled_warn_time[key] = now

    @staticmethod
    def format_raw(value):
        try:
            if isinstance(value, (float, np.floating)):
                return f'{float(value):.6g}'
            return str(value)
        except Exception:
            return str(value)


def main(args=None):
    rclpy.init(args=args)
    node = ContactBoxPoseDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
