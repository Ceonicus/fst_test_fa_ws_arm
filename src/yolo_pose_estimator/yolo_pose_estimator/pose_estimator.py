import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped

from sensor_msgs_py import point_cloud2
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        # --- Parameters ---
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('point_cloud_topic', '/camera/camera/depth/points')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        yolo_topic = self.get_parameter('yolo_detections_topic').get_parameter_value().string_value
        pc_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to YOLO detections on: {yolo_topic}")
        self.get_logger().info(f"Subscribing to PointCloud2 on: {pc_topic}")
        self.get_logger().info(f"Subscribing to CameraInfo on: {cam_info_topic}")

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers with message_filters
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, yolo_topic)
        self.point_cloud_sub = message_filters.Subscriber(self, PointCloud2, pc_topic, qos_profile=qos_profile)
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, cam_info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.point_cloud_sub, self.cam_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def synchronized_callback(self, detections_msg, pc_msg, cam_info_msg):
        # Convert point cloud to numpy
        pc_list = list(point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))
        pc_numpy = np.array(pc_list, dtype=np.float32)

        if pc_numpy.shape[0] == 0:
            self.get_logger().warn("Empty point cloud received.")
            return

        # Camera intrinsics
        fx = cam_info_msg.k[0]
        fy = cam_info_msg.k[4]
        cx = cam_info_msg.k[2]
        cy = cam_info_msg.k[5]

        for detection in detections_msg.detections:
            bbox = detection.bbox
            center_x = bbox.center.x
            center_y = bbox.center.y
            width = bbox.size_x
            height = bbox.size_y

            xmin = int(center_x - width / 2)
            xmax = int(center_x + width / 2)
            ymin = int(center_y - height / 2)
            ymax = int(center_y + height / 2)

            # --- Project 3D points into pixel space ---
            X, Y, Z = pc_numpy[:, 0], pc_numpy[:, 1], pc_numpy[:, 2]
            u = (fx * X / Z + cx).astype(np.int32)
            v = (fy * Y / Z + cy).astype(np.int32)

            # Select points inside bbox
            mask = (u >= xmin) & (u <= xmax) & (v >= ymin) & (v <= ymax) & (Z > 0)
            object_pc = pc_numpy[mask]

            if object_pc.shape[0] < 10:
                self.get_logger().warn("Not enough points in bbox cloud for PCA.")
                continue

            # --- 6D Pose Estimation (PCA) ---
            centroid = np.mean(object_pc, axis=0)
            centered_pc = object_pc - centroid
            covariance_matrix = np.cov(centered_pc, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

            rotation_matrix = eigenvectors.T
            if np.linalg.det(rotation_matrix) < 0:
                rotation_matrix[:, 0] *= -1

            rotation = Rotation.from_matrix(rotation_matrix)
            quaternion = rotation.as_quat()

            # PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = pc_msg.header
            pose_msg.pose.position = Point(x=centroid[0], y=centroid[1], z=centroid[2])
            pose_msg.pose.orientation = Quaternion(
                x=quaternion[0], y=quaternion[1],
                z=quaternion[2], w=quaternion[3]
            )
            self.pose_publisher.publish(pose_msg)

            # TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = pc_msg.header.frame_id
            t.child_frame_id = f"object_{detection.results[0].id if detection.results else 'unknown'}"
            t.transform.translation.x = centroid[0]
            t.transform.translation.y = centroid[1]
            t.transform.translation.z = centroid[2]
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
