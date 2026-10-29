#!/usr/bin/env python3

import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from builtin_interfaces.msg import Duration as DurationMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState


MOVEIT_SUCCESS = 1
APRILTAG_DETECTIONS_TOPIC = "/detections"


def quat_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        raise ValueError("Cannot normalize zero quaternion")
    return [value / norm for value in q]


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_inverse(q):
    q = quat_normalize(q)
    return [-q[0], -q[1], -q[2], q[3]]


def quat_rotate(q, vector):
    q = quat_normalize(q)
    qv = [vector[0], vector[1], vector[2], 0.0]
    q_inv = quat_inverse(q)
    return quat_multiply(quat_multiply(q, qv), q_inv)[:3]


def compose_transform(t1, q1, t2, q2):
    rotated_t2 = quat_rotate(q1, t2)

    translation = [
        t1[0] + rotated_t2[0],
        t1[1] + rotated_t2[1],
        t1[2] + rotated_t2[2],
    ]
    rotation = quat_normalize(quat_multiply(q1, q2))

    return translation, rotation


def inverse_transform(translation, rotation):
    q_inv = quat_inverse(rotation)
    inv_translation = quat_rotate(
        q_inv,
        [-translation[0], -translation[1], -translation[2]],
    )
    return inv_translation, q_inv


def make_pose(translation, rotation):
    pose = Pose()
    pose.position.x = float(translation[0])
    pose.position.y = float(translation[1])
    pose.position.z = float(translation[2])
    pose.orientation.x = float(rotation[0])
    pose.orientation.y = float(rotation[1])
    pose.orientation.z = float(rotation[2])
    pose.orientation.w = float(rotation[3])
    return pose


def duration_to_seconds(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) / 1_000_000_000.0


def duration_from_seconds(seconds):
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    return DurationMsg(sec=whole_seconds, nanosec=nanoseconds)


def sanitize_filename(value):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return cleaned.strip("_") or "pose"


class CameraPoseCapture(Node):
    def __init__(self):
        super().__init__("camera_pose_capture")

        self.declare_parameter("execute", False)
        self.declare_parameter("save_depth", True)
        self.declare_parameter("output_dir", "~/Desktop/fst_test_fa_ws_arm/camera_pose_captures")
        self.declare_parameter("settle_seconds", 2.0)
        self.declare_parameter("step_mm", 5.0)
        self.declare_parameter("group_name", "fairino5_v6_group")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mount_link", "wrist2_link")
        self.declare_parameter("control_link", "wrist3_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("lock_joints", ["j6"])
        self.declare_parameter("joint_lock_tolerance_deg", 0.1)
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("cartesian_fraction_threshold", 0.99)
        self.declare_parameter("return_to_center", True)
        self.declare_parameter("apriltag_max_age_sec", 0.5)
        self.declare_parameter("apriltag_frame", "tag36h11_0")
        self.declare_parameter("apriltag_wait_timeout_sec", 3.0)

        self.execute = self.get_bool_parameter("execute")
        self.save_depth = self.get_bool_parameter("save_depth")
        self.output_root = Path(self.get_string_parameter("output_dir")).expanduser()
        self.settle_seconds = self.get_float_parameter("settle_seconds")
        self.step_mm = self.get_float_parameter("step_mm")
        self.group_name = self.get_string_parameter("group_name")
        self.base_frame = self.get_string_parameter("base_frame")
        self.mount_link = self.get_string_parameter("mount_link").strip()
        self.control_link = self.get_string_parameter("control_link")
        self.camera_frame = self.get_string_parameter("camera_frame")
        self.lock_joints = self.get_string_list_parameter("lock_joints")

        if len(self.lock_joints) == 1 and str(self.lock_joints[0]).strip().lower() in ("none", "false", "off", "disable", "disabled", "no"):
            self.lock_joints = []
        self.joint_lock_tolerance_deg = self.get_float_parameter("joint_lock_tolerance_deg")
        self.joint_lock_tolerance_rad = math.radians(self.joint_lock_tolerance_deg)
        self.rgb_topic = self.get_string_parameter("rgb_topic")
        self.depth_topic = self.get_string_parameter("depth_topic")
        self.camera_info_topic = self.get_string_parameter("camera_info_topic")
        self.cartesian_fraction_threshold = self.get_float_parameter("cartesian_fraction_threshold")
        self.return_to_center = self.get_bool_parameter("return_to_center")
        self.apriltag_max_age_sec = self.get_float_parameter("apriltag_max_age_sec")
        self.apriltag_frame = self.get_string_parameter("apriltag_frame").strip()
        self.apriltag_wait_timeout_sec = self.get_float_parameter("apriltag_wait_timeout_sec")
        self.apriltag_topic = APRILTAG_DETECTIONS_TOPIC

        if self.step_mm <= 0.0:
            raise ValueError("step_mm must be greater than zero")
        if self.settle_seconds < 0.0:
            raise ValueError("settle_seconds must be non-negative")
        if not 0.0 < self.cartesian_fraction_threshold <= 1.0:
            raise ValueError("cartesian_fraction_threshold must be in (0.0, 1.0]")
        if self.joint_lock_tolerance_deg < 0.0:
            raise ValueError("joint_lock_tolerance_deg must be non-negative")
        if self.apriltag_max_age_sec < 0.0:
            raise ValueError("apriltag_max_age_sec must be non-negative")
        if not self.apriltag_frame:
            raise ValueError("apriltag_frame must not be empty")
        if self.apriltag_wait_timeout_sec < 0.0:
            raise ValueError("apriltag_wait_timeout_sec must be non-negative")

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_joint_state = None
        self.latest_apriltag_detection_array = None
        self.latest_apriltag_detection_received_time_sec = None

        self.tf_buffer = None
        self.tf_listener = None
        self.cartesian_client = None
        self.execute_client = None

    def get_bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def get_float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def get_string_parameter(self, name):
        return str(self.get_parameter(name).value)

    def get_string_list_parameter(self, name):
        value = self.get_parameter(name).value

        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            return [str(item).strip() for item in value if str(item).strip()]
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return []
            if text.startswith("[") and text.endswith("]"):
                text = text[1:-1]
            return [
                part.strip().strip("'\"")
                for part in text.split(",")
                if part.strip().strip("'\"")
            ]

        text = str(value).strip()
        return [text] if text else []

    def mount_link_compensation_active(self):
        return bool(self.mount_link) and self.mount_link != self.control_link

    def build_pose_sequence(self):
        step_m = self.step_mm / 1000.0
        sequence = [("center_start", 0.0, 0.0, 0.0)]

        offsets = [
            ("camera_right", +step_m, 0.0, 0.0),
            ("camera_left", -step_m, 0.0, 0.0),
            ("camera_up", 0.0, -step_m, 0.0),
            ("camera_down", 0.0, +step_m, 0.0),
            ("camera_forward", 0.0, 0.0, +step_m),
            ("camera_back", 0.0, 0.0, -step_m),
        ]

        for name, dx, dy, dz in offsets:
            sequence.append((name, dx, dy, dz))
            if self.return_to_center:
                sequence.append(("center", 0.0, 0.0, 0.0))

        return sequence

    def print_config(self, pose_sequence, run_dir=None):
        print("")
        print("camera_pose_capture configuration")
        print("=================================")
        print(f"execute: {self.execute}")
        print(f"output_dir: {self.output_root}")
        if run_dir is not None:
            print(f"run_dir: {run_dir}")
        print(f"save_depth: {self.save_depth}")
        print(f"settle_seconds: {self.settle_seconds:.3f}")
        print(f"step_mm: {self.step_mm:.3f}")
        print(f"group_name: {self.group_name}")
        print(f"base_frame: {self.base_frame}")
        print(f"mount_link: {self.mount_link}")
        print(f"mount_link_compensation_active: {self.mount_link_compensation_active()}")
        print(f"control_link: {self.control_link}")
        print(f"camera_frame: {self.camera_frame}")
        print(f"lock_joints: {self.lock_joints}")
        print(f"joint_lock_tolerance_deg: {self.joint_lock_tolerance_deg:.3f}")
        print(f"rgb_topic: {self.rgb_topic}")
        print(f"depth_topic: {self.depth_topic}")
        print(f"camera_info_topic: {self.camera_info_topic}")
        print(f"cartesian_fraction_threshold: {self.cartesian_fraction_threshold:.3f}")
        print(f"return_to_center: {self.return_to_center}")
        print(f"apriltag_topic: {self.apriltag_topic}")
        print(f"apriltag_max_age_sec: {self.apriltag_max_age_sec:.3f}")
        print(f"apriltag_frame: {self.apriltag_frame}")
        print(f"apriltag_wait_timeout_sec: {self.apriltag_wait_timeout_sec:.3f}")
        print("")
        print("Pose sequence uses camera optical-frame offsets:")
        print("  +X = camera_right")
        print("  -X = camera_left")
        print("  -Y = camera_up")
        print("  +Y = camera_down")
        print("  +Z = camera_forward")
        print("  -Z = camera_back")
        print("")

        for index, (name, dx, dy, dz) in enumerate(pose_sequence):
            print(
                f"{index:02d} {name}: "
                f"dx={dx * 1000.0:+.3f} mm, "
                f"dy={dy * 1000.0:+.3f} mm, "
                f"dz={dz * 1000.0:+.3f} mm"
            )

        print("")

    def setup_runtime_interfaces(self):
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, qos_profile_sensor_data)

        if self.save_depth:
            self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(
            AprilTagDetectionArray,
            self.apriltag_topic,
            self.apriltag_detection_callback,
            qos_profile_sensor_data,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def rgb_callback(self, message):
        self.latest_rgb = message

    def depth_callback(self, message):
        self.latest_depth = message

    def camera_info_callback(self, message):
        self.latest_camera_info = message

    def joint_state_callback(self, message):
        self.latest_joint_state = message

    def apriltag_detection_callback(self, message):
        self.latest_apriltag_detection_array = message
        self.latest_apriltag_detection_received_time_sec = self.now_seconds()

    def wait_for_runtime_inputs(self):
        self.wait_for_message(lambda: self.latest_rgb is not None, "RGB image", 15.0)

        if self.save_depth:
            self.wait_for_message(lambda: self.latest_depth is not None, "depth image", 15.0)

        self.wait_for_message(lambda: self.latest_joint_state is not None, "/joint_states", 15.0)
        self.wait_for_required_transforms(timeout_sec=15.0)

        self.get_logger().info("Waiting for /compute_cartesian_path service")
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_cartesian_path service is not available")

        self.get_logger().info("Waiting for /execute_trajectory action")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory action is not available")

    def wait_for_message(self, predicate, label, timeout_sec):
        self.get_logger().info(f"Waiting for {label}")
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout_sec:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(f"Timed out waiting for {label}")

    def wait_for_required_transforms(self, timeout_sec):
        required = []

        def add_required(target_frame, source_frame):
            if not target_frame or not source_frame or target_frame == source_frame:
                return
            pair = (target_frame, source_frame)
            if pair not in required:
                required.append(pair)

        add_required(self.base_frame, self.camera_frame)
        add_required(self.base_frame, self.control_link)

        if self.mount_link_compensation_active():
            add_required(self.base_frame, self.mount_link)
        else:
            add_required(self.control_link, self.camera_frame)

        start_time = time.time()
        missing = []

        while rclpy.ok() and time.time() - start_time < timeout_sec:
            missing = []

            for target_frame, source_frame in required:
                try:
                    self.lookup_transform(target_frame, source_frame, timeout_sec=0.2)
                except Exception:
                    missing.append(f"{target_frame} -> {source_frame}")

            if not missing:
                return

            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError("Timed out waiting for required TF: " + ", ".join(missing))

    def lookup_transform(self, target_frame, source_frame, timeout_sec=1.0):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=RclpyDuration(seconds=timeout_sec),
        )

    def lookup_transform_lists(self, target_frame, source_frame):
        transform = self.lookup_transform(target_frame, source_frame)
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        return (
            [float(translation.x), float(translation.y), float(translation.z)],
            quat_normalize([float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]),
        )

    def build_motion_reference(self):
        base_to_camera_translation, base_to_camera_rotation = self.lookup_transform_lists(
            self.base_frame,
            self.camera_frame,
        )

        if not self.mount_link_compensation_active():
            control_to_camera_translation, control_to_camera_rotation = self.lookup_transform_lists(
                self.control_link,
                self.camera_frame,
            )
            inverse_control_to_camera_translation, inverse_control_to_camera_rotation = inverse_transform(
                control_to_camera_translation,
                control_to_camera_rotation,
            )

            return {
                "base_to_camera": (base_to_camera_translation, base_to_camera_rotation),
                "mount_to_camera": None,
                "mount_to_control": None,
                "inverse_control_to_camera": (
                    inverse_control_to_camera_translation,
                    inverse_control_to_camera_rotation,
                ),
            }

        base_to_mount_translation, base_to_mount_rotation = self.lookup_transform_lists(
            self.base_frame,
            self.mount_link,
        )
        base_to_control_translation, base_to_control_rotation = self.lookup_transform_lists(
            self.base_frame,
            self.control_link,
        )

        inverse_base_to_mount_translation, inverse_base_to_mount_rotation = inverse_transform(
            base_to_mount_translation,
            base_to_mount_rotation,
        )
        mount_to_camera_translation, mount_to_camera_rotation = compose_transform(
            inverse_base_to_mount_translation,
            inverse_base_to_mount_rotation,
            base_to_camera_translation,
            base_to_camera_rotation,
        )
        mount_to_control_translation, mount_to_control_rotation = compose_transform(
            inverse_base_to_mount_translation,
            inverse_base_to_mount_rotation,
            base_to_control_translation,
            base_to_control_rotation,
        )

        return {
            "base_to_camera": (base_to_camera_translation, base_to_camera_rotation),
            "mount_to_camera": (mount_to_camera_translation, mount_to_camera_rotation),
            "mount_to_control": (mount_to_control_translation, mount_to_control_rotation),
            "inverse_control_to_camera": None,
        }

    def compute_target_poses(
        self,
        motion_reference,
        camera_offset,
    ):
        start_camera_translation, start_camera_rotation = motion_reference["base_to_camera"]
        base_delta = quat_rotate(start_camera_rotation, camera_offset)
        target_camera_translation = [
            start_camera_translation[0] + base_delta[0],
            start_camera_translation[1] + base_delta[1],
            start_camera_translation[2] + base_delta[2],
        ]
        target_camera_rotation = list(start_camera_rotation)

        if self.mount_link_compensation_active():
            mount_to_camera_translation, mount_to_camera_rotation = motion_reference[
                "mount_to_camera"
            ]
            mount_to_control_translation, mount_to_control_rotation = motion_reference[
                "mount_to_control"
            ]
            camera_to_mount_translation, camera_to_mount_rotation = inverse_transform(
                mount_to_camera_translation,
                mount_to_camera_rotation,
            )
            target_mount_translation, target_mount_rotation = compose_transform(
                target_camera_translation,
                target_camera_rotation,
                camera_to_mount_translation,
                camera_to_mount_rotation,
            )
            target_control_translation, target_control_rotation = compose_transform(
                target_mount_translation,
                target_mount_rotation,
                mount_to_control_translation,
                mount_to_control_rotation,
            )
        else:
            inverse_control_to_camera_translation, inverse_control_to_camera_rotation = (
                motion_reference["inverse_control_to_camera"]
            )
            target_control_translation, target_control_rotation = compose_transform(
                target_camera_translation,
                target_camera_rotation,
                inverse_control_to_camera_translation,
                inverse_control_to_camera_rotation,
            )
            target_mount_translation = None
            target_mount_rotation = None

            if self.mount_link:
                target_mount_translation = list(target_control_translation)
                target_mount_rotation = list(target_control_rotation)

        return {
            "camera": (target_camera_translation, target_camera_rotation),
            "mount": (
                (target_mount_translation, target_mount_rotation)
                if target_mount_translation is not None
                else None
            ),
            "control": (target_control_translation, target_control_rotation),
        }

    def get_fresh_joint_state(self):
        rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_joint_state is None:
            raise RuntimeError("No /joint_states message is available")

        return self.latest_joint_state

    def build_joint_lock_constraints(self, joint_state):
        locked_positions = []
        lock_metadata = {
            "lock_joints": list(self.lock_joints),
            "joint_lock_tolerance_deg": float(self.joint_lock_tolerance_deg),
            "joint_lock_tolerance_rad": float(self.joint_lock_tolerance_rad),
            "locked_joint_positions": locked_positions,
            "path_constraints": None,
        }

        if not self.lock_joints:
            return None, lock_metadata

        joint_positions = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        missing_joints = [
            joint_name
            for joint_name in self.lock_joints
            if joint_name not in joint_positions
        ]

        if missing_joints:
            available = ", ".join(joint_state.name)
            missing = ", ".join(missing_joints)
            raise RuntimeError(
                f"Cannot lock joint(s) not present in /joint_states: {missing}. "
                f"Available joints: {available}"
            )

        constraints = Constraints()
        constraints.name = "camera_pose_capture_joint_lock"

        for joint_name in self.lock_joints:
            position_rad = joint_positions[joint_name]
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position_rad
            joint_constraint.tolerance_above = self.joint_lock_tolerance_rad
            joint_constraint.tolerance_below = self.joint_lock_tolerance_rad
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

            locked_positions.append(
                {
                    "name": joint_name,
                    "radians": position_rad,
                    "degrees": math.degrees(position_rad),
                }
            )

        lock_metadata["path_constraints"] = {
            "name": constraints.name,
            "joint_constraints": [
                {
                    "joint_name": joint_constraint.joint_name,
                    "position_rad": float(joint_constraint.position),
                    "position_deg": math.degrees(joint_constraint.position),
                    "tolerance_above_rad": float(joint_constraint.tolerance_above),
                    "tolerance_above_deg": math.degrees(joint_constraint.tolerance_above),
                    "tolerance_below_rad": float(joint_constraint.tolerance_below),
                    "tolerance_below_deg": math.degrees(joint_constraint.tolerance_below),
                    "weight": float(joint_constraint.weight),
                }
                for joint_constraint in constraints.joint_constraints
            ],
        }

        return constraints, lock_metadata

    def compute_cartesian_path(self, target_control_translation, target_control_rotation):
        current_joint_state = self.get_fresh_joint_state()
        path_constraints, lock_metadata = self.build_joint_lock_constraints(current_joint_state)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state = current_joint_state
        request.start_state.is_diff = False
        request.group_name = self.group_name
        request.link_name = self.control_link
        request.waypoints = [make_pose(target_control_translation, target_control_rotation)]
        request.max_step = 0.002
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        if path_constraints is not None:
            request.path_constraints = path_constraints

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            raise RuntimeError("/compute_cartesian_path timed out")

        response = future.result()
        if response is None:
            raise RuntimeError("/compute_cartesian_path returned no response")

        return response, lock_metadata

    def ensure_trajectory_timing(self, robot_trajectory):
        points = robot_trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("Cartesian path returned an empty joint trajectory")

        times = [duration_to_seconds(point.time_from_start) for point in points]
        already_timed = (
            all(times[index] > times[index - 1] for index in range(1, len(times)))
            and times[-1] > 0.0
        )

        if already_timed:
            return False

        total_duration = max(3.0, 0.10 * len(points))

        if len(points) == 1:
            points[0].time_from_start = duration_from_seconds(total_duration)
            return True

        for index, point in enumerate(points):
            point.time_from_start = duration_from_seconds(
                total_duration * float(index + 1) / float(len(points))
            )

        return True

    def execute_trajectory(self, robot_trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory

        send_goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=15.0)

        if not send_goal_future.done():
            raise RuntimeError("Timed out sending /execute_trajectory goal")

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            raise RuntimeError("/execute_trajectory returned no goal handle")
        if not goal_handle.accepted:
            return {"accepted": False, "error_code": None, "success": False}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        if not result_future.done():
            raise RuntimeError("Timed out waiting for /execute_trajectory result")

        result_wrapper = result_future.result()
        if result_wrapper is None:
            raise RuntimeError("/execute_trajectory returned no result")

        error_code = int(result_wrapper.result.error_code.val)

        return {
            "accepted": True,
            "error_code": error_code,
            "success": error_code == MOVEIT_SUCCESS,
        }

    def plan_and_execute_pose(
        self,
        target_control_translation,
        target_control_rotation,
        pose_name=None,
        camera_offset=None,
    ):
        response, lock_metadata = self.compute_cartesian_path(
            target_control_translation,
            target_control_rotation,
        )
        error_code = int(response.error_code.val)
        fraction = float(response.fraction)
        trajectory_points = len(response.solution.joint_trajectory.points)

        cartesian_result = {
            "fraction": fraction,
            "error_code": error_code,
            "trajectory_points": trajectory_points,
        }

        if pose_name is not None and camera_offset is not None:
            locked_joint_names = [
                entry["name"]
                for entry in lock_metadata["locked_joint_positions"]
            ]
            locked_joint_values = ", ".join(
                f"{entry['name']}={entry['radians']:.6f} rad "
                f"({entry['degrees']:.3f} deg)"
                for entry in lock_metadata["locked_joint_positions"]
            )
            if not locked_joint_values:
                locked_joint_values = "none"

            self.get_logger().info(
                f"Cartesian path {pose_name}: "
                f"requested_camera_offset_mm=("
                f"{camera_offset[0] * 1000.0:+.3f}, "
                f"{camera_offset[1] * 1000.0:+.3f}, "
                f"{camera_offset[2] * 1000.0:+.3f}), "
                f"mount_link={self.mount_link}, "
                f"control_link={self.control_link}, "
                f"mount_link_compensation_active={self.mount_link_compensation_active()}, "
                f"locked_joints={locked_joint_names}, "
                f"locked_joint_targets={locked_joint_values}, "
                f"fraction={fraction:.3f}"
            )

        if error_code != MOVEIT_SUCCESS:
            raise RuntimeError(f"Cartesian path failed with MoveIt error code {error_code}")

        if fraction < self.cartesian_fraction_threshold:
            raise RuntimeError(
                f"Cartesian path fraction {fraction:.3f} is below threshold "
                f"{self.cartesian_fraction_threshold:.3f}"
            )

        timing_added = self.ensure_trajectory_timing(response.solution)
        cartesian_result["timing_added"] = timing_added

        execute_result = self.execute_trajectory(response.solution)
        if not execute_result["success"]:
            raise RuntimeError(f"/execute_trajectory failed: {execute_result}")

        return {
            "cartesian_path": cartesian_result,
            "execute_trajectory": execute_result,
            "lock_joints": lock_metadata["lock_joints"],
            "joint_lock_tolerance_deg": lock_metadata["joint_lock_tolerance_deg"],
            "joint_lock_tolerance_rad": lock_metadata["joint_lock_tolerance_rad"],
            "locked_joint_positions": lock_metadata["locked_joint_positions"],
            "path_constraints": lock_metadata["path_constraints"],
        }

    def offset_to_dict(self, camera_offset):
        return {
            "meters": {
                "x": float(camera_offset[0]),
                "y": float(camera_offset[1]),
                "z": float(camera_offset[2]),
            },
            "millimeters": {
                "x": float(camera_offset[0]) * 1000.0,
                "y": float(camera_offset[1]) * 1000.0,
                "z": float(camera_offset[2]) * 1000.0,
            },
        }

    def pose_tuple_to_dict(self, frame_id, pose_tuple):
        if pose_tuple is None:
            return None

        translation, rotation = pose_tuple

        return {
            "frame_id": frame_id,
            "position": {
                "x": float(translation[0]),
                "y": float(translation[1]),
                "z": float(translation[2]),
            },
            "orientation": {
                "x": float(rotation[0]),
                "y": float(rotation[1]),
                "z": float(rotation[2]),
                "w": float(rotation[3]),
            },
        }

    def now_seconds(self):
        return float(self.get_clock().now().nanoseconds) / 1_000_000_000.0

    def stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def transform_age_sec(self, transform):
        stamp_sec = self.stamp_to_seconds(transform.header.stamp)

        if stamp_sec <= 0.0:
            return None

        return max(0.0, self.now_seconds() - stamp_sec)

    def spin_for_duration(self, duration_sec, spin_timeout_sec=0.05):
        deadline = time.monotonic() + max(0.0, duration_sec)

        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(
                self,
                timeout_sec=min(spin_timeout_sec, remaining),
            )

        rclpy.spin_once(self, timeout_sec=0.0)

    def point_to_dict(self, point):
        if point is None:
            return None

        output = {
            "x": float(point.x),
            "y": float(point.y),
        }

        if hasattr(point, "z"):
            output["z"] = float(point.z)

        return output

    def json_scalar_or_list(self, value):
        if value is None or isinstance(value, (bool, int, float, str)):
            return value

        if hasattr(value, "tolist"):
            return self.json_scalar_or_list(value.tolist())

        if isinstance(value, (list, tuple)):
            return [self.json_scalar_or_list(item) for item in value]

        return str(value)

    def detection_to_dict(self, detection):
        center = None
        center_field = None

        if hasattr(detection, "center"):
            center = getattr(detection, "center")
            center_field = "center"
        elif hasattr(detection, "centre"):
            center = getattr(detection, "centre")
            center_field = "centre"

        corners = None
        if hasattr(detection, "corners"):
            corners = [self.point_to_dict(corner) for corner in detection.corners]

        return {
            "family": getattr(detection, "family", None),
            "id": self.json_scalar_or_list(getattr(detection, "id", None)),
            "hamming": (
                int(detection.hamming)
                if hasattr(detection, "hamming")
                else None
            ),
            "decision_margin": (
                float(detection.decision_margin)
                if hasattr(detection, "decision_margin")
                else None
            ),
            "center_field": center_field,
            "center": self.point_to_dict(center),
            "corners": corners,
        }

    def collect_apriltag_metadata(self):
        message = self.latest_apriltag_detection_array
        metadata = {
            "apriltag_detection_available": False,
            "apriltag_detection_fresh": False,
            "apriltag_detection_age_sec": None,
            "apriltag_detections": 0,
            "apriltag_first_detection": None,
        }

        if message is None:
            return metadata

        header_stamp_sec = self.stamp_to_seconds(message.header.stamp)
        stamp_sec = (
            header_stamp_sec
            if header_stamp_sec > 0.0
            else self.latest_apriltag_detection_received_time_sec
        )

        age_sec = None
        if stamp_sec is not None:
            age_sec = max(0.0, self.now_seconds() - stamp_sec)

        detections = list(getattr(message, "detections", []))
        detection_is_fresh = (
            age_sec is not None
            and age_sec <= self.apriltag_max_age_sec
        )

        metadata["apriltag_detection_fresh"] = bool(detection_is_fresh)
        metadata["apriltag_detection_age_sec"] = age_sec
        metadata["apriltag_detections"] = len(detections)

        if not detection_is_fresh:
            return metadata

        metadata["apriltag_detection_available"] = len(detections) > 0

        if detections:
            metadata["apriltag_first_detection"] = self.detection_to_dict(detections[0])

        return metadata

    def lookup_camera_to_tag_dict(self, timeout_sec=0.2):
        output = {
            "source": "tf",
            "parent_frame": self.camera_frame,
            "child_frame": self.apriltag_frame,
            "lookup_success": False,
            "lookup_fresh": False,
            "lookup_age_sec": None,
            "error": None,
            "stamp": None,
            "translation": None,
            "rotation": None,
        }

        try:
            transform = self.lookup_transform(
                self.camera_frame,
                self.apriltag_frame,
                timeout_sec=timeout_sec,
            )
        except Exception as error:
            output["error"] = str(error)
            return output

        transform_dict = self.transform_to_dict(transform)
        lookup_age_sec = self.transform_age_sec(transform)
        lookup_fresh = (
            lookup_age_sec is None
            or lookup_age_sec <= self.apriltag_max_age_sec
        )
        output.update(
            {
                "lookup_success": True,
                "lookup_fresh": bool(lookup_fresh),
                "lookup_age_sec": lookup_age_sec,
                "parent_frame": transform_dict["target_frame"],
                "child_frame": transform_dict["source_frame"],
                "stamp": transform_dict["stamp"],
                "translation": transform_dict["translation"],
                "rotation": transform_dict["rotation"],
            }
        )

        return output

    def wait_for_fresh_apriltag_data(self):
        deadline = time.monotonic() + self.apriltag_wait_timeout_sec
        last_apriltag_metadata = self.collect_apriltag_metadata()
        last_camera_to_tag = self.lookup_camera_to_tag_dict(timeout_sec=0.0)

        while rclpy.ok():
            last_apriltag_metadata = self.collect_apriltag_metadata()
            if last_apriltag_metadata["apriltag_detection_available"]:
                return {
                    "apriltag_wait_succeeded": True,
                    "apriltag_wait_reason": "fresh_detection",
                }

            last_camera_to_tag = self.lookup_camera_to_tag_dict(timeout_sec=0.02)
            if last_camera_to_tag["lookup_success"] and last_camera_to_tag["lookup_fresh"]:
                return {
                    "apriltag_wait_succeeded": True,
                    "apriltag_wait_reason": "fresh_camera_to_tag_tf",
                }

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break

            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        self.get_logger().warn(
            "Fresh AprilTag data was not available before capture; saving anyway. "
            f"timeout_sec={self.apriltag_wait_timeout_sec:.3f}, "
            f"latest_detection_age_sec="
            f"{last_apriltag_metadata['apriltag_detection_age_sec']}, "
            f"camera_to_tag_lookup_success={last_camera_to_tag['lookup_success']}, "
            f"camera_to_tag_lookup_age_sec={last_camera_to_tag['lookup_age_sec']}, "
            f"camera_to_tag_error={last_camera_to_tag['error']}"
        )

        return {
            "apriltag_wait_succeeded": False,
            "apriltag_wait_reason": "timeout",
        }

    def save_capture(self, run_dir, pose_index, pose_name, motion_result):
        safe_name = sanitize_filename(pose_name)
        prefix = f"{pose_index:02d}_{safe_name}"
        apriltag_metadata = self.collect_apriltag_metadata()
        camera_to_tag = self.lookup_camera_to_tag_dict()

        rgb_filename = f"{prefix}_rgb.png"
        rgb_path = run_dir / rgb_filename

        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding="bgr8")
        if not cv2.imwrite(str(rgb_path), rgb_image):
            raise RuntimeError(f"Failed to write RGB image: {rgb_path}")

        depth_filename = None
        depth_encoding_note = None

        if self.save_depth and self.latest_depth is not None:
            depth_filename = f"{prefix}_depth.png"
            depth_path = run_dir / depth_filename
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")

            if np.issubdtype(depth_image.dtype, np.floating):
                depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
                depth_image = np.clip(depth_image * 1000.0, 0.0, 65535.0).astype(np.uint16)
                depth_encoding_note = "float depth converted to uint16 millimeters for PNG"
            else:
                depth_encoding_note = f"saved with dtype {depth_image.dtype}"

            if not cv2.imwrite(str(depth_path), depth_image):
                raise RuntimeError(f"Failed to write depth image: {depth_path}")

        metadata_filename = f"{prefix}_metadata.json"
        metadata_path = run_dir / metadata_filename
        apriltag_wait = motion_result.get("apriltag_wait", {})

        metadata = {
            "pose_name": pose_name,
            "pose_index": pose_index,
            "timestamp": {
                "unix_seconds": time.time(),
                "iso8601_local": datetime.now().isoformat(timespec="seconds"),
            },
            "execute": self.execute,
            "step_mm": self.step_mm,
            "settle_seconds": self.settle_seconds,
            "group_name": self.group_name,
            "base_frame": self.base_frame,
            "mount_link": self.mount_link,
            "mount_link_compensation_active": self.mount_link_compensation_active(),
            "control_link": self.control_link,
            "camera_frame": self.camera_frame,
            "lock_joints": motion_result.get("lock_joints", list(self.lock_joints)),
            "joint_lock_tolerance_deg": motion_result.get(
                "joint_lock_tolerance_deg",
                self.joint_lock_tolerance_deg,
            ),
            "locked_joint_positions": motion_result.get("locked_joint_positions"),
            "path_constraints": motion_result.get("path_constraints"),
            "requested_camera_offset": motion_result.get("requested_camera_offset"),
            "target_camera_pose": motion_result.get("target_camera_pose"),
            "target_mount_pose": motion_result.get("target_mount_pose"),
            "target_control_pose": motion_result.get("target_control_pose"),
            "camera_topics": {
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
                "camera_info": self.camera_info_topic,
                "apriltag_detections": self.apriltag_topic,
            },
            "settle_spin_enabled": True,
            "apriltag_max_age_sec": self.apriltag_max_age_sec,
            "apriltag_frame": self.apriltag_frame,
            "apriltag_wait_timeout_sec": self.apriltag_wait_timeout_sec,
            "apriltag_wait_succeeded": apriltag_wait.get("apriltag_wait_succeeded", False),
            "apriltag_wait_reason": apriltag_wait.get("apriltag_wait_reason", "unknown"),
            **apriltag_metadata,
            "camera_to_tag": camera_to_tag,
            "joint_states": self.joint_state_to_dict(self.latest_joint_state),
            "tf_snapshots": self.collect_tf_snapshots(),
            "cartesian_path": motion_result.get("cartesian_path"),
            "execute_trajectory": motion_result.get("execute_trajectory"),
            "output_files": {
                "rgb": rgb_filename,
                "depth": depth_filename,
                "metadata": metadata_filename,
            },
            "depth_encoding_note": depth_encoding_note,
            "camera_info": self.camera_info_to_dict(self.latest_camera_info),
        }

        with metadata_path.open("w", encoding="utf-8") as file:
            json.dump(metadata, file, indent=2)

        self.get_logger().info(f"Saved capture {pose_index:02d} {pose_name} to {run_dir}")

    def collect_tf_snapshots(self):
        pairs = [
            ("base_to_wrist2", self.base_frame, "wrist2_link"),
            ("base_to_wrist3", self.base_frame, "wrist3_link"),
            ("base_to_camera", self.base_frame, self.camera_frame),
            ("base_to_control", self.base_frame, self.control_link),
        ]

        output = {}
        for name, target_frame, source_frame in pairs:
            output[name] = self.lookup_transform_dict(target_frame, source_frame)

        output["base_to_mount"] = (
            self.lookup_transform_dict(self.base_frame, self.mount_link)
            if self.mount_link
            else None
        )
        output["mount_to_camera"] = (
            self.lookup_transform_dict(self.mount_link, self.camera_frame)
            if self.mount_link
            else None
        )
        output["mount_to_control"] = (
            self.lookup_transform_dict(self.mount_link, self.control_link)
            if self.mount_link
            else None
        )

        return output

    def lookup_transform_dict(self, target_frame, source_frame):
        if target_frame == source_frame:
            stamp = self.get_clock().now().to_msg()
            return self.transform_lists_to_dict(
                target_frame,
                source_frame,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                stamp,
            )

        transform = self.lookup_transform(target_frame, source_frame)
        return self.transform_to_dict(transform)

    def transform_to_dict(self, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp

        return self.transform_lists_to_dict(
            transform.header.frame_id,
            transform.child_frame_id,
            [float(translation.x), float(translation.y), float(translation.z)],
            quat_normalize([float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]),
            stamp,
        )

    def transform_lists_to_dict(self, target_frame, source_frame, translation, rotation, stamp):
        return {
            "target_frame": target_frame,
            "source_frame": source_frame,
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "translation": {
                "x": float(translation[0]),
                "y": float(translation[1]),
                "z": float(translation[2]),
            },
            "rotation": {
                "x": float(rotation[0]),
                "y": float(rotation[1]),
                "z": float(rotation[2]),
                "w": float(rotation[3]),
            },
        }

    def joint_state_to_dict(self, joint_state):
        if joint_state is None:
            return None

        stamp = joint_state.header.stamp

        return {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "name": list(joint_state.name),
            "position": [float(value) for value in joint_state.position],
            "velocity": [float(value) for value in joint_state.velocity],
            "effort": [float(value) for value in joint_state.effort],
        }

    def camera_info_to_dict(self, camera_info):
        if camera_info is None:
            return None

        return {
            "width": int(camera_info.width),
            "height": int(camera_info.height),
            "distortion_model": camera_info.distortion_model,
            "d": [float(value) for value in camera_info.d],
            "k": [float(value) for value in camera_info.k],
            "r": [float(value) for value in camera_info.r],
            "p": [float(value) for value in camera_info.p],
        }

    def run(self):
        pose_sequence = self.build_pose_sequence()
        self.output_root.mkdir(parents=True, exist_ok=True)

        if not self.execute:
            self.print_config(pose_sequence)
            print("DRY RUN COMPLETE: no MoveIt services/actions were created or called.")
            return

        run_dir = self.output_root / f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        run_dir.mkdir(parents=True, exist_ok=False)

        self.print_config(pose_sequence, run_dir=run_dir)
        self.setup_runtime_interfaces()
        self.wait_for_runtime_inputs()
        motion_reference = self.build_motion_reference()

        for pose_index, (pose_name, dx, dy, dz) in enumerate(pose_sequence):
            camera_offset = [dx, dy, dz]
            motion_result = {
                "cartesian_path": None,
                "execute_trajectory": None,
                "requested_camera_offset": self.offset_to_dict(camera_offset),
                "target_camera_pose": None,
                "target_mount_pose": None,
                "target_control_pose": None,
                "lock_joints": list(self.lock_joints),
                "joint_lock_tolerance_deg": self.joint_lock_tolerance_deg,
                "locked_joint_positions": None,
                "path_constraints": None,
            }

            if pose_index > 0:
                target_poses = self.compute_target_poses(motion_reference, camera_offset)
                target_control_translation, target_control_rotation = target_poses["control"]

                motion_result["target_camera_pose"] = self.pose_tuple_to_dict(
                    self.base_frame,
                    target_poses["camera"],
                )
                motion_result["target_mount_pose"] = self.pose_tuple_to_dict(
                    self.base_frame,
                    target_poses["mount"],
                )
                motion_result["target_control_pose"] = self.pose_tuple_to_dict(
                    self.base_frame,
                    target_poses["control"],
                )

                self.get_logger().info(
                    f"Planning/executing {pose_index:02d} {pose_name}: "
                    f"dx={dx * 1000.0:+.3f} mm, "
                    f"dy={dy * 1000.0:+.3f} mm, "
                    f"dz={dz * 1000.0:+.3f} mm, "
                    f"mount_link={self.mount_link}, "
                    f"control_link={self.control_link}, "
                    f"mount_link_compensation_active={self.mount_link_compensation_active()}"
                )
                plan_result = self.plan_and_execute_pose(
                    target_control_translation,
                    target_control_rotation,
                    pose_name=pose_name,
                    camera_offset=camera_offset,
                )
                motion_result.update(plan_result)

            self.get_logger().info(
                f"Settling for {self.settle_seconds:.3f} seconds with ROS callback spinning"
            )
            self.spin_for_duration(self.settle_seconds)

            self.get_logger().info(
                f"Waiting up to {self.apriltag_wait_timeout_sec:.3f} seconds "
                "for fresh AprilTag detection or camera_to_tag TF"
            )
            motion_result["apriltag_wait"] = self.wait_for_fresh_apriltag_data()

            self.save_capture(run_dir, pose_index, pose_name, motion_result)

        print("")
        print(f"Capture complete. Output: {run_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0

    try:
        node = CameraPoseCapture()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"camera_pose_capture failed: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
