#!/usr/bin/env python3

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
import tf2_ros

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration

from builtin_interfaces.msg import Duration as BuiltinDuration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

ACTION_NAME = "/fairino5_controller/follow_joint_trajectory"

TF_PAIRS = [
    ("base_to_wrist2", "base_link", "wrist2_link"),
    ("wrist2_to_camera", "wrist2_link", "camera_color_optical_frame"),
    ("camera_to_tag", "camera_color_optical_frame", "tag36h11_0"),
    ("base_to_tag", "base_link", "tag36h11_0"),
]


class AutoJointValidationRecorder(Node):
    def __init__(self, output_csv, execute):
        super().__init__("auto_joint_validation_recorder")

        self.output_csv = Path(output_csv)
        self.execute = execute
        self.latest_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg):
        joint_map = {}

        for name, position in zip(msg.name, msg.position):
            joint_map[name] = position

        if all(name in joint_map for name in JOINT_NAMES):
            self.latest_joint_state = joint_map

    def wait_for_joint_state(self, timeout_sec=10.0):
        self.get_logger().info("Waiting for /joint_states...")

        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_joint_state is not None:
                self.get_logger().info("Joint state received.")
                return dict(self.latest_joint_state)

        raise RuntimeError("Timed out waiting for /joint_states.")

    def wait_for_action_server(self, timeout_sec=10.0):
        self.get_logger().info(f"Waiting for action server: {ACTION_NAME}")

        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Action server not available: {ACTION_NAME}")

        self.get_logger().info("Action server available.")

    def move_to_joint_pose(self, target_joint_pose, move_duration_sec=3):
        target_positions = [target_joint_pose[name] for name in JOINT_NAMES]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = BuiltinDuration(
            sec=int(move_duration_sec),
            nanosec=0,
        )

        goal.trajectory.points.append(point)

        if not self.execute:
            print("")
            print("[DRY RUN] Would move to:")

            for name, pos in zip(JOINT_NAMES, target_positions):
                print(f"  {name}: {pos:+.6f} rad / {math.degrees(pos):+.3f} deg")

            return True

        send_goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Failed to send trajectory goal.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        self.get_logger().info("Trajectory goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error("No result received from trajectory action.")
            return False

        self.get_logger().info(f"Trajectory result code: {result.result.error_code}")

        return result.result.error_code == 0

    def lookup_transform(self, target_frame, source_frame, timeout_sec=5.0):
        start_time = time.time()
        last_error = None

        while rclpy.ok() and time.time() - start_time < timeout_sec:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=RclpyDuration(seconds=0.2),
                )

                return transform

            except Exception as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(
            f"Could not get TF {target_frame} -> {source_frame}: {last_error}"
        )

    def record_sample(self, sample_id, label):
        row = {
            "sample_id": sample_id,
            "label": label,
            "timestamp_sec": time.time(),
        }

        for prefix, target_frame, source_frame in TF_PAIRS:
            transform = self.lookup_transform(target_frame, source_frame)

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            row[f"{prefix}_tx"] = translation.x
            row[f"{prefix}_ty"] = translation.y
            row[f"{prefix}_tz"] = translation.z
            row[f"{prefix}_qx"] = rotation.x
            row[f"{prefix}_qy"] = rotation.y
            row[f"{prefix}_qz"] = rotation.z
            row[f"{prefix}_qw"] = rotation.w

        base_x = row["base_to_tag_tx"]
        base_y = row["base_to_tag_ty"]
        base_z = row["base_to_tag_tz"]

        print("")
        print(f"Recorded sample {sample_id}: {label}")
        print("base_to_tag translation:")
        print(f"  x = {base_x:+.4f} m")
        print(f"  y = {base_y:+.4f} m")
        print(f"  z = {base_z:+.4f} m")

        return row

    def save_rows(self, rows):
        if not rows:
            return

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)

        with self.output_csv.open("w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        print("")
        print(f"Saved {len(rows)} samples to: {self.output_csv}")


def make_target_pose(center_pose, offsets_deg):
    target_pose = dict(center_pose)

    for joint_name, offset_deg in offsets_deg.items():
        target_pose[joint_name] += math.radians(offset_deg)

    return target_pose


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "output_csv",
        help="Output CSV path",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually command the robot. Without this, dry-run only.",
    )

    parser.add_argument(
        "--settle-sec",
        type=float,
        default=2.0,
        help="Seconds to wait after each move before recording.",
    )

    parser.add_argument(
        "--move-sec",
        type=int,
        default=3,
        help="Trajectory duration in seconds.",
    )

    parser.add_argument(
        "--step-deg",
        type=float,
        default=0.5,
        help="Joint offset size in degrees.",
    )

    args = parser.parse_args()

    rclpy.init()

    node = AutoJointValidationRecorder(
        output_csv=args.output_csv,
        execute=args.execute,
    )

    try:
        center_pose = node.wait_for_joint_state()
        node.wait_for_action_server()

        step = args.step_deg

        pose_plan = [
            ("center_start", {}),
            ("j1_plus", {"j1": +step}),
            ("center_after_j1_plus", {}),
            ("j1_minus", {"j1": -step}),
            ("center_after_j1_minus", {}),
            ("j2_plus", {"j2": +step}),
            ("center_after_j2_plus", {}),
            ("j2_minus", {"j2": -step}),
            ("center_after_j2_minus", {}),
            ("j3_plus", {"j3": +step}),
            ("center_after_j3_plus", {}),
            ("j3_minus", {"j3": -step}),
            ("center_after_j3_minus", {}),
            ("j5_plus", {"j5": +step}),
            ("center_after_j5_plus", {}),
            ("j5_minus", {"j5": -step}),
            ("center_end", {}),
        ]

        print("")
        print("Center joint pose:")

        for name in JOINT_NAMES:
            value = center_pose[name]
            print(f"  {name}: {value:+.6f} rad / {math.degrees(value):+.3f} deg")

        print("")
        print(f"Step size: +/- {step:.3f} degrees")
        print("Movement pattern: always return to center between offsets.")

        print("")

        if args.execute:
            print("EXECUTE MODE: robot will move using small joint offsets.")
        else:
            print("DRY RUN MODE: robot will NOT move.")

        print("")
        print("Keep one hand near the robot stop / teach pendant.")
        print("Make sure the workspace is clear.")
        print("Use the camera viewer to confirm the tag is visible.")
        print("You will press Enter before each move/sample.")
        print("")

        rows = []

        for sample_id, (label, offsets_deg) in enumerate(pose_plan):
            target_pose = make_target_pose(center_pose, offsets_deg)

            print("")
            print("=" * 80)
            print(f"Next sample {sample_id}: {label}")
            print("Offsets:")

            if offsets_deg:
                for joint_name, offset_deg in offsets_deg.items():
                    print(f"  {joint_name}: {offset_deg:+.3f} deg")
            else:
                print("  none / center pose")

            input("Press Enter to move to this pose and record, or Ctrl+C to stop...")

            move_ok = node.move_to_joint_pose(
                target_pose,
                move_duration_sec=args.move_sec,
            )

            if not move_ok:
                print(f"Skipping sample {sample_id} because movement failed.")
                continue

            if args.execute:
                print(f"Waiting {args.settle_sec:.1f} seconds for robot/tag to settle...")
                time.sleep(args.settle_sec)

            try:
                row = node.record_sample(sample_id, label)
            except Exception as error:
                print("")
                print(f"Could not record sample {sample_id}: {error}")
                print("Skipping this sample. Make sure tag is visible and try the next one.")
                continue

            rows.append(row)
            node.save_rows(rows)

        print("")
        print("Auto validation recording complete.")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
