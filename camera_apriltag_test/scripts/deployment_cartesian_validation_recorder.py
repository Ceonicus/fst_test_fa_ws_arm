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

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from shape_msgs.msg import SolidPrimitive


MOVE_GROUP_ACTION = "/move_action"
MOVE_GROUP_NAME = "manipulator"

BASE_FRAME = "base_link"
CONTROL_LINK = "wrist2_link"
CAMERA_FRAME = "camera_color_optical_frame"
TAG_FRAME = "tag36h11_0"

TF_PAIRS = [
    ("base_to_wrist2", BASE_FRAME, CONTROL_LINK),
    ("base_to_camera", BASE_FRAME, CAMERA_FRAME),
    ("wrist2_to_camera", CONTROL_LINK, CAMERA_FRAME),
    ("camera_to_tag", CAMERA_FRAME, TAG_FRAME),
    ("base_to_tag", BASE_FRAME, TAG_FRAME),
]


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    return [v / norm for v in q]


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


def quat_rotate(q, v):
    q = quat_normalize(q)
    qv = [v[0], v[1], v[2], 0.0]
    q_inv = quat_inverse(q)

    return quat_multiply(quat_multiply(q, qv), q_inv)[:3]


def compose_transform(t1, q1, t2, q2):
    rotated_t2 = quat_rotate(q1, t2)

    t = [
        t1[0] + rotated_t2[0],
        t1[1] + rotated_t2[1],
        t1[2] + rotated_t2[2],
    ]

    q = quat_normalize(quat_multiply(q1, q2))

    return t, q


def inverse_transform(t, q):
    q_inv = quat_inverse(q)
    neg_t = [-t[0], -t[1], -t[2]]
    inv_t = quat_rotate(q_inv, neg_t)

    return inv_t, q_inv


def distance_mm(t1, t2):
    return math.sqrt(
        (t1[0] - t2[0]) ** 2
        + (t1[1] - t2[1]) ** 2
        + (t1[2] - t2[2]) ** 2
    ) * 1000.0


def quat_angle_deg(q1, q2):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)

    dot = abs(
        q1[0] * q2[0]
        + q1[1] * q2[1]
        + q1[2] * q2[2]
        + q1[3] * q2[3]
    )

    dot = max(-1.0, min(1.0, dot))
    angle_rad = 2.0 * math.acos(dot)

    return math.degrees(angle_rad)


class DeploymentCartesianValidationRecorder(Node):
    def __init__(self, output_csv, execute, step_mm, settle_sec):
        super().__init__("deployment_cartesian_validation_recorder")

        self.output_csv = Path(output_csv)
        self.execute = execute
        self.step_m = step_mm / 1000.0
        self.settle_sec = settle_sec

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            MOVE_GROUP_ACTION,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait_for_move_group(self, timeout_sec=10.0):
        self.get_logger().info(f"Waiting for MoveIt action server: {MOVE_GROUP_ACTION}")

        if not self.move_group_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"MoveIt action server not available: {MOVE_GROUP_ACTION}")

        self.get_logger().info("MoveIt action server available.")

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

                t = transform.transform.translation
                q = transform.transform.rotation

                return (
                    [float(t.x), float(t.y), float(t.z)],
                    quat_normalize([float(q.x), float(q.y), float(q.z), float(q.w)]),
                )

            except Exception as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(f"Could not get TF {target_frame} -> {source_frame}: {last_error}")

    def wait_for_required_tfs(self):
        print("Waiting for required TF frames...")

        while rclpy.ok():
            missing = []

            for _, target_frame, source_frame in TF_PAIRS:
                try:
                    self.lookup_transform(target_frame, source_frame, timeout_sec=0.5)
                except Exception:
                    missing.append(f"{target_frame} -> {source_frame}")

            if not missing:
                print("All required TF frames available.")
                return

            print("Still missing:")
            for item in missing:
                print(f"  {item}")

            time.sleep(0.5)

    def create_move_group_goal(self, target_wrist2_t, target_wrist2_q, plan_only):
        request = MotionPlanRequest()
        request.group_name = MOVE_GROUP_NAME
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.05
        request.max_acceleration_scaling_factor = 0.05
        request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "fixed_orientation_cartesian_target"

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = CONTROL_LINK
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.002]

        sphere_pose = Pose()
        sphere_pose.orientation.w = 1.0
        sphere_pose.position.x = target_wrist2_t[0]
        sphere_pose.position.y = target_wrist2_t[1]
        sphere_pose.position.z = target_wrist2_t[2]

        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(sphere_pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = BASE_FRAME
        orientation_constraint.link_name = CONTROL_LINK
        orientation_constraint.orientation.x = target_wrist2_q[0]
        orientation_constraint.orientation.y = target_wrist2_q[1]
        orientation_constraint.orientation.z = target_wrist2_q[2]
        orientation_constraint.orientation.w = target_wrist2_q[3]
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)

        planning_options = PlanningOptions()
        planning_options.plan_only = plan_only
        planning_options.look_around = False
        planning_options.replan = False
        planning_options.planning_scene_diff.is_diff = True
        planning_options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        return goal

    def send_moveit_goal(self, target_wrist2_t, target_wrist2_q, label):
        plan_only = not self.execute

        goal = self.create_move_group_goal(
            target_wrist2_t=target_wrist2_t,
            target_wrist2_q=target_wrist2_q,
            plan_only=plan_only,
        )

        if plan_only:
            print("")
            print(f"[PLAN ONLY] Planning target for: {label}")
        else:
            print("")
            print(f"[EXECUTE] Moving to target for: {label}")

        send_goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            print("MoveIt goal failed to send.")
            return False

        if not goal_handle.accepted:
            print("MoveIt goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrapper = result_future.result()

        if result_wrapper is None:
            print("MoveIt did not return a result.")
            return False

        result = result_wrapper.result
        error_code = result.error_code.val

        print(f"MoveIt error_code: {error_code}")

        return error_code == 1

    def record_sample(
        self,
        sample_id,
        label,
        commanded_dx,
        commanded_dy,
        commanded_dz,
        center_camera_t,
        center_camera_q,
        previous_camera_t,
    ):
        row = {
            "sample_id": sample_id,
            "label": label,
            "commanded_dx_camera_m": commanded_dx,
            "commanded_dy_camera_m": commanded_dy,
            "commanded_dz_camera_m": commanded_dz,
            "timestamp_sec": time.time(),
        }

        for prefix, target_frame, source_frame in TF_PAIRS:
            t, q = self.lookup_transform(target_frame, source_frame)

            row[f"{prefix}_tx"] = t[0]
            row[f"{prefix}_ty"] = t[1]
            row[f"{prefix}_tz"] = t[2]
            row[f"{prefix}_qx"] = q[0]
            row[f"{prefix}_qy"] = q[1]
            row[f"{prefix}_qz"] = q[2]
            row[f"{prefix}_qw"] = q[3]

        current_camera_t = [
            row["base_to_camera_tx"],
            row["base_to_camera_ty"],
            row["base_to_camera_tz"],
        ]

        current_camera_q = [
            row["base_to_camera_qx"],
            row["base_to_camera_qy"],
            row["base_to_camera_qz"],
            row["base_to_camera_qw"],
        ]

        movement_from_center_mm = distance_mm(current_camera_t, center_camera_t)
        movement_from_previous_mm = distance_mm(current_camera_t, previous_camera_t)
        rotation_error_deg = quat_angle_deg(current_camera_q, center_camera_q)

        row["movement_from_center_mm"] = movement_from_center_mm
        row["movement_from_previous_mm"] = movement_from_previous_mm
        row["camera_rotation_error_deg"] = rotation_error_deg

        print("")
        print(f"Recorded sample {sample_id}: {label}")
        print("base_to_camera movement proof:")
        print(f"  movement from center:   {movement_from_center_mm:.2f} mm")
        print(f"  movement from previous: {movement_from_previous_mm:.2f} mm")
        print(f"  camera rotation error:  {rotation_error_deg:.4f} deg")

        print("base_to_tag translation:")
        print(f"  x = {row['base_to_tag_tx']:+.4f} m")
        print(f"  y = {row['base_to_tag_ty']:+.4f} m")
        print(f"  z = {row['base_to_tag_tz']:+.4f} m")

        return row, current_camera_t

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


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("output_csv")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--step-mm", type=float, default=10.0)
    parser.add_argument("--settle-sec", type=float, default=2.0)

    args = parser.parse_args()

    rclpy.init()

    node = DeploymentCartesianValidationRecorder(
        output_csv=args.output_csv,
        execute=args.execute,
        step_mm=args.step_mm,
        settle_sec=args.settle_sec,
    )

    try:
        node.wait_for_move_group()
        node.wait_for_required_tfs()

        center_camera_t, center_camera_q = node.lookup_transform(BASE_FRAME, CAMERA_FRAME)
        wrist2_to_camera_t, wrist2_to_camera_q = node.lookup_transform(CONTROL_LINK, CAMERA_FRAME)

        inverse_wrist2_to_camera_t, inverse_wrist2_to_camera_q = inverse_transform(
            wrist2_to_camera_t,
            wrist2_to_camera_q,
        )

        step = node.step_m

        pose_plan = [
            ("center_start", 0.0, 0.0, 0.0),
            ("right_10mm", +step, 0.0, 0.0),
            ("center_after_right", 0.0, 0.0, 0.0),
            ("left_10mm", -step, 0.0, 0.0),
            ("center_after_left", 0.0, 0.0, 0.0),
            ("up_10mm", 0.0, -step, 0.0),
            ("center_after_up", 0.0, 0.0, 0.0),
            ("down_10mm", 0.0, +step, 0.0),
            ("center_after_down", 0.0, 0.0, 0.0),
            ("forward_towards_tag_10mm", 0.0, 0.0, +step),
            ("center_after_forward", 0.0, 0.0, 0.0),
            ("backward_away_from_tag_10mm", 0.0, 0.0, -step),
            ("center_end", 0.0, 0.0, 0.0),
        ]

        print("")
        print("Deployment-style Cartesian validation")
        print("")
        print(f"Step size: {args.step_mm:.1f} mm")
        print("Frame: camera_color_optical_frame")
        print("Orientation: fixed")
        print("Pattern: return to center after every movement")
        print("")
        print("Camera optical frame convention:")
        print("  +X = image right")
        print("  -X = image left")
        print("  -Y = image up")
        print("  +Y = image down")
        print("  +Z = forward, toward tag/part")
        print("  -Z = backward, away from tag/part")
        print("")

        if args.execute:
            print("EXECUTE MODE: robot will move using MoveIt.")
        else:
            print("PLAN-ONLY MODE: robot will NOT move.")

        print("")
        print("Keep one hand near the robot stop / teach pendant.")
        print("Make sure the workspace is clear.")
        print("Make sure the tag is visible and TAG OK in the viewer.")
        print("")

        rows = []
        previous_camera_t = list(center_camera_t)

        for sample_id, (label, dx, dy, dz) in enumerate(pose_plan):
            local_delta = [dx, dy, dz]
            base_delta = quat_rotate(center_camera_q, local_delta)

            target_camera_t = [
                center_camera_t[0] + base_delta[0],
                center_camera_t[1] + base_delta[1],
                center_camera_t[2] + base_delta[2],
            ]

            target_camera_q = list(center_camera_q)

            target_wrist2_t, target_wrist2_q = compose_transform(
                target_camera_t,
                target_camera_q,
                inverse_wrist2_to_camera_t,
                inverse_wrist2_to_camera_q,
            )

            print("")
            print("=" * 80)
            print(f"Next sample {sample_id}: {label}")
            print("Commanded camera-frame offset:")
            print(f"  dx = {dx * 1000:+.1f} mm")
            print(f"  dy = {dy * 1000:+.1f} mm")
            print(f"  dz = {dz * 1000:+.1f} mm")

            input("Press Enter to plan/move and record, or Ctrl+C to stop...")

            ok = node.send_moveit_goal(
                target_wrist2_t=target_wrist2_t,
                target_wrist2_q=target_wrist2_q,
                label=label,
            )

            if not ok:
                print(f"Skipping sample {sample_id}: MoveIt failed.")
                continue

            if args.execute:
                print(f"Waiting {args.settle_sec:.1f} seconds for robot/tag to settle...")
                time.sleep(args.settle_sec)

            try:
                row, current_camera_t = node.record_sample(
                    sample_id=sample_id,
                    label=label,
                    commanded_dx=dx,
                    commanded_dy=dy,
                    commanded_dz=dz,
                    center_camera_t=center_camera_t,
                    center_camera_q=center_camera_q,
                    previous_camera_t=previous_camera_t,
                )
            except Exception as error:
                print("")
                print(f"Could not record sample {sample_id}: {error}")
                print("Skipping this sample. Make sure the tag is visible.")
                continue

            if args.execute:
                expected_non_center = abs(dx) > 1e-9 or abs(dy) > 1e-9 or abs(dz) > 1e-9

                if expected_non_center and row["movement_from_center_mm"] < 3.0:
                    print("")
                    print("WARNING: Expected movement, but measured movement from center is < 3 mm.")

            rows.append(row)
            previous_camera_t = current_camera_t
            node.save_rows(rows)

        print("")
        print("Recording complete.")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
