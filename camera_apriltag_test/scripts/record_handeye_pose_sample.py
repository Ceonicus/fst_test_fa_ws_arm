#!/usr/bin/env python3

import csv
import sys
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class HandeyePoseRecorder(Node):
    def __init__(self, output_csv):
        super().__init__("handeye_pose_recorder")

        self.output_csv = Path(output_csv)

        self.frames_to_record = [
            ("base_link", "wrist2_link", "base_to_wrist2"),
            ("wrist2_link", "camera_color_optical_frame", "wrist2_to_camera"),
            ("camera_color_optical_frame", "tag36h11_0", "camera_to_tag"),
            ("base_link", "tag36h11_0", "base_to_tag"),
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sample_id = 0
        self.fieldnames = ["sample_id", "timestamp_sec"]

        for _, _, name in self.frames_to_record:
            self.fieldnames.extend([
                f"{name}_tx",
                f"{name}_ty",
                f"{name}_tz",
                f"{name}_qx",
                f"{name}_qy",
                f"{name}_qz",
                f"{name}_qw",
            ])

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)

        if not self.output_csv.exists():
            with self.output_csv.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writeheader()

        self.get_logger().info("Hand-eye pose recorder started.")
        self.get_logger().info(f"Output CSV: {self.output_csv}")
        self.get_logger().info("Move robot manually only. Press Enter to record each sample.")

    def wait_for_required_transforms(self, timeout_sec=10.0):
        print("\nWaiting for required TF frames...")

        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            missing = []

            for parent_frame, child_frame, _ in self.frames_to_record:
                ok = self.tf_buffer.can_transform(
                    parent_frame,
                    child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )

                if not ok:
                    missing.append(f"{parent_frame} -> {child_frame}")

            if not missing:
                print("All required TF frames available.")
                return True

            print("Still missing:")
            for item in missing:
                print(f"  {item}")

        return False

    def lookup_transform(self, parent_frame, child_frame):
        return self.tf_buffer.lookup_transform(
            parent_frame,
            child_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )

    def record_one_sample(self):
        if not self.wait_for_required_transforms(timeout_sec=5.0):
            print("\nCould not record sample because required TF frames are missing.")
            print("Check robot stack, RealSense, and AprilTag node.")
            return

        row = {
            "sample_id": self.sample_id,
            "timestamp_sec": self.get_clock().now().nanoseconds / 1e9,
        }

        for parent_frame, child_frame, name in self.frames_to_record:
            transform = self.lookup_transform(parent_frame, child_frame)

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            row[f"{name}_tx"] = translation.x
            row[f"{name}_ty"] = translation.y
            row[f"{name}_tz"] = translation.z
            row[f"{name}_qx"] = rotation.x
            row[f"{name}_qy"] = rotation.y
            row[f"{name}_qz"] = rotation.z
            row[f"{name}_qw"] = rotation.w

        with self.output_csv.open("a", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow(row)

        print("")
        print(f"Recorded sample {self.sample_id}")
        print("base_to_tag translation:")
        print(f"  x = {row['base_to_tag_tx']:+.4f} m")
        print(f"  y = {row['base_to_tag_ty']:+.4f} m")
        print(f"  z = {row['base_to_tag_tz']:+.4f} m")
        print(f"Saved to: {self.output_csv}")

        self.sample_id += 1


def main():
    output_csv = "camera_apriltag_test/handeye_pose_samples_current_urdf.csv"

    if len(sys.argv) >= 2:
        output_csv = sys.argv[1]

    rclpy.init()
    node = HandeyePoseRecorder(output_csv)

    try:
        if not node.wait_for_required_transforms(timeout_sec=15.0):
            print("\nRequired TF frames are not available.")
            print("Make sure these are running:")
            print("  Terminal 1: robot / URDF / TF")
            print("  Terminal 2: RealSense")
            print("  Terminal 3: apriltag_node")
            return

        while rclpy.ok():
            input("\nManually move robot/camera to a stable tag view, then press Enter to record. Ctrl+C to stop.")
            node.record_one_sample()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
