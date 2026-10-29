#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TagAlignmentHelper(Node):
    def __init__(self):
        super().__init__("tag_alignment_helper")

        self.target_distance_m = 0.40
        if len(sys.argv) >= 2:
            self.target_distance_m = float(sys.argv[1])

        self.x_tolerance_m = 0.02
        self.y_tolerance_m = 0.02
        self.z_tolerance_m = 0.03

        self.camera_frame = "camera_color_optical_frame"
        self.tag_frame = "tag36h11_0"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.print_alignment)

        self.get_logger().info("Tag alignment helper started.")
        self.get_logger().info(f"Target distance: {self.target_distance_m:.3f} m")
        self.get_logger().info("Move robot manually only. This script does not command robot motion.")

    def print_alignment(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            print("\nTag visible: NO")
            print("Action: move camera until AprilTag is visible.")
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

        distance_error = z - self.target_distance_m

        horizontal_status = "OK"
        vertical_status = "OK"
        distance_status = "OK"

        horizontal_action = "hold horizontal position"
        vertical_action = "hold vertical position"
        distance_action = "hold distance"

        if x > self.x_tolerance_m:
            horizontal_status = "TAG RIGHT"
            horizontal_action = "move camera RIGHT manually"
        elif x < -self.x_tolerance_m:
            horizontal_status = "TAG LEFT"
            horizontal_action = "move camera LEFT manually"

        if y > self.y_tolerance_m:
            vertical_status = "TAG LOW"
            vertical_action = "move camera DOWN manually"
        elif y < -self.y_tolerance_m:
            vertical_status = "TAG HIGH"
            vertical_action = "move camera UP manually"

        if distance_error > self.z_tolerance_m:
            distance_status = "TOO FAR"
            distance_action = "move camera CLOSER manually"
        elif distance_error < -self.z_tolerance_m:
            distance_status = "TOO CLOSE"
            distance_action = "move camera FARTHER manually"

        is_good = (
            abs(x) <= self.x_tolerance_m
            and abs(y) <= self.y_tolerance_m
            and abs(distance_error) <= self.z_tolerance_m
        )

        print("\n==============================")
        print("Tag visible: YES")
        print(f"camera -> tag x: {x:+.3f} m")
        print(f"camera -> tag y: {y:+.3f} m")
        print(f"camera -> tag z: {z:+.3f} m")
        print(f"target distance: {self.target_distance_m:.3f} m")
        print(f"distance error: {distance_error:+.3f} m")
        print("")
        print(f"Horizontal: {horizontal_status} | {horizontal_action}")
        print(f"Vertical:   {vertical_status} | {vertical_action}")
        print(f"Distance:   {distance_status} | {distance_action}")
        print("")
        print(f"Overall: {'GOOD' if is_good else 'ADJUST'}")


def main():
    rclpy.init()
    node = TagAlignmentHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
