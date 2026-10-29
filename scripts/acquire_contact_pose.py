#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(
        description="Acquire and average a locked contact-box pose from live vision."
    )
    parser.add_argument("--topic", default="/contact_box_pose_debug")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-spread", type=float, default=0.015)
    parser.add_argument("--output", default="/tmp/contact_box_locked_pose.env")
    args = parser.parse_args()

    if args.samples <= 0:
        print("ERROR: --samples must be > 0", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("ERROR: --timeout must be > 0", file=sys.stderr)
        return 2
    if args.max_spread <= 0.0:
        print("ERROR: --max-spread must be > 0", file=sys.stderr)
        return 2

    try:
        os.unlink(args.output)
    except FileNotFoundError:
        pass
    except OSError as exc:
        print(
            f"ERROR: failed to remove stale contact pose env file {args.output}: {exc}",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = rclpy.create_node("acquire_contact_pose")
    samples = []
    invalid_message_seen = False
    invalid_message_reason = ""
    stale_message_count = 0
    acquisition_start_monotonic = time.monotonic()
    acquisition_start_ros_ns = node.get_clock().now().nanoseconds
    deadline_monotonic = time.monotonic() + args.timeout

    def pose_message_is_invalid(msg):
        validity_fields = ("valid", "is_valid")
        for field_name in validity_fields:
            if hasattr(msg, field_name) and not bool(getattr(msg, field_name)):
                return True, f"{field_name} is false"

        position = msg.pose.position
        orientation = msg.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return True, "pose contains non-finite values"

        return False, ""

    def callback(msg):
        nonlocal invalid_message_seen
        nonlocal invalid_message_reason
        nonlocal stale_message_count

        receive_ros_ns = node.get_clock().now().nanoseconds
        if receive_ros_ns < acquisition_start_ros_ns:
            stale_message_count += 1
            return

        msg_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg_stamp_ns > 0 and msg_stamp_ns < acquisition_start_ros_ns:
            stale_message_count += 1
            return

        is_invalid, reason = pose_message_is_invalid(msg)
        if is_invalid:
            invalid_message_seen = True
            invalid_message_reason = reason
            return

        if len(samples) < args.samples:
            samples.append(
                (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            )

    subscription = node.create_subscription(PoseStamped, args.topic, callback, 10)
    tmp_output = f"{args.output}.tmp.{os.getpid()}"

    try:
        while rclpy.ok() and len(samples) < args.samples:
            if invalid_message_seen:
                print(
                    "ERROR: Contact-box pose message is invalid. Refusing to lock target.",
                    file=sys.stderr,
                )
                if invalid_message_reason:
                    print(f"Invalid message detail: {invalid_message_reason}", file=sys.stderr)
                elapsed = time.monotonic() - acquisition_start_monotonic
                print(f"elapsed_sec: {elapsed:.3f}", file=sys.stderr)
                return 1
            if time.monotonic() > deadline_monotonic:
                if not samples:
                    print(
                        "ERROR: No fresh contact-box pose samples received. "
                        "Is the contact box visible to YOLO?",
                        file=sys.stderr,
                    )
                    if stale_message_count:
                        print(
                            f"Ignored stale contact pose samples: {stale_message_count}",
                            file=sys.stderr,
                        )
                    elapsed = time.monotonic() - acquisition_start_monotonic
                    print(f"elapsed_sec: {elapsed:.3f}", file=sys.stderr)
                    return 1
                print(
                    f"ERROR: timed out after {args.timeout:.3f} sec waiting for "
                    f"{args.samples} fresh samples on {args.topic}; got {len(samples)}",
                    file=sys.stderr,
                )
                elapsed = time.monotonic() - acquisition_start_monotonic
                print(f"elapsed_sec: {elapsed:.3f}", file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.05)

        if invalid_message_seen:
            print(
                "ERROR: Contact-box pose message is invalid. Refusing to lock target.",
                file=sys.stderr,
            )
            if invalid_message_reason:
                print(f"Invalid message detail: {invalid_message_reason}", file=sys.stderr)
            elapsed = time.monotonic() - acquisition_start_monotonic
            print(f"elapsed_sec: {elapsed:.3f}", file=sys.stderr)
            return 1

        avg_x = sum(sample[0] for sample in samples) / len(samples)
        avg_y = sum(sample[1] for sample in samples) / len(samples)
        avg_z = sum(sample[2] for sample in samples) / len(samples)

        max_deviation = 0.0
        for sample in samples:
            dx = sample[0] - avg_x
            dy = sample[1] - avg_y
            dz = sample[2] - avg_z
            max_deviation = max(max_deviation, math.sqrt(dx * dx + dy * dy + dz * dz))

        if max_deviation > args.max_spread:
            print(
                "ERROR: Contact-box pose samples are unstable. "
                "Refusing to lock target.",
                file=sys.stderr,
            )
            print(
                f"Max sample deviation {max_deviation:.4f} m exceeds "
                f"limit {args.max_spread:.4f} m",
                file=sys.stderr,
            )
            elapsed = time.monotonic() - acquisition_start_monotonic
            print(f"elapsed_sec: {elapsed:.3f}", file=sys.stderr)
            return 1

        acquired_at_unix = time.time()
        elapsed = time.monotonic() - acquisition_start_monotonic
        with open(tmp_output, "w", encoding="utf-8") as env_file:
            env_file.write(f"LOCKED_CONTACT_X={avg_x:.9f}\n")
            env_file.write(f"LOCKED_CONTACT_Y={avg_y:.9f}\n")
            env_file.write(f"LOCKED_CONTACT_Z={avg_z:.9f}\n")
            env_file.write(f"CONTACT_POSE_MAX_DEVIATION={max_deviation:.9f}\n")
            env_file.write(f"CONTACT_POSE_ACQUIRED_AT_UNIX={acquired_at_unix:.6f}\n")
        os.replace(tmp_output, args.output)

        print("Keypoint/contact pose acquired from live vision")
        print(f"average x/y/z: {avg_x:.4f}, {avg_y:.4f}, {avg_z:.4f}")
        print(f"max deviation: {max_deviation:.4f} m")
        print(f"acquired_at_unix: {acquired_at_unix:.6f}")
        print(f"elapsed_sec: {elapsed:.3f}")
        print(f"env file: {args.output}")
        return 0
    finally:
        try:
            os.unlink(tmp_output)
        except FileNotFoundError:
            pass
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
