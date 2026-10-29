#!/usr/bin/env python3
import argparse
import json
import math
import sys

import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_vector_by_quaternion(qx, qy, qz, qw, vector):
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("TF quaternion is invalid")
    return qx / norm, qy / norm, qz / norm, qw / norm


def wait_for_transform(node, buffer, timeout_sec):
    deadline = node.get_clock().now() + Duration(seconds=timeout_sec)
    last_error = None
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            return buffer.lookup_transform("base_link", "wrist3_link", rclpy.time.Time())
        except TransformException as exc:
            last_error = exc
    raise RuntimeError(f"failed to lookup TF base_link -> wrist3_link: {last_error}")


def main():
    parser = argparse.ArgumentParser(
        description="Estimate suction-tip standoff from locked contact pose and wrist3_link TF."
    )
    parser.add_argument("--contact-x", type=float, required=True)
    parser.add_argument("--contact-y", type=float, required=True)
    parser.add_argument("--contact-z", type=float, required=True)
    parser.add_argument("--tool-offset-z", type=float, default=0.263)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--env", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_locked_standoff")
    buffer = Buffer()
    TransformListener(buffer, node)

    try:
        transform = wait_for_transform(node, buffer, args.timeout)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = normalize_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        tool_axis = rotate_vector_by_quaternion(
            qx, qy, qz, qw, (0.0, 0.0, 1.0)
        )
        tool_offset = (
            tool_axis[0] * args.tool_offset_z,
            tool_axis[1] * args.tool_offset_z,
            tool_axis[2] * args.tool_offset_z,
        )
        wrist = (translation.x, translation.y, translation.z)
        suction_tip = (
            wrist[0] + tool_offset[0],
            wrist[1] + tool_offset[1],
            wrist[2] + tool_offset[2],
        )
        contact = (args.contact_x, args.contact_y, args.contact_z)
        tip_to_contact = (
            contact[0] - suction_tip[0],
            contact[1] - suction_tip[1],
            contact[2] - suction_tip[2],
        )
        signed_standoff = (
            tip_to_contact[0] * tool_axis[0]
            + tip_to_contact[1] * tool_axis[1]
            + tip_to_contact[2] * tool_axis[2]
        )
        lateral = (
            tip_to_contact[0] - signed_standoff * tool_axis[0],
            tip_to_contact[1] - signed_standoff * tool_axis[1],
            tip_to_contact[2] - signed_standoff * tool_axis[2],
        )
        lateral_error = math.sqrt(
            lateral[0] * lateral[0]
            + lateral[1] * lateral[1]
            + lateral[2] * lateral[2]
        )

        result = {
            "wrist_x": wrist[0],
            "wrist_y": wrist[1],
            "wrist_z": wrist[2],
            "wrist_qx": qx,
            "wrist_qy": qy,
            "wrist_qz": qz,
            "wrist_qw": qw,
            "suction_tip_x": suction_tip[0],
            "suction_tip_y": suction_tip[1],
            "suction_tip_z": suction_tip[2],
            "signed_standoff_m": signed_standoff,
            "signed_standoff_cm": signed_standoff * 100.0,
            "lateral_error_m": lateral_error,
            "lateral_error_cm": lateral_error * 100.0,
        }

        if args.json:
            print(json.dumps(result, sort_keys=True))
        elif args.env:
            print(f"WRIST_X={result['wrist_x']:.9f}")
            print(f"WRIST_Y={result['wrist_y']:.9f}")
            print(f"WRIST_Z={result['wrist_z']:.9f}")
            print(f"SUCTION_TIP_X={result['suction_tip_x']:.9f}")
            print(f"SUCTION_TIP_Y={result['suction_tip_y']:.9f}")
            print(f"SUCTION_TIP_Z={result['suction_tip_z']:.9f}")
            print(f"SIGNED_STANDOFF_M={result['signed_standoff_m']:.9f}")
            print(f"SIGNED_STANDOFF_CM={result['signed_standoff_cm']:.6f}")
            print(f"LATERAL_ERROR_M={result['lateral_error_m']:.9f}")
            print(f"LATERAL_ERROR_CM={result['lateral_error_cm']:.6f}")
        else:
            print("Locked contact standoff check")
            print(
                "wrist3_link: "
                f"x={wrist[0]:.4f}, y={wrist[1]:.4f}, z={wrist[2]:.4f}, "
                f"qxyzw=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})"
            )
            print(
                "suction tip estimate: "
                f"x={suction_tip[0]:.4f}, y={suction_tip[1]:.4f}, "
                f"z={suction_tip[2]:.4f}"
            )
            print(
                "signed standoff: "
                f"{signed_standoff:.4f} m ({signed_standoff * 100.0:.2f} cm)"
            )
            print(
                "lateral error: "
                f"{lateral_error:.4f} m ({lateral_error * 100.0:.2f} cm)"
            )
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
