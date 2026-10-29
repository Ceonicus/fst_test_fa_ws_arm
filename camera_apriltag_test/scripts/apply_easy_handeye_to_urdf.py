#!/usr/bin/env python3

import math
import re
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from rclpy.node import Node


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


def quat_rotate(q, v):
    q = quat_normalize(q)

    qv = [v[0], v[1], v[2], 0.0]
    q_inv = [-q[0], -q[1], -q[2], q[3]]

    return quat_multiply(quat_multiply(q, qv), q_inv)[:3]


def compose(t1, q1, t2, q2):
    rotated_t2 = quat_rotate(q1, t2)

    t = [
        t1[0] + rotated_t2[0],
        t1[1] + rotated_t2[1],
        t1[2] + rotated_t2[2],
    ]

    q = quat_normalize(quat_multiply(q1, q2))

    return t, q


def inverse_transform(t, q):
    q = quat_normalize(q)

    q_inv = [-q[0], -q[1], -q[2], q[3]]
    neg_t = [-t[0], -t[1], -t[2]]
    inv_t = quat_rotate(q_inv, neg_t)

    return inv_t, q_inv


def quat_to_rpy(q):
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)

    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def load_calibration(calib_path):
    data = yaml.safe_load(Path(calib_path).read_text())

    transform = data["transform"]
    translation = transform["translation"]
    rotation = transform["rotation"]

    t = [
        float(translation["x"]),
        float(translation["y"]),
        float(translation["z"]),
    ]

    q = quat_normalize([
        float(rotation["x"]),
        float(rotation["y"]),
        float(rotation["z"]),
        float(rotation["w"]),
    ])

    return t, q


def transform_msg_to_tq(transform_stamped):
    tr = transform_stamped.transform.translation
    qt = transform_stamped.transform.rotation

    t = [
        float(tr.x),
        float(tr.y),
        float(tr.z),
    ]

    q = quat_normalize([
        float(qt.x),
        float(qt.y),
        float(qt.z),
        float(qt.w),
    ])

    return t, q


def wait_for_transform(buffer, node, target_frame, source_frame, timeout_sec=10.0):
    start_time = time.time()

    while rclpy.ok() and time.time() - start_time < timeout_sec:
        try:
            transform = buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )

            return transform_msg_to_tq(transform)

        except Exception as error:
            node.get_logger().info(
                f"Waiting for TF {target_frame} -> {source_frame}: {error}"
            )
            rclpy.spin_once(node, timeout_sec=0.2)

    raise RuntimeError(f"Could not get TF {target_frame} -> {source_frame}")


def patch_joint_aa(urdf_text, xyz, rpy):
    joint_start = urdf_text.find('<joint name="aa"')

    if joint_start == -1:
        raise RuntimeError('Could not find joint name="aa" in URDF.')

    joint_end = urdf_text.find("</joint>", joint_start)

    if joint_end == -1:
        raise RuntimeError('Could not find closing </joint> for joint name="aa".')

    joint_block = urdf_text[joint_start:joint_end]

    new_origin = (
        f'<origin xyz="{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f}" '
        f'rpy="{rpy[0]:.9f} {rpy[1]:.9f} {rpy[2]:.9f}"/>'
    )

    new_joint_block = re.sub(
        r"<origin[^>]*/>",
        new_origin,
        joint_block,
        count=1,
    )

    if new_joint_block == joint_block:
        raise RuntimeError('Could not replace <origin .../> inside joint name="aa".')

    return urdf_text[:joint_start] + new_joint_block + urdf_text[joint_end:]


def main():
    if len(sys.argv) != 4:
        print("Usage:")
        print("  python3 apply_easy_handeye_to_urdf.py <calibration.calib> <input.urdf> <output.urdf>")
        sys.exit(1)

    calib_path = Path(sys.argv[1])
    input_urdf_path = Path(sys.argv[2])
    output_urdf_path = Path(sys.argv[3])

    if not calib_path.exists():
        print(f"Calibration file not found: {calib_path}")
        sys.exit(1)

    if not input_urdf_path.exists():
        print(f"Input URDF not found: {input_urdf_path}")
        sys.exit(1)

    rclpy.init()

    node = Node("apply_easy_handeye_to_urdf")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)

    desired_wrist2_to_color_t, desired_wrist2_to_color_q = load_calibration(calib_path)

    camera_base_to_color_t, camera_base_to_color_q = wait_for_transform(
        buffer,
        node,
        "camera_base",
        "camera_color_optical_frame",
    )

    inverse_camera_base_to_color_t, inverse_camera_base_to_color_q = inverse_transform(
        camera_base_to_color_t,
        camera_base_to_color_q,
    )

    new_wrist2_to_camera_base_t, new_wrist2_to_camera_base_q = compose(
        desired_wrist2_to_color_t,
        desired_wrist2_to_color_q,
        inverse_camera_base_to_color_t,
        inverse_camera_base_to_color_q,
    )

    new_rpy = quat_to_rpy(new_wrist2_to_camera_base_q)

    print("")
    print("Desired calibrated wrist2_link -> camera_color_optical_frame:")
    print(f"xyz: {desired_wrist2_to_color_t}")
    print(f"quat xyzw: {desired_wrist2_to_color_q}")

    print("")
    print("Existing downstream camera_base -> camera_color_optical_frame:")
    print(f"xyz: {camera_base_to_color_t}")
    print(f"quat xyzw: {camera_base_to_color_q}")

    print("")
    print("New URDF joint aa should be wrist2_link -> camera_base:")
    print(
        "xyz: "
        f"{new_wrist2_to_camera_base_t[0]:.9f} "
        f"{new_wrist2_to_camera_base_t[1]:.9f} "
        f"{new_wrist2_to_camera_base_t[2]:.9f}"
    )
    print(
        "rpy: "
        f"{new_rpy[0]:.9f} "
        f"{new_rpy[1]:.9f} "
        f"{new_rpy[2]:.9f}"
    )

    input_text = input_urdf_path.read_text()

    output_text = patch_joint_aa(
        input_text,
        new_wrist2_to_camera_base_t,
        new_rpy,
    )

    output_urdf_path.parent.mkdir(parents=True, exist_ok=True)
    output_urdf_path.write_text(output_text)

    print("")
    print(f"Wrote patched URDF to: {output_urdf_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
