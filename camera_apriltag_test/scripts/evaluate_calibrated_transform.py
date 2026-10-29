#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import yaml


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
    x, y, z, w = q
    vx, vy, vz = v

    # q * [v,0] * q^-1
    qv = [vx, vy, vz, 0.0]
    q_inv = [-x, -y, -z, w]
    rotated = quat_multiply(quat_multiply(q, qv), q_inv)
    return rotated[:3]


def compose_transform(t1, q1, t2, q2):
    rotated_t2 = quat_rotate(q1, t2)

    t = [
        t1[0] + rotated_t2[0],
        t1[1] + rotated_t2[1],
        t1[2] + rotated_t2[2],
    ]

    q = quat_normalize(quat_multiply(q1, q2))

    return t, q


def mean(values):
    return sum(values) / len(values)


def std(values):
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


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

    return data, t, q


def get_transform_from_row(row, prefix):
    t = [
        float(row[f"{prefix}_tx"]),
        float(row[f"{prefix}_ty"]),
        float(row[f"{prefix}_tz"]),
    ]

    q = quat_normalize([
        float(row[f"{prefix}_qx"]),
        float(row[f"{prefix}_qy"]),
        float(row[f"{prefix}_qz"]),
        float(row[f"{prefix}_qw"]),
    ])

    return t, q


def main():
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python3 evaluate_calibrated_transform.py <samples.csv> <calibration.calib>")
        sys.exit(1)

    samples_path = Path(sys.argv[1])
    calib_path = Path(sys.argv[2])

    if not samples_path.exists():
        print(f"Samples file not found: {samples_path}")
        sys.exit(1)

    if not calib_path.exists():
        print(f"Calibration file not found: {calib_path}")
        sys.exit(1)

    calib_data, wrist2_to_camera_t, wrist2_to_camera_q = load_calibration(calib_path)

    rows = []
    with samples_path.open("r", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            rows.append(row)

    if not rows:
        print("No samples found.")
        sys.exit(1)

    computed_positions = []

    print("")
    print("Calibration used:")
    print(f"name: {calib_data['parameters']['name']}")
    print(f"robot_base_frame: {calib_data['parameters']['robot_base_frame']}")
    print(f"robot_effector_frame: {calib_data['parameters']['robot_effector_frame']}")
    print(f"tracking_base_frame: {calib_data['parameters']['tracking_base_frame']}")
    print(f"tracking_marker_frame: {calib_data['parameters']['tracking_marker_frame']}")
    print("")
    print("Using calibrated wrist2_link -> camera_color_optical_frame:")
    print(f"translation: x={wrist2_to_camera_t[0]:+.6f}, y={wrist2_to_camera_t[1]:+.6f}, z={wrist2_to_camera_t[2]:+.6f}")
    print(f"quaternion:  x={wrist2_to_camera_q[0]:+.6f}, y={wrist2_to_camera_q[1]:+.6f}, z={wrist2_to_camera_q[2]:+.6f}, w={wrist2_to_camera_q[3]:+.6f}")
    print("")

    for row in rows:
        base_to_wrist_t, base_to_wrist_q = get_transform_from_row(row, "base_to_wrist2")
        camera_to_tag_t, camera_to_tag_q = get_transform_from_row(row, "camera_to_tag")

        base_to_camera_t, base_to_camera_q = compose_transform(
            base_to_wrist_t,
            base_to_wrist_q,
            wrist2_to_camera_t,
            wrist2_to_camera_q,
        )

        base_to_tag_t, base_to_tag_q = compose_transform(
            base_to_camera_t,
            base_to_camera_q,
            camera_to_tag_t,
            camera_to_tag_q,
        )

        computed_positions.append((int(row["sample_id"]), base_to_tag_t))

    xs = [p[1][0] for p in computed_positions]
    ys = [p[1][1] for p in computed_positions]
    zs = [p[1][2] for p in computed_positions]

    mean_x = mean(xs)
    mean_y = mean(ys)
    mean_z = mean(zs)

    errors = []

    print("Per-sample calibrated base_link -> tag36h11_0 position error from mean:")
    print("")

    for sample_id, position in computed_positions:
        x, y, z = position
        dx = x - mean_x
        dy = y - mean_y
        dz = z - mean_z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        errors.append(error)

        print(
            f"sample {sample_id:02d}: "
            f"x={x:+.4f}, y={y:+.4f}, z={z:+.4f}, "
            f"error={error * 1000:.1f} mm"
        )

    print("")
    print("Summary:")
    print(f"sample count: {len(rows)}")
    print(f"mean position: x={mean_x:+.4f} m, y={mean_y:+.4f} m, z={mean_z:+.4f} m")
    print("")
    print("Axis standard deviation:")
    print(f"x std: {std(xs) * 1000:.1f} mm")
    print(f"y std: {std(ys) * 1000:.1f} mm")
    print(f"z std: {std(zs) * 1000:.1f} mm")
    print("")
    print("3D error from mean:")
    print(f"mean error: {mean(errors) * 1000:.1f} mm")
    print(f"max error:  {max(errors) * 1000:.1f} mm")
    print("")

    if mean(errors) < 0.010 and max(errors) < 0.025:
        verdict = "GOOD"
    elif mean(errors) < 0.030 and max(errors) < 0.060:
        verdict = "USABLE / CHECK"
    else:
        verdict = "POOR / RETAKE CALIBRATION OR CHECK FRAMES"

    print(f"Verdict: {verdict}")


if __name__ == "__main__":
    main()
