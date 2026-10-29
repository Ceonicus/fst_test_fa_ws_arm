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


def mean(values):
    return sum(values) / len(values)


def std(values):
    if len(values) < 2:
        return 0.0

    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def load_calib(path):
    data = yaml.safe_load(Path(path).read_text())
    tr = data["transform"]["translation"]
    qt = data["transform"]["rotation"]

    t = [float(tr["x"]), float(tr["y"]), float(tr["z"])]
    q = quat_normalize([float(qt["x"]), float(qt["y"]), float(qt["z"]), float(qt["w"])])

    return data, t, q


def row_tf(row, prefix):
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


def evaluate(rows, wrist_to_camera_t, wrist_to_camera_q, label):
    positions = []

    for row in rows:
        base_to_wrist_t, base_to_wrist_q = row_tf(row, "base_to_wrist2")
        camera_to_tag_t, camera_to_tag_q = row_tf(row, "camera_to_tag")

        base_to_camera_t, base_to_camera_q = compose(
            base_to_wrist_t,
            base_to_wrist_q,
            wrist_to_camera_t,
            wrist_to_camera_q,
        )

        base_to_tag_t, _ = compose(
            base_to_camera_t,
            base_to_camera_q,
            camera_to_tag_t,
            camera_to_tag_q,
        )

        positions.append((int(row["sample_id"]), base_to_tag_t))

    xs = [p[1][0] for p in positions]
    ys = [p[1][1] for p in positions]
    zs = [p[1][2] for p in positions]

    mean_x = mean(xs)
    mean_y = mean(ys)
    mean_z = mean(zs)

    errors = []

    print("")
    print("=" * 80)
    print(label)
    print("=" * 80)

    for sample_id, position in positions:
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

    if mean(errors) < 0.010 and max(errors) < 0.025:
        verdict = "GOOD"
    elif mean(errors) < 0.030 and max(errors) < 0.060:
        verdict = "USABLE / CHECK"
    else:
        verdict = "POOR"

    print(f"Verdict: {verdict}")


def main():
    if len(sys.argv) != 3:
        print("Usage:")
        print("  python3 evaluate_calibration_both_directions.py <samples.csv> <calibration.calib>")
        sys.exit(1)

    samples_path = Path(sys.argv[1])
    calib_path = Path(sys.argv[2])

    data, calib_t, calib_q = load_calib(calib_path)

    rows = []
    with samples_path.open("r", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            rows.append(row)

    inverse_t, inverse_q = inverse_transform(calib_t, calib_q)

    print("")
    print("Calibration file:")
    print(calib_path)
    print("")
    print("Saved transform from file:")
    print(f"t = [{calib_t[0]:+.6f}, {calib_t[1]:+.6f}, {calib_t[2]:+.6f}]")
    print(f"q = [{calib_q[0]:+.6f}, {calib_q[1]:+.6f}, {calib_q[2]:+.6f}, {calib_q[3]:+.6f}]")
    print("")
    print("Inverse of saved transform:")
    print(f"t = [{inverse_t[0]:+.6f}, {inverse_t[1]:+.6f}, {inverse_t[2]:+.6f}]")
    print(f"q = [{inverse_q[0]:+.6f}, {inverse_q[1]:+.6f}, {inverse_q[2]:+.6f}, {inverse_q[3]:+.6f}]")

    evaluate(
        rows,
        calib_t,
        calib_q,
        "TEST A: using saved transform directly as wrist2_link -> camera_color_optical_frame",
    )

    evaluate(
        rows,
        inverse_t,
        inverse_q,
        "TEST B: using INVERSE saved transform as wrist2_link -> camera_color_optical_frame",
    )


if __name__ == "__main__":
    main()
