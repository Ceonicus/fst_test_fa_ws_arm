#!/usr/bin/env python3

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


def transform_from_dict(data):
    tr = data["translation"]
    qt = data["rotation"]

    t = [
        float(tr["x"]),
        float(tr["y"]),
        float(tr["z"]),
    ]

    q = quat_normalize([
        float(qt["x"]),
        float(qt["y"]),
        float(qt["z"]),
        float(qt["w"]),
    ])

    return t, q


def load_calibration(calib_path):
    data = yaml.safe_load(Path(calib_path).read_text())
    return data, transform_from_dict(data["transform"])


def load_samples(samples_path):
    data = yaml.safe_load(Path(samples_path).read_text())
    return data, data["samples"]


def evaluate(samples, calib_t, calib_q, robot_mode, tracking_mode, calib_mode):
    positions = []

    for sample_index, sample in enumerate(samples):
        robot_t, robot_q = transform_from_dict(sample["robot"])
        tracking_t, tracking_q = transform_from_dict(sample["tracking"])

        if robot_mode == "direct":
            base_to_wrist_t, base_to_wrist_q = robot_t, robot_q
        else:
            base_to_wrist_t, base_to_wrist_q = inverse_transform(robot_t, robot_q)

        if tracking_mode == "direct":
            camera_to_tag_t, camera_to_tag_q = tracking_t, tracking_q
        else:
            camera_to_tag_t, camera_to_tag_q = inverse_transform(tracking_t, tracking_q)

        if calib_mode == "direct":
            wrist_to_camera_t, wrist_to_camera_q = calib_t, calib_q
        else:
            wrist_to_camera_t, wrist_to_camera_q = inverse_transform(calib_t, calib_q)

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

        positions.append((sample_index, base_to_tag_t))

    xs = [p[1][0] for p in positions]
    ys = [p[1][1] for p in positions]
    zs = [p[1][2] for p in positions]

    mean_x = mean(xs)
    mean_y = mean(ys)
    mean_z = mean(zs)

    errors = []

    for _, position in positions:
        x, y, z = position

        dx = x - mean_x
        dy = y - mean_y
        dz = z - mean_z

        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        errors.append(error)

    result = {
        "robot_mode": robot_mode,
        "tracking_mode": tracking_mode,
        "calib_mode": calib_mode,
        "positions": positions,
        "mean_position": [mean_x, mean_y, mean_z],
        "x_std_mm": std(xs) * 1000,
        "y_std_mm": std(ys) * 1000,
        "z_std_mm": std(zs) * 1000,
        "mean_error_mm": mean(errors) * 1000,
        "max_error_mm": max(errors) * 1000,
        "errors_mm": [error * 1000 for error in errors],
    }

    return result


def print_result_summary(result):
    print(
        f"robot={result['robot_mode']:<7} "
        f"tracking={result['tracking_mode']:<7} "
        f"calib={result['calib_mode']:<7} | "
        f"mean={result['mean_error_mm']:7.1f} mm | "
        f"max={result['max_error_mm']:7.1f} mm | "
        f"std xyz=({result['x_std_mm']:.1f}, {result['y_std_mm']:.1f}, {result['z_std_mm']:.1f}) mm"
    )


def print_detailed_result(result):
    print("")
    print("=" * 100)
    print("BEST HYPOTHESIS DETAILS")
    print("=" * 100)

    print(f"robot transform mode:    {result['robot_mode']}")
    print(f"tracking transform mode: {result['tracking_mode']}")
    print(f"calibration mode:        {result['calib_mode']}")

    mean_x, mean_y, mean_z = result["mean_position"]

    print("")
    print("Per-sample base_link -> tag36h11_0 position error from mean:")
    print("")

    for (sample_index, position), error_mm in zip(result["positions"], result["errors_mm"]):
        x, y, z = position

        print(
            f"sample {sample_index:02d}: "
            f"x={x:+.4f}, y={y:+.4f}, z={z:+.4f}, "
            f"error={error_mm:.1f} mm"
        )

    print("")
    print("Summary:")
    print(f"sample count: {len(result['positions'])}")
    print(f"mean position: x={mean_x:+.4f} m, y={mean_y:+.4f} m, z={mean_z:+.4f} m")
    print("")
    print("Axis standard deviation:")
    print(f"x std: {result['x_std_mm']:.1f} mm")
    print(f"y std: {result['y_std_mm']:.1f} mm")
    print(f"z std: {result['z_std_mm']:.1f} mm")
    print("")
    print("3D error from mean:")
    print(f"mean error: {result['mean_error_mm']:.1f} mm")
    print(f"max error:  {result['max_error_mm']:.1f} mm")

    if result["mean_error_mm"] < 10 and result["max_error_mm"] < 25:
        verdict = "GOOD"
    elif result["mean_error_mm"] < 30 and result["max_error_mm"] < 60:
        verdict = "USABLE / CHECK"
    else:
        verdict = "POOR"

    print(f"Verdict: {verdict}")


def main():
    if len(sys.argv) != 3:
        print("Usage:")
        print("  python3 evaluate_easy_handeye_samples.py <samples.samples> <calibration.calib>")
        sys.exit(1)

    samples_path = Path(sys.argv[1])
    calib_path = Path(sys.argv[2])

    samples_data, samples = load_samples(samples_path)
    calib_data, (calib_t, calib_q) = load_calibration(calib_path)

    print("")
    print("Samples file:")
    print(samples_path)
    print("")
    print("Calibration file:")
    print(calib_path)
    print("")
    print("Calibration parameters:")
    for key, value in calib_data["parameters"].items():
        print(f"{key}: {value}")

    print("")
    print(f"Sample count: {len(samples)}")

    results = []

    for robot_mode in ["direct", "inverse"]:
        for tracking_mode in ["direct", "inverse"]:
            for calib_mode in ["direct", "inverse"]:
                result = evaluate(
                    samples=samples,
                    calib_t=calib_t,
                    calib_q=calib_q,
                    robot_mode=robot_mode,
                    tracking_mode=tracking_mode,
                    calib_mode=calib_mode,
                )

                results.append(result)

    results.sort(key=lambda result: (result["mean_error_mm"], result["max_error_mm"]))

    print("")
    print("=" * 100)
    print("ALL HYPOTHESES, SORTED BEST TO WORST")
    print("=" * 100)

    for result in results:
        print_result_summary(result)

    print_detailed_result(results[0])


if __name__ == "__main__":
    main()
