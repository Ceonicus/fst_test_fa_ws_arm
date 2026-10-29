#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Verifying latest hand-eye capture run"
RUN_DIR=$(ls -td camera_pose_captures/run_* | head -1)

python3 - "${RUN_DIR}" <<'PY'
import json
import math
import sys
from pathlib import Path


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
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


def quat_dot(q1, q2):
    return sum(a * b for a, b in zip(q1, q2))


def quat_angle_deg(q1, q2):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = min(1.0, max(-1.0, abs(quat_dot(q1, q2))))
    return math.degrees(2.0 * math.acos(dot))


def transform_from_dict(data):
    translation = data["translation"]
    rotation = data["rotation"]
    return (
        [
            float(translation["x"]),
            float(translation["y"]),
            float(translation["z"]),
        ],
        quat_normalize(
            [
                float(rotation["x"]),
                float(rotation["y"]),
                float(rotation["z"]),
                float(rotation["w"]),
            ]
        ),
    )


def compose(t1, q1, t2, q2):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    moved_t2 = quat_rotate(q1, t2)
    return (
        [
            t1[0] + moved_t2[0],
            t1[1] + moved_t2[1],
            t1[2] + moved_t2[2],
        ],
        quat_normalize(quat_multiply(q1, q2)),
    )


def distance_mm(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))) * 1000.0


def mean(values):
    values = list(values)
    return sum(values) / len(values)


def format_float(value, precision=3):
    if value is None:
        return "None"
    return f"{value:.{precision}f}"


def load_sample(path):
    data = json.loads(path.read_text())
    base_to_camera = data["tf_snapshots"]["base_to_camera"]
    camera_to_tag = data["camera_to_tag"]
    base_t_camera = transform_from_dict(base_to_camera)
    camera_t_tag = transform_from_dict(camera_to_tag)
    base_t_tag = compose(*base_t_camera, *camera_t_tag)
    fresh_valid = (
        bool(data.get("apriltag_detection_available"))
        and bool(data.get("apriltag_detection_fresh"))
        and bool(camera_to_tag.get("lookup_success"))
        and bool(camera_to_tag.get("lookup_fresh"))
    )
    return {
        "path": path,
        "pose": data.get("pose_name", path.stem.replace("_metadata", "")),
        "pose_index": data.get("pose_index"),
        "det_age": data.get("apriltag_detection_age_sec"),
        "fresh_valid": fresh_valid,
        "base_t_camera": base_t_camera,
        "base_t_tag": base_t_tag,
    }


def average_quaternion(quaternions):
    reference = quaternions[0]
    accumulator = [0.0, 0.0, 0.0, 0.0]
    for q in quaternions:
        q = quat_normalize(q)
        if quat_dot(reference, q) < 0.0:
            q = [-v for v in q]
        for index, value in enumerate(q):
            accumulator[index] += value
    return quat_normalize(accumulator)


def main():
    run_dir = Path(sys.argv[1])
    metadata_paths = sorted(run_dir.glob("*_metadata.json"))
    if not metadata_paths:
        print(f"No metadata files found in {run_dir}")
        sys.exit(1)

    samples = [load_sample(path) for path in metadata_paths]
    base_tag_positions = [sample["base_t_tag"][0] for sample in samples]
    base_tag_quaternions = [sample["base_t_tag"][1] for sample in samples]
    mean_base_tag_position = [mean(axis) for axis in zip(*base_tag_positions)]
    mean_base_tag_quaternion = average_quaternion(base_tag_quaternions)
    first_camera_position = samples[0]["base_t_camera"][0]

    print("")
    print("Per-pose results:")
    for sample in samples:
        camera_move_mm = distance_mm(sample["base_t_camera"][0], first_camera_position)
        base_tag_error_mm = distance_mm(sample["base_t_tag"][0], mean_base_tag_position)
        base_tag_rot_deg = quat_angle_deg(sample["base_t_tag"][1], mean_base_tag_quaternion)
        sample["camera_move_mm"] = camera_move_mm
        sample["base_tag_error_mm"] = base_tag_error_mm
        sample["base_tag_rot_deg"] = base_tag_rot_deg
        print(
            f"pose={sample['pose']} "
            f"det_age={format_float(sample['det_age'], 3)} "
            f"camera_move_mm={camera_move_mm:.3f} "
            f"base_tag_error_mm={base_tag_error_mm:.3f} "
            f"base_tag_rot_deg={base_tag_rot_deg:.4f}"
        )

    fresh_all_valid = all(sample["fresh_valid"] for sample in samples)
    max_camera_movement = max(sample["camera_move_mm"] for sample in samples)
    mean_position_error = mean(sample["base_tag_error_mm"] for sample in samples)
    max_position_error = max(sample["base_tag_error_mm"] for sample in samples)
    mean_rotation_error = mean(sample["base_tag_rot_deg"] for sample in samples)
    max_rotation_error = max(sample["base_tag_rot_deg"] for sample in samples)
    passed = fresh_all_valid and max_position_error <= 2.0

    print("")
    print("Summary:")
    print(f"run_dir: {run_dir}")
    print(f"samples: {len(samples)}")
    print(f"fresh detections all valid: {fresh_all_valid}")
    print(f"max camera movement: {max_camera_movement:.3f} mm")
    print(f"mean base_T_tag position error: {mean_position_error:.3f} mm")
    print(f"max base_T_tag position error: {max_position_error:.3f} mm")
    print(f"mean rotation error: {mean_rotation_error:.4f} deg")
    print(f"max rotation error: {max_rotation_error:.4f} deg")
    print(f"result: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
PY
