#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path


def mean(values):
    return sum(values) / len(values)


def std(values):
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_tag_stability.py <samples.csv>")
        sys.exit(1)

    csv_path = Path(sys.argv[1])

    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        sys.exit(1)

    rows = []

    with csv_path.open("r", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            rows.append(row)

    if not rows:
        print("No samples found.")
        sys.exit(1)

    xs = [float(row["base_to_tag_tx"]) for row in rows]
    ys = [float(row["base_to_tag_ty"]) for row in rows]
    zs = [float(row["base_to_tag_tz"]) for row in rows]

    mean_x = mean(xs)
    mean_y = mean(ys)
    mean_z = mean(zs)

    errors = []

    print("")
    print("Per-sample base_link -> tag36h11_0 position error from mean:")
    print("")

    for row, x, y, z in zip(rows, xs, ys, zs):
        dx = x - mean_x
        dy = y - mean_y
        dz = z - mean_z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        errors.append(error)

        print(
            f"sample {int(row['sample_id']):02d}: "
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
        verdict = "POOR BASELINE / CALIBRATION NEEDED"

    print(f"Verdict: {verdict}")


if __name__ == "__main__":
    main()
