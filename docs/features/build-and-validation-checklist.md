# Build And Validation Checklist

Use this as the normal end-to-end checklist. Do not run robot-moving steps unless the user explicitly asks.

## Safe Checks

```bash
./scripts/check_system.sh
```

Expected:

- Package prefixes resolve.
- No active legacy workspace references are printed.

```bash
./scripts/build_safe.sh
```

Expected:

- Previously reported by user: full build succeeds with 32 packages finished.
- Needs revalidation if build-related files changed.

```bash
./scripts/verify_latest_handeye.sh
```

Safety:

- Analysis-only, no movement.

Expected pass thresholds:

```text
fresh detections all valid: True
max base_T_tag position error <= 2.0 mm
```

Latest verified during this documentation pass:

```text
run_dir: camera_pose_captures/run_20260629_160327
samples: 13
fresh detections all valid: True
max camera movement: 15.378 mm
mean base_T_tag position error: 0.080 mm
max base_T_tag position error: 0.208 mm
mean rotation error: 0.1441 deg
max rotation error: 0.2860 deg
result: PASS
```

Previously reported by user for an older run:

```text
max base_T_tag position error: 0.27 mm
result: PASS
```

## Runtime Flow

Run only when hardware use is intended.

Terminal 1:

```bash
./scripts/launch_robot.sh
```

Terminal 2:

```bash
./scripts/launch_realsense.sh
```

Terminal 3:

```bash
./scripts/launch_apriltag.sh
```

Terminal 4:

```bash
./scripts/view_camera.sh
```

Terminal 5, robot-moving:

```bash
./scripts/capture.sh 10
```

After capture:

```bash
./scripts/verify_latest_handeye.sh
```

## Diagnostics

When the robot stack is intentionally running:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```
