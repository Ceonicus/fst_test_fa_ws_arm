# Workspace Porting

Purpose: document the current self-contained workspace state after moving work into `~/Desktop/fst_test_fa_ws_arm`.

## Verified State

- Current repo root from `pwd`: `/home/jetson-nano/Desktop/fst_test_fa_ws_arm`.
- `AGENTS.md` did not exist before this documentation pass.
- `docs/` did not exist before this documentation pass.
- `colcon list` returned 32 package entries in `src/`.
- `./scripts/check_system.sh` completed successfully.
- The old-reference check in `./scripts/check_system.sh` returned no active matches under `src/` with backup and binary exclusions.

Package prefixes verified by `./scripts/check_system.sh`:

```text
fairino_hardware: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/fairino_hardware
fairino_mtc_demo: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/fairino_mtc_demo
fairino5_v6_moveit2_config: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/fairino5_v6_moveit2_config
fairino_description: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/fairino_description
camera_pose_capture: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/camera_pose_capture
realsense2_camera: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/realsense2_camera
apriltag_ros: /opt/ros/humble
```

RealSense package resolution is self-contained for `realsense2_camera`: it resolves from this workspace install tree. `apriltag_ros` resolves from the system ROS Humble installation.

## User-Reported State

- Full build succeeds with `32 packages finished`.
- Old workspace is no longer needed for normal operation.
- RealSense, robot MoveIt launch, AprilTag detection, viewer, capture, and validation were previously reported working.

These runtime claims were not revalidated by launching hardware during this documentation pass.

## Practical Guidance

- Use `./scripts/check_system.sh` after path or package-discovery changes.
- Use `./scripts/build_safe.sh` for slow, low-parallelism Jetson builds.
- Avoid adding absolute paths. Prefer package-relative CMake, launch, and URDF references.
