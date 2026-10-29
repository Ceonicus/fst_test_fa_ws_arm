# Helper Scripts

Purpose: avoid long manual ROS commands and keep setup consistent.

All shell scripts are executable and use `set -euo pipefail`. They temporarily disable nounset while sourcing ROS setup files because ROS Humble setup scripts may reference unset variables.

Current contact-box source of truth: [Contact Box Vision-to-Motion Developer Guide](contact-box-vision-to-motion-developer-guide.md). That guide owns the current validated contact-box sequence, safety assumptions, TCP/tool offset, staging/ready pose meanings, and perception startup procedure.

## config/contact_box_motion.env

- Purpose: env-style contact-box motion tuning file used by reset, approach, direct, press, cover pickup-out, and target-visible pose-acquisition helpers.
- Loader: `scripts/load_contact_box_motion_config.sh`.
- Behavior: loader finds the workspace root, loads `config/contact_box_motion.env` if present, otherwise uses built-in safe defaults, exports values, prints the loaded config path and speed scaling, validates numeric ranges, and refuses unsafe values.
- Main speed variables: `CONTACT_BOX_RESET_VELOCITY_SCALING`, `CONTACT_BOX_RESET_ACCELERATION_SCALING`, `CONTACT_BOX_APPROACH_VELOCITY_SCALING`, `CONTACT_BOX_APPROACH_ACCELERATION_SCALING`, `CONTACT_BOX_PRESS_VELOCITY_SCALING`, `CONTACT_BOX_PRESS_ACCELERATION_SCALING`.
- Planner/safety variables include `CONTACT_BOX_CARTESIAN_EEF_STEP`, `CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD`, `CONTACT_BOX_CARTESIAN_MIN_FRACTION`, `CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE`, `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD`, `CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD`, and press-specific max wrist/joint limits.
- Joint-delta split: saved-pose resets use `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD`, direct approach uses `CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD`, and press uses `CONTACT_BOX_PRESS_MAX_JOINT_DELTA_RAD`.
- Contact/pose variables include `CONTACT_BOX_DEFAULT_STANDOFF_M`, `CONTACT_BOX_DEFAULT_PRESS_DEPTH_M`, `CONTACT_BOX_MAX_PRESS_DEPTH_M`, `CONTACT_BOX_TARGET_OFFSET_X_M`, `CONTACT_BOX_TARGET_OFFSET_Y_M`, `CONTACT_BOX_TARGET_OFFSET_Z_M`, `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M`, `CONTACT_BOX_COVER_LIFT_Z_M`, `CONTACT_BOX_MIN_ACTUAL_LIFT_DELTA_Z_M`, `CONTACT_BOX_COVER_PULL_OUT_AXIS`, `CONTACT_BOX_COVER_PULL_OUT_M`, `CONTACT_BOX_SUCTION_SETTLE_DELAY_SEC`, `CONTACT_BOX_POSE_SAMPLE_COUNT`, `CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC`, `CONTACT_BOX_POSE_MAX_SPREAD_M`, and `CONTACT_BOX_MAX_LIVE_VS_MANUAL_DELTA_M`.
- Target offset: direct/staged contact scripts use the fresh live contact pose as a raw reference, then plan to `live_contact_pose + CONTACT_BOX_TARGET_OFFSET_*_M`; the current default shifts Z by `-0.030 m` in `base_link`.
- Pre-approach lateral limit: `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M` controls the direct script's allowed starting lateral error to the adjusted target. The target offset can intentionally create initial lateral error from the true 30 cm ready pose, so this gate is wider than the old `0.020 m` check while still bounded by the loader.
- Cover pickup-out config: `CONTACT_BOX_COVER_LIFT_Z_M=0.015` lifts in `base_link` +Z after suction is ON. The action server logs commanded-vs-actual lift and requires only `CONTACT_BOX_MIN_ACTUAL_LIFT_DELTA_Z_M=0.003` before pull-out, matching the bash flow's permissive lift behavior. `CONTACT_BOX_COVER_PULL_OUT_AXIS=opposite_tool_axis` pulls opposite the current contact approach/tool direction; the action-server prototype projects that direction onto `base_link` XY and keeps lifted Z constant during pull-out. `CONTACT_BOX_COVER_PULL_OUT_M=0.050` is the current development pull-out distance, and the pickup-out script fails closed if it is blank or invalid. The action-server prototype additionally uses `CONTACT_BOX_SUCTION_SETTLE_DELAY_SEC` between suction ON and lift.
- Cover mount offset config: `CONTACT_BOX_COVER_MOUNT_OFFSET_X/Y/Z_M` applies the same runtime mount correction to `cover_mount_approach` and `cover_mount_place`. Keep these zero when the correction is already baked into `config/recorded_robot_poses.json`.
- Command-line flags still override where applicable: `--press-depth-m VALUE` overrides default press depth, and direct `--fast` / staged `--fast-demo` remain explicit speed modes.
- Safety: keep plan-only testing first, increase speed gradually, keep emergency stop reachable, and do not increase speed while suction fault debugging remains unresolved.

## scripts/launch_robot.sh

- Purpose: start the FAIRINO MoveIt/ros2_control/RViz stack.
- Safety: no motion command by itself, but starts robot control infrastructure.
- Contact-box requirement: keep this running before starting the one-shot contact approach or full cover-removal wrapper.
- Usage:

```bash
./scripts/launch_robot.sh
```

## scripts/launch_realsense.sh

- Purpose: start RealSense with aligned depth and pointcloud enabled.
- Safety: non-moving.
- Important: `publish_tf:=false`.
- Contact-box requirement: keep this running before `scripts/launch_contact_box_perception.sh` and before the one-shot/full wrapper.
- Usage:

```bash
./scripts/launch_realsense.sh
```

## scripts/launch_apriltag.sh

- Purpose: start `apriltag_ros` on the RealSense color image/camera info topics.
- Safety: non-moving.
- Usage:

```bash
./scripts/launch_apriltag.sh
```

## scripts/view_camera.sh

- Purpose: start the camera/AprilTag viewer.
- Safety: non-moving.
- Usage:

```bash
./scripts/view_camera.sh
```

## scripts/capture.sh

- Purpose: run camera pose capture with optional step size in millimeters.
- Safety: ROBOT-MOVING.
- Default step size: `10`.
- Usage:

```bash
./scripts/capture.sh
./scripts/capture.sh 10
./scripts/capture.sh 2
```

The script prints the selected step size, warns that the robot will physically move, and counts down for 3 seconds.

## scripts/approach_contact_box.sh

- Purpose: launch the validated locked-contact Cartesian contact-box approach for a supplied standoff distance in meters.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Defaults: validated locked contact `(0.3323, -0.7376, 0.6131)`, `tool_offset_z: 0.263`, and fixed wrist quaternion xyzw `(0.500, 0.490, 0.011, 0.714)`.
- Config: loads `scripts/load_contact_box_motion_config.sh`; stage-specific defaults are preserved while the approach config values remain at template defaults, and edited approach/cartesian/safety values override the MoveIt launch parameters.
- Usage:

```bash
./scripts/approach_contact_box.sh 0.050
./scripts/approach_contact_box.sh 0.050 --execute
```

## scripts/approach_*cm.sh

- Purpose: short wrappers for staged contact-box standoff testing.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Available stages: `30 cm`, `25 cm`, `20 cm`, `15 cm`, `10 cm`, `7.5 cm`, and `5 cm`.
- Usage:

```bash
./scripts/approach_30cm.sh
./scripts/approach_25cm.sh
./scripts/approach_20cm.sh
./scripts/approach_15cm.sh
./scripts/approach_10cm.sh
./scripts/approach_7_5cm.sh
./scripts/approach_5cm.sh
```

## scripts/reset_to_36cm.sh

- Purpose: launch the saved-joint reset utility for the old `testing_home_pose_v5_36cm_reset` pose.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Note: the name is retained for compatibility, but it was based on the old incorrect `tool_offset_z: 0.180` assumption. After correcting `tool_offset_z` to `0.263`, treat this as the current development `contact_box_staging_pose`, not a true 36 cm pose.
- Config: loads `scripts/load_contact_box_motion_config.sh`; reset speed comes from `CONTACT_BOX_RESET_VELOCITY_SCALING` and `CONTACT_BOX_RESET_ACCELERATION_SCALING`.
- Usage:

```bash
./scripts/reset_to_36cm.sh
./scripts/reset_to_36cm.sh --execute
```

## scripts/reset_to_contact_box_staging_pose.sh

- Purpose: clearer wrapper for the current development contact-box staging/recovery pose.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Behavior: delegates to `scripts/reset_to_36cm.sh` to avoid duplicating saved joint values.
- Important: development/testing pose only, not a calibrated deployment pose and not guaranteed safe from every possible robot configuration.
- Usage:

```bash
./scripts/reset_to_contact_box_staging_pose.sh
./scripts/reset_to_contact_box_staging_pose.sh --execute
```

## scripts/reset_to_30cm.sh

- Purpose: launch the saved-joint reset utility for the true 30 cm contact-box ready pose in `pose_records/testing_pose_v6_true_30cm/`.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Config: loads `scripts/load_contact_box_motion_config.sh`; reset speed comes from `CONTACT_BOX_RESET_VELOCITY_SCALING` and `CONTACT_BOX_RESET_ACCELERATION_SCALING`, and saved-pose reset range comes from `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD`.
- Usage:

```bash
./scripts/reset_to_30cm.sh
./scripts/reset_to_30cm.sh --execute
```

The true 30 cm reset script hard-codes logical joint order `j1..j6`, mapped from the recorded `/joint_states` order `j1, j2, j4, j5, j3, j6`.

## scripts/launch_contact_box_perception.sh

- Purpose: launch only the contact-box perception / pose-estimation layer after the operator has manually launched robot / MoveIt and RealSense.
- Launches: `ros2 run pose_estimater object_detect` and `ros2 run pose_estimater contact_box_pose_debug`.
- Does not launch: robot bringup, MoveIt, RealSense, suction, gripper, or any motion command.
- Contact-box requirement: keep this running before one-shot contact approach or the full cover-removal wrapper. Fresh pose acquisition fails closed if this perception layer is unavailable.
- Logs: `logs/contact_box_perception/YYYYMMDD_HHMMSS/`.
- Stop: Ctrl+C stops child processes.
- Usage:

```bash
./scripts/launch_contact_box_perception.sh
```

## scripts/check_contact_box_perception.sh

- Purpose: check whether robot stack, RealSense, TF, YOLO keypoint detection, and contact-box pose estimation are ready.
- Default checks: runtime-only infrastructure readiness: `/joint_states`, camera color/depth/camera info topics, YOLO/object-detection topic presence, `/contact_box_pose_debug` publisher presence, and TF `base_link -> wrist3_link` plus `base_link -> camera_color_optical_frame`.
- Target-visible checks: `--require-target-visible` additionally requires recent keypoint/contact-pose messages and fresh stable sample acquisition. Run it only after the robot is at `true_30cm` / `contact_box_ready_pose`.
- Config: target-visible acquisition uses `CONTACT_BOX_POSE_SAMPLE_COUNT`, `CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC`, and `CONTACT_BOX_POSE_MAX_SPREAD_M`.
- Output: runtime mode prints `CONTACT-BOX RUNTIME CHECK PASS`; target mode prints `PASS: Fresh stable contact-box pose acquired` or `FAIL: No fresh stable contact-box pose. Ensure contact box is visible.`
- Safety: check-only, no robot motion, no suction, no gripper.
- Usage:

```bash
./scripts/check_contact_box_perception.sh
./scripts/check_contact_box_perception.sh --runtime-only
./scripts/check_contact_box_perception.sh --require-target-visible
```

## scripts/check_suction_interface.sh

- Purpose: verify the integrated `fairino_hardware` suction interface.
- Checks: `/initialize_suction` topic, `/fairino_suction_command` service, and exact service type `fairino_msgs/srv/FairinoSuctionCommand`.
- Note: `/initialize_suction` is explicit suction preparation, not suction ON.
- Safety: check-only, no robot motion, no suction, no gripper.
- Current warning: previously reported by user on 2026-07-03, the peripheral/axle-Lua suction path can leave the FAIRINO controller in a non-motion state requiring full controller power cycle even when later ROS/MoveIt motion reports success.
- Usage:

```bash
./scripts/check_suction_interface.sh
```

## scripts/suction_init.sh

- Purpose: prepare the EVS08 suction peripheral by publishing `/initialize_suction`.
- Safety: dry-run by default; publishes the setup topic only with `--execute`; no suction ON command and no robot motion.
- Behavior: with `--execute`, checks that `ros2` is available, warns if `/initialize_suction` has no visible subscriber, publishes `std_msgs/msg/Bool "{data: true}"`, then polls read-only `/fairino_suction_command` command `0 = STATUS` every `0.25 s` until suction preparation is confirmed or the `25 s` timeout expires.
- Timing: prints a compact timing summary for ROS setup, topic/service checks, setup publish, STATUS polling attempts, and total elapsed time.
- Failure handling: exits nonzero if setup cannot be confirmed; check robot stack logs before continuing.
- Usage:

```bash
./scripts/suction_init.sh
./scripts/suction_init.sh --execute
```

## scripts/suction_on.sh

- Purpose: turn on suction A / top two cups only through `/fairino_suction_command`.
- Safety: dry-run by default; real suction command only with `--execute`; no robot motion. Requires successful `./scripts/suction_init.sh --execute` first.
- Current validation note: previously reported by user, A_ON physically turns on the top two suction cups after suction initialization.
- Behavior: sends service command `1`, which maps to `MoveGripper(1,100,70,60,6000,1,1,0,70,60)`. This forces suction B / bottom cups OFF.
- Timing: prints a timing summary for ROS setup, service-call duration, and total elapsed time.
- Failure handling: exits nonzero if the ROS service call fails or the service response is not confirmed as `success=true`.
- Usage:

```bash
./scripts/suction_on.sh
./scripts/suction_on.sh --execute
```

## scripts/suction_off.sh

- Purpose: release suction using EVS08 stop-all/break-vacuum through `/fairino_suction_command`.
- Safety: dry-run by default; real suction command only with `--execute`; no robot motion.
- Current warning: this uses the peripheral/axle-Lua gripper protocol path and still requires cautious validation; successful service response does not by itself prove the controller can still move afterward.
- Behavior: sends service command `6`, which maps to `MoveGripper(1,0,70,60,6000,1,1,0,70,60)`. This forces A/top cups OFF and B/bottom cups OFF, and intentionally avoids `ActGripper(1,0)` for normal release.
- Timing: prints a timing summary for ROS setup, service-call duration, and total elapsed time.
- Failure handling: exits nonzero if the ROS service call fails or the service response is not confirmed as `success=true`. If OFF returns `success=false`, stop suction testing and recover/release manually before continuing.
- Usage:

```bash
./scripts/suction_off.sh
./scripts/suction_off.sh --execute
```

## Legacy suction aliases

- `scripts/suction_a_on.sh` is a backward-compatible alias to `scripts/suction_on.sh`.
- `scripts/suction_a_off.sh` is a backward-compatible alias to `scripts/suction_off.sh`.
- `scripts/suction_all_off.sh` is a backward-compatible safe release alias to `scripts/suction_off.sh`.
- `scripts/suction_all_on.sh` is disabled and exits nonzero even with `--execute`; this project workflow never turns on suction B / bottom cups.
- Service commands `3 = B_ON`, `4 = B_OFF`, and `5 = ALL_ON` are rejected before any SDK call.

Normal operator-facing workflow is only `suction_init.sh`, `suction_on.sh`, and `suction_off.sh`.

## scripts/suction_tool_do_on.sh

- Purpose: dry-run-only proposal for a future suction ON path through FAIRINO `SetToolDO`.
- Behavior: prints the intended SDK call shape `SetToolDO(id, 1, smooth=0, block=1)` and the physical wiring/config items that must be confirmed.
- Safety: never commands hardware. `--execute` is intentionally refused until wiring is confirmed and a ToolDO service using the active `fairino_hardware` `_ptr_robot` connection is implemented.
- Usage:

```bash
./scripts/suction_tool_do_on.sh
./scripts/suction_tool_do_on.sh --id 0
./scripts/suction_tool_do_on.sh --id 1
```

## scripts/suction_tool_do_off.sh

- Purpose: dry-run-only proposal for a future suction OFF path through FAIRINO `SetToolDO`.
- Behavior: prints the intended SDK call shape `SetToolDO(id, 0, smooth=0, block=1)` and the physical wiring/config items that must be confirmed.
- Safety: never commands hardware. `--execute` is intentionally refused until wiring is confirmed and a ToolDO service using the active `fairino_hardware` `_ptr_robot` connection is implemented.
- Usage:

```bash
./scripts/suction_tool_do_off.sh
./scripts/suction_tool_do_off.sh --id 0
./scripts/suction_tool_do_off.sh --id 1
```

## scripts/test_suction_cycle.sh

- Purpose: legacy placeholder for the old integrated suction cycle.
- Behavior: checks the suction interface first. Dry-run reports that the repeated B_OFF -> B_ON -> B_OFF cycle is disabled. `--execute` exits nonzero without sending suction commands.
- Safety: real suction commands require `--execute`; no robot motion; no real suction/grip feedback is currently available.
- Current validation note: OFF now uses EVS08 full-state `MoveGripper` release instead of `ActGripper(1,0)`, but repeated cycle testing remains disabled until the peripheral path is hardware-validated.
- Usage:

```bash
./scripts/test_suction_cycle.sh
```

## scripts/acquire_contact_pose.py

- Purpose: collect and average live `/contact_box_pose_debug` samples, then write `/tmp/contact_box_locked_pose.env` only on success.
- Safety: perception-only, no robot motion.
- Defaults: `5` samples, `5.0 sec` timeout, `0.015 m` max spread.
- Freshness: removes the output env file at startup, accepts only samples received after startup, ignores nonzero header timestamps older than startup, and writes no env file on failure.
- Output: `LOCKED_CONTACT_X/Y/Z`, `CONTACT_POSE_MAX_DEVIATION`, and `CONTACT_POSE_ACQUIRED_AT_UNIX`.
- Usage:

```bash
./scripts/acquire_contact_pose.py
```

## scripts/check_locked_standoff.py

- Purpose: estimate signed suction-tip standoff from `base_link -> wrist3_link` TF and a locked contact point.
- Safety: TF-only, no robot motion.
- Supports human output, `--env`, and `--json`.
- Usage:

```bash
./scripts/check_locked_standoff.py \
  --contact-x 0.3323 \
  --contact-y -0.7376 \
  --contact-z 0.6131
```

## scripts/verify_keypoint_and_touch_contact_box.sh

- Purpose: one-shot weekly demo workflow that acquires a live contact keypoint, applies the configured target offset, checks standoff to the adjusted suction target, and runs staged Cartesian approach planning to the validated contact standoff.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Freshness: removes stale `/tmp/contact_box_locked_pose.env`, requires fresh stable acquisition, validates `LOCKED_CONTACT_X/Y/Z`, and fails closed if acquisition fails.
- Diagnostics: logs live-vs-manual delta against the development manual point `(0.3323, -0.7376, 0.6131)`.
- Final validated contact target: `standoff_distance: 0.002`.
- Config: loads `scripts/load_contact_box_motion_config.sh` for pose sampling, live-vs-manual warning distance, Cartesian jump/min-fraction, and optional approach speed/safety overrides. The existing per-stage speed/safety profile is preserved while config values remain at template defaults.
- Hardware: does not command suction or gripper hardware.
- Note: this is the safer engineering validation path because it uses staged Cartesian targets.
- Usage:

```bash
./scripts/verify_keypoint_and_touch_contact_box.sh
./scripts/verify_keypoint_and_touch_contact_box.sh --execute
./scripts/verify_keypoint_and_touch_contact_box.sh --execute --fast-demo
```

`--fast-demo` is only valid with `--execute`. It skips the separate plan-only launch per stage for filming, but each stage still plans, safety-checks, retimes, and executes inside `contact_box_plan_debug`.

## scripts/verify_keypoint_and_touch_contact_box_direct.sh

- Purpose: deployment-style one-shot test from the true 30 cm testing pose to contact.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Assumption: robot is already at a pose where the contact box should be visible, normally `true_30cm` / `contact_box_ready_pose`.
- Behavior: removes stale `/tmp/contact_box_locked_pose.env`, acquires live keypoint/contact pose once, validates `LOCKED_CONTACT_X/Y/Z`, applies `CONTACT_BOX_TARGET_OFFSET_*_M`, logs raw-live-vs-manual and adjusted-target-vs-manual deltas, checks current standoff/lateral error to the adjusted suction target using `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M` for the pre-approach lateral gate, then calls `contact_box_plan_debug` once for `standoff_distance: 0.002`.
- Press mode: `--press` or `--press-depth-m VALUE` adds an explicit slow compression move after the default `0.002 m` target. Press remains no-motion without `--execute` and never commands suction hardware.
- Config: loads `scripts/load_contact_box_motion_config.sh` for default standoff, pose sampling, press depth/limits, press speed, Cartesian parameters, and optional direct approach speed/safety overrides. The true-30cm-to-contact envelope remains at the existing defaults unless `CONTACT_BOX_APPROACH_*`, `CONTACT_BOX_CARTESIAN_EEF_STEP`, `CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE`, or `CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD` are edited away from template defaults.
- Hardware: does not command suction or gripper hardware.
- Usage:

```bash
./scripts/verify_keypoint_and_touch_contact_box_direct.sh
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --fast
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --press
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --press-depth-m 0.003
```

`--fast` is only valid with `--execute` and overrides direct approach velocity/acceleration scaling to `0.005`.
`--press-depth-m` implies `--press`; requests above `CONTACT_BOX_MAX_PRESS_DEPTH_M=0.005` are refused, and negative target standoff is clamped to `0.000 m` by default.

## scripts/run_contact_box_development_sequence.sh

- Purpose: run the current full contact-box development sequence.
- Sequence: current pose where camera may not see target, development contact-box staging pose, true 30 cm contact-box ready pose, fresh contact-pose acquisition, direct one-shot keypoint-to-contact approach, optional press.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Fail-closed behavior: if staging reset or true 30 cm reset returns nonzero, the sequence exits immediately and does not run keypoint acquisition or direct contact approach.
- Fast flag: `--execute --fast` forwards `--fast` only to the direct one-shot contact script.
- Press flags: `--press` and `--press-depth-m VALUE` are forwarded only to the final direct contact script, never to staging or ready-pose reset.
- Config: loads `scripts/load_contact_box_motion_config.sh` at startup so invalid motion config fails before child motion scripts are launched. Child scripts also load and validate the same config before passing MoveIt launch parameters.
- Freshness: no target visibility is required before staging or true 30 cm. Before the final approach, the direct script must acquire a fresh stable pose from the true 30 cm camera view; stale env reuse is refused.
- Usage:

```bash
./scripts/run_contact_box_development_sequence.sh
./scripts/run_contact_box_development_sequence.sh --execute
./scripts/run_contact_box_development_sequence.sh --execute --fast
./scripts/run_contact_box_development_sequence.sh --execute --press
./scripts/run_contact_box_development_sequence.sh --execute --press-depth-m 0.003
```

Current contact-box staging pose, true 30 cm ready pose, manual contact point, fixed orientation, and press depth are development/testing values only. Actual test lift car top deployment requires recalibration as documented in [Contact Box Vision-to-Motion Developer Guide](contact-box-vision-to-motion-developer-guide.md).

## scripts/run_contact_box_cover_pickout_sequence.sh

- Purpose: pickup-out motion only, starting after the robot is already pressed/contacting the cover and `./scripts/suction_on.sh --execute` has already succeeded.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied. Does not command suction, does not release suction, and does not move to `cover_mount_approach` or `cover_mount_place`.
- Behavior: reads current `base_link -> wrist3_link` TF, plans a Cartesian lift by `CONTACT_BOX_COVER_LIFT_Z_M` in `base_link` +Z, then plans a Cartesian pull opposite the current tool +Z/contact approach direction by `CONTACT_BOX_COVER_PULL_OUT_M`.
- Fail-closed config: `CONTACT_BOX_COVER_PULL_OUT_M` must be set in `config/contact_box_motion.env`; it is blank by default until the pull-out distance is visually/mechanically validated.
- Postcondition: cover should be clear of the contact box and suction remains ON. Table-mount transfer/place and suction release are separate later steps.
- Usage:

```bash
./scripts/run_contact_box_cover_pickout_sequence.sh
./scripts/run_contact_box_cover_pickout_sequence.sh --execute
```

## scripts/run_contact_box_cover_to_mount_sequence.sh

- Purpose: move the already-picked-out cover to the recorded table-mount poses.
- Preconditions: `scripts/run_contact_box_cover_pickout_sequence.sh --execute` has completed, the cover is clear of the contact box, suction A/top cups are still ON, and `config/recorded_robot_poses.json` contains `cover_mount_approach` and `cover_mount_place`.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied. Does not command suction, does not call `suction_off.sh`, does not release suction, and does not retreat from `cover_mount_place`.
- Behavior: validates `config/recorded_robot_poses.json`, applies `CONTACT_BOX_COVER_MOUNT_OFFSET_*_M` to both recorded mount tool poses at runtime, moves to recorded `cover_mount_approach` with the saved recorded-pose joint helper, makes a small Cartesian nudge to shifted `cover_mount_approach` when the runtime mount offset is nonzero, then moves from shifted `cover_mount_approach` to shifted `cover_mount_place` with `contact_box_plan_debug` in `planning_mode:=cartesian`. The recorded JSON is not modified.
- Speed/timing note: if the runtime mount offset is effectively zero (`<= 0.0005 m`), the no-op Cartesian nudge from recorded approach to shifted approach is skipped. The script still runs the recorded approach move and final placement move, and prints a timing summary for config/pose loading, each motion stage, and total elapsed time.
- Mount offset defaults: `CONTACT_BOX_COVER_MOUNT_OFFSET_X_M=0.000`, `CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M=0.000`, `CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M=-0.000`. These are separate from the contact-box vision target offset.
- Motion note: shifted `cover_mount_approach -> shifted cover_mount_place` is Cartesian only; there is no joint-space fallback for that step. The script warns if the recorded approach/place poses are nearly identical or if their orientations differ.
- Postcondition: cover should be at `cover_mount_place` and suction remains ON. Suction release is a separate manual step.
- Usage:

```bash
./scripts/run_contact_box_cover_to_mount_sequence.sh
./scripts/run_contact_box_cover_to_mount_sequence.sh --execute
```

## scripts/run_contact_box_cover_mount_retreat_sequence.sh

- Purpose: retreat from shifted `cover_mount_place` to shifted `cover_mount_approach` after the cover has been released.
- Preconditions: the cover is already at shifted `cover_mount_place`, and `./scripts/suction_off.sh --execute` has already succeeded.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied. Does not command suction and does not call `suction_off.sh`.
- Behavior: validates `config/recorded_robot_poses.json`, applies `CONTACT_BOX_COVER_MOUNT_OFFSET_*_M` to the recorded mount poses at runtime, then moves from shifted place to shifted approach with `contact_box_plan_debug` in `planning_mode:=cartesian`.
- Timing: prints a timing summary for config/pose loading, Cartesian retreat, and total elapsed time.
- Motion note: shifted `cover_mount_place -> shifted cover_mount_approach` is Cartesian only; there is no joint-space fallback.
- Usage:

```bash
./scripts/run_contact_box_cover_mount_retreat_sequence.sh
./scripts/run_contact_box_cover_mount_retreat_sequence.sh --execute
```

## scripts/run_contact_box_full_cover_removal_sequence.sh

- Purpose: one operator-facing wrapper for the full contact-box cover-removal workflow.
- Safety: dry-run by default. Without `--execute`, it prints and logs the planned stage order only. With `--execute`, it runs child scripts sequentially and stops at the first failure.
- Stage order: `SUCTION_INIT`, `CONTACT_APPROACH_PRESS`, `SUCTION_ON`, `PICKOUT`, `MOVE_TO_MOUNT`, `SUCTION_OFF`, `MOUNT_RETREAT`.
- Suction order: `suction_init.sh --execute` runs before contact approach and retries up to three total attempts. `suction_on.sh --execute` runs only after contact approach/press succeeds. `suction_off.sh --execute` runs only after move-to-mount succeeds. Mount retreat runs only after suction OFF succeeds.
- Logging: writes `logs/contact_box_full_cover_removal_YYYYmmdd_HHMMSS.log` with stage start, exact command, success/failure, exit code, per-stage duration, final failed stage name, and a final timing summary.
- Timing: child scripts also print nested summaries for config loading, fresh pose acquisition/validation, individual MoveIt launches, suction service calls, pickup-out, mount transfer, and retreat.
- Usage:

```bash
./scripts/run_contact_box_full_cover_removal_sequence.sh
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

## scripts/run_contact_box_cover_removal_action.sh

- Purpose: developer wrapper for the persistent `moveit_move` contact-box cover-removal action-server prototype.
- Action: sends goals to `/contact_box_cover_removal` with type `moveit_move/action/ContactBoxCoverRemoval`.
- Safety: dry-run by default (`execute=false`, `dry_run=true`) for a fast action plumbing check. `--plan-only` sends `execute=false`, `dry_run=false` for full non-motion planning validation. No robot motion or suction is commanded unless `--execute` is explicitly supplied.
- Press behavior: `--press` is the default to match the current full cover-removal workflow; `--no-press` skips the optional controlled press-in stage.
- Preconditions: the action server is already running, normally through `ros2 launch moveit_move contact_box_cover_removal_action_server.launch.py`. Real execution also requires the robot/MoveIt, RealSense, and contact-box perception terminals to be running.
- Prototype status: keeps one `MoveGroupInterface` alive across the sequence to reduce repeated MoveIt node startup overhead. It does not replace the current validated bash wrapper yet.
- Usage:

```bash
./scripts/run_contact_box_cover_removal_action.sh
./scripts/run_contact_box_cover_removal_action.sh --dry-run
./scripts/run_contact_box_cover_removal_action.sh --plan-only
./scripts/run_contact_box_cover_removal_action.sh --execute
./scripts/run_contact_box_cover_removal_action.sh --execute --no-press
```

## scripts/demo_contact_box_from_30cm.sh

- Purpose: optional wrapper that can run `reset_to_30cm.sh` first, then run the one-shot contact-box validation script.
- Safety: plan-only by default; robot-moving only when `--execute` is explicitly supplied.
- Usage:

```bash
./scripts/demo_contact_box_from_30cm.sh --reset-first
./scripts/demo_contact_box_from_30cm.sh --reset-first --execute
```

## scripts/verify_latest_handeye.sh

- Purpose: analyze the latest `camera_pose_captures/run_*` metadata directory.
- Safety: analysis-only, no movement.
- Pass condition: fresh detections all valid and max `base_T_tag` position error <= `2.0 mm`.
- Usage:

```bash
./scripts/verify_latest_handeye.sh
```

## scripts/check_system.sh

- Purpose: verify package prefixes and check active source files for legacy absolute workspace references.
- Safety: non-moving.
- Usage:

```bash
./scripts/check_system.sh
```

## scripts/build_safe.sh

- Purpose: slow, low-parallelism Jetson build.
- Safety: non-moving build command.
- Usage:

```bash
./scripts/build_safe.sh
```

## scripts/stop_ros.sh

- Purpose: stop common ROS processes and restart the ROS daemon.
- Safety: non-moving, but disruptive to running ROS sessions.
- Usage:

```bash
./scripts/stop_ros.sh
```

## scripts/README.md

- Purpose: user-facing quick terminal flow.
- Safety: documentation only.
