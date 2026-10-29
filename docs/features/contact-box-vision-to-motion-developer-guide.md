# Contact-Box Cover-Removal Developer Guide

This is the source of truth for the FAIRINO/ROS 2 contact-box cover-removal workflow in this workspace.

The workflow is hardware-moving when run with `--execute`. Read this guide before changing contact-box motion, suction, perception, recorded poses, or tuning values.

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware and Software Context](#2-hardware-and-software-context)
3. [Required Launch Terminals](#3-required-launch-terminals)
4. [Operator Quick Start](#4-operator-quick-start)
5. [Full Workflow Preconditions](#5-full-workflow-preconditions)
6. [Full Workflow Sequence](#6-full-workflow-sequence)
7. [Script Reference](#7-script-reference)
8. [Configuration Reference](#8-configuration-reference)
9. [Recorded Poses](#9-recorded-poses)
10. [New Site Calibration Procedure](#10-new-site-calibration-procedure)
11. [Pose Calibration Commands](#11-pose-calibration-commands)
12. [Safe Validation Procedure](#12-safe-validation-procedure)
13. [Failure Handling and Logs](#13-failure-handling-and-logs)
14. [Common Tuning Examples](#14-common-tuning-examples)
15. [Safety Rules](#15-safety-rules)
16. [Known Limitations / TODOs](#16-known-limitations--todos)

## 1. Overview

The full contact-box cover-removal workflow uses a FAIRINO FR5-V6 arm to:

1. Prepare the EVS08 suction peripheral.
2. Move to the contact-box ready pose.
3. Acquire a fresh YOLO/depth contact-box pose.
4. Press the active top two suction cups onto the cover.
5. Turn suction A/top cups ON.
6. Lift and pull the cover out of the contact box.
7. Move the held cover to the table-mount placement pose.
8. Release suction using the validated EVS08 stop-all/break-vacuum command.
9. Retreat from the table-mount place pose to the table-mount approach pose.

The one-shot operator command is:

```bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

That command does not replace the runtime launch terminals. The robot, RealSense camera, and contact-box perception pipelines must already be running.

## 2. Hardware and Software Context

| Item | Current state |
| --- | --- |
| Robot | FAIRINO FR5-V6 |
| ROS | ROS 2 Humble |
| Motion planning | MoveIt / ros2_control through the local FAIRINO workspace |
| Camera | Intel RealSense |
| Perception | YOLO/depth contact-box pose pipeline |
| End effector | JODELL EVS08 electric suction gripper |
| Suction transport | RS485 through FAIRINO Peripheral / axle-Lua protocol |
| Suction gripper ID | `1` |
| Finger gripper ID | `2` |
| Suction used by this workflow | A / top two cups only |
| Suction intentionally disabled | B / bottom cups |
| Normal release | `suction_off.sh`, EVS08 `MoveGripper(1,0,70,60,6000,1,1,0,70,60)` |
| Forbidden normal release | `ActGripper(1,0)` |

Lua packages, protocol files, and robot-end firmware are vendor/controller-side artifacts. Do not modify Lua or firmware for normal ROS workflow changes.

## 3. Required Launch Terminals

Four terminals are normally required. Terminals 1-3 keep the runtime stack alive. Terminal 4 runs the operator workflow.

### Terminal 1 - Robot / MoveIt

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/launch_robot.sh
```

Keeps the robot, ros2_control, and MoveIt stack alive. This launch script does not command a trajectory by itself, but it starts robot control infrastructure.

### Terminal 2 - RealSense

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/launch_realsense.sh
```

Keeps the RealSense camera pipeline alive. The contact-box perception stack depends on the camera topics.

### Terminal 3 - Contact-Box Perception

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/launch_contact_box_perception.sh
```

Starts the contact-box YOLO/depth pose pipeline. The one-shot/contact approach requires fresh contact-box pose acquisition. If this perception layer is not running, fresh acquisition should fail closed.

### Terminal 4 - Full Cover Removal

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

The full wrapper assumes Terminals 1-3 are already running. It does not launch the robot stack, RealSense, or contact-box perception.

## 4. Operator Quick Start

Plan-only dry run:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_full_cover_removal_sequence.sh
```

The plan-only full wrapper prints the planned stage order and commands only. It must not run robot motion or suction commands.

Hardware execution:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

Before hardware execution, verify that the robot, RealSense, and contact-box perception terminals are still running.

## 5. Full Workflow Preconditions

Before running either:

```bash
./scripts/run_contact_box_development_sequence.sh --execute --press
```

or:

```bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

the following must already be running:

- Robot stack through `./scripts/launch_robot.sh`.
- RealSense through `./scripts/launch_realsense.sh`.
- Contact-box perception through `./scripts/launch_contact_box_perception.sh`.

Perception matters because the final contact approach acquires a fresh live contact-box pose. Stale `/tmp/contact_box_locked_pose.env` reuse is intentionally prevented.

## 6. Full Workflow Sequence

`scripts/run_contact_box_full_cover_removal_sequence.sh --execute` runs each stage synchronously. It waits for the current stage to finish, checks the exit code, and starts the next stage only on success.

The wrapper and major child scripts print timestamped stage logs and a compact `Timing summary:`. The full wrapper records top-level stages (`SUCTION_INIT`, `CONTACT_APPROACH_PRESS`, `SUCTION_ON`, `PICKOUT`, `MOVE_TO_MOUNT`, `SUCTION_OFF`, `MOUNT_RETREAT`) and total elapsed time. The nested scripts break those top-level buckets down further:

- `run_contact_box_development_sequence.sh`: config load, stale pose clearing, staging move, true 30 cm ready move, direct pose validation/final approach.
- `verify_keypoint_and_touch_contact_box_direct.sh`: config/ROS setup, fresh contact-pose acquisition, target offset, live-vs-manual diagnostics, standoff validation, pre-approach safety gates, final approach, optional press, and post-approach verification checks.
- `run_contact_box_cover_pickout_sequence.sh`: config load, pickup-out target computation, lift, and pull-out.
- `run_contact_box_cover_to_mount_sequence.sh`: config/pose loading, move to `cover_mount_approach`, optional mount-offset nudge, and move to `cover_mount_place`.
- `run_contact_box_cover_mount_retreat_sequence.sh`: config/pose loading and Cartesian retreat.
- `suction_init.sh`, `suction_on.sh`, and `suction_off.sh`: ROS setup, service/topic checks or service-call time, and total elapsed time.

Current low-risk speedups are intentionally narrow:

- `suction_init.sh` polls read-only suction STATUS every `0.25 s` instead of every `1.0 s`, with the same `25 s` timeout and the same success confirmation requirement.
- `run_contact_box_cover_to_mount_sequence.sh` skips the Cartesian mount-offset nudge only when the runtime mount offset is effectively zero (`<= 0.0005 m`). The recorded approach move and final placement move still run.

| Stage | Script | Preconditions | Postconditions | Fail-closed behavior |
| --- | --- | --- | --- | --- |
| `SUCTION_INIT` | `./scripts/suction_init.sh --execute` | Robot stack running; suction peripheral available. | EVS08 suction peripheral prepared; suction still OFF. | Retries up to 3 total attempts. If all fail, stop before robot presses onto cover. |
| `CONTACT_APPROACH_PRESS` | `./scripts/run_contact_box_development_sequence.sh --execute --press` | Robot, RealSense, and perception terminals running. | Robot reaches slow contact/press pose against cover; suction still OFF. | If reset, fresh pose acquisition, planning, execution, or press fails, stop before suction ON. |
| `SUCTION_ON` | `./scripts/suction_on.sh --execute` | Press/contact pose succeeded. | Suction A/top cups ON; suction B/bottom cups forced OFF. | If suction ON fails, do not pick/lift/pull. |
| `PICKOUT` | `./scripts/run_contact_box_cover_pickout_sequence.sh --execute` | Suction ON succeeded; cover should be attached. | Cover lifted in `base_link` +Z and pulled clear of contact box; suction remains ON. | If lift or pull fails, stop with suction state requiring operator assessment. |
| `MOVE_TO_MOUNT` | `./scripts/run_contact_box_cover_to_mount_sequence.sh --execute` | Cover is clear and held by suction. | Cover reaches shifted `cover_mount_place`; suction remains ON. | If mount approach/place fails, stop before release. |
| `SUCTION_OFF` | `./scripts/suction_off.sh --execute` | Cover has reached shifted `cover_mount_place`. | EVS08 stop-all/break-vacuum release commanded; suction OFF. | If release fails, do not retreat because cover may still be attached. |
| `MOUNT_RETREAT` | `./scripts/run_contact_box_cover_mount_retreat_sequence.sh --execute` | `suction_off.sh --execute` succeeded. | Robot retreats Cartesian-only to shifted `cover_mount_approach`; cover remains released/supported at mount. | If retreat fails, stop and log `MOUNT_RETREAT`. |

## 6.1 Persistent Action Prototype

`moveit_move` now includes a first persistent action-server prototype:

```text
Node executable: contact_box_cover_removal_action_server
Launch file: src/moveit_move/launch/contact_box_cover_removal_action_server.launch.py
Action: /contact_box_cover_removal
Action type: moveit_move/action/ContactBoxCoverRemoval
Developer wrapper: scripts/run_contact_box_cover_removal_action.sh
```

Purpose: reduce the repeated `ros2 launch` / MoveIt node startup / current-state monitor startup overhead in the shell-script sequence. The action server creates one `MoveGroupInterface`, starts the current-state monitor once, loads `config/contact_box_motion.env` once, reads `config/recorded_robot_poses.json` once, and reuses the same process for staging, ready pose, final approach, press, pickup-out, mount transfer, and retreat.

This is a prototype and does not remove or replace the existing working bash scripts. The bash wrapper remains the validated operator path until the action path is hardware-tested.

Action goal:

```text
bool execute
bool press
bool dry_run
```

Action result:

```text
bool success
string message
string failed_stage
float64 total_elapsed_sec
```

Action feedback:

```text
string current_stage
float64 stage_elapsed_sec
float64 total_elapsed_sec
```

Safety behavior:

- `dry_run=true` is a fast action-plumbing check. It publishes the stage names with `DRY_RUN`, performs no planning, commands no robot motion, and commands no suction.
- `execute=false` and `dry_run=false` is the full plan-only validation mode. It walks every sequence stage, publishes `PLAN_ONLY` feedback, uses the loaded config and recorded poses, waits for live state as needed, acquires a fresh contact pose, computes/checks MoveIt plans where possible, and skips `MoveGroupInterface::execute()`.
- Plan-only suction stages check that `/fairino_suction_command` is discoverable, but do not publish `/initialize_suction` and do not send suction ON/OFF/status commands.
- Real hardware execution requires `execute=true` and `dry_run=false`.
- The server does not shell out to the movement scripts for each motion stage.
- Fresh contact-pose acquisition is in-memory from `/contact_box_pose_debug` after the true 30 cm ready pose. It does not read or reuse `/tmp/contact_box_locked_pose.env`.
- Suction is still commanded only in the preserved order: init, ON after contact/press, OFF after mount placement.
- Action-server pickup-out uses a two-step target: `lift_target=(current.x, current.y, current.z + CONTACT_BOX_COVER_LIFT_Z_M)`, then `pull_target=lift_target + normalize((-tool_axis.x, -tool_axis.y, 0.0)) * CONTACT_BOX_COVER_PULL_OUT_M`, where `tool_axis` is the initial `wrist3_link` tool +Z direction. The pull target keeps the lifted `base_link` Z constant to avoid hook re-latching.
- In real execute mode, the action server logs commanded-vs-actual lift after `LIFT_COVER`, but only requires `actual_delta_z >= CONTACT_BOX_MIN_ACTUAL_LIFT_DELTA_Z_M` before continuing. This mirrors the bash flow, which continues after the lift motion rather than requiring exact target convergence.
- If any stage fails, the action aborts with `success=false`, `failed_stage`, and elapsed time, and later stages do not run.
- Motion speeds, acceleration scaling, Cartesian parameters, target offsets, joint-delta limits, press limits, pickup-out distances, and mount offsets come from the existing config/recorded-pose files.
- Current plan-only validation plans/checks each stage against the current live robot state because the robot is not moved between stages. Stages that depend on the robot physically being at the ready/contact/mount pose can fail closed unless the hardware is already in a compatible pose.

Implemented prototype stages:

```text
SUCTION_INIT
MOVE_TO_STAGING_POSE
MOVE_TO_TRUE_30CM_READY_POSE
ACQUIRE_FRESH_CONTACT_POSE
FINAL_CONTACT_APPROACH
CONTROLLED_PRESS_IN   # only when goal.press=true
SUCTION_ON
SUCTION_SETTLE_DELAY
LIFT_COVER
PULL_COVER_OUT
MOVE_TO_MOUNT
SUCTION_OFF
MOUNT_RETREAT
```

## 7. Script Reference

| Script | Purpose | Executes motion? | Executes suction? | Preconditions | End state |
| --- | --- | --- | --- | --- | --- |
| `scripts/launch_robot.sh` | Launch FAIRINO robot / MoveIt stack. | No trajectory by itself. | No. | Robot/controller available. | Robot control stack stays running. |
| `scripts/launch_realsense.sh` | Launch RealSense camera pipeline. | No. | No. | Camera connected. | Camera topics stay running. |
| `scripts/launch_contact_box_perception.sh` | Launch contact-box YOLO/depth pose pipeline. | No. | No. | Robot TF and RealSense running. | `/contact_box_pose_debug` pipeline stays running. |
| `scripts/run_contact_box_full_cover_removal_sequence.sh` | Full operator-facing cover-removal wrapper. | Yes, only with `--execute` through child scripts. | Yes, only with `--execute` through suction child scripts. | Robot, RealSense, perception running. | Cover released at mount and robot retreated, or fail-closed at failed stage. |
| `scripts/run_contact_box_development_sequence.sh` | Staging, true 30 cm ready pose, fresh pose acquisition, direct contact approach, optional press. | Yes, only with `--execute`. | No. | Robot, RealSense, perception running for final approach. | Robot at contact/press pose. |
| `scripts/verify_keypoint_and_touch_contact_box_direct.sh` | One-shot fresh live contact pose to adjusted suction target. | Yes, only with `--execute`. | No. | Robot at true 30 cm ready pose; perception running. | Robot at final contact/standoff or explicit press pose. |
| `scripts/run_contact_box_cover_pickout_sequence.sh` | Lift and pull cover out after suction ON. | Yes, only with `--execute`. | No. | Robot at press/contact pose; suction A ON. | Cover clear of contact box; suction still ON. |
| `scripts/run_contact_box_cover_to_mount_sequence.sh` | Move held cover to shifted mount approach/place. | Yes, only with `--execute`. | No. | Cover picked out; suction A ON; recorded mount poses exist. | Cover at shifted `cover_mount_place`; suction still ON. |
| `scripts/run_contact_box_cover_mount_retreat_sequence.sh` | Retreat after release. | Yes, only with `--execute`. | No. | Cover at shifted `cover_mount_place`; suction OFF succeeded. | Robot at shifted `cover_mount_approach`. |
| `scripts/run_contact_box_cover_removal_action.sh` | Developer wrapper for the persistent action-server prototype. | No in dry-run or `--plan-only`; yes only with `--execute` goal. | No in dry-run or `--plan-only`; yes only with `--execute` goal. | Action server already launched; robot/RealSense/perception required for plan-only and real execution. | Dry-run feedback, plan-only validation, or action-controlled full sequence. |
| `scripts/suction_init.sh` | Prepare EVS08 peripheral; poll read-only STATUS when available. | No. | Prepares peripheral only; suction should stay OFF. | Robot stack and suction service available. | Suction prepared. |
| `scripts/suction_on.sh` | Turn ON suction A/top cups only. | No. | Yes, only with `--execute`. | `suction_init.sh --execute` succeeded; robot already pressed onto cover in full workflow. | A/top cups ON; B/bottom cups OFF. |
| `scripts/suction_off.sh` | Release cover with EVS08 stop-all/break-vacuum. | No. | Yes, only with `--execute`. | Cover at mount place in full workflow. | A and B OFF via full-state `MoveGripper`; no `ActGripper(1,0)`. |
| `scripts/load_contact_box_motion_config.sh` | Shared config loader/validator. | No. | No. | Sourced by scripts. | Exports validated `CONTACT_BOX_*` values. |

## 8. Configuration Reference

Current values below come from `config/contact_box_motion.env` at the time of this documentation update. Treat them as development values, not universal deployment values.

| Variable | Meaning | Typical/current value | When to tune | Safety notes |
| --- | --- | --- | --- | --- |
| `CONTACT_BOX_RESET_VELOCITY_SCALING` | Saved-pose reset speed scaling. | `0.03` | Reset moves too slow/fast. | Increase gradually. |
| `CONTACT_BOX_RESET_ACCELERATION_SCALING` | Saved-pose reset acceleration scaling. | `0.03` | Reset moves too slow/fast. | Increase gradually. |
| `CONTACT_BOX_APPROACH_VELOCITY_SCALING` | Cartesian approach speed scaling. | `0.03` | Contact approach or cover transfer speed tuning. | Keep low near contact. |
| `CONTACT_BOX_APPROACH_ACCELERATION_SCALING` | Cartesian approach acceleration scaling. | `0.03` | Contact approach or cover transfer tuning. | Keep low near contact. |
| `CONTACT_BOX_PRESS_VELOCITY_SCALING` | Optional press-in speed. | `0.001` | Press too slow/fast. | Keep very conservative. |
| `CONTACT_BOX_PRESS_ACCELERATION_SCALING` | Optional press-in acceleration. | `0.001` | Press too slow/fast. | Keep very conservative. |
| `CONTACT_BOX_CARTESIAN_EEF_STEP` | MoveIt Cartesian interpolation step. | `0.0005` | Cartesian fraction or path quality. | Smaller can be slower. |
| `CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD` | MoveIt Cartesian jump threshold. | `0.0` | Only after planning review. | Do not mask unsafe jumps. |
| `CONTACT_BOX_CARTESIAN_MIN_FRACTION` | Minimum accepted Cartesian path fraction. | `0.99` | Planning strictness. | Lowering accepts less complete paths. |
| `CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE` | General max wrist target distance for guarded contact paths. | `0.5` in current env | Only when target envelope is understood. | The loader historically bounds this tightly; if validation rejects the value, reduce it before execution. |
| `CONTACT_BOX_PRESS_MAX_WRIST_TARGET_DISTANCE` | Max wrist move for press. | `0.010` | Press validation. | Keep strict. |
| `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD` | Max joint delta for saved-pose resets. | `1.50` | Wider deployment start poses. | Do not remove the limit. |
| `CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD` | Max joint delta for Cartesian approach/transfer planning. | `1.20` | Approach planning fails on expected move. | Keep separate from press. |
| `CONTACT_BOX_PRESS_MAX_JOINT_DELTA_RAD` | Max joint delta for small press. | `0.03` | Rarely. | Keep strict; do not inherit reset/approach value. |
| `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M` | Allowed starting lateral error to adjusted suction target. | `0.1` | Intentional contact target offset creates a larger starting lateral error. | Does not replace final validation. |
| `CONTACT_BOX_MAX_LIVE_VS_MANUAL_DELTA_M` | Warning threshold comparing live pose to manual development point. | `0.020` | Perception/site calibration diagnostics. | Warning/diagnostic, not a substitute for visual checks. |
| `CONTACT_BOX_DEFAULT_STANDOFF_M` | Normal final contact standoff before optional press. | `0.002` | Contact tuning. | Do not make `0.000` normal without explicit press logic. |
| `CONTACT_BOX_DEFAULT_PRESS_DEPTH_M` | Default explicit press depth. | `0.003` | Suction seal compression. | Must remain within max press depth. |
| `CONTACT_BOX_MAX_PRESS_DEPTH_M` | Max allowed explicit press depth. | `0.005` | Site/tool compliance. | Keep conservative. |
| `CONTACT_BOX_TARGET_OFFSET_X_M` | Base-frame X offset from fresh live contact pose to suction target. | `0.010` | Top cups miss solid cover material. | This belongs only to contact-box approach, not table mount. |
| `CONTACT_BOX_TARGET_OFFSET_Y_M` | Base-frame Y offset from fresh live contact pose to suction target. | `0.000` | Top cups miss solid cover material. | Visually verify direction. |
| `CONTACT_BOX_TARGET_OFFSET_Z_M` | Base-frame Z offset from fresh live contact pose to suction target. | `-0.030` | Move top cups away from cover holes. | Direction depends on base frame/site. |
| `CONTACT_BOX_COVER_LIFT_Z_M` | Pickup-out lift distance in `base_link` +Z after suction ON. | `0.015` | Screw/hook clearance. | Lift is a separate vertical step before pull-out. |
| `CONTACT_BOX_MIN_ACTUAL_LIFT_DELTA_Z_M` | Minimum measured positive `base_link` Z lift required before action-server pull-out. | `0.003` | Lift execution undershoots but still clears hooks. | Must be between `0.0` and `CONTACT_BOX_COVER_LIFT_Z_M`; action logs larger target error but continues if this minimum is met. |
| `CONTACT_BOX_COVER_PULL_OUT_AXIS` | Pull-out direction source. | `opposite_tool_axis` | Only if approach geometry changes. | Action-server pull-out starts from the lifted pose and moves opposite the approach/tool direction projected onto `base_link` XY while keeping lifted Z constant. |
| `CONTACT_BOX_COVER_PULL_OUT_M` | Pull-out distance after lift. | `0.050` | Cover not clear or excessive travel. | Calibrate visually. |
| `CONTACT_BOX_SUCTION_SETTLE_DELAY_SEC` | Action-server delay after suction ON and before cover lift. | `2.0` | Suction needs more or less time to seal before lift. | Executes no robot motion and sends no extra suction command during the delay. |
| `CONTACT_BOX_COVER_MOUNT_OFFSET_X_M` | Runtime X offset for both mount poses. | `0.000` | Placement barely misses in X. | Use either runtime offset or baked JSON offset, not both. |
| `CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M` | Runtime Y offset for both mount poses. | `0.000` | Placement barely misses in Y. | Apply to both approach and place. |
| `CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M` | Runtime Z offset for both mount poses. | `-0.000` | Placement height nudge. | Be careful near table contact. |
| `CONTACT_BOX_POSE_SAMPLE_COUNT` | Number of fresh contact pose samples. | `5` | Perception stability. | More samples can take longer. |
| `CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC` | Time allowed for fresh samples. | `5.0` | Camera/perception latency. | Too short causes false fail. |
| `CONTACT_BOX_POSE_MAX_SPREAD_M` | Max allowed spread among fresh samples. | `0.015` | Perception noise. | Higher accepts noisier target. |

Runtime mount offsets should normally be zero if the offsets have already been baked into `config/recorded_robot_poses.json`.

## 9. Recorded Poses

Recorded poses live in `config/recorded_robot_poses.json`. The current schema is `fairino_pose_recorder_v1`, generated by `scripts/pose_recorder_gui.py`.

| Pose name | Purpose | When used | How to calibrate | Safety notes |
| --- | --- | --- | --- | --- |
| `cover_contact_box_clear` | Cover lifted/pulled out of contact box; suction still ON. Safe clear pose for future reinsertion or recovery workflows. | Not used by the current full wrapper, but retained as a useful calibrated clear pose. | After successful pickup-out, record the current clear pose with suction still ON and cover clear of the box. | Do not assume this pose is safe at a new site without recalibration. |
| `cover_mount_approach` | Safe pose above the table mount. Intended to be directly above `cover_mount_place`. | Used before placement and after release/retreat. | From `cover_mount_place`, move straight upward only, keep x/y and orientation nearly unchanged, then record. | Approach -> place and place -> retreat should be mostly vertical Cartesian moves. |
| `cover_mount_place` | Final supported table-mount placement pose. | Full wrapper releases suction only after reaching this pose. | Hold the cover in its final supported table-mount position and record. | Suction should be released only here in the full workflow. |

Validate mount geometry after recording:

- `cover_mount_approach.z` should be higher than `cover_mount_place.z`.
- x/y should be close.
- orientation should be close.
- Cartesian planning fraction should meet `CONTACT_BOX_CARTESIAN_MIN_FRACTION`.

## 10. New Site Calibration Procedure

Use this checklist when moving to a new site/setup.

1. Physically mount the robot, RealSense camera, contact box, and table mount.
2. Start the robot stack, RealSense, and contact-box perception in separate terminals as shown in [Required Launch Terminals](#3-required-launch-terminals).
3. Confirm RealSense and YOLO/depth perception are working. Use runtime checks first; require target visibility only when the robot is at the ready pose.
4. Confirm suction init/on/off using only the validated suction scripts:

   ```bash
   ./scripts/suction_init.sh --execute
   ./scripts/suction_on.sh --execute
   ./scripts/suction_off.sh --execute
   ```

5. Calibrate contact-box target offsets:

   ```text
   CONTACT_BOX_TARGET_OFFSET_X_M
   CONTACT_BOX_TARGET_OFFSET_Y_M
   CONTACT_BOX_TARGET_OFFSET_Z_M
   ```

   These shift the detected live contact point so the active top two suction cups land on solid cover material instead of holes.

6. Calibrate pre-approach lateral error:

   ```text
   CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M
   ```

   This must allow the intentional target offset while still blocking unexpected target jumps.

7. Calibrate pickup-out:

   ```text
   CONTACT_BOX_COVER_LIFT_Z_M
   CONTACT_BOX_MIN_ACTUAL_LIFT_DELTA_Z_M
   CONTACT_BOX_COVER_PULL_OUT_AXIS
   CONTACT_BOX_COVER_PULL_OUT_M
   CONTACT_BOX_SUCTION_SETTLE_DELAY_SEC
   ```

   Lift should move only in `base_link` +Z and clear screws before the pull-out starts. Action-server pull-out should start from the lifted pose and move opposite the contact approach/tool direction projected onto `base_link` XY, with lifted Z held constant to prevent hook re-latching. The action-server settle delay gives suction time to seal before lift without commanding motion or suction again.

8. Record `cover_contact_box_clear` after a successful pickup-out.
9. Record `cover_mount_place`: hold the cover in the final supported table-mount position and save the pose.
10. Record `cover_mount_approach`: from `cover_mount_place`, move straight upward only, keep x/y/orientation nearly unchanged, and save the pose.
11. Validate approach/place geometry: approach z higher than place z, x/y close, orientation close, Cartesian fraction acceptable.
12. If placement barely misses, choose one method:

    - Tune runtime mount offsets in `config/contact_box_motion.env`, or
    - Bake the offset into `config/recorded_robot_poses.json` and reset runtime mount offsets to zero.

    Do not apply both a baked JSON offset and runtime mount offsets.

13. Run plan-only first:

    ```bash
    ./scripts/run_contact_box_full_cover_removal_sequence.sh
    ```

14. Run hardware execution only with the emergency stop ready:

    ```bash
    ./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
    ```

## 11. Pose Calibration Commands

The actual pose recording tool in this repo is:

```bash
python3 scripts/pose_recorder_gui.py
```

The GUI writes to:

```text
config/recorded_robot_poses.json
```

It records:

- Latest `/joint_states`.
- TF `base_link -> wrist3_link`.
- Pose name and notes.

Before using the GUI, the robot/MoveIt stack must be running so `/joint_states` and TF are available. If the GUI cannot read `/joint_states` or TF, it refuses to save.

Use these exact pose names for the current cover workflow:

```text
cover_contact_box_clear
cover_mount_place
cover_mount_approach
```

Needs revalidation: the GUI is an interactive Tk tool. It is the located repo tool for recording `config/recorded_robot_poses.json`, but this documentation update did not launch it because GUI/runtime hardware commands were out of scope.

## 12. Safe Validation Procedure

Documentation/static checks:

```bash
cd ~/Desktop/fst_test_fa_ws_arm

bash -n scripts/run_contact_box_full_cover_removal_sequence.sh
bash -n scripts/run_contact_box_cover_mount_retreat_sequence.sh
bash -n scripts/run_contact_box_cover_to_mount_sequence.sh
bash -n scripts/run_contact_box_cover_pickout_sequence.sh
bash -n scripts/run_contact_box_development_sequence.sh
bash -n scripts/suction_init.sh
bash -n scripts/suction_on.sh
bash -n scripts/suction_off.sh
bash -n scripts/run_contact_box_cover_removal_action.sh
bash -n scripts/launch_robot.sh
bash -n scripts/launch_realsense.sh
bash -n scripts/launch_contact_box_perception.sh

python3 -m py_compile scripts/acquire_contact_pose.py
python3 -m json.tool config/recorded_robot_poses.json >/tmp/recorded_robot_poses.validated.json || true
```

Build the action-server prototype after changing its C++ or action definition:

```bash
colcon build --packages-select moveit_move
source install/setup.bash
```

Useful config/doc inspection:

```bash
grep -n "CONTACT_BOX_" config/contact_box_motion.env
grep -R "run_contact_box_full_cover_removal_sequence\|cover_mount_approach\|cover_mount_place" -n docs scripts config | head -n 150
```

Plan-only full wrapper:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_full_cover_removal_sequence.sh
```

Hardware execution, only after Terminals 1-3 are running:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

Persistent action-server prototype launch:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_move contact_box_cover_removal_action_server.launch.py
```

In another terminal:

```bash
cd ~/Desktop/fst_test_fa_ws_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_contact_box_cover_removal_action.sh --dry-run
```

Full non-motion plan-only validation:

```bash
./scripts/run_contact_box_cover_removal_action.sh --plan-only
```

Persistent action-server prototype, hardware execution only after robot/MoveIt, RealSense, and contact-box perception are running and the dry-run and plan-only goals have succeeded:

```bash
./scripts/run_contact_box_cover_removal_action.sh --execute
```

Collect the real timing report from the full wrapper log after a hardware run:

```bash
tail -n 160 "$(ls -t logs/contact_box_full_cover_removal_*.log | head -n 1)"
```

## 13. Failure Handling and Logs

The full wrapper writes:

```text
logs/contact_box_full_cover_removal_YYYYmmdd_HHMMSS.log
```

The log includes timestamped stage starts, exact child commands, exit codes, per-stage durations, nested child timing summaries, and a final top-level timing summary. Use the timing output to identify whether the dominant blocker is suction setup, staging/ready reset, fresh perception acquisition, final approach planning/execution, press, pickup-out, table-mount transfer, suction release, or retreat.

Find and tail the latest log:

```bash
ls -lt logs/contact_box_full_cover_removal_*.log | head
tail -n 100 "$(ls -t logs/contact_box_full_cover_removal_*.log | head -n 1)"
```

Failure message:

```text
FULL COVER REMOVAL FAILED at stage: <STAGE_NAME>
```

Stage meanings:

| Failed stage | Meaning | Operator response |
| --- | --- | --- |
| `SUCTION_INIT` | Peripheral setup did not confirm after up to 3 attempts. | Do not run contact approach. Check robot stack and suction service logs. |
| `CONTACT_APPROACH_PRESS` | Reset, fresh pose acquisition, planning, execution, or press failed. | Do not turn suction ON. Check perception and MoveIt logs. |
| `SUCTION_ON` | A/top-cups ON command failed. | Do not pick out. Check suction service and hardware state. |
| `PICKOUT` | Lift/pull failed after suction may be ON. | Stop and assess cover attachment manually. Do not guess recovery. |
| `MOVE_TO_MOUNT` | Transfer/place failed while holding cover. | Keep suction state in mind; cover may still be attached. |
| `SUCTION_OFF` | Release command failed at mount. | Do not retreat; cover may still be attached. Recover manually. |
| `MOUNT_RETREAT` | Release succeeded but Cartesian retreat failed. | Cover should be released; inspect MoveIt/planning state. |

If any failure occurs after `SUCTION_ON`, assume the cover may be attached until a human verifies otherwise.

## 14. Common Tuning Examples

Adjust contact target to move active top cups on the cover:

```bash
CONTACT_BOX_TARGET_OFFSET_X_M=0.005
CONTACT_BOX_TARGET_OFFSET_Y_M=0.000
CONTACT_BOX_TARGET_OFFSET_Z_M=-0.030
```

Increase pickup pull-out distance after visual validation:

```bash
CONTACT_BOX_COVER_PULL_OUT_M=0.060
```

Tune table-mount placement at runtime:

```bash
CONTACT_BOX_COVER_MOUNT_OFFSET_X_M=0.010
CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M=0.000
CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M=0.000
```

If the mount correction is baked into `config/recorded_robot_poses.json`, reset runtime mount offsets:

```bash
CONTACT_BOX_COVER_MOUNT_OFFSET_X_M=0.000
CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M=0.000
CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M=0.000
```

Increase `CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE` only when the expected target distance is understood. This limit exists to catch unexpected target jumps and should not be used to paper over bad perception or stale pose data.

## 15. Safety Rules

- Do not use `ActGripper(1,0)` for normal release.
- Do not enable suction B / bottom cups.
- Do not run `suction_on.sh --execute` before contact press succeeds.
- Do not pick out unless `suction_on.sh --execute` succeeds.
- Do not run `suction_off.sh --execute` before the cover reaches `cover_mount_place`.
- Do not retreat from `cover_mount_place` unless `suction_off.sh --execute` succeeds.
- Do not reuse stale `/tmp/contact_box_locked_pose.env`.
- Do not run the one-shot or full sequence unless robot, RealSense, and contact-box perception terminals are already running.
- Do not use arbitrary joint-space movement for approach -> place or place -> retreat; use Cartesian-only wrappers.
- Keep the emergency stop ready during hardware validation.
- Do not modify Lua or firmware for normal ROS workflow tuning.

## 16. Known Limitations / TODOs

- `config/recorded_robot_poses.json` contains `NaN` in velocity/effort arrays. Python's `json` tooling tolerates this by default, but strict JSON parsers may reject it.
- Current numeric poses, offsets, and speeds are development/site values. New site deployment requires recalibration.
- Runtime mount offsets and baked JSON offsets must not both be applied.
- The full wrapper assumes the robot, RealSense, and contact-box perception terminals remain alive for the whole run.
- The current tool/TCP and suction contact geometry are validated for this setup but should be rechecked after any end-effector, camera, cover, or mount change.
- The pose recorder GUI is the located tool for `recorded_robot_poses.json`; it was not launched during this documentation-only update.
