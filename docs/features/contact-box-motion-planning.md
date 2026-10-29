# Contact Box Motion Planning

Purpose: document the MoveIt motion-planning pieces used by the contact-box workflow.

Current source of truth: [Contact Box Vision-to-Motion Developer Guide](contact-box-vision-to-motion-developer-guide.md). Read that guide before changing contact-box poses, TCP/tool offset assumptions, scripts, launch parameters, perception assumptions, or safety thresholds.

## Status

Implemented plan-only/default-safe node:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py
```

This node subscribes to the safe perception pose:

```text
/contact_box_pose_debug
```

It does not subscribe to `/object_pose`, does not publish `/object_pose`, and does not call gripper or suction services. Execution remains disabled by default and requires both `execute_motion:=true` and `allow_standoff_execution:=true`.

Previously reported by user: the direct one-shot contact-box approach has successfully reached physical contact from the current true 30 cm ready pose with `standoff_distance:=0.002`, `tool_offset_z:=0.263`, and fixed orientation xyzw `(0.500, 0.490, 0.011, 0.714)`.

Current wrapper scripts enforce fresh contact-pose acquisition before the final contact approach, after the robot reaches `true_30cm` / `contact_box_ready_pose`. The camera does not need to see the contact box at the initial robot pose. Stale `/tmp/contact_box_locked_pose.env` is removed, fresh `/contact_box_pose_debug` samples are required for the final approach, and missing or unstable acquisition fails closed. Optional press-in is explicit via `--press` or `--press-depth-m`; the normal final target remains `standoff_distance:=0.002`.

## Old Motion Workflow Risk

Verified from source inspection:

- `src/moveit_move/src/moveit_move.cpp` subscribes to `/object_pose` as `geometry_msgs/msg/PoseStamped`.
- It uses planning group `fairino5_v6_group`.
- It calls `setPoseTarget(msg->pose, "camera_link")`, so the target link is `camera_link`.
- It plans and immediately calls `move_group_->execute(plan)` on the first received pose.
- After execution, it calls the `/gripper_command` service.
- It adds a hardcoded table collision object, but does not model the contact box or suction tool.
- It has no configurable approach offsets.

Related unsafe helpers:

- `src/moveit_move/src/hello_moveit.cpp` plans to a hardcoded pose and executes immediately.
- `src/pose_estimater/pose_estimater/send_goal_pose.py` publishes RViz plan and execute triggers.
- `object_pose_estimate.py` publishes `/object_pose` with identity orientation.
- `object_pose_estimate2.py` publishes `/object_pose` using a rough direction-derived orientation and also applies legacy point scaling.

This old workflow is unsafe for contact-box pickup because a perception message on `/object_pose` can directly trigger robot motion and gripper actuation.

## New Plan-Only Workflow

New file:

```text
src/moveit_move/src/contact_box_plan_debug.cpp
```

Entry point:

```text
contact_box_plan_debug
```

Inputs:

```text
/contact_box_pose_debug   geometry_msgs/msg/PoseStamped
MoveIt move_group action/services
TF and robot_state required by MoveIt
```

Default parameters:

```text
input_topic: /contact_box_pose_debug
planning_group: fairino5_v6_group
target_link: wrist3_link
planning_mode: pose_target
execute_motion: false
approach_offset_x: 0.0
approach_offset_y: 0.0
approach_offset_z: 0.0
use_tool_offset: true
tool_offset_x: 0.0
tool_offset_y: 0.0
tool_offset_z: 0.263
use_fixed_orientation: false
target_qx: 0.0
target_qy: 0.0
target_qz: 0.0
target_qw: 1.0
plan_once: true
use_tool_standoff: true
standoff_distance: 0.05
allow_standoff_execution: false
velocity_scaling: 0.001
acceleration_scaling: 0.001
max_wrist_target_distance: 0.05
cartesian_eef_step: 0.001
cartesian_jump_threshold: 0.0
cartesian_min_fraction: 0.99
max_joint_delta_rad: 0.08
lock_contact_pose: true
contact_pose_sample_count: 5
contact_pose_sample_timeout_sec: 5.0
contact_pose_max_spread_m: 0.015
locked_contact_pose_source: live_average
locked_contact_x: 0.0
locked_contact_y: 0.0
locked_contact_z: 0.0
exit_after_completion: true
```

Behavior:

- Receives a debug pose in `base_link`.
- In `planning_mode: cartesian` with `lock_contact_pose: true`, locks the contact target before planning and ignores later YOLO/contact-pose updates for that move.
- `locked_contact_pose_source: live_average` collects `contact_pose_sample_count` samples from `/contact_box_pose_debug`, averages x/y/z, computes the maximum Euclidean sample deviation, and refuses to plan if the spread is greater than `contact_pose_max_spread_m` or samples do not arrive before `contact_pose_sample_timeout_sec`.
- `locked_contact_pose_source: manual_params` uses `locked_contact_x/y/z` directly in the MoveIt planning frame and still applies the same motion safety checks.
- Applies `approach_offset_*` directly in the input pose frame to form a desired temporary suction contact pose.
- If `use_tool_offset` is true, interprets `tool_offset_*` as a temporary `target_link -> suction contact point` vector in the target link frame.
- Computes the `target_link` pose by subtracting the rotated tool offset from the desired suction contact pose.
- Before planning, reads the latest available TF transform from the MoveIt planning frame to `target_link` and compares that current position to the planned `target_link` target.
- If the TF lookup fails, if the target pose frame differs from the MoveIt planning frame, or if the target is farther than `max_wrist_target_distance`, it refuses to plan or execute.
- Uses cached `/joint_states` values, mapped by joint name rather than array order, to seed the MoveIt planning start state.
- `planning_mode: pose_target` keeps the original coarse-positioning path: it calls `setPoseTarget(target_pose, target_link)` and then `plan()`.
- `planning_mode: cartesian` does not call `setPoseTarget`; it builds straight-line waypoints from the current `target_link` pose to `target_pose`, holds `target_pose` orientation on every waypoint, and calls `computeCartesianPath()`.
- Cartesian mode requires the returned fraction to be at least `cartesian_min_fraction`; otherwise it logs an error and refuses execution.
- Cartesian mode retimes the raw `computeCartesianPath()` trajectory with MoveIt's iterative parabolic time parameterization using `velocity_scaling` and `acceleration_scaling`. Retiming failure blocks execution.
- After any successful pose-target or Cartesian plan, it compares the first and final joint trajectory points, logs every joint delta, and blocks if any absolute delta is greater than `max_joint_delta_rad`.
- Calls MoveIt planning and logs whether planning succeeded.
- Logs the desired temporary suction contact pose and the planned `target_link` pose.
- Only calls `execute()` when both `execute_motion` and `allow_standoff_execution` are true.
- With `plan_once: true` and `exit_after_completion: true`, exits after one successful plan, successful execution, planning failure, or safety block.
- Never calls gripper or suction services.

Previously reported by user after real-robot validation:

- Previous reset pose: `testing_home_pose_v5_36cm_reset`.
- Current true 30 cm testing pose: `pose_records/testing_pose_v6_true_30cm/`.
- Validated locked contact pose: `(0.3323, -0.7376, 0.6131)` in `base_link`.
- Validated corrected suction contact offset: `tool_offset_z: 0.263` m along `wrist3_link` local +Z.
- Ruler checks matched at `20 cm`, `15 cm`, `10 cm`, `7.5 cm`, `5 cm`, and `2.5 cm` after replacing the old temporary `0.180 m` offset.
- Physical contact occurred at `standoff_distance: 0.002`; do not use `standoff_distance: 0.000` as the normal contact target.
- Validated 5 cm `wrist3_link` pose approximately: position `(0.110, -0.517, 0.607)`, quaternion xyzw `(0.500, 0.490, 0.011, 0.714)`.

The source and launch defaults now use `tool_offset_z: 0.263`. The final production model should still add the measured suction contact point to the active URDF as `suction_tcp`.

Safe plan-only command, only after the MoveIt planning stack is intentionally running:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_move contact_box_plan_debug.launch.py
```

Example with a temporary vertical standoff:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py approach_offset_z:=0.05
```

Example with the temporary old-URDF suction estimate enabled:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py \
	  use_tool_offset:=true \
	  tool_offset_x:=0.0 \
	  tool_offset_y:=0.0 \
	  tool_offset_z:=0.263
```

Example coarse far-standoff plan:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py \
  planning_mode:=pose_target \
  standoff_distance:=0.40
```

Example close Cartesian approach from an already reached standoff pose toward the contact box using the validated default contact target:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py \
  planning_mode:=cartesian \
  standoff_distance:=0.002 \
  cartesian_min_fraction:=0.99
```

Example close Cartesian approach with a manually supplied locked contact point:

```bash
ros2 launch moveit_move contact_box_plan_debug.launch.py \
  planning_mode:=cartesian \
  locked_contact_pose_source:=manual_params \
  locked_contact_x:=0.3323 \
  locked_contact_y:=-0.7376 \
  locked_contact_z:=0.6131
```

## Validated Helper Scripts

The following scripts wrap the validated locked-contact Cartesian approach. They are plan-only unless `--execute` is explicitly supplied.

The contact-box shell wrappers load the shared config file before passing MoveIt launch parameters:

```text
config/contact_box_motion.env
scripts/load_contact_box_motion_config.sh
```

Reset speed uses `CONTACT_BOX_RESET_VELOCITY_SCALING` and `CONTACT_BOX_RESET_ACCELERATION_SCALING`. Press speed uses `CONTACT_BOX_PRESS_VELOCITY_SCALING` and `CONTACT_BOX_PRESS_ACCELERATION_SCALING`. `CONTACT_BOX_CARTESIAN_*`, `CONTACT_BOX_MAX_*`, and pose-acquisition variables are validated by the loader before launch. The direct and staged approach scripts preserve their existing conservative profiles while the approach/safety values remain at template defaults; editing the config intentionally overrides those launch parameters. `--press-depth-m` overrides the configured default press depth for that command, and `--fast` / `--fast-demo` remain explicit command-line speed modes.

Plan-only examples:

```bash
./scripts/approach_30cm.sh
./scripts/approach_25cm.sh
./scripts/approach_20cm.sh
./scripts/approach_15cm.sh
./scripts/approach_10cm.sh
./scripts/approach_7_5cm.sh
./scripts/approach_5cm.sh
./scripts/reset_to_30cm.sh
./scripts/reset_to_36cm.sh
./scripts/reset_to_contact_box_staging_pose.sh
./scripts/run_contact_box_development_sequence.sh
```

Generic standoff form:

```bash
./scripts/approach_contact_box.sh 0.050
```

Execution examples, only for supervised robot testing:

```bash
./scripts/approach_5cm.sh --execute
./scripts/reset_to_30cm.sh --execute
./scripts/reset_to_36cm.sh --execute
./scripts/reset_to_contact_box_staging_pose.sh --execute
./scripts/run_contact_box_development_sequence.sh --execute
```

`scripts/reset_to_contact_box_staging_pose.sh` is the preferred name for the current development staging/recovery pose. It delegates to the old `scripts/reset_to_36cm.sh`; the old name is misleading after correcting `tool_offset_z` from `0.180` to `0.263`.

`scripts/reset_to_30cm.sh` is the current true 30 cm contact-box ready pose. This is the validated start pose for the direct one-shot approach and is also development/testing only.

The approach scripts pass:

```text
planning_mode: cartesian
lock_contact_pose: true
locked_contact_pose_source: manual_params
locked_contact_x/y/z: 0.3323, -0.7376, 0.6131
tool_offset_z: 0.263
target_qx/qy/qz/qw: 0.500, 0.490, 0.011, 0.714
cartesian_eef_step: 0.001
cartesian_jump_threshold: 0.0
cartesian_min_fraction: 0.99
```

For `30 cm`, `25 cm`, `20 cm`, `15 cm`, and `10 cm`, the template-preserved profile uses `max_wrist_target_distance: 0.07`, `max_joint_delta_rad: 0.12`, `velocity_scaling: 0.005`, and `acceleration_scaling: 0.005`.

For `7.5 cm` and `5 cm`, the template-preserved profile uses `max_wrist_target_distance: 0.04`, `max_joint_delta_rad: 0.12`, `velocity_scaling: 0.003`, and `acceleration_scaling: 0.003`.

## Staged Validation Demo

The staged standup/demo workflow is the safer engineering validation path:

```bash
./scripts/verify_keypoint_and_touch_contact_box.sh
./scripts/verify_keypoint_and_touch_contact_box.sh --execute
./scripts/verify_keypoint_and_touch_contact_box.sh --execute --fast-demo
```

Optional reset wrapper:

```bash
./scripts/demo_contact_box_from_30cm.sh --reset-first
./scripts/demo_contact_box_from_30cm.sh --reset-first --execute
```

The demo script first removes stale `/tmp/contact_box_locked_pose.env`, acquires the live keypoint-derived `/contact_box_pose_debug` pose, validates sample stability, writes a fresh env file, logs live-vs-manual delta against `(0.3323, -0.7376, 0.6131)`, and then uses that locked pose for every Cartesian stage. YOLO/keypoint updates after acquisition are not used by the staged movement.

Staged targets:

```text
0.300, 0.250, 0.200, 0.150, 0.100, 0.075, 0.050, 0.025, 0.010, 0.005, 0.002 m
```

The final validated contact target is `standoff_distance: 0.002`. The script never commands suction or gripper hardware. In plan-only mode it validates the next forward stage and stops before motion; in normal execution mode it performs a separate plan-only check before executing each stage and re-checks standoff from TF after each execution.

For filming, `--execute --fast-demo` skips the separate plan-only launch per stage. Each stage still runs through `contact_box_plan_debug`, which plans, safety-checks, retimes, and executes the stage, then the wrapper re-checks standoff from TF before continuing.

## Direct Deployment-Style Test

The direct one-shot deployment-style test is:

```bash
./scripts/verify_keypoint_and_touch_contact_box_direct.sh
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --fast
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --press
./scripts/verify_keypoint_and_touch_contact_box_direct.sh --execute --press-depth-m 0.003
```

This script is intended to start from the true 30 cm testing pose, where the contact box should be visible. It removes stale `/tmp/contact_box_locked_pose.env`, acquires live YOLO/keypoint-derived `/contact_box_pose_debug` samples once, locks the averaged 3D contact point, logs live-vs-manual delta, checks current standoff and lateral error from TF, and then calls `contact_box_plan_debug` exactly once for a direct Cartesian approach to `standoff_distance: 0.002`.

The direct test does not use staged targets. It still uses `tool_offset_z: 0.263`, fixed suction orientation xyzw `(0.500, 0.490, 0.011, 0.714)`, locked manual contact pose parameters after acquisition, trajectory retiming, and the existing `contact_box_plan_debug` safety checks. The script does not command suction or gripper hardware.

With `--press` or `--press-depth-m`, the direct script performs the default approach first and then runs a separate slow Cartesian press target using the configured press scaling, Cartesian parameters, and press-specific wrist/joint limits. Press mode is opt-in and still does not command suction or gripper hardware.

Do not run `moveit_move`, `hello_moveit`, gripper nodes, suction nodes, or any `/object_pose` publisher during this debug workflow.

## Current Development Sequence

The current full development flow is:

```bash
./scripts/run_contact_box_development_sequence.sh
./scripts/run_contact_box_development_sequence.sh --execute
./scripts/run_contact_box_development_sequence.sh --execute --fast
./scripts/run_contact_box_development_sequence.sh --execute --press
./scripts/run_contact_box_development_sequence.sh --execute --press-depth-m 0.003
```

It runs:

```text
current pose, where camera may not see contact box
-> contact_box_staging_pose
-> true_30cm / contact_box_ready_pose
-> fresh contact-pose acquisition
-> direct one-shot keypoint-to-contact approach
```

The saved-pose steps do not require `/contact_box_pose_debug` samples. If fresh acquisition fails after `true_30cm` / `contact_box_ready_pose`, the workflow stops and refuses the final contact approach.

Older shorthand:

```text
contact_box_staging_pose
-> true_30cm / contact_box_ready_pose
-> direct one-shot keypoint-to-contact approach
```

The `--fast`, `--press`, and `--press-depth-m` flags are forwarded only to `scripts/verify_keypoint_and_touch_contact_box_direct.sh`. Staging and reset motions keep their normal reset parameters and never use press mode.

The current staging pose, true 30 cm pose, manual contact point, fixed suction orientation, press depth, and collision/clearance assumptions are development/testing values only. Real deployment on the test lift car top requires recalibration; see the developer guide for the exact list.

## Why `ros2 run` Fails

`contact_box_plan_debug` constructs a MoveIt `MoveGroupInterface`, which loads the robot model from node parameters such as:

```text
robot_description
robot_description_semantic
robot_description_kinematics
planning_pipelines
```

In ROS 2 these are per-node parameters. A standalone `ros2 run moveit_move contact_box_plan_debug` process does not inherit `robot_description_semantic` or other MoveIt parameters just because `/move_group` is already running. The expected symptom is:

```text
Could not find parameter robot_description_semantic
Unable to parse SRDF.
Unable to construct robot model.
```

Use the launch file instead. It mirrors the existing `fairino_mtc_demo mtc_fairino.launch.py` MoveIt config source:

```text
MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
URDF xacro: config/fairino5_v6_robot.urdf.xacro
controllers: config/moveit_controllers.yaml
planning pipelines: ompl, pilz_industrial_motion_planner
```

If duplicate-node warnings or stale graph state appears while testing, stop duplicate ROS sessions and clean the daemon:

```bash
./scripts/stop_ros.sh
```

## Tool Frame And Orientation Notes

Verified from active robot files:

- MoveIt group `fairino5_v6_group` is a chain from `base_link` to `wrist3_link`.
- The active `fairino5_v6.urdf` contains the wrist links and the calibrated RealSense camera chain.
- No active suction cup TCP frame or contact-point tool frame was found in the `fairino5_v6` URDF/SRDF.

Design implications:

- `wrist3_link` is the safest temporary target link because it is the MoveIt chain tip.
- `camera_link` is not a correct motion target for suction pickup; it is part of the wrist-mounted camera model.
- `tool_offset_*` is a temporary plan-only approximation for `wrist3_link -> suction contact point`; it is not a verified TCP.
- The final target should be a real suction tool/contact TCP frame once the physical offset is measured and added to the robot model.
- The offset from `wrist3_link` to the suction cup contact point is not currently verified.
- The debug pose orientation is not a validated grasp orientation. The suction cup orientation must be chosen so the cup normal faces the contact-box surface along a safe approach vector.

## Remaining Development Requirements

Previously reported by user: the current development setup has validated the temporary `wrist3_link` suction contact offset `tool_offset_z:=0.263` and direct contact at `standoff_distance:=0.002`.

Before deployment on the real test lift car top:

- Recalibrate `contact_box_staging_pose` and `true_30cm` / `contact_box_ready_pose`.
- Revalidate camera/contact-box setup and contact-box detection pose.
- Revalidate TCP/tool offset if suction hardware changes.
- Add or verify a real suction TCP frame in URDF/SRDF.
- Add surface normal/orientation estimation or another validated way to set suction orientation.
- Add suction activation, grip verification, and retract behavior only after the peripheral/axle-Lua suction non-motion fault is resolved or a simple DO/ToolDO path is physically validated.
- Add a contact box, lift car top, camera, and suction tooling collision scene as needed.
