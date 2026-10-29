# Agent Guide

This repository is a ROS 2 Humble workspace for a Jetson Orin Nano controlling a FAIRINO FR5-V6 workflow with MoveIt2, Intel RealSense, AprilTag detection, camera pose capture, and hand-eye validation.

Future Codex/AI agents should treat this file and `docs/` as the first orientation layer. Read `AGENTS.md` and the relevant `docs/features/*.md` files before changing source code.

## Ground Rules

- Work from the repo root: `~/Desktop/fst_test_fa_ws_arm`.
- Do not run robot movement, capture, or trajectory execution commands unless the user explicitly asks.
- Do not run `./scripts/capture.sh` or `ros2 launch camera_pose_capture ...` unless explicitly instructed.
- Do not delete files or revert user changes unless explicitly instructed.
- Do not commit unless explicitly instructed.
- Never reintroduce legacy absolute workspace references.
- Use helper scripts instead of long manual ROS commands when possible.
- Prefer `./scripts/build_safe.sh` for Jetson builds.
- After path, build, install, or package-discovery changes, run `./scripts/check_system.sh`.

## Documentation Policy

- Every meaningful new feature or modified feature needs a matching Markdown file under `docs/features/`.
- Update an existing feature doc when changing that feature.
- Keep docs concise and practical for future agents.
- Separate verified facts from user-reported state. If a claim is not verified from files or safe commands, label it as `Previously reported by user`, `Expected working state`, or `Needs revalidation`.
- Do not paste huge logs or private chat transcripts into docs.

## Developer Documentation Rules for All Features

- For every feature implemented in this workspace, developer documentation must be treated as the source of truth.
- Any new feature must include developer documentation that explains:
  - current status
  - high-level architecture
  - relevant scripts/nodes/launch files
  - important parameters/constants
  - safety assumptions
  - usage commands
  - validation results
  - known limitations
  - future work
  - troubleshooting notes
- Any change to an existing feature must update the relevant developer documentation in the same change.
- Documentation must be detailed enough that another developer, Codex session, or ChatGPT chat can continue the work without relying on hidden conversation history.
- Do not leave major feature behavior documented only in chat logs, terminal history, or memory.
- If a feature has a dedicated developer guide under `docs/features/`, read it before modifying that feature.
- If no developer guide exists for a feature being modified, create one.
- Keep `docs/README.md` and any feature index updated when new developer documentation is added.
- Keep helper-script documentation consistent with actual script behavior.
- Plan-only must remain the default for robot-motion scripts unless there is a very explicit and documented reason otherwise.
- Robot motion must require explicit `--execute` or explicit `execute_motion:=true` plus the matching `allow_*` execution gate.
- Do not command suction hardware unless explicitly requested and the command is known/validated.
- Do not command gripper hardware unless explicitly requested and the command is known/validated.
- Do not run `capture.sh` unless explicitly requested.
- Be careful with `/joint_states` order. It may appear as `j1, j2, j4, j5, j3, j6`, but saved/reset scripts should map by joint name and use logical robot order `j1, j2, j3, j4, j5, j6`.
- If a feature requires multiple runtime nodes, provide or maintain startup/check scripts where practical.
- Runtime startup scripts must be documented in the feature developer guide.
- Startup scripts must clearly state what they launch and what they assume is already running.
- Startup scripts must not command robot motion unless their name and documentation clearly say they are motion scripts.
- Startup scripts for perception, camera, TF, or MoveIt should be separated from scripts that execute robot motion.
- If startup commands depend on package/node names, inspect the workspace and use the actual commands.
- Do not invent placeholder commands without marking them unresolved.
- Any change to runtime startup procedure must update the relevant developer documentation.
- If a feature has calibration values, documentation must clearly distinguish development/testing values from deployment values.
- If deployment requires recalibration, document exactly what must be recalibrated before changing or using the feature in the real environment.

## Contact Box Vision-to-Motion Feature Rules

- The source of truth for the contact-box vision-to-motion feature is `docs/features/contact-box-vision-to-motion-developer-guide.md`.
- Read that developer guide before modifying contact-box approach, reset, pose, perception, TCP, or MoveIt scripts.
- Any change to contact-box behavior, scripts, launch parameters, safety thresholds, poses, TCP/tool offset, perception assumptions, or workflow must update the developer guide in the same change.
- Keep `docs/features/contact-box-motion-planning.md` and `docs/features/helper-scripts.md` consistent with the developer guide.
- Contact-box reset/approach/press speed and related validated motion parameters are configured through `config/contact_box_motion.env` via `scripts/load_contact_box_motion_config.sh`; do not reintroduce per-script hardcoded tuning unless there is a documented safety reason.
- Saved-pose reset range uses `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD`; direct approach uses `CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD`; press uses `CONTACT_BOX_PRESS_MAX_JOINT_DELTA_RAD`. Do not remove these limits entirely, and keep press stricter than reset/approach.
- `scripts/run_contact_box_development_sequence.sh` must fail closed if staging or true 30 cm reset fails; do not continue to keypoint acquisition or direct contact approach after a safety-blocked reset.
- Do not use `standoff_distance:=0.000` as the normal contact target. The validated contact target is `standoff_distance:=0.002`.
- `standoff_distance:=0.000` or suction-cup press-in behavior must be explicit, slow, opt-in, and documented.
- The current `contact_box_staging_pose`, `true_30cm` / `contact_box_ready_pose`, fixed suction orientation, and manual contact point are development/testing values only. Real deployment requires recalibration on the actual test lift car top.
- Stale `/tmp/contact_box_locked_pose.env` must not be reused.
- The contact-box development sequence may start from a pose where the camera does not see the contact box.
- Do not require `/contact_box_pose_debug` or fresh keypoint acquisition before moving to `contact_box_staging_pose` or `true_30cm` / `contact_box_ready_pose`.
- Fresh stable contact-pose acquisition must be enforced only before the final contact approach, after the robot has reached `true_30cm` / `contact_box_ready_pose`.
- The final suction target is the fresh live contact pose plus `CONTACT_BOX_TARGET_OFFSET_X/Y/Z_M` from `config/contact_box_motion.env`; current default is `(0.000, 0.000, -0.030) m` in `base_link`. This offset is a development calibration value to avoid cover holes and must be visually verified/recalibrated before deployment.
- `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M` controls the direct approach starting lateral-error gate to the adjusted suction target. Current default is `0.050 m` because the `-0.030 m` target offset can intentionally produce about `0.029 m` initial lateral error from `true_30cm`; do not remove final contact validation.
- Current operator-facing suction workflow is `scripts/suction_init.sh`, `scripts/suction_on.sh`, and `scripts/suction_off.sh`. This project uses suction A/top two cups only; B/bottom cups and ALL_ON are intentionally disabled. Normal release must use EVS08 `MoveGripper(1,0,70,60,6000,1,1,0,70,60)` and must not use `ActGripper(1,0)`.
- Robot / MoveIt stack is launched manually by the operator for this feature.
- RealSense is launched manually by the operator for this feature.
- `scripts/launch_contact_box_perception.sh` should only launch the contact-box perception / pose-estimation layer.
- `scripts/check_contact_box_perception.sh` should verify runtime readiness by default without requiring the target to be visible. Use `--require-target-visible` only after the robot is at `true_30cm` / `contact_box_ready_pose`.

## Verification Policy

- Do not blindly trust earlier summaries, including this file.
- Verify from source files and safe local commands before documenting or changing behavior.
- Safe analysis commands include file reads, `git status`, `colcon list`, `ros2 pkg prefix ...`, `./scripts/check_system.sh`, and `./scripts/verify_latest_handeye.sh`.
- Do not launch hardware or movement workflows during verification unless the user explicitly asks.

## Validated Or Reported Frames

Verified from current files and latest capture metadata:

- Robot base frame: `base_link`
- MoveIt group: `fairino5_v6_group`
- Mount link: `wrist2_link`
- Control link: `wrist3_link`
- Camera frame: `camera_color_optical_frame`
- AprilTag frame: `tag36h11_0`
- AprilTag family/id/size: `36h11`, id `0`, size `0.080 m`

Verified from latest capture metadata, `wrist2_link -> camera_color_optical_frame`:

- Translation: `[-0.035352633085738314, 0.0978096074408002, 0.21313794136825306]`
- Quaternion xyzw: `[-0.7155925088883929, 0.009538534188972466, -0.00026898389092081833, 0.6984527938493822]`

Verified from active URDF camera patch:

- Parent: `wrist2_link`
- Child: `camera_base`
- Origin xyz: `0.098409181 0.096648299 0.116823635`
- Origin rpy: `0.012943658 0.024329645 1.557082441`

## Do Not Touch Unless Needed

The calibrated camera transform is embedded in:

- `src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino_description/urdf/fairino5_v6.urdf`
- `camera_base` fixed joint block under `BEGIN CALIBRATED WRIST2 REALSENSE CAMERA PATCH`

Only modify this block for an intentional recalibration or camera mount change. If changed, update `docs/features/handeye-calibration.md` and rerun safe validation as appropriate.

## Known Validation State

Verified by `./scripts/verify_latest_handeye.sh` during this documentation pass:

- Run: `camera_pose_captures/run_20260629_160327`
- Samples: `13`
- Fresh detections all valid: `True`
- Max camera movement: `15.378 mm`
- Mean base_T_tag position error: `0.080 mm`
- Max base_T_tag position error: `0.208 mm`
- Mean rotation error: `0.1441 deg`
- Max rotation error: `0.2860 deg`
- Result: `PASS`

Full build success and hardware workflow operation were previously reported by user, but were not rerun during this documentation pass.

## Common Commands

Safe:

```bash
./scripts/check_system.sh
./scripts/build_safe.sh
./scripts/verify_latest_handeye.sh
./scripts/stop_ros.sh
```

Normal workflow, with robot-moving steps marked:

```bash
./scripts/launch_robot.sh        # starts robot/MoveIt stack, no motion command by itself
./scripts/launch_realsense.sh    # starts camera
./scripts/launch_apriltag.sh     # starts AprilTag detector
./scripts/view_camera.sh         # starts viewer
./scripts/capture.sh 10          # ROBOT-MOVING
./scripts/verify_latest_handeye.sh
```

Feature docs:

- `docs/features/workspace-porting.md`
- `docs/features/handeye-calibration.md`
- `docs/features/camera-pose-capture.md`
- `docs/features/realsense-apriltag.md`
- `docs/features/robot-moveit-control.md`
- `docs/features/helper-scripts.md`
- `docs/features/fairino-hardware-fixes.md`
- `docs/features/build-and-validation-checklist.md`
- `docs/features/contact-box-vision-to-motion-developer-guide.md`
