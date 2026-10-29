# Documentation Index

This folder is the orientation layer for future Codex/AI agents working in this ROS 2 workspace.

Read `../AGENTS.md` first, then the relevant feature docs:

- [Workspace Porting](features/workspace-porting.md)
- [Hand-Eye Calibration](features/handeye-calibration.md)
- [Camera Pose Capture](features/camera-pose-capture.md)
- [RealSense And AprilTag](features/realsense-apriltag.md)
- [Robot MoveIt Control](features/robot-moveit-control.md)
- [Helper Scripts](features/helper-scripts.md)
- [FAIRINO Hardware Fixes](features/fairino-hardware-fixes.md)
- [EVS08 Suction Peripheral](features/evs08-suction-peripheral.md)
- [Build And Validation Checklist](features/build-and-validation-checklist.md)
- [Contact Box Pose Estimation](features/contact-box-pose-estimation.md)
- [Contact Box Motion Planning](features/contact-box-motion-planning.md)
- [Contact-Box Cover-Removal Developer Guide](features/contact-box-vision-to-motion-developer-guide.md) - current source of truth for full contact-box cover removal, required terminals, script sequence, config, calibration, validation, and failure handling.
- [Contact-Box Cover Removal README](features/contact-box-cover-removal/README.md)
- [Contact-Box Perception README](features/contact-box-perception/README.md)
- [Contact-Box Pose Calibration README](features/contact-box-pose-calibration/README.md)
- [Suction Control README](features/suction-control/README.md)
- [Table-Mount Placement README](features/table-mount-placement/README.md)
- [Saved Joint Reset](features/saved-joint-reset.md)

Documentation rules:

- Add or update `docs/features/<feature-name>.md` for every meaningful feature change.
- If a feature has a dedicated developer guide, treat that guide as the source of truth and update it in the same change.
- Mark unverified claims as `Previously reported by user`, `Expected working state`, or `Needs revalidation`.
- Do not run robot movement or capture commands just to update docs.

Contact-box notes:

- The full operator-facing cover-removal command is `./scripts/run_contact_box_full_cover_removal_sequence.sh --execute`.
- The full wrapper does not launch runtime infrastructure. Before running the one-shot/contact approach or full wrapper, keep these running in separate terminals: `./scripts/launch_robot.sh`, `./scripts/launch_realsense.sh`, and `./scripts/launch_contact_box_perception.sh`.
- Contact-box motion tuning lives in `config/contact_box_motion.env` and is loaded by `scripts/load_contact_box_motion_config.sh`. Use it for reset/approach/press speed and related validated motion parameters instead of editing each script. Saved-pose reset range is controlled by `CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD`; direct approach and press use separate joint-delta limits.
- `scripts/reset_to_36cm.sh` is now treated as the development `contact_box_staging_pose`; the old name is misleading after correcting `tool_offset_z` to `0.263`.
- `scripts/reset_to_30cm.sh` is the current development `contact_box_ready_pose`.
- `scripts/verify_keypoint_and_touch_contact_box_direct.sh` is the validated direct one-shot keypoint-to-contact test.
- The full development sequence may start from a pose where the camera does not see the contact box. It moves by saved joint poses to staging and true 30 cm first.
- `scripts/run_contact_box_development_sequence.sh` must stop before keypoint acquisition or final approach if staging or true 30 cm reset fails or is safety-blocked.
- Fresh contact-pose acquisition is mandatory only before the final contact approach from true 30 cm; stale `/tmp/contact_box_locked_pose.env` must not be reused.
- The final suction target is the fresh live contact pose plus `CONTACT_BOX_TARGET_OFFSET_X/Y/Z_M`. Current local config is `(0.010, 0.000, -0.030) m` in `base_link` to move the top two suction cups away from small cover holes and toward solid cover material.
- `CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M` controls the direct approach starting lateral-error gate to the adjusted suction target. The target offset can intentionally create initial lateral error from the true 30 cm ready pose; final contact validation is still reported against the adjusted target.
- `scripts/check_contact_box_perception.sh` defaults to runtime readiness. Use `--require-target-visible` only from true 30 cm / contact-box ready pose.
- Default contact target remains `standoff_distance:=0.002`; press-in or `standoff_distance:=0.000` behavior must be explicit, slow, and documented.
- Peripheral/axle-Lua suction remains manual-only for this workflow. Operator-facing suction commands are `./scripts/suction_init.sh --execute`, `./scripts/suction_on.sh --execute`, and `./scripts/suction_off.sh --execute`. The current EVS08 ROS path uses suction A/top cups only, forces B/bottom cups OFF, releases with full-state `MoveGripper(1,0,70,60,6000,1,1,0,70,60)`, and avoids `ActGripper(1,0)` for normal OFF.
- Table-mount placement uses `config/recorded_robot_poses.json` poses `cover_mount_approach` and `cover_mount_place`. Runtime `CONTACT_BOX_COVER_MOUNT_OFFSET_*_M` values are applied to both poses; keep them zero if the correction is already baked into the JSON.
- Current contact-box poses, manual contact point, fixed orientation, and press-depth assumptions are development/testing values only; deployment on the real test lift car top requires recalibration as listed in the developer guide.
