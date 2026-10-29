# Contact-Box Cover Removal

Purpose: remove the contact-box cover with the FAIRINO FR5-V6, EVS08 suction gripper, YOLO/depth contact pose, pickup-out motion, table-mount placement, suction release, and retreat.

Source of truth: [Contact-Box Cover-Removal Developer Guide](../contact-box-vision-to-motion-developer-guide.md).

## Main Scripts

- `scripts/run_contact_box_full_cover_removal_sequence.sh`
- `scripts/run_contact_box_development_sequence.sh`
- `scripts/run_contact_box_cover_pickout_sequence.sh`
- `scripts/run_contact_box_cover_to_mount_sequence.sh`
- `scripts/run_contact_box_cover_mount_retreat_sequence.sh`
- `scripts/suction_init.sh`
- `scripts/suction_on.sh`
- `scripts/suction_off.sh`

## Required Launch Terminals

Run these before the full wrapper:

```bash
./scripts/launch_robot.sh
./scripts/launch_realsense.sh
./scripts/launch_contact_box_perception.sh
```

The full wrapper does not launch robot, camera, or perception.

## Operator Commands

Plan-only:

```bash
./scripts/run_contact_box_full_cover_removal_sequence.sh
```

Execute:

```bash
./scripts/run_contact_box_full_cover_removal_sequence.sh --execute
```

## Config And Poses

Config: `config/contact_box_motion.env`.

Recorded poses: `config/recorded_robot_poses.json`.

Required pose names:

- `cover_contact_box_clear`
- `cover_mount_approach`
- `cover_mount_place`

## Calibration Notes

New sites must recalibrate contact target offsets, pickup-out lift/pull, and table-mount recorded poses. `cover_mount_approach` should be directly above `cover_mount_place` so placement and retreat remain Cartesian vertical moves.

## Safety Notes

Only suction A/top cups are used. Suction B is disabled. `suction_off.sh` uses EVS08 stop-all/break-vacuum and must not use `ActGripper(1,0)`.
