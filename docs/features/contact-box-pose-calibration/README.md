# Contact-Box Pose Calibration

Purpose: record and validate the saved poses used by the cover-removal workflow.

Source of truth: [Contact-Box Cover-Removal Developer Guide](../contact-box-vision-to-motion-developer-guide.md).

## Main Files And Tool

- Pose file: `config/recorded_robot_poses.json`
- Recorder tool: `scripts/pose_recorder_gui.py`

Command:

```bash
python3 scripts/pose_recorder_gui.py
```

The GUI requires `/joint_states` and TF `base_link -> wrist3_link`, so the robot/MoveIt stack must already be running.

## Required Recorded Poses

- `cover_contact_box_clear`
- `cover_mount_approach`
- `cover_mount_place`

## Calibration Notes

- Record `cover_mount_place` with the cover in its final supported table-mount position.
- Record `cover_mount_approach` by moving straight upward from `cover_mount_place`, keeping x/y and orientation nearly unchanged.
- `cover_mount_approach -> cover_mount_place` should be mostly vertical and Cartesian-plannable.
- If runtime mount offsets are baked into the JSON poses, set `CONTACT_BOX_COVER_MOUNT_OFFSET_*_M` to zero.

## Safety Notes

Do not silently edit pose JSON while a hardware test is in progress. Validate changes plan-only before using `--execute`.
