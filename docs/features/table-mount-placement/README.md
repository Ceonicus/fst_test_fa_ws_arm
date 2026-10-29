# Table-Mount Placement

Purpose: move the picked contact-box cover to the table mount, release suction, and retreat.

Source of truth: [Contact-Box Cover-Removal Developer Guide](../contact-box-vision-to-motion-developer-guide.md).

## Main Scripts

- `scripts/run_contact_box_cover_to_mount_sequence.sh`
- `scripts/suction_off.sh`
- `scripts/run_contact_box_cover_mount_retreat_sequence.sh`

## Required Recorded Poses

- `cover_mount_approach`
- `cover_mount_place`

## Required Config Values

- `CONTACT_BOX_COVER_MOUNT_OFFSET_X_M`
- `CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M`
- `CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M`
- `CONTACT_BOX_CARTESIAN_MIN_FRACTION`
- `CONTACT_BOX_APPROACH_VELOCITY_SCALING`
- `CONTACT_BOX_APPROACH_ACCELERATION_SCALING`

## Operator Flow

The full wrapper runs this automatically:

```bash
./scripts/run_contact_box_cover_to_mount_sequence.sh --execute
./scripts/suction_off.sh --execute
./scripts/run_contact_box_cover_mount_retreat_sequence.sh --execute
```

## Calibration Notes

Apply the same runtime mount offset to both approach and place, or bake the offset into `recorded_robot_poses.json` and set runtime offsets to zero. Do not offset only `cover_mount_place`; that breaks the intended vertical placement geometry.

## Safety Notes

The approach-to-place and place-to-retreat legs are Cartesian-only. Do not replace them with arbitrary joint-space movement.
