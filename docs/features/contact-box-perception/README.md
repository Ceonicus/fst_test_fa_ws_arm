# Contact-Box Perception

Purpose: provide the live YOLO/depth contact-box pose required by the contact approach.

Source of truth: [Contact-Box Cover-Removal Developer Guide](../contact-box-vision-to-motion-developer-guide.md). See also [Contact Box Pose Estimation](../contact-box-pose-estimation.md).

## Required Launch Terminals

RealSense:

```bash
./scripts/launch_realsense.sh
```

Contact-box perception:

```bash
./scripts/launch_contact_box_perception.sh
```

The robot/MoveIt stack must also be running so TF is available.

## Main Scripts

- `scripts/launch_realsense.sh`
- `scripts/launch_contact_box_perception.sh`
- `scripts/check_contact_box_perception.sh`
- `scripts/verify_keypoint_and_touch_contact_box_direct.sh`

## Required Config Values

- `CONTACT_BOX_POSE_SAMPLE_COUNT`
- `CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC`
- `CONTACT_BOX_POSE_MAX_SPREAD_M`
- `CONTACT_BOX_TARGET_OFFSET_X_M`
- `CONTACT_BOX_TARGET_OFFSET_Y_M`
- `CONTACT_BOX_TARGET_OFFSET_Z_M`

## Safety Notes

The final contact approach requires fresh pose acquisition. Stale `/tmp/contact_box_locked_pose.env` must not be reused. If perception is not running or the contact box is not visible from the ready pose, acquisition should fail closed.
