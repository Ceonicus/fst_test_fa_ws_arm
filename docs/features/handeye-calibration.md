# Hand-Eye Calibration

Purpose: document the robot-camera-AprilTag consistency checks and the active camera transform.

## Frames And Tag

Verified from launch/config files and latest capture metadata:

- Robot base frame: `base_link`
- Mount link: `wrist2_link`
- Control link: `wrist3_link`
- Camera frame: `camera_color_optical_frame`
- AprilTag frame: `tag36h11_0`
- AprilTag config: `camera_apriltag_test/tags_36h11_test.yaml`
- AprilTag family: `36h11`
- Tag id: `0`
- Tag size: `0.080 m`

## Camera Transform

Verified from latest capture metadata, transform `wrist2_link -> camera_color_optical_frame`:

```text
translation:
  x: -0.035352633085738314
  y:  0.0978096074408002
  z:  0.21313794136825306
quaternion xyzw:
  x: -0.7155925088883929
  y:  0.009538534188972466
  z: -0.00026898389092081833
  w:  0.6984527938493822
```

The prompt provided near-identical values with slightly different rounding. Treat the values above as verified from files during this pass.

## Active URDF Camera Chain

Verified from `src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino_description/urdf/fairino5_v6.urdf`:

```text
wrist2_link
  -> camera_base
  -> camera_bottom_screw_frame
  -> camera_link
  -> camera_depth_frame
  -> camera_color_frame
  -> camera_color_optical_frame
```

The calibrated base joint block is:

```text
parent: wrist2_link
child: camera_base
origin xyz: 0.098409181 0.096648299 0.116823635
origin rpy: 0.012943658 0.024329645 1.557082441
```

Do not change this URDF block unless recalibrating or physically changing the camera mount.

## Validation Method

The verifier composes:

```text
base_T_tag = base_T_camera x camera_T_tag
```

Implementation location: `scripts/verify_latest_handeye.sh`.

Important quaternion rule:

- Normalize only actual rotation quaternions.
- Do not normalize vector quaternions during `q * [v, 0] * q^-1`.

## Latest Verified Result

Verified by running `./scripts/verify_latest_handeye.sh`, which is analysis-only and does not move the robot:

```text
run_dir: camera_pose_captures/run_20260629_160327
samples: 13
fresh detections all valid: True
max camera movement: 15.378 mm
mean base_T_tag position error: 0.080 mm
max base_T_tag position error: 0.208 mm
mean rotation error: 0.1441 deg
max rotation error: 0.2860 deg
result: PASS
```

Previously reported by user for an older run:

```text
run_dir: camera_pose_captures/run_20260629_145530
samples: 13
result: PASS
```

## Scope Of Confidence

This verifies internal consistency among robot TF, camera TF, and AprilTag TF. It does not prove external ruler/metrology accuracy.
