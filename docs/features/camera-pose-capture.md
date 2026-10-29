# Camera Pose Capture

Package: `camera_pose_capture`

Purpose: move the camera through small optical-frame offsets, save RGB/depth/metadata, and collect TF snapshots for hand-eye validation.

## Safety

`./scripts/capture.sh` is robot-moving. Do not run it unless the user explicitly asks.

## Verified Implementation

Launch file:

- `src/camera_pose_capture/launch/camera_pose_capture.launch.py`

Node:

- `src/camera_pose_capture/camera_pose_capture/capture_node.py`

Output path:

- `camera_pose_captures/run_*`

Verified pose sequence from `build_pose_sequence()`:

```text
center_start
camera_right
center
camera_left
center
camera_up
center
camera_down
center
camera_forward
center
camera_back
center
```

Verified camera optical-frame offset directions:

```text
+X camera_right
-X camera_left
-Y camera_up
+Y camera_down
+Z camera_forward
-Z camera_back
```

## Current Script Launch Parameters

Verified from `scripts/capture.sh`:

```text
execute:=true
use_realsense:=false
group_name:=fairino5_v6_group
mount_link:=wrist2_link
control_link:=wrist3_link
lock_joints:=[none]
apriltag_frame:=tag36h11_0
apriltag_max_age_sec:=0.5
apriltag_wait_timeout_sec:=3.0
step_mm: ${STEP_MM}
settle_seconds:=2.0
```

`STEP_MM` defaults to `10` in `scripts/capture.sh`.

The launch file default `step_mm` is `5.0`, but the helper script overrides it.

## Metadata

Each capture writes `*_metadata.json` with:

- pose name/index
- selected frames and links
- camera topics
- AprilTag freshness status
- `camera_to_tag`
- TF snapshots, including `base_to_camera` and `mount_to_camera`
- output filenames

Latest verified metadata run:

- `camera_pose_captures/run_20260629_160327`
- `execute: true`
- `step_mm: 10.0`
- `samples: 13`
