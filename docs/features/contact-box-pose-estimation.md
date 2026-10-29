# Contact Box Pose Estimation

Purpose: document the safe debug-only contact-box pose workflow in `pose_estimater`.

## Status

Implemented debug-only node:

```bash
ros2 run pose_estimater contact_box_pose_debug
```

This node estimates a position-only debug pose and publishes:

```text
/contact_box_pose_debug
```

It does not publish `/object_pose`, does not call MoveIt, and does not command gripper or suction hardware.

## Old Workflow Summary

Verified from source inspection:

- `pose_estimater.object_detect` runs YOLO on `/camera/camera/color/image_raw`.
- It filters the exact class label `Interlock w/ Cover`.
- It publishes an annotated image on `/yolo/detections`.
- It publishes the detected box center on `/pixel_coordinates`.
- `object_pose_estimate` and `object_pose_estimate2` estimate a pose from point cloud data and publish `/object_pose`.
- `moveit_move` subscribes `/object_pose`, plans/executes motion, and sends a gripper service command.

## Contact-Box Detector

Detector command:

```bash
ros2 run pose_estimater object_detect
```

Safety:

- Vision-only.
- Does not publish `/object_pose`.
- Does not call MoveIt.
- Does not command gripper or suction hardware.

Verified implementation file:

```text
src/pose_estimater/pose_estimater/object_detect.py
```

Default input:

```text
/camera/camera/color/image_raw
```

Default outputs:

```text
/pixel_coordinates     geometry_msgs/msg/Point
/yolo/detections       sensor_msgs/msg/Image
```

Important: `/yolo/detections` is an annotated image, not a `Detection2DArray`.

Default model path:

```text
~/Desktop/fst_test_fa_ws_arm/yolo_custom.pt
```

The detector expands the path and fails at startup if the model file is missing or cannot be loaded.

Default target logic:

```text
target_class: Interlock w/ Cover
confidence_threshold: 0.5
```

The class comparison normalizes case and whitespace. The detector can also accept additional comma-separated class names or a class id override. If several target boxes are present, it publishes only the highest-confidence target center as integer pixel coordinates in the `x` and `y` fields of `/pixel_coordinates`; `z` remains `0.0`.

Detector parameters:

```text
image_topic: /camera/camera/color/image_raw
pixel_topic: /pixel_coordinates
annotated_topic: /yolo/detections
model_path: ~/Desktop/fst_test_fa_ws_arm/yolo_custom.pt
target_class: Interlock w/ Cover
accepted_class_names: ""
target_class_id: -1
confidence_threshold: 0.5
publish_annotated: true
log_all_detections: true
rejected_log_throttle_sec: 2.0
display_window: false
window_name: Contact Box Detector
display_scale: 1.0
```

`output_topic` is still accepted as a legacy alias for the annotated image topic, but new usage should prefer `annotated_topic`.

Optional GUI display:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater object_detect --ros-args -p display_window:=true
```

The GUI window shows the RealSense RGB image with the accepted contact-box bounding box, class label, confidence score, and center pixel. If no accepted target is present, it overlays `No accepted target`. The GUI is disabled by default and should only be used from a Jetson desktop session or another environment with working display access.

Optional GUI name and scale:

```bash
ros2 run pose_estimater object_detect --ros-args \
  -p display_window:=true \
  -p window_name:="Contact Box Detector" \
  -p display_scale:=0.75
```

Use `rqt_image_view` instead of `display_window` when running headless, over SSH without display forwarding, or when another ROS machine should monitor the annotated stream:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /yolo/detections
```

Warning: OpenCV GUI windows require desktop/display access on the Jetson. If `display_window:=true` is used without a display, the detector logs a warning, disables the GUI path, and continues publishing `/yolo/detections`.

Known detector limitations:

- The YOLO model quality has not been revalidated during this documentation pass.
- The detector still depends on the model's trained label names matching the configured target.
- `/pixel_coordinates` carries only the center point, not bbox size or confidence.
- The detector intentionally publishes nothing when the target is absent; downstream debug nodes should use fresh pixel messages only.
- This is not enough for robot motion. A stable `/contact_box_pose_debug` pose and a verified tool/suction offset are still required first.

## Why The Old Workflow Is Unsafe Or Inaccurate

Verified from source inspection:

- The old pose nodes use `camera_depth_optical_frame`, while the validated calibration is for `camera_color_optical_frame`.
- They use point cloud topics instead of aligned depth plus color intrinsics.
- One path applies a hardcoded `y * 0.95` pixel adjustment.
- One path multiplies XYZ by `0.5`.
- They publish `/object_pose`, which can trigger robot motion if a motion node is active.
- The MoveIt consumer targets `camera_link`, not a validated suction/tool control link.
- No verified suction cup/tool offset compensation was found.

## New Safe Debug Workflow

New file:

```text
src/pose_estimater/pose_estimater/contact_box_pose_debug.py
```

Entry point:

```text
contact_box_pose_debug = pose_estimater.contact_box_pose_debug:main
```

Inputs:

```text
/pixel_coordinates
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
TF: camera_color_optical_frame -> base_link
```

Output:

```text
/contact_box_pose_debug
```

The node computes a position-only debug pose:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median aligned depth in meters
```

Depth behavior:

- Supports `16UC1` depth in millimeters.
- Supports `32FC1` depth in meters.
- Rejects zero, NaN, inf, and depths outside configured bounds.
- Uses the median valid depth inside a configurable pixel window.
- Logs the raw center depth, median raw depth, depth in meters, valid pixel count, camera-frame XYZ, base-frame XYZ, TF status, and TF mode.

## Parameters

Defaults:

```text
pixel_topic: /pixel_coordinates
depth_topic: /camera/camera/aligned_depth_to_color/image_raw
camera_info_topic: /camera/camera/color/camera_info
camera_frame: camera_color_optical_frame
target_frame: base_link
window_px: 7
min_depth_m: 0.05
max_depth_m: 2.0
publish_topic: /contact_box_pose_debug
use_latest_tf: true
tf_timeout_sec: 0.2
```

TF timing behavior:

- `use_latest_tf: true` asks TF for the latest available `camera_color_optical_frame -> base_link` transform instead of requiring a transform at the exact depth image timestamp.
- Exact timestamp lookup can fail with `extrapolation into the future` when the depth frame timestamp is slightly newer than the latest transform in the TF buffer.
- Latest-TF mode is acceptable for stationary debug testing because the robot and wrist-mounted camera are expected to be still while checking contact-box depth and base-frame position.
- Set `use_latest_tf:=false` to restore message timestamp lookup when validating timestamp synchronization.
- `tf_timeout_sec` controls how long the lookup waits before rejecting the frame.

Warning: final robot-motion integration should revisit timestamp synchronization if the robot, camera, or target can move during detection. Latest-TF lookup trades timestamp exactness for robustness during stationary debug checks.

## Camera-Only Validation Commands

Terminal 1:

```bash
./scripts/launch_realsense.sh
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater object_detect
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater contact_box_pose_debug
```

Use exact message timestamps only when deliberately testing TF synchronization:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater contact_box_pose_debug --ros-args -p use_latest_tf:=false
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /pixel_coordinates --once
ros2 topic echo /contact_box_pose_debug --once
```

Do not run `moveit_move`, gripper nodes, suction nodes, or any `/object_pose` publisher during camera-only validation.

Detector-only checks:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater object_detect
ros2 topic echo /pixel_coordinates --once
ros2 topic hz /pixel_coordinates
ros2 topic list | grep -E "pixel|yolo|camera"
```

Use these only with RealSense intentionally running. They are vision-only and do not move the robot.

## Future Steps Before Robot Movement

Before any robot motion:

- Verify YOLO class labels and confidence on live images.
- Confirm `/contact_box_pose_debug` is stable while the robot is stationary.
- Compare debug pose against the visible contact-box location and expected range.
- Use the separate `contact_box_plan_debug` MoveIt validation path for plan-only checks.
- Define and validate the suction cup/tool offset from `wrist3_link`.
- Only publish motion targets after explicit user approval for robot movement.
