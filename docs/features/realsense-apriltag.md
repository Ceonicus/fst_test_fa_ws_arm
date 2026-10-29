# RealSense And AprilTag

Purpose: document the camera and AprilTag pieces used by the capture and validation workflow.

## RealSense

Launch script:

```bash
./scripts/launch_realsense.sh
```

Verified command:

```bash
ros2 launch realsense2_camera rs_launch.py \
  publish_tf:=false \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

`publish_tf:=false` is intentional. Camera TF is supplied by the robot URDF camera chain, not by the RealSense driver.

Verified topic assumptions from launch files and metadata:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
```

Package prefix verified:

```text
realsense2_camera: /home/jetson-nano/Desktop/fst_test_fa_ws_arm/install/realsense2_camera
```

## AprilTag

Launch script:

```bash
./scripts/launch_apriltag.sh
```

Verified command:

```bash
ros2 run apriltag_ros apriltag_node \
  --ros-args \
  --params-file camera_apriltag_test/tags_36h11_test.yaml \
  -r /image_rect:=/camera/camera/color/image_raw \
  -r /camera_info:=/camera/camera/color/camera_info
```

Config file:

- `camera_apriltag_test/tags_36h11_test.yaml`

Verified config values:

```text
family: 36h11
size: 0.080
tag ids: [0]
tag frames: [tag36h11_0]
tag sizes: [0.080]
pose_estimation_method: pnp
```

Package prefix verified:

```text
apriltag_ros: /opt/ros/humble
```

## Viewer

Viewer script:

```bash
./scripts/view_camera.sh
```

Underlying Python file:

```text
camera_apriltag_test/scripts/view_camera_for_samples.py
```

Runtime camera and detection behavior was previously reported by user and was not revalidated by launching hardware during this documentation pass.
