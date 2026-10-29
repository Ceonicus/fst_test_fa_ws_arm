# test_fa_ws Workspace Guide

## Purpose

`test_fa_ws` is the main integration workspace for a FAIRINO ROS 2 setup. It brings together robot motion, MoveIt 2 planning, gripper control, RealSense camera input, YOLO perception, object pose estimation, and hand-eye calibration.

The workspace currently contains both local experiment packages and large third-party/vendor packages. Treat local packages as the primary place for project-specific documentation and code notes.

## Workspace Layout

```text
/home/fst/test_fa_ws
|-- src/
|   |-- moveit_move/
|   |-- rg75_300_gripper/
|   |-- ev_08_suction_gripper/
|   |-- yolo_pose_estimator/
|   |-- pose_estimater/
|   |-- frcobot_ros2-V3.0.0_robotV3.8.0/
|   |-- realsense-ros/
|   |-- yolo_ros/
|   |-- easy_handeye2/
|   `-- moveit_task_constructor/
`-- working_yolo_custom/
```

## Package Inventory

Packages detected with `colcon list`:

- `moveit_move`: C++ MoveIt planning examples. Installs `moveit_move` and `hello_moveit`.
- `rg75_300_gripper`: C++ gripper node. Installs `gripper_node`.
- `ev_08_suction_gripper`: C++ suction gripper node. Installs `suction_node`.
- `yolo_pose_estimator`: Python pose-estimation package. Installs `pose_estimator`.
- `pose_estimater`: Python object detection and pose-estimation package. Installs `object_detect`, `object_pose_estimate`, and `object_pose_estimate2`.
- `fairino_msgs`, `fairino_hardware`, `fairino_description`, `fairino_mtc_demo`: FAIRINO robot support packages.
- `fairino*_v6_moveit2_config`, `robot5_v6_moveit2_config`: generated MoveIt 2 configs for different FAIRINO robot variants.
- `realsense2_camera`, `realsense2_camera_msgs`, `realsense2_description`: Intel RealSense camera packages.
- `yolo_ros`, `yolo_msgs`, `yolo_bringup`: YOLO ROS 2 packages.
- `easy_handeye2`, `easy_handeye2_msgs`: hand-eye calibration packages.
- `moveit_task_constructor_*`, `rviz_marker_tools`: MoveIt Task Constructor packages.

## Build

From the workspace root:

```bash
cd /home/fst/test_fa_ws
source /opt/ros/<ros-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For faster iteration on local project packages:

```bash
colcon build --symlink-install --packages-select moveit_move rg75_300_gripper ev_08_suction_gripper yolo_pose_estimator pose_estimater
source install/setup.bash
```

## Common Launches and Runs

RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py
```

YOLO bringup:

```bash
ros2 launch yolo_bringup yolo.launch.py
```

Other YOLO launch variants are available for `yolov5`, `yolov8`, `yolov9`, `yolov10`, `yolov11`, `yolov12`, and `yolo-world`.

FAIRINO 5 MoveIt 2 demo:

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

FAIRINO 5 hand-eye calibration launch:

```bash
ros2 launch fairino5_v6_moveit2_config handeye_calibrate.launch.py
```

MoveIt C++ examples:

```bash
ros2 run moveit_move hello_moveit
ros2 run moveit_move moveit_move
```

Gripper and suction nodes:

```bash
ros2 run rg75_300_gripper gripper_node
ros2 run ev_08_suction_gripper suction_node
```

Pose estimation nodes:

```bash
ros2 run yolo_pose_estimator pose_estimator
ros2 run pose_estimater object_detect
ros2 run pose_estimater object_pose_estimate
ros2 run pose_estimater object_pose_estimate2
```

## Suggested Bringup Notes to Capture Next

Document the real launch order once it has been tested on hardware. A useful format is:

```text
Terminal 1: source ROS and workspace, start robot driver or MoveIt.
Terminal 2: start RealSense camera.
Terminal 3: start YOLO detector.
Terminal 4: start pose-estimation node.
Terminal 5: start motion or pick-place task.
```

For each terminal, record:

- exact command
- required environment variables
- robot IP address or controller mode, if any
- camera serial number or stream settings, if any
- important topics and frames
- expected RViz result
- common failure symptoms and fixes

## Open Questions

- Which ROS 2 distribution is the intended target?
- Which FAIRINO arm model is the primary robot for this workspace?
- Which gripper is normally mounted: RG75-300, EV-08 suction, or both?
- Which RealSense model and serial number are used?
- What TF frame names are considered stable for camera, robot base, end effector, and detected object?
- Which launch file is the preferred production/demo entry point?
