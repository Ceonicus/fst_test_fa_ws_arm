# test_fa_ws

ROS 2 workspace for FAIRINO robot experiments that combine robot description and MoveIt 2 configuration, RealSense camera input, YOLO perception, hand-eye calibration, gripper control, and MoveIt Task Constructor examples.

This documentation is a starting point. It captures the current workspace layout and the commands that are visible from the repository. Hardware details such as robot IP addresses, camera serial numbers, calibration names, and preferred demo launch order should be filled in as the setup is validated.

## Documentation

- [test_fa_ws workspace guide](docs/test_fa_ws.md)
- [ros2_pose_estimate_ws workspace guide](docs/ros2_pose_estimate_ws.md)
- [Robotics documentation completion draft](docs/robotics_documentation_completion_draft.md)
- [Google Docs-ready HTML export](docs/robotics_documentation_google_docs.html)
- [Google Docs-ready DOCX export](docs/robotics_documentation_google_docs.docx)
- [Documentation roadmap](docs/documentation_roadmap.md)

## Quick Start

Open a terminal in this workspace:

```bash
cd /home/fst/test_fa_ws
source /opt/ros/<ros-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros-distro>` with the ROS 2 distribution installed on the robot workstation, for example `humble` or `jazzy`.

## Package Overview

Main local integration packages:

- `moveit_move`: C++ MoveIt motion examples.
- `rg75_300_gripper`: ROS 2 interface for the FAIRINO RG75-300 gripper.
- `ev_08_suction_gripper`: ROS 2 interface for the FAIRINO EV-08 suction gripper.
- `yolo_pose_estimator`: Python node for YOLO-based pose estimation.
- `pose_estimater`: Python object detection and pose estimation experiments.

Vendored or external packages included in `src/`:

- `frcobot_ros2-V3.0.0_robotV3.8.0`: FAIRINO robot descriptions, hardware interface, MoveIt configs, messages, and demos.
- `realsense-ros`: Intel RealSense ROS 2 camera driver and descriptions.
- `yolo_ros`: YOLO ROS 2 packages and messages.
- `easy_handeye2`: hand-eye calibration tooling.
- `moveit_task_constructor`: MoveIt Task Constructor core, demos, visualization, messages, and RViz marker tools.

## Useful Commands

List packages:

```bash
colcon list
```

Build selected local packages while iterating:

```bash
colcon build --symlink-install --packages-select moveit_move rg75_300_gripper ev_08_suction_gripper yolo_pose_estimator pose_estimater
source install/setup.bash
```

Run common entry points after building:

```bash
ros2 run moveit_move hello_moveit
ros2 run moveit_move moveit_move
ros2 run rg75_300_gripper gripper_node
ros2 run ev_08_suction_gripper suction_node
ros2 run yolo_pose_estimator pose_estimator
ros2 run pose_estimater object_detect
ros2 run pose_estimater object_pose_estimate
ros2 run pose_estimater object_pose_estimate2
```

Launch common subsystems:

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch yolo_bringup yolo.launch.py
ros2 launch fairino5_v6_moveit2_config demo.launch.py
ros2 launch fairino5_v6_moveit2_config handeye_calibrate.launch.py
```

## Notes

- Keep generated build artifacts (`build/`, `install/`, `log/`) out of documentation changes.
- Prefer documenting tested launch sequences with the exact terminal order, environment variables, and expected topics.
- Add screenshots or RViz views only after the launch path has been verified on the target machine.
