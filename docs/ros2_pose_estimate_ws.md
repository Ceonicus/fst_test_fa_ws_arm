# ros2_pose_estimate_ws Workspace Guide

## Purpose

`ros2_pose_estimate_ws` is a smaller ROS 2 workspace focused on FAIRINO 5 robot description, MoveIt 2 configuration variants, and MoveIt Task Constructor examples.

The workspace is located at:

```text
/home/fst/ros2_pose_estimate_ws
```

This page currently lives in `test_fa_ws/docs/` so both related workspaces can be documented from one shared starting point.

## Workspace Layout

```text
/home/fst/ros2_pose_estimate_ws
`-- src/
    |-- fairino5_v6_description/
    |-- fairino5_v6_config/
    |-- fairino5_v6_moveit_config/
    |-- fairino5_v62_moveit_config/
    |-- fairino5_v63_moveit_config/
    `-- mtc_tutorial/
```

## Package Inventory

Packages detected with `colcon list`:

- `fairino5_v6_description`: FAIRINO 5 V6 robot description, URDF/Xacro files, controller config, and RViz launch.
- `fairino5_v6_config`: MoveIt 2 config package for FAIRINO 5 V6.
- `fairino5_v6_moveit_config`: MoveIt 2 config package for FAIRINO 5 V6.
- `fairino5_v62_moveit_config`: alternate MoveIt 2 config package with `sensors_3d.yaml`.
- `fairino5_v63_moveit_config`: alternate MoveIt 2 config package.
- `mtc_tutorial`: MoveIt Task Constructor tutorial and FAIRINO demo code. Installs `mtc_tutorial` and `mtc_tutorial2`.

## Build

From the workspace root:

```bash
cd /home/fst/ros2_pose_estimate_ws
source /opt/ros/<ros-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros-distro>` with the installed ROS 2 distribution.

## Common Launches and Runs

Robot description RViz view:

```bash
ros2 launch fairino5_v6_description robot.launch.py
```

MoveIt 2 demo for the base FAIRINO 5 V6 config:

```bash
ros2 launch fairino5_v6_moveit_config demo.launch.py
```

MoveIt 2 planning stack components are also available as separate launch files:

```bash
ros2 launch fairino5_v6_moveit_config move_group.launch.py
ros2 launch fairino5_v6_moveit_config moveit_rviz.launch.py
ros2 launch fairino5_v6_moveit_config rsp.launch.py
ros2 launch fairino5_v6_moveit_config spawn_controllers.launch.py
```

MoveIt Task Constructor tutorials:

```bash
ros2 run mtc_tutorial mtc_tutorial
ros2 run mtc_tutorial mtc_tutorial2
```

MTC launch files currently available:

```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
ros2 launch mtc_tutorial mtc_demo_finger_suction.launch.py
ros2 launch mtc_tutorial mtc_demo_finger_suction2.launch.py
ros2 launch mtc_tutorial mtc_fairino.launch.py
ros2 launch mtc_tutorial mtc_fairino2.launch.py
ros2 launch mtc_tutorial mtc_fairino3.launch.py
ros2 launch mtc_tutorial mtc_node.launch.py
ros2 launch mtc_tutorial move_group_fairino.launch.py
ros2 launch mtc_tutorial spawn_robot_moveit.launch.py
```

## Notes on Config Variants

There are several FAIRINO 5 MoveIt config packages with similar file structure:

- `fairino5_v6_config`
- `fairino5_v6_moveit_config`
- `fairino5_v62_moveit_config`
- `fairino5_v63_moveit_config`

The next documentation pass should explain what each variant changes, especially robot description differences, controller settings, planning groups, end-effector setup, and whether `sensors_3d.yaml` is expected.

## Open Questions

- Which config package is the current baseline for demonstrations?
- What is the difference between `v6`, `v62`, and `v63` in this workspace?
- Which MTC launch file is the preferred starting point?
- Does this workspace connect to real hardware, simulation only, or both?
- Are perception outputs from `test_fa_ws` expected to feed into MTC tasks here?
