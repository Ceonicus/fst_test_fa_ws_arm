# Robot MoveIt Control

Purpose: document the FAIRINO FR5-V6 MoveIt2 stack used by the helper scripts and capture workflow.

## Launch

Robot launch script:

```bash
./scripts/launch_robot.sh
```

Verified command:

```bash
ros2 launch fairino_mtc_demo mtc_fairino.launch.py
```

The launch file uses:

- `robotname = fairino5_v6_robot`
- `packagename = fairino5_v6_moveit2_config`
- `ros2controllername = fairino5_controller`
- planning pipelines: `ompl`, `pilz_industrial_motion_planner`
- nodes: RViz, static TF `world -> base_link`, `robot_state_publisher`, `move_group`, `ros2_control_node`

## MoveIt Group

Verified from `fairino5_v6_robot.srdf`:

```text
group: fairino5_v6_group
joints: j1, j2, j3, j4, j5, j6
chain: base_link -> wrist3_link
```

## Controllers

Verified from `ros2_controllers.yaml`:

```text
joint_state_broadcaster
fairino5_controller
```

Verified from `moveit_controllers.yaml`:

```text
fairino5_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  joints: j1..j6
```

Verified from `fairino5_v6_robot.ros2_control.xacro`:

```text
hardware plugin: fairino_hardware/FairinoHardwareInterface
```

The xacro macro instance is named `FakeSystem`, but the configured plugin is the FAIRINO hardware interface.

## Diagnostics

Safe when the robot stack is already intentionally running:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Expected joint states topic:

```text
/joint_states
```

## Runtime State

Physical robot movement after controller/hardware fixes was previously reported by user. It was not independently revalidated during this documentation pass because robot movement commands were intentionally not run.
