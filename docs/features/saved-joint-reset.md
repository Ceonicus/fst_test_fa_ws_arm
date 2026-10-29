# Saved Joint Reset

Purpose: document the safe joint-target reset utility for returning the FR5-V6 arm to a recorded camera-visible pose.

## Status

Implemented node:

```bash
ros2 launch moveit_move return_to_saved_joint_pose.launch.py
```

Default behavior is planning-only. Execution requires both:

```text
execute_motion: true
allow_reset_execution: true
```

The node does not call `setPoseTarget`, does not call `computeCartesianPath`, and does not command gripper or suction hardware.
It reads current joints directly from `/joint_states` for reset safety checks and planning start state construction, so it does not depend on MoveIt's current-state monitor accepting the latest joint-state timestamp.

## Recorded Reset Pose

Previously reported by user: the manual reset pose was recorded from `/joint_states` in this message order:

```text
j1, j2, j4, j5, j3, j6
```

The launch defaults remap that recording into logical robot joint order:

```text
j1: -1.4155149936414828
j2: -1.4576652436883306
j3: -1.8781316413347113
j4:  0.2190782725278865
j5:  2.206984498534039
j6:  2.3956715922950256
```

The implementation maps targets by MoveIt joint name, not by `/joint_states` array order.

## Parameters

Defaults:

```text
planning_group: fairino5_v6_group
j1: -1.4155149936414828
j2: -1.4576652436883306
j3: -1.8781316413347113
j4: 0.2190782725278865
j5: 2.206984498534039
j6: 2.3956715922950256
max_joint_delta_rad: 0.50
already_at_target_tolerance_rad: 0.002
planning_time_sec: 5.0
velocity_scaling: 0.002
acceleration_scaling: 0.002
execute_motion: false
allow_reset_execution: false
```

## Safety Behavior

Verified from source:

- Subscribes to `/joint_states` as `sensor_msgs/msg/JointState` and waits up to 5 seconds for the first message.
- Reads current joint values from the cached `/joint_states` message by joint name, not array order.
- Uses the MoveIt planning group's joint names to map current and target values.
- Prints current values, target values, and current-to-target deltas before planning.
- Refuses to plan if any current-to-target joint delta exceeds `max_joint_delta_rad`.
- Exits cleanly without planning or execution if every current-to-target joint delta is less than or equal to `already_at_target_tolerance_rad`.
- Sets MoveIt planning time to `planning_time_sec` and uses one planning attempt.
- Constructs a MoveIt `RobotState` start state from cached `/joint_states` values and passes it to `setStartState(...)`.
- Sets only a joint value target with `setJointValueTarget(...)`.
- After planning, inspects the first and final joint trajectory points and prints planned joint deltas.
- Refuses execution if any planned joint delta exceeds `max_joint_delta_rad`.
- Only executes when both execution flags are true.
- Returns nonzero when safety-blocked, when planning fails, when execution gates block a requested execution, or when execution fails. Returns zero only for true plan-only success, already-at-target success, or successful execution.

Known limitation: post-execution actual joint feedback verification is not implemented in this node yet. A future hardening pass should wait for fresh `/joint_states` after `execute()` succeeds, compare actual joints to the target by joint name, and return nonzero if the final error exceeds tolerance.

Planning-only command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_move return_to_saved_joint_pose.launch.py
```

Robot-moving execution should only be run deliberately under supervision:

```bash
ros2 launch moveit_move return_to_saved_joint_pose.launch.py \
  execute_motion:=true \
  allow_reset_execution:=true
```

Needs revalidation: this documentation pass built the package but did not launch the reset utility against live robot state and did not execute robot motion.
