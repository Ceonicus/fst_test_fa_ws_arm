# camera_pose_capture

Safe camera pose capture workflow for the FAIRINO FR5-V6 RealSense setup.

This package does not launch the robot environment, does not launch MTC, and does not include gripper, suction, tool-change, or IO logic. The robot/MoveIt environment and RealSense camera should be started separately.

The launch default is `execute:=false`. In dry-run mode the node prints the configured pose sequence and exits without creating or calling MoveIt motion interfaces.

## Dry Run

```bash
ros2 launch camera_pose_capture camera_pose_capture.launch.py execute:=false
```

## Execute With Current Workspace Group

```bash
ros2 launch camera_pose_capture camera_pose_capture.launch.py execute:=true group_name:=fairino5_v6_group step_mm:=5
```

## Execute With Reference MTC Environment Group

```bash
ros2 launch camera_pose_capture camera_pose_capture.launch.py execute:=true group_name:=arm step_mm:=5
```

Before execute mode, verify the FAIRINO web app has no alarm/brake issue, the arm is in a safe pose, the workspace is clear, `/joint_states` is live, camera topics are live, and TF includes `base_link -> wrist2_link`, `base_link -> wrist3_link`, and `base_link -> camera_color_optical_frame`.
