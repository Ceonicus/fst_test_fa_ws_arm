# Documentation Roadmap

Use this file as the lightweight plan for turning the workspace notes into dependable project documentation.

## Current Documentation

- `README.md`: front door for `test_fa_ws`.
- `docs/test_fa_ws.md`: package inventory, build commands, and common launches for the integration workspace.
- `docs/ros2_pose_estimate_ws.md`: package inventory and common launches for the FAIRINO pose-estimation/MoveIt workspace.

## Next High-Value Pages

1. Hardware setup

   Document the robot controller, network settings, RealSense model, gripper type, emergency-stop assumptions, and startup checklist.

2. Bringup sequence

   Record the tested terminal-by-terminal launch order for camera, robot driver, MoveIt, YOLO, pose estimation, calibration, and task execution.

3. TF and frames

   List the canonical frames for robot base, flange/tool, camera optical frame, calibration target, detected object, and planning scene object.

4. Perception pipeline

   Describe the image topics, YOLO detector outputs, depth association, pose-estimation assumptions, and published object pose topics.

5. Calibration workflow

   Document the `easy_handeye2` flow, calibration launch file, sample collection procedure, output files, and how to verify the result in RViz.

6. MoveIt and MTC workflow

   Explain which MoveIt config is the baseline, how planning groups are named, how grippers are represented, and how MTC demos are expected to run.

7. Troubleshooting

   Collect recurring issues such as missing transforms, camera not detected, robot not reachable, failed planning, controller startup problems, and package build failures.

## Documentation Rules

- Prefer exact commands that have been tested on the robot workstation.
- Include expected topics, services, actions, and frames for each launch.
- Mark assumptions explicitly.
- Keep vendor package documentation minimal and link to upstream docs when available.
- Put project-specific behavior in local package docs or workspace docs.

## Verification Checklist

Before calling a page complete, verify:

- build command works from a clean terminal
- launch command starts without missing package errors
- expected ROS topics are visible with `ros2 topic list`
- expected TF frames are visible with `ros2 run tf2_tools view_frames`
- RViz config opens without missing fixed-frame errors
- hardware-specific steps include safety notes and shutdown steps
