# Robotics Documentation Completion Draft

This draft contains text that can be pasted into the existing Google Docs/Word documentation. It is organized by the same section numbers found in `Robotics Docummentation compiled.docx`.

The content below is based on the current files in `/home/fst/test_fa_ws` and `/home/fst/ros2_pose_estimate_ws`. Hardware-specific values such as final wiring pinouts, camera serial number, robot safety settings, and calibrated TF values should still be verified on the physical setup before release.

## 1. Hardware Setup

### 1.1 Main Hardware Components

The robot cell consists of the following main hardware components:

- FAIRINO collaborative robot arm with controller box.
- Tool changer mounted between the robot flange and the end effector.
- End effectors used for testing:
  - RG75-300 parallel gripper.
  - EV-08 suction gripper.
- Intel RealSense depth camera for RGB image and point cloud capture.
- Ubuntu 22.04 workstation or Jetson device running ROS 2 Humble.
- Ethernet connection between the computer and the robot controller.
- 24 V power supply and signal wiring for end-effector power, RS485 communication, and digital I/O.

### 1.2 Basic Safety Checks Before Power-On

Before powering the robot or running ROS commands:

1. Make sure the robot is mechanically mounted and cannot move during operation.
2. Check that the emergency stop is reachable.
3. Confirm that the tool changer is fully locked and the end effector is secure.
4. Confirm that the end-effector wiring is strain-relieved and will not be pulled during robot motion.
5. Check that the work area is clear of people and loose objects.
6. Start with low robot speed while testing new ROS nodes, MoveIt plans, gripper commands, or suction commands.
7. Verify the robot IP address is reachable before starting ROS hardware or gripper nodes.

### 1.3 Robot Network Setup

The default robot controller address used in the current code is:

```text
192.168.58.2
```

This value is configured in:

```text
src/rg75_300_gripper/src/gripper_config.hpp
src/ev_08_suction_gripper/src/suction_config.hpp
```

The computer should be connected to the same subnet as the robot controller. For example, the computer can use a static IP address such as:

```text
192.168.58.x
```

where `x` is not already used by the robot controller.

Verify connectivity with:

```bash
ping 192.168.58.2
```

### 1.4 Tool Changer Setup

The tool changer provides a repeatable mechanical interface between the robot flange and the active end effector. After mounting:

1. Confirm the robot-side tool changer plate is rigidly attached to the robot flange.
2. Confirm the tool-side plate is rigidly attached to the gripper or suction tool.
3. Check that all alignment pins fully seat when the tool is connected.
4. Check that power, RS485, and I/O lines pass correctly through the tool changer.
5. Label the robot-side and tool-side connectors to avoid incorrect reconnection.

Recommended figure captions:

- Figure: Complete robot tool changer setup.
- Figure: Robot-side tool changer plate mounted on robot flange.
- Figure: Tool-side plate mounted to gripper/suction tool.
- Figure: Cable routing from robot flange to tool changer connector.

## 2. Electrical Wirings

### 2.1 Suction Gripper Wiring

The EV-08 suction gripper uses 24 V power, RS485 communication, and digital I/O wiring. The current wiring list is:

```text
Brown wire: 24 V
Blue wire: 0 V
White wire: RS485+ / RS485A
Black wire: RS485- / RS485B
Green wire: IN1
Orange wire: IN2
Half black/white wire: RS485 GND
Purple wire: IN COM
Yellow wire: OUT1
Pink wire: OUT2
Half yellow/black wire: OUT3
Red wire: OUT COM
```

Before connecting power, verify the suction gripper datasheet and the 12-pin connector pinout. Incorrect 24 V or RS485 wiring can damage the device.

### 2.2 Parallel Gripper Wiring

The RG75-300 parallel gripper is controlled through the FAIRINO controller using the gripper interface exposed by the Fairino SDK. In the current ROS package, the gripper configuration is stored in:

```text
src/rg75_300_gripper/src/gripper_config.hpp
```

Important configuration values currently used:

```text
ROBOT_IP = 192.168.58.2
GRIPPER_INDEX = 2
GRIPPER_COMPANY = 6
GRIPPER_DEVICE = 0
GRIPPER_SOFTWARE = 0
GRIPPER_BUS = 2
```

These values must match the robot Web UI peripheral settings. If the Web UI settings and ROS code do not match, the ROS service may start but the gripper will not respond correctly.

### 2.3 Wiring Verification Checklist

Use this checklist after wiring and before running ROS commands:

1. 24 V and 0 V are connected to the correct pins.
2. RS485 A/B polarity is correct.
3. RS485 GND is connected when required by the device.
4. Digital input common and output common are wired to the correct reference.
5. The tool changer passes through all required signals.
6. No exposed wire strands are visible.
7. The cable path does not collide with the robot during full-range movement.
8. Robot Web UI peripheral settings match the end effector being tested.

## 3.3 Controlling the Suction Gripper

### Method 1: ROS Method

The suction gripper is controlled by the `ev_08_suction_gripper` package. The ROS node connects to the FAIRINO robot controller using the SDK and exposes a ROS 2 service.

Start the suction node:

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ev_08_suction_gripper suction_node
```

The node provides:

```text
Service: /suction_command
Type: ev_08_suction_gripper/srv/SuctionCommand
Topic: /suction_state
Type: ev_08_suction_gripper/msg/SuctionState
```

The service request uses one integer command:

```text
1 = suction A on
2 = suction A off
3 = suction B on
4 = suction B off
5 = suction A and B on
```

Example service calls:

```bash
ros2 service call /suction_command ev_08_suction_gripper/srv/SuctionCommand "{command: 1}"
ros2 service call /suction_command ev_08_suction_gripper/srv/SuctionCommand "{command: 2}"
ros2 service call /suction_command ev_08_suction_gripper/srv/SuctionCommand "{command: 3}"
ros2 service call /suction_command ev_08_suction_gripper/srv/SuctionCommand "{command: 4}"
ros2 service call /suction_command ev_08_suction_gripper/srv/SuctionCommand "{command: 5}"
```

The service response returns:

```text
bool success
string message
```

If the service fails, check:

- Robot controller is reachable at `192.168.58.2`.
- The suction gripper is powered.
- RS485 wiring and robot Web UI settings are correct.
- The `suction_config.hpp` values match the robot setup.
- The robot controller has no active error state.

## 4.5.9 moveit_move

The `moveit_move` package contains C++ examples that use MoveIt 2 to plan and execute FAIRINO robot motion.

The package builds two executables:

```text
hello_moveit
moveit_move
```

### hello_moveit

`hello_moveit` is a simple MoveIt test program. It creates a `MoveGroupInterface` for the planning group:

```text
fairino5_v6_group
```

It sets a fixed target pose, plans a trajectory, and executes the plan if planning succeeds. This is useful for confirming that MoveIt, the robot model, controllers, and planning group are working.

Run it with:

```bash
ros2 run moveit_move hello_moveit
```

### moveit_move

`moveit_move` is the integration node used with perception. It subscribes to:

```text
/object_pose
```

The subscribed message type is:

```text
geometry_msgs/msg/PoseStamped
```

When an object pose is received, the node sends the pose to MoveIt as a target pose for the robot. It then plans and executes the motion. After motion execution, it calls the RG75 gripper service:

```text
/gripper_command
```

This connects the perception output to robot motion and gripper actuation.

Run it with:

```bash
ros2 run moveit_move moveit_move
```

Important notes:

- The MoveIt planning group must be named `fairino5_v6_group`.
- `/object_pose` must be published before motion can begin.
- `/gripper_command` must be available if the gripper command step is required.
- The node adds a table collision object to the planning scene before planning.

## 4.5.10 pose_estimater

The `pose_estimater` package contains Python ROS 2 nodes for object detection and 3D pose estimation using RealSense camera data.

Executables:

```text
object_detect
object_pose_estimate
object_pose_estimate2
```

### object_detect

`object_detect` runs a YOLO model on the RGB camera stream. The default image topic is:

```text
/camera/camera/color/image_raw
```

The default YOLO model path is:

```text
/home/fst/test_fa_ws/yolo_custom.pt
```

The node publishes an annotated detection image and the detected object's pixel center. The pixel coordinate output is:

```text
/pixel_coordinates
```

The current code searches for the class label:

```text
Interlock w/ Cover
```

Run it with:

```bash
ros2 run pose_estimater object_detect
```

### object_pose_estimate2

`object_pose_estimate2` combines the 2D pixel coordinate from YOLO with the RealSense point cloud. It subscribes to:

```text
/camera/camera/depth/color/points
/pixel_coordinates
```

It looks up the transform between the camera frame and robot base frame, converts the detected pixel to a 3D point, transforms it into the robot base frame, and publishes:

```text
/object_pose
```

The default frames are:

```text
base_frame = base_link
camera_frame = camera_depth_optical_frame
```

Run it with:

```bash
ros2 run pose_estimater object_pose_estimate2
```

This node depends on a valid TF transform between `base_link` and `camera_depth_optical_frame`. If no transform exists, `/object_pose` will not be published.

## 4.5.11 yolo_pose_estimator

The `yolo_pose_estimator` package is a separate Python package that estimates object pose from YOLO detections, point cloud data, and camera info.

Executable:

```text
pose_estimator
```

Default input topics:

```text
/yolo/detections
/camera/camera/depth/points
/camera/camera/color/camera_info
```

Default output:

```text
/object_pose
```

It uses `message_filters` to synchronize YOLO detections, point cloud data, and camera calibration information. It then calculates an object pose and broadcasts a TF child frame named from the detected object ID.

Run it with:

```bash
ros2 run yolo_pose_estimator pose_estimator
```

Use this package when the detector publishes `vision_msgs/msg/Detection2DArray`. Use `pose_estimater` when the workflow is based on the custom `/pixel_coordinates` output.

## 6. Integration of Packages

The complete system combines hardware control, camera perception, object pose estimation, MoveIt planning, and gripper actuation.

### 6.1 System Data Flow

The current perception-to-motion flow is:

```text
RealSense camera
  -> /camera/camera/color/image_raw
  -> object_detect
  -> /pixel_coordinates
  -> object_pose_estimate2
  -> /object_pose
  -> moveit_move
  -> MoveIt plan and execute
  -> /gripper_command
  -> RG75 gripper action
```

The RealSense point cloud is also used:

```text
RealSense camera
  -> /camera/camera/depth/color/points
  -> object_pose_estimate2
```

### 6.2 Required TF Frames

For the pose estimation pipeline to work, ROS must know the transform between the camera and robot base. The important frames are:

```text
base_link
camera_depth_optical_frame
camera_link
```

During early testing, a static transform can be published manually:

```bash
ros2 run tf2_ros static_transform_publisher 0.2 0 0.3 0 0 0 base_link camera_link
```

For the final system, the static transform should come from hand-eye calibration rather than a guessed value.

### 6.3 Recommended Five-Terminal Test Flow

Terminal 1: launch MoveIt.

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

Terminal 2: launch the RealSense camera with point cloud enabled.

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Terminal 3: run object detection.

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater object_detect
```

Terminal 4: run object pose estimation.

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pose_estimater object_pose_estimate2
```

Terminal 5: run MoveIt motion from object pose.

```bash
cd ~/test_fa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run moveit_move moveit_move
```

### 6.4 Verification Commands

Check active topics:

```bash
ros2 topic list
```

Check the object pose:

```bash
ros2 topic echo /object_pose
```

Check pixel coordinates from YOLO:

```bash
ros2 topic echo /pixel_coordinates
```

Check TF tree:

```bash
ros2 run tf2_tools view_frames
```

Check services:

```bash
ros2 service list
```

Check gripper command service type:

```bash
ros2 service type /gripper_command
```

Check suction command service type:

```bash
ros2 service type /suction_command
```

### 6.5 Common Integration Problems

Problem: Object is detected in the camera image but `/object_pose` is not published.

Likely causes:

- `/camera/camera/depth/color/points` is not active.
- `/pixel_coordinates` is not being published.
- The point cloud frame does not match `camera_depth_optical_frame`.
- TF from `base_link` to `camera_depth_optical_frame` is missing.

Problem: MoveIt receives `/object_pose` but planning fails.

Likely causes:

- Target pose is outside the robot workspace.
- Target orientation is not reachable.
- Collision object blocks the path.
- MoveIt planning group name does not match `fairino5_v6_group`.

Problem: Gripper or suction service exists but hardware does not move.

Likely causes:

- Robot IP address is wrong or robot is unreachable.
- Web UI peripheral settings do not match the ROS config.
- End-effector wiring is incorrect.
- Robot controller has an active error state.
- The selected gripper index does not match the configured device.

## 8. YOLOv11 Computer Vision

### 8.1 Purpose

YOLOv11 is used to detect the target object in the RealSense RGB image. The detection result is used as the first step in the robot perception pipeline. In the current local workflow, the detector publishes the center pixel of the target object, and a separate pose estimation node combines that pixel with RealSense point cloud data.

### 8.2 Input and Output

Input image topic:

```text
/camera/camera/color/image_raw
```

Detected target class:

```text
Interlock w/ Cover
```

Pixel coordinate output:

```text
/pixel_coordinates
```

Annotated image output:

```text
/yolo/detections
```

### 8.3 Running YOLO Detection

Start the RealSense camera first:

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Then run the detection node:

```bash
ros2 run pose_estimater object_detect
```

If using the `yolo_ros` bringup package instead:

```bash
ros2 launch yolo_bringup yolo.launch.py model:=yolo_custom.pt
```

### 8.4 Model File

The default custom model path used by `pose_estimater/object_detect.py` is:

```text
/home/fst/test_fa_ws/yolo_custom.pt
```

If the model file is moved, update the launch command or the node parameter:

```bash
ros2 run pose_estimater object_detect --ros-args -p model_path:=/path/to/model.pt
```

### 8.5 Tuning Detection Confidence

The default confidence threshold is:

```text
0.5
```

To change it:

```bash
ros2 run pose_estimater object_detect --ros-args -p confidence_threshold:=0.7
```

Use a higher threshold to reduce false positives. Use a lower threshold if the target object is often missed.

## Additional Section: Hand-Eye Calibration Workflow

Hand-eye calibration is needed to accurately transform camera measurements into the robot base frame. Without calibration, the robot may detect the object correctly in the image but move to the wrong physical position.

### Calibration Goal

The goal is to obtain a static transform between:

```text
robot base or end effector
camera frame
```

depending on whether the camera is mounted in the environment or on the robot arm.

### Current Package

The workspace contains:

```text
easy_handeye2
easy_handeye2_msgs
```

The FAIRINO MoveIt config also includes:

```text
fairino5_v6_moveit2_config/launch/handeye_calibrate.launch.py
```

### Basic Workflow

1. Launch the robot and MoveIt stack.
2. Launch the RealSense camera.
3. Launch the calibration target detector, such as AprilTag or ArUco detection.
4. Launch the hand-eye calibration tool.
5. Move the robot through multiple poses where the calibration target is visible.
6. Collect samples.
7. Compute and save the calibration.
8. Publish the saved transform during normal robot startup.
9. Verify in RViz that the camera frame and robot frame are aligned correctly.

### Verification

After calibration, verify:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf_static
```

The final transform should be stable and should not be replaced by a rough manual static transform in production runs.
