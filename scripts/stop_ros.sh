#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Stopping common ROS processes"
pkill -f camera_pose_capture || true
pkill -f apriltag_node || true
pkill -f realsense2_camera || true
pkill -f robot_state_publisher || true
pkill -f move_group || true
pkill -f ros2_control_node || true
pkill -f controller_manager || true

echo "Restarting ROS daemon"
ros2 daemon stop || true
ros2 daemon start || true
