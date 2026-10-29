#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Checking ROS package prefixes"
ros2 pkg prefix fairino_hardware
ros2 pkg prefix fairino_mtc_demo
ros2 pkg prefix fairino5_v6_moveit2_config
ros2 pkg prefix fairino_description
ros2 pkg prefix camera_pose_capture
ros2 pkg prefix realsense2_camera
ros2 pkg prefix apriltag_ros

echo ""
echo "Checking active old workspace references"
grep -R "/home/fst/test_fa_ws\|/home/fst/ros2_pose_estimate_ws\|Desktop/fst_test_fa_ws/" -n src \
  --exclude-dir=.git \
  --exclude-dir=__pycache__ \
  --exclude="*.bak" \
  --exclude="*.bak_*" \
  --exclude="*.2026*.bak" \
  --exclude="*.zip" \
  --exclude="*.so" \
  --exclude="*.so.*" || true
