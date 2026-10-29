#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Launching RealSense camera"
ros2 launch realsense2_camera rs_launch.py \
  publish_tf:=false \
  align_depth.enable:=true \
  pointcloud.enable:=true
