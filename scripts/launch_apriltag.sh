#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Launching AprilTag detector"
ros2 run apriltag_ros apriltag_node \
  --ros-args \
  --params-file camera_apriltag_test/tags_36h11_test.yaml \
  -r /image_rect:=/camera/camera/color/image_raw \
  -r /camera_info:=/camera/camera/color/camera_info
