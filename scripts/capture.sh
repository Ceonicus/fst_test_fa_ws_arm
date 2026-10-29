#!/usr/bin/env bash
set -euo pipefail

STEP_MM="${1:-10}"

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Launching camera pose capture"
echo "Selected step size: ${STEP_MM} mm"
echo "WARNING: the robot will physically move."
for second in 3 2 1; do
  echo "Starting in ${second}..."
  sleep 1
done

ros2 launch camera_pose_capture camera_pose_capture.launch.py \
  execute:=true \
  use_realsense:=false \
  group_name:=fairino5_v6_group \
  mount_link:=wrist2_link \
  control_link:=wrist3_link \
  lock_joints:="[none]" \
  apriltag_frame:=tag36h11_0 \
  apriltag_max_age_sec:=0.5 \
  apriltag_wait_timeout_sec:=3.0 \
  step_mm:=${STEP_MM} \
  settle_seconds:=2.0
