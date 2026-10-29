#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
set -u

echo "Building workspace slowly and safely on Jetson"
colcon build --symlink-install \
  --parallel-workers 1 \
  --executor sequential \
  --event-handlers console_direct+
