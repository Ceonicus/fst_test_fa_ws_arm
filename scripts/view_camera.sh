#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Launching camera sample viewer"
python3 camera_apriltag_test/scripts/view_camera_for_samples.py
