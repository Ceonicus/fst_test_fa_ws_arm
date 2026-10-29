#!/usr/bin/env bash
set -euo pipefail

cd ~/Desktop/fst_test_fa_ws_arm
set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/fst_test_fa_ws_arm/install/setup.bash
set -u

echo "Launching Fairino MoveIt robot demo"
ros2 launch fairino_mtc_demo mtc_fairino.launch.py
