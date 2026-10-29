#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WORKSPACE_ROOT}/logs/contact_box_perception/${STAMP}"

mkdir -p "${LOG_DIR}"

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

echo "Launching contact-box perception / pose-estimation layer only."
echo "This script does NOT launch robot bringup, MoveIt, or RealSense."
echo "Logs: ${LOG_DIR}"

mapfile -t CURRENT_TOPICS < <(ros2 topic list 2>/dev/null || true)

has_topic() {
  local topic="$1"
  local existing
  for existing in "${CURRENT_TOPICS[@]}"; do
    if [[ "${existing}" == "${topic}" ]]; then
      return 0
    fi
  done
  return 1
}

warn_missing_topic() {
  local topic="$1"
  local description="$2"
  if has_topic "${topic}"; then
    echo "PASS: ${description} topic is present (${topic})"
  else
    echo "WARN: ${description} topic is not currently visible (${topic})"
  fi
}

warn_missing_topic /joint_states "robot joint state"
warn_missing_topic /tf "TF"
warn_missing_topic /camera/camera/color/image_raw "RealSense color image"
warn_missing_topic /camera/camera/aligned_depth_to_color/image_raw "RealSense aligned depth"
warn_missing_topic /camera/camera/color/camera_info "RealSense color camera info"

PIDS=()

cleanup() {
  echo
  echo "Stopping contact-box perception child processes..."
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait "${PIDS[@]:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo
echo "Starting YOLO/keypoint detector:"
echo "  ros2 run pose_estimater object_detect"
ros2 run pose_estimater object_detect \
  >"${LOG_DIR}/object_detect.log" 2>&1 &
PIDS+=("$!")
echo "object_detect PID: ${PIDS[-1]}"

echo
echo "Starting contact-box 3D pose debug node:"
echo "  ros2 run pose_estimater contact_box_pose_debug"
ros2 run pose_estimater contact_box_pose_debug \
  >"${LOG_DIR}/contact_box_pose_debug.log" 2>&1 &
PIDS+=("$!")
echo "contact_box_pose_debug PID: ${PIDS[-1]}"

echo
echo "Expected topics:"
echo "  /pixel_coordinates"
echo "  /contact_box_pose_debug"
echo "  /yolo/detections"
echo "  /camera/camera/color/image_raw"
echo "  /camera/camera/aligned_depth_to_color/image_raw"
echo "  /camera/camera/color/camera_info"
echo
echo "This script will keep running until Ctrl+C."

while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "ERROR: child process ${pid} exited. Check logs in ${LOG_DIR}." >&2
      exit 1
    fi
  done
  sleep 1
done
