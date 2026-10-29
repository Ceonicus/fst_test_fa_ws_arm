#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CHECK_ENV_FILE=/tmp/contact_box_perception_check.env
LOCKED_ENV_FILE=/tmp/contact_box_locked_pose.env
FAILURES=0
REQUIRE_TARGET_VISIBLE=false

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --runtime-only"
  echo "  $0 --require-target-visible"
  echo
  echo "Default is --runtime-only: checks robot/camera/perception infrastructure only."
  echo "--require-target-visible also requires fresh stable /contact_box_pose_debug samples."
  echo "Run --require-target-visible only after the robot is at true_30cm/contact_box_ready_pose."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-only)
      REQUIRE_TARGET_VISIBLE=false
      ;;
    --require-target-visible)
      REQUIRE_TARGET_VISIBLE=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument '$1'" >&2
      usage
      exit 2
      ;;
  esac
  shift
done

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

pass() {
  echo "PASS: $1"
}

fail() {
  echo "FAIL: $1" >&2
  FAILURES=$((FAILURES + 1))
}

check_topic_present() {
  local topic="$1"
  local label="$2"
  if topic_is_present "${topic}"; then
    pass "${label} topic is present (${topic})"
  else
    fail "${label} topic is missing (${topic})"
  fi
}

topic_is_present() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "${topic}"
}

check_any_topic_present() {
  local label="$1"
  shift
  local topic
  for topic in "$@"; do
    if topic_is_present "${topic}"; then
      pass "${label} topic is present (${topic})"
      return
    fi
  done
  fail "${label} topic is missing; checked: $*"
}

check_topic_once() {
  local topic="$1"
  local label="$2"
  if timeout 5 ros2 topic echo "${topic}" --once >/tmp/contact_box_topic_check.txt 2>&1; then
    pass "${label} is publishing recent data (${topic})"
  else
    fail "${label} did not publish data within timeout (${topic})"
    sed -n '1,5p' /tmp/contact_box_topic_check.txt >&2 || true
  fi
}

check_tf() {
  local target="$1"
  local source="$2"
  local label="$3"
  local output="/tmp/contact_box_tf_${target}_${source}.txt"
  if timeout 5 ros2 run tf2_ros tf2_echo "${target}" "${source}" >"${output}" 2>&1 &&
    grep -q "Translation:" "${output}"
  then
    pass "${label} TF is available (${target} -> ${source})"
  else
    fail "${label} TF is unavailable (${target} -> ${source})"
    sed -n '1,8p' "${output}" >&2 || true
  fi
}

if [[ "${REQUIRE_TARGET_VISIBLE}" == "true" ]]; then
  echo "Checking robot stack, RealSense, contact-box runtime, and target visibility."
  echo "Assumption: robot is already at true_30cm/contact_box_ready_pose."
else
  echo "Checking robot stack, RealSense, and contact-box runtime readiness."
  echo "Target visibility is not required in runtime-only mode."
fi
echo "This script does not move the robot and does not command suction/gripper hardware."
echo

check_topic_present /joint_states "robot joint state"
check_topic_present /camera/camera/color/image_raw "RealSense color image"
check_topic_present /camera/camera/aligned_depth_to_color/image_raw "RealSense aligned depth"
check_topic_present /camera/camera/color/camera_info "RealSense color camera info"
check_any_topic_present "YOLO/object detection" /pixel_coordinates /yolo/detections
check_topic_present /contact_box_pose_debug "contact-box 3D pose publisher"

echo
check_tf base_link wrist3_link "wrist"
check_tf base_link camera_color_optical_frame "camera"

if [[ "${REQUIRE_TARGET_VISIBLE}" == "true" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"

  echo
  check_topic_once /pixel_coordinates "YOLO keypoint"
  check_topic_once /contact_box_pose_debug "contact-box 3D pose"

  echo
  rm -f -- "${LOCKED_ENV_FILE}" "${CHECK_ENV_FILE}"
  unset LOCKED_CONTACT_X LOCKED_CONTACT_Y LOCKED_CONTACT_Z
  unset CONTACT_POSE_MAX_DEVIATION CONTACT_POSE_ACQUIRED_AT_UNIX

  if "${SCRIPT_DIR}/acquire_contact_pose.py" \
    --samples "${CONTACT_BOX_POSE_SAMPLE_COUNT}" \
    --timeout "${CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC}" \
    --max-spread "${CONTACT_BOX_POSE_MAX_SPREAD_M}" \
    --output "${CHECK_ENV_FILE}"
  then
    if [[ -f "${CHECK_ENV_FILE}" ]]; then
      # shellcheck disable=SC1090
      if ! source "${CHECK_ENV_FILE}"; then
        fail "No fresh stable contact-box pose. Ensure contact box is visible."
      fi
    fi
    if [[ -n "${LOCKED_CONTACT_X:-}" && -n "${LOCKED_CONTACT_Y:-}" && -n "${LOCKED_CONTACT_Z:-}" ]]; then
      pass "Fresh stable contact-box pose acquired"
    else
      fail "No fresh stable contact-box pose. Ensure contact box is visible."
    fi
  else
    echo "Contact-box keypoint/contact pose acquisition failed. Refusing to move." >&2
    fail "No fresh stable contact-box pose. Ensure contact box is visible."
  fi
fi

echo
if [[ "${FAILURES}" -eq 0 ]]; then
  if [[ "${REQUIRE_TARGET_VISIBLE}" == "true" ]]; then
    echo "CONTACT-BOX TARGET VISIBILITY CHECK PASS"
  else
    echo "CONTACT-BOX RUNTIME CHECK PASS"
  fi
  exit 0
fi

if [[ "${REQUIRE_TARGET_VISIBLE}" == "true" ]]; then
  echo "CONTACT-BOX TARGET VISIBILITY CHECK FAIL: ${FAILURES} issue(s)" >&2
else
  echo "CONTACT-BOX RUNTIME CHECK FAIL: ${FAILURES} issue(s)" >&2
fi
exit 1
