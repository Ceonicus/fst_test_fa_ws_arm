#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --execute"
  echo "  $0 --execute --fast"
  echo "  $0 --execute --press"
  echo "  $0 --execute --press-depth-m 0.003"
  echo
  echo "Plan-only by default. --execute performs one direct Cartesian approach."
  echo "Assumes the robot is already at true_30cm/contact_box_ready_pose."
  echo "--fast is valid only with --execute and raises scaling conservatively."
  echo "--press adds an explicit slow compression move after the default 0.002 m target."
}

EXECUTE=false
FAST=false
PRESS=false
PRESS_DEPTH_M=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=true
      ;;
    --fast)
      FAST=true
      ;;
    --press)
      PRESS=true
      ;;
    --press-depth-m)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --press-depth-m requires a value in meters." >&2
        usage
        exit 2
      fi
      PRESS=true
      PRESS_DEPTH_M="$2"
      shift
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

if [[ "${FAST}" == "true" && "${EXECUTE}" != "true" ]]; then
  echo "ERROR: --fast is only valid together with --execute." >&2
  usage
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE=/tmp/contact_box_locked_pose.env

declare -a TIMING_LABELS=()
declare -a TIMING_DURATIONS_MS=()
SEQUENCE_START_MS="$(date +%s%3N)"

now_ms() {
  date +%s%3N
}

format_duration_ms() {
  local duration_ms="$1"
  printf '%d.%03d s' "$((duration_ms / 1000))" "$((duration_ms % 1000))"
}

record_timing() {
  local label="$1"
  local duration_ms="$2"
  TIMING_LABELS+=("${label}")
  TIMING_DURATIONS_MS+=("${duration_ms}")
}

print_timing_summary() {
  local sequence_elapsed_ms
  local i

  sequence_elapsed_ms="$(( $(now_ms) - SEQUENCE_START_MS ))"

  echo
  echo "Timing summary:"
  for i in "${!TIMING_LABELS[@]}"; do
    echo "- ${TIMING_LABELS[$i]}: $(format_duration_ms "${TIMING_DURATIONS_MS[$i]}")"
  done
  echo "- total: $(format_duration_ms "${sequence_elapsed_ms}")"
}

CONFIG_LOAD_START_MS="$(now_ms)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"
record_timing "load config" "$(( $(now_ms) - CONFIG_LOAD_START_MS ))"

# Easy-to-edit deployment-test thresholds.
TOOL_OFFSET_Z=0.263
FINAL_STANDOFF_M="${CONTACT_BOX_DEFAULT_STANDOFF_M}"
FINAL_TARGET_MARGIN_M=0.001
MIN_CURRENT_STANDOFF_M=0.015
MAX_CURRENT_STANDOFF_M=0.400
MAX_LATERAL_ERROR_M="${CONTACT_BOX_MAX_PRE_APPROACH_LATERAL_ERROR_M}"
CONTACT_SAMPLE_COUNT="${CONTACT_BOX_POSE_SAMPLE_COUNT}"
CONTACT_SAMPLE_TIMEOUT_SEC="${CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC}"
CONTACT_MAX_SPREAD_M="${CONTACT_BOX_POSE_MAX_SPREAD_M}"
DEVELOPMENT_MANUAL_CONTACT_X=0.3323
DEVELOPMENT_MANUAL_CONTACT_Y=-0.7376
DEVELOPMENT_MANUAL_CONTACT_Z=0.6131
MAX_LIVE_VS_MANUAL_DELTA_M="${CONTACT_BOX_MAX_LIVE_VS_MANUAL_DELTA_M}"
DEFAULT_PRESS_DEPTH_M="${CONTACT_BOX_DEFAULT_PRESS_DEPTH_M}"
MAX_PRESS_DEPTH_M="${CONTACT_BOX_MAX_PRESS_DEPTH_M}"

VELOCITY_SCALING=0.003
ACCELERATION_SCALING=0.003
if contact_box_config_value_changed CONTACT_BOX_APPROACH_VELOCITY_SCALING; then
  VELOCITY_SCALING="${CONTACT_BOX_APPROACH_VELOCITY_SCALING}"
fi
if contact_box_config_value_changed CONTACT_BOX_APPROACH_ACCELERATION_SCALING; then
  ACCELERATION_SCALING="${CONTACT_BOX_APPROACH_ACCELERATION_SCALING}"
fi

CARTESIAN_EEF_STEP=0.001
if contact_box_config_value_changed CONTACT_BOX_CARTESIAN_EEF_STEP; then
  CARTESIAN_EEF_STEP="${CONTACT_BOX_CARTESIAN_EEF_STEP}"
fi
CARTESIAN_JUMP_THRESHOLD="${CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD}"
CARTESIAN_MIN_FRACTION="${CONTACT_BOX_CARTESIAN_MIN_FRACTION}"

MAX_WRIST_TARGET_DISTANCE=0.350
if contact_box_config_value_changed CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE; then
  MAX_WRIST_TARGET_DISTANCE="${CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE}"
fi
MAX_JOINT_DELTA_RAD=1.20
if contact_box_config_value_changed CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD; then
  MAX_JOINT_DELTA_RAD="${CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD}"
fi

PRESS_VELOCITY_SCALING="${CONTACT_BOX_PRESS_VELOCITY_SCALING}"
PRESS_ACCELERATION_SCALING="${CONTACT_BOX_PRESS_ACCELERATION_SCALING}"
PRESS_CARTESIAN_EEF_STEP="${CONTACT_BOX_CARTESIAN_EEF_STEP}"
PRESS_MAX_WRIST_TARGET_DISTANCE="${CONTACT_BOX_PRESS_MAX_WRIST_TARGET_DISTANCE}"
PRESS_MAX_JOINT_DELTA_RAD="${CONTACT_BOX_PRESS_MAX_JOINT_DELTA_RAD}"
PRESS_CARTESIAN_JUMP_THRESHOLD="${CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD}"
PRESS_CARTESIAN_MIN_FRACTION="${CONTACT_BOX_CARTESIAN_MIN_FRACTION}"

if [[ "${PRESS}" == "true" && -z "${PRESS_DEPTH_M}" ]]; then
  PRESS_DEPTH_M="${DEFAULT_PRESS_DEPTH_M}"
fi

if [[ "${PRESS}" == "true" ]]; then
  if ! PRESS_TARGET_VALUES="$(python3 - "${FINAL_STANDOFF_M}" "${PRESS_DEPTH_M}" "${MAX_PRESS_DEPTH_M}" <<'PY'
import math
import sys

try:
    final_standoff = float(sys.argv[1])
    press_depth = float(sys.argv[2])
    max_press_depth = float(sys.argv[3])
except ValueError:
    print("ERROR: --press-depth-m must be a decimal meter value.", file=sys.stderr)
    sys.exit(2)

if not math.isfinite(press_depth):
    print("ERROR: --press-depth-m must be finite.", file=sys.stderr)
    sys.exit(2)
if press_depth < 0.0:
    print("ERROR: --press-depth-m must be non-negative.", file=sys.stderr)
    sys.exit(2)
if press_depth > max_press_depth:
    print(
        f"ERROR: --press-depth-m {press_depth:.6f} exceeds safety limit "
        f"{max_press_depth:.6f} m.",
        file=sys.stderr,
    )
    sys.exit(2)

raw_target = final_standoff - press_depth
target = max(0.0, raw_target)
clamped = 1 if raw_target < 0.0 else 0
print(f"{target:.9f} {raw_target:.9f} {clamped}")
PY
)"; then
    usage
    exit 2
  fi
  read -r PRESS_TARGET_STANDOFF_M PRESS_RAW_TARGET_STANDOFF_M PRESS_TARGET_CLAMPED <<< "${PRESS_TARGET_VALUES}"
fi

if [[ "${FAST}" == "true" ]]; then
  VELOCITY_SCALING=0.005
  ACCELERATION_SCALING=0.005
fi

cd "${WORKSPACE_ROOT}"
ROS_SETUP_START_MS="$(now_ms)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
record_timing "source ROS workspace setup" "$(( $(now_ms) - ROS_SETUP_START_MS ))"

if [[ "${EXECUTE}" == "true" ]]; then
  if [[ "${FAST}" == "true" ]]; then
    echo "DIRECT CONTACT VALIDATION: execution enabled with conservative fast scaling."
  else
    echo "DIRECT CONTACT VALIDATION: execution enabled."
  fi
else
  echo "DIRECT CONTACT VALIDATION: plan-only mode. No robot motion will be commanded."
fi
echo "Expected start: true 30 cm testing pose."
echo "Final target standoff: ${FINAL_STANDOFF_M} m"
echo "Direct approach launch scaling: velocity=${VELOCITY_SCALING}, acceleration=${ACCELERATION_SCALING}"
echo "Direct approach Cartesian safety: eef_step=${CARTESIAN_EEF_STEP}, jump_threshold=${CARTESIAN_JUMP_THRESHOLD}, min_fraction=${CARTESIAN_MIN_FRACTION}, max_wrist_target_distance=${MAX_WRIST_TARGET_DISTANCE}, max_joint_delta_rad=${MAX_JOINT_DELTA_RAD}"

fail_contact_acquisition() {
  echo "Contact-box keypoint/contact pose acquisition failed. Refusing to move." >&2
  print_timing_summary
  exit 1
}

acquire_locked_contact_pose() {
  local start_ms
  local duration_ms

  start_ms="$(now_ms)"
  rm -f -- "${ENV_FILE}"
  unset LOCKED_CONTACT_X LOCKED_CONTACT_Y LOCKED_CONTACT_Z
  unset CONTACT_POSE_MAX_DEVIATION CONTACT_POSE_ACQUIRED_AT_UNIX

  if ! "${SCRIPT_DIR}/acquire_contact_pose.py" \
    --samples "${CONTACT_SAMPLE_COUNT}" \
    --timeout "${CONTACT_SAMPLE_TIMEOUT_SEC}" \
    --max-spread "${CONTACT_MAX_SPREAD_M}" \
    --output "${ENV_FILE}"
  then
    duration_ms="$(( $(now_ms) - start_ms ))"
    record_timing "acquire fresh contact pose failed" "${duration_ms}"
    fail_contact_acquisition
  fi

  if [[ ! -f "${ENV_FILE}" ]]; then
    duration_ms="$(( $(now_ms) - start_ms ))"
    record_timing "acquire fresh contact pose failed" "${duration_ms}"
    fail_contact_acquisition
  fi

  # shellcheck disable=SC1090
  if ! source "${ENV_FILE}"; then
    duration_ms="$(( $(now_ms) - start_ms ))"
    record_timing "source locked contact pose failed" "${duration_ms}"
    fail_contact_acquisition
  fi

  if [[ -z "${LOCKED_CONTACT_X:-}" || -z "${LOCKED_CONTACT_Y:-}" || -z "${LOCKED_CONTACT_Z:-}" ]]; then
    duration_ms="$(( $(now_ms) - start_ms ))"
    record_timing "validate locked contact pose failed" "${duration_ms}"
    fail_contact_acquisition
  fi

  duration_ms="$(( $(now_ms) - start_ms ))"
  record_timing "acquire fresh contact pose" "${duration_ms}"
  echo "TIMING: acquire fresh contact pose: $(format_duration_ms "${duration_ms}")"
}

if [[ "${PRESS}" == "true" ]]; then
  echo "WARNING: Press mode performs a slow suction-cup compression move after the default ${FINAL_STANDOFF_M} m target."
  echo "Press depth: ${PRESS_DEPTH_M} m"
  echo "Press target standoff: ${PRESS_TARGET_STANDOFF_M} m"
  if [[ "${PRESS_TARGET_CLAMPED}" == "1" ]]; then
    echo "WARNING: requested press depth would create negative standoff (${PRESS_RAW_TARGET_STANDOFF_M} m); clamped to 0.000000000 m."
  fi
  if [[ "${EXECUTE}" != "true" ]]; then
    echo "Press mode is plan-only because --execute was not supplied."
  fi
fi

acquire_locked_contact_pose

echo "Raw locked contact pose from fresh live YOLO/depth acquisition:"
echo "  x=${LOCKED_CONTACT_X}"
echo "  y=${LOCKED_CONTACT_Y}"
echo "  z=${LOCKED_CONTACT_Z}"
echo "  max_deviation=${CONTACT_POSE_MAX_DEVIATION}"
if [[ -n "${CONTACT_POSE_ACQUIRED_AT_UNIX:-}" ]]; then
  echo "  acquired_at_unix=${CONTACT_POSE_ACQUIRED_AT_UNIX}"
fi

float_lt() {
  python3 - "$1" "$2" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) < float(sys.argv[2]) else 1)
PY
}

float_le() {
  python3 - "$1" "$2" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)
PY
}

float_gt() {
  python3 - "$1" "$2" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)
PY
}

float_add() {
  python3 - "$1" "$2" <<'PY'
import sys
print(f"{float(sys.argv[1]) + float(sys.argv[2]):.9f}")
PY
}

apply_target_offset() {
  RAW_CONTACT_X="${LOCKED_CONTACT_X}"
  RAW_CONTACT_Y="${LOCKED_CONTACT_Y}"
  RAW_CONTACT_Z="${LOCKED_CONTACT_Z}"

  local adjusted_values
  adjusted_values="$(python3 - \
    "${RAW_CONTACT_X}" \
    "${RAW_CONTACT_Y}" \
    "${RAW_CONTACT_Z}" \
    "${CONTACT_BOX_TARGET_OFFSET_X_M}" \
    "${CONTACT_BOX_TARGET_OFFSET_Y_M}" \
    "${CONTACT_BOX_TARGET_OFFSET_Z_M}" <<'PY'
import sys

raw = tuple(float(value) for value in sys.argv[1:4])
offset = tuple(float(value) for value in sys.argv[4:7])
adjusted = tuple(raw[index] + offset[index] for index in range(3))
print(f"{adjusted[0]:.9f} {adjusted[1]:.9f} {adjusted[2]:.9f}")
PY
)"
  read -r ADJUSTED_CONTACT_X ADJUSTED_CONTACT_Y ADJUSTED_CONTACT_Z <<< "${adjusted_values}"

  echo "Fresh live contact pose:"
  echo "  x=${RAW_CONTACT_X}"
  echo "  y=${RAW_CONTACT_Y}"
  echo "  z=${RAW_CONTACT_Z}"
  echo "Configured target offset:"
  echo "  dx=${CONTACT_BOX_TARGET_OFFSET_X_M}"
  echo "  dy=${CONTACT_BOX_TARGET_OFFSET_Y_M}"
  echo "  dz=${CONTACT_BOX_TARGET_OFFSET_Z_M}"
  echo "Adjusted suction target:"
  echo "  x=${ADJUSTED_CONTACT_X}"
  echo "  y=${ADJUSTED_CONTACT_Y}"
  echo "  z=${ADJUSTED_CONTACT_Z}"
  echo "Mechanical note: adjusted target is used so the top two suction cups land on the solid center cover area."
}

print_live_vs_manual_delta() {
  local delta_values
  local raw_delta_m
  local raw_delta_cm
  local adjusted_delta_m
  local adjusted_delta_cm
  delta_values="$(python3 - \
    "${RAW_CONTACT_X}" \
    "${RAW_CONTACT_Y}" \
    "${RAW_CONTACT_Z}" \
    "${ADJUSTED_CONTACT_X}" \
    "${ADJUSTED_CONTACT_Y}" \
    "${ADJUSTED_CONTACT_Z}" \
    "${DEVELOPMENT_MANUAL_CONTACT_X}" \
    "${DEVELOPMENT_MANUAL_CONTACT_Y}" \
    "${DEVELOPMENT_MANUAL_CONTACT_Z}" <<'PY'
import math
import sys

raw = tuple(float(value) for value in sys.argv[1:4])
adjusted = tuple(float(value) for value in sys.argv[4:7])
manual = tuple(float(value) for value in sys.argv[7:10])
raw_delta = math.sqrt(sum((raw[index] - manual[index]) ** 2 for index in range(3)))
adjusted_delta = math.sqrt(sum((adjusted[index] - manual[index]) ** 2 for index in range(3)))
print(
    f"{raw_delta:.9f} {raw_delta * 100.0:.6f} "
    f"{adjusted_delta:.9f} {adjusted_delta * 100.0:.6f}"
)
PY
)"
  read -r raw_delta_m raw_delta_cm adjusted_delta_m adjusted_delta_cm <<< "${delta_values}"

  echo "Raw live contact pose: x=${RAW_CONTACT_X}, y=${RAW_CONTACT_Y}, z=${RAW_CONTACT_Z}"
  echo "Adjusted suction target: x=${ADJUSTED_CONTACT_X}, y=${ADJUSTED_CONTACT_Y}, z=${ADJUSTED_CONTACT_Z}"
  echo "Development manual contact pose: x=${DEVELOPMENT_MANUAL_CONTACT_X}, y=${DEVELOPMENT_MANUAL_CONTACT_Y}, z=${DEVELOPMENT_MANUAL_CONTACT_Z}"
  echo "Raw-live-vs-manual delta: ${raw_delta_m} m (${raw_delta_cm} cm)"
  echo "Adjusted-target-vs-manual delta: ${adjusted_delta_m} m (${adjusted_delta_cm} cm)"
  echo "Note: adjusted-target-vs-manual delta includes the configured suction target offset and may intentionally be larger."
  if float_gt "${raw_delta_m}" "${MAX_LIVE_VS_MANUAL_DELTA_M}"; then
    echo "WARNING: raw live contact point differs significantly from the validated development manual point."
  fi
}

check_current_standoff() {
  eval "$("${SCRIPT_DIR}/check_locked_standoff.py" \
    --contact-x "${ADJUSTED_CONTACT_X}" \
    --contact-y "${ADJUSTED_CONTACT_Y}" \
    --contact-z "${ADJUSTED_CONTACT_Z}" \
    --tool-offset-z "${TOOL_OFFSET_Z}" \
    --env)"
  echo "Current signed standoff to adjusted suction target: ${SIGNED_STANDOFF_M} m (${SIGNED_STANDOFF_CM} cm)"
  echo "Current lateral error to adjusted suction target: ${LATERAL_ERROR_M} m (${LATERAL_ERROR_CM} cm)"
}

fail_preflight() {
  echo "SAFETY BLOCKED: $1" >&2
  print_timing_summary
  exit 1
}

VALIDATION_START_MS="$(now_ms)"
apply_target_offset
record_timing "apply target offset" "$(( $(now_ms) - VALIDATION_START_MS ))"
VALIDATION_START_MS="$(now_ms)"
print_live_vs_manual_delta
record_timing "live-vs-manual validation" "$(( $(now_ms) - VALIDATION_START_MS ))"
VALIDATION_START_MS="$(now_ms)"
check_current_standoff
record_timing "current standoff validation" "$(( $(now_ms) - VALIDATION_START_MS ))"

PREFLIGHT_START_MS="$(now_ms)"
FINAL_PLUS_MARGIN="$(float_add "${FINAL_STANDOFF_M}" "${FINAL_TARGET_MARGIN_M}")"
if float_lt "${SIGNED_STANDOFF_M}" "${MIN_CURRENT_STANDOFF_M}"; then
  fail_preflight "current standoff ${SIGNED_STANDOFF_M} m is below minimum ${MIN_CURRENT_STANDOFF_M} m."
fi
if float_gt "${SIGNED_STANDOFF_M}" "${MAX_CURRENT_STANDOFF_M}"; then
  fail_preflight "current standoff ${SIGNED_STANDOFF_M} m exceeds maximum ${MAX_CURRENT_STANDOFF_M} m."
fi
if float_le "${SIGNED_STANDOFF_M}" "${FINAL_PLUS_MARGIN}"; then
  fail_preflight "current standoff ${SIGNED_STANDOFF_M} m is already <= final target plus margin ${FINAL_PLUS_MARGIN} m."
fi
echo "Pre-approach lateral error limit: ${MAX_LATERAL_ERROR_M} m"
if float_gt "${LATERAL_ERROR_M}" "${MAX_LATERAL_ERROR_M}"; then
  fail_preflight "lateral error ${LATERAL_ERROR_M} m exceeds limit ${MAX_LATERAL_ERROR_M} m."
fi
record_timing "freshness and pre-approach safety gates" "$(( $(now_ms) - PREFLIGHT_START_MS ))"

EXECUTE_MOTION=false
ALLOW_STANDOFF_EXECUTION=false
if [[ "${EXECUTE}" == "true" ]]; then
  EXECUTE_MOTION=true
  ALLOW_STANDOFF_EXECUTION=true
fi

FINAL_APPROACH_START_MS="$(now_ms)"
set +e
ros2 launch moveit_move contact_box_plan_debug.launch.py \
  planning_mode:=cartesian \
  lock_contact_pose:=true \
  locked_contact_pose_source:=manual_params \
  locked_contact_x:="${ADJUSTED_CONTACT_X}" \
  locked_contact_y:="${ADJUSTED_CONTACT_Y}" \
  locked_contact_z:="${ADJUSTED_CONTACT_Z}" \
  use_tool_offset:=true \
  tool_offset_z:="${TOOL_OFFSET_Z}" \
  use_fixed_orientation:=true \
  target_qx:=0.500 \
  target_qy:=0.490 \
  target_qz:=0.011 \
  target_qw:=0.714 \
  plan_once:=true \
  use_tool_standoff:=true \
  standoff_distance:="${FINAL_STANDOFF_M}" \
  max_wrist_target_distance:="${MAX_WRIST_TARGET_DISTANCE}" \
  max_joint_delta_rad:="${MAX_JOINT_DELTA_RAD}" \
  cartesian_eef_step:="${CARTESIAN_EEF_STEP}" \
  cartesian_jump_threshold:="${CARTESIAN_JUMP_THRESHOLD}" \
  cartesian_min_fraction:="${CARTESIAN_MIN_FRACTION}" \
  velocity_scaling:="${VELOCITY_SCALING}" \
  acceleration_scaling:="${ACCELERATION_SCALING}" \
  exit_after_completion:=true \
  execute_motion:="${EXECUTE_MOTION}" \
  allow_standoff_execution:="${ALLOW_STANDOFF_EXECUTION}"
FINAL_APPROACH_STATUS=$?
set -e
record_timing "final contact approach" "$(( $(now_ms) - FINAL_APPROACH_START_MS ))"
if [[ "${FINAL_APPROACH_STATUS}" -ne 0 ]]; then
  echo "ERROR: final contact approach failed." >&2
  print_timing_summary
  exit "${FINAL_APPROACH_STATUS}"
fi

if [[ "${PRESS}" == "true" ]]; then
  if [[ "${EXECUTE}" == "true" ]]; then
    PRESS_VALIDATION_START_MS="$(now_ms)"
    check_current_standoff
    record_timing "press pre-check standoff validation" "$(( $(now_ms) - PRESS_VALIDATION_START_MS ))"
    echo "WARNING: Press mode performs a slow suction-cup compression move after the default ${FINAL_STANDOFF_M} m target."
    echo "Executing controlled press-in to standoff ${PRESS_TARGET_STANDOFF_M} m."
    echo "Press launch scaling: velocity=${PRESS_VELOCITY_SCALING}, acceleration=${PRESS_ACCELERATION_SCALING}"
    echo "Press Cartesian safety: eef_step=${PRESS_CARTESIAN_EEF_STEP}, jump_threshold=${PRESS_CARTESIAN_JUMP_THRESHOLD}, min_fraction=${PRESS_CARTESIAN_MIN_FRACTION}, max_wrist_target_distance=${PRESS_MAX_WRIST_TARGET_DISTANCE}, max_joint_delta_rad=${PRESS_MAX_JOINT_DELTA_RAD}"
    PRESS_START_MS="$(now_ms)"
    set +e
    ros2 launch moveit_move contact_box_plan_debug.launch.py \
      planning_mode:=cartesian \
      lock_contact_pose:=true \
      locked_contact_pose_source:=manual_params \
      locked_contact_x:="${ADJUSTED_CONTACT_X}" \
      locked_contact_y:="${ADJUSTED_CONTACT_Y}" \
      locked_contact_z:="${ADJUSTED_CONTACT_Z}" \
      use_tool_offset:=true \
      tool_offset_z:="${TOOL_OFFSET_Z}" \
      use_fixed_orientation:=true \
      target_qx:=0.500 \
      target_qy:=0.490 \
      target_qz:=0.011 \
      target_qw:=0.714 \
      plan_once:=true \
      use_tool_standoff:=true \
      standoff_distance:="${PRESS_TARGET_STANDOFF_M}" \
      velocity_scaling:="${PRESS_VELOCITY_SCALING}" \
      acceleration_scaling:="${PRESS_ACCELERATION_SCALING}" \
      cartesian_eef_step:="${PRESS_CARTESIAN_EEF_STEP}" \
      max_wrist_target_distance:="${PRESS_MAX_WRIST_TARGET_DISTANCE}" \
      max_joint_delta_rad:="${PRESS_MAX_JOINT_DELTA_RAD}" \
      cartesian_jump_threshold:="${PRESS_CARTESIAN_JUMP_THRESHOLD}" \
      cartesian_min_fraction:="${PRESS_CARTESIAN_MIN_FRACTION}" \
      exit_after_completion:=true \
      execute_motion:=true \
      allow_standoff_execution:=true
    PRESS_STATUS=$?
    set -e
    record_timing "controlled press-in" "$(( $(now_ms) - PRESS_START_MS ))"
    if [[ "${PRESS_STATUS}" -ne 0 ]]; then
      echo "ERROR: controlled press-in failed." >&2
      print_timing_summary
      exit "${PRESS_STATUS}"
    fi
  else
    echo "PRESS MODE PLAN-ONLY: press target standoff ${PRESS_TARGET_STANDOFF_M} m was computed; no press motion was commanded."
  fi
fi

if [[ "${EXECUTE}" == "true" ]]; then
  FINAL_VERIFICATION_START_MS="$(now_ms)"
  check_current_standoff
  record_timing "post-approach verification checks" "$(( $(now_ms) - FINAL_VERIFICATION_START_MS ))"
  if [[ "${PRESS}" == "true" ]]; then
    echo "DIRECT CONTACT VALIDATION COMPLETE: final commanded standoff ${FINAL_STANDOFF_M} m using adjusted suction target plus press target ${PRESS_TARGET_STANDOFF_M} m"
  else
    echo "DIRECT CONTACT VALIDATION COMPLETE: final commanded standoff ${FINAL_STANDOFF_M} m using adjusted suction target"
  fi
else
  if [[ "${PRESS}" == "true" ]]; then
    echo "DIRECT CONTACT PLAN-ONLY COMPLETE: default target ${FINAL_STANDOFF_M} m using adjusted suction target; press target ${PRESS_TARGET_STANDOFF_M} m was not executed."
  else
    echo "DIRECT CONTACT PLAN-ONLY COMPLETE: final commanded standoff ${FINAL_STANDOFF_M} m using adjusted suction target"
  fi
  echo "No robot motion was commanded."
fi

print_timing_summary
