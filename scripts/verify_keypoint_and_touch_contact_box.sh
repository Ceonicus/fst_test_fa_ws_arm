#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --execute"
  echo "  $0 --execute --fast-demo"
  echo
  echo "Plan-only by default. --execute performs each stage after its plan-only check."
  echo "--fast-demo is valid only with --execute and skips the separate plan-only launch per stage."
}

EXECUTE=false
FAST_DEMO=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=true
      ;;
    --fast-demo)
      FAST_DEMO=true
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

if [[ "${FAST_DEMO}" == "true" && "${EXECUTE}" != "true" ]]; then
  echo "ERROR: --fast-demo is only valid together with --execute." >&2
  usage
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE=/tmp/contact_box_locked_pose.env

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"

TOOL_OFFSET_Z=0.263
SKIP_MARGIN_M=0.001
CONTACT_SAMPLE_COUNT="${CONTACT_BOX_POSE_SAMPLE_COUNT}"
CONTACT_SAMPLE_TIMEOUT_SEC="${CONTACT_BOX_POSE_SAMPLE_TIMEOUT_SEC}"
CONTACT_MAX_SPREAD_M="${CONTACT_BOX_POSE_MAX_SPREAD_M}"
DEVELOPMENT_MANUAL_CONTACT_X=0.3323
DEVELOPMENT_MANUAL_CONTACT_Y=-0.7376
DEVELOPMENT_MANUAL_CONTACT_Z=0.6131
MAX_LIVE_VS_MANUAL_DELTA_M="${CONTACT_BOX_MAX_LIVE_VS_MANUAL_DELTA_M}"
CARTESIAN_JUMP_THRESHOLD="${CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD}"
CARTESIAN_MIN_FRACTION="${CONTACT_BOX_CARTESIAN_MIN_FRACTION}"

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if [[ "${EXECUTE}" == "true" ]]; then
  if [[ "${FAST_DEMO}" == "true" ]]; then
    echo "CONTACT VALIDATION: fast demo execution enabled."
    echo "Each stage still plans, safety-checks, retimes, and executes inside contact_box_plan_debug."
  else
    echo "CONTACT VALIDATION: execution enabled after each successful plan-only stage."
  fi
else
  echo "CONTACT VALIDATION: plan-only mode. No robot motion will be commanded."
fi

fail_contact_acquisition() {
  echo "Contact-box keypoint/contact pose acquisition failed. Refusing to move." >&2
  exit 1
}

acquire_locked_contact_pose() {
  rm -f -- "${ENV_FILE}"
  unset LOCKED_CONTACT_X LOCKED_CONTACT_Y LOCKED_CONTACT_Z
  unset CONTACT_POSE_MAX_DEVIATION CONTACT_POSE_ACQUIRED_AT_UNIX

  if ! "${SCRIPT_DIR}/acquire_contact_pose.py" \
    --samples "${CONTACT_SAMPLE_COUNT}" \
    --timeout "${CONTACT_SAMPLE_TIMEOUT_SEC}" \
    --max-spread "${CONTACT_MAX_SPREAD_M}" \
    --output "${ENV_FILE}"
  then
    fail_contact_acquisition
  fi

  if [[ ! -f "${ENV_FILE}" ]]; then
    fail_contact_acquisition
  fi

  # shellcheck disable=SC1090
  if ! source "${ENV_FILE}"; then
    fail_contact_acquisition
  fi

  if [[ -z "${LOCKED_CONTACT_X:-}" || -z "${LOCKED_CONTACT_Y:-}" || -z "${LOCKED_CONTACT_Z:-}" ]]; then
    fail_contact_acquisition
  fi
}

acquire_locked_contact_pose

echo "Raw locked contact pose from fresh live YOLO/depth acquisition:"
echo "  x=${LOCKED_CONTACT_X}"
echo "  y=${LOCKED_CONTACT_Y}"
echo "  z=${LOCKED_CONTACT_Z}"
echo "  max_deviation=${CONTACT_POSE_MAX_DEVIATION}"
if [[ -n "${CONTACT_POSE_ACQUIRED_AT_UNIX:-}" ]]; then
  echo "  acquired_at_unix=${CONTACT_POSE_ACQUIRED_AT_UNIX}"
fi

float_le() {
  python3 - "$1" "$2" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) <= float(sys.argv[2]) else 1)
PY
}

float_add() {
  python3 - "$1" "$2" <<'PY'
import sys
print(f"{float(sys.argv[1]) + float(sys.argv[2]):.9f}")
PY
}

float_gt() {
  python3 - "$1" "$2" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)
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

stage_limits() {
  case "$1" in
    0.300|0.250|0.200|0.150|0.100)
      MAX_WRIST_TARGET_DISTANCE=0.07
      MAX_JOINT_DELTA_RAD=0.12
      if [[ "${FAST_DEMO}" == "true" ]]; then
        VELOCITY_SCALING=0.008
        ACCELERATION_SCALING=0.008
      else
        VELOCITY_SCALING=0.005
        ACCELERATION_SCALING=0.005
      fi
      CARTESIAN_EEF_STEP=0.001
      ;;
    0.075|0.050)
      MAX_WRIST_TARGET_DISTANCE=0.04
      MAX_JOINT_DELTA_RAD=0.12
      if [[ "${FAST_DEMO}" == "true" ]]; then
        VELOCITY_SCALING=0.005
        ACCELERATION_SCALING=0.005
      else
        VELOCITY_SCALING=0.003
        ACCELERATION_SCALING=0.003
      fi
      CARTESIAN_EEF_STEP=0.001
      ;;
    0.025)
      MAX_WRIST_TARGET_DISTANCE=0.04
      MAX_JOINT_DELTA_RAD=0.08
      VELOCITY_SCALING=0.003
      ACCELERATION_SCALING=0.003
      CARTESIAN_EEF_STEP=0.0005
      ;;
    0.010|0.005|0.002)
      MAX_WRIST_TARGET_DISTANCE=0.02
      MAX_JOINT_DELTA_RAD=0.05
      if [[ "${FAST_DEMO}" == "true" ]]; then
        VELOCITY_SCALING=0.002
        ACCELERATION_SCALING=0.002
      else
        VELOCITY_SCALING=0.001
        ACCELERATION_SCALING=0.001
      fi
      CARTESIAN_EEF_STEP=0.0005
      ;;
    *)
      echo "ERROR: no safety limits configured for stage '$1'" >&2
      exit 2
      ;;
  esac

  if contact_box_config_value_changed CONTACT_BOX_APPROACH_VELOCITY_SCALING; then
    VELOCITY_SCALING="${CONTACT_BOX_APPROACH_VELOCITY_SCALING}"
  fi
  if contact_box_config_value_changed CONTACT_BOX_APPROACH_ACCELERATION_SCALING; then
    ACCELERATION_SCALING="${CONTACT_BOX_APPROACH_ACCELERATION_SCALING}"
  fi
  if contact_box_config_value_changed CONTACT_BOX_CARTESIAN_EEF_STEP; then
    CARTESIAN_EEF_STEP="${CONTACT_BOX_CARTESIAN_EEF_STEP}"
  fi
  if contact_box_config_value_changed CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE; then
    MAX_WRIST_TARGET_DISTANCE="${CONTACT_BOX_MAX_WRIST_TARGET_DISTANCE}"
  fi
  if contact_box_config_value_changed CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD; then
    MAX_JOINT_DELTA_RAD="${CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD}"
  fi
}

run_stage() {
  local standoff="$1"
  local execute_motion="$2"
  local allow_standoff_execution="$2"

  stage_limits "${standoff}"
  if [[ "${execute_motion}" == "true" ]]; then
    echo "Executing validated Cartesian stage to ${standoff} m"
  else
    echo "Planning validated Cartesian stage to ${standoff} m"
  fi

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
    standoff_distance:="${standoff}" \
    cartesian_jump_threshold:="${CARTESIAN_JUMP_THRESHOLD}" \
    cartesian_min_fraction:="${CARTESIAN_MIN_FRACTION}" \
    cartesian_eef_step:="${CARTESIAN_EEF_STEP}" \
    max_wrist_target_distance:="${MAX_WRIST_TARGET_DISTANCE}" \
    max_joint_delta_rad:="${MAX_JOINT_DELTA_RAD}" \
    velocity_scaling:="${VELOCITY_SCALING}" \
    acceleration_scaling:="${ACCELERATION_SCALING}" \
    exit_after_completion:=true \
    execute_motion:="${execute_motion}" \
    allow_standoff_execution:="${allow_standoff_execution}"
}

apply_target_offset
print_live_vs_manual_delta
check_current_standoff

STAGES=(0.300 0.250 0.200 0.150 0.100 0.075 0.050 0.025 0.010 0.005 0.002)
ran_stage=false
final_stage_commanded=false

for stage in "${STAGES[@]}"; do
  threshold="$(float_add "${stage}" "${SKIP_MARGIN_M}")"
  if float_le "${SIGNED_STANDOFF_M}" "${threshold}"; then
    echo "Skipping ${stage} m; current standoff ${SIGNED_STANDOFF_M} m is already <= ${threshold} m."
    continue
  fi

  if [[ "${EXECUTE}" != "true" ]]; then
    run_stage "${stage}" false
    ran_stage=true
    echo "Plan-only mode complete for next forward stage ${stage} m."
    echo "No robot motion was commanded."
    exit 0
  fi

  if [[ "${FAST_DEMO}" != "true" ]]; then
    run_stage "${stage}" false
  else
    echo "Fast demo mode: skipping separate plan-only launch for ${stage} m."
  fi
  ran_stage=true

  run_stage "${stage}" true
  if [[ "${stage}" == "0.002" ]]; then
    final_stage_commanded=true
  fi
  check_current_standoff
done

if [[ "${EXECUTE}" == "true" && "${final_stage_commanded}" == "true" ]]; then
  echo "CONTACT VALIDATION COMPLETE: final commanded standoff 0.002 m using adjusted suction target"
elif [[ "${ran_stage}" == "false" ]]; then
  echo "No forward stage was needed from the current standoff."
  echo "CONTACT VALIDATION COMPLETE: final commanded standoff 0.002 m using adjusted suction target"
else
  echo "CONTACT VALIDATION COMPLETE: final commanded standoff 0.002 m using adjusted suction target"
fi
