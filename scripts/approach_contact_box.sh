#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 STANDOFF_M [--execute]"
  echo
  echo "Plan-only by default. --execute passes both MoveIt execution gates."
  echo "Example plan-only: $0 0.050"
  echo "Example execute:   $0 0.050 --execute"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

STANDOFF_DISTANCE="$1"
shift

if [[ ! "${STANDOFF_DISTANCE}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: STANDOFF_M must be a non-negative decimal meter value." >&2
  usage
  exit 2
fi

EXECUTE_MOTION=false
ALLOW_STANDOFF_EXECUTION=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE_MOTION=true
      ALLOW_STANDOFF_EXECUTION=true
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

MAX_WRIST_TARGET_DISTANCE=0.07
MAX_JOINT_DELTA_RAD=0.12
VELOCITY_SCALING=0.005
ACCELERATION_SCALING=0.005

case "${STANDOFF_DISTANCE}" in
  0.075|0.050|0.05)
    MAX_WRIST_TARGET_DISTANCE=0.04
    VELOCITY_SCALING=0.003
    ACCELERATION_SCALING=0.003
    ;;
  0.025)
    MAX_WRIST_TARGET_DISTANCE=0.04
    MAX_JOINT_DELTA_RAD=0.08
    VELOCITY_SCALING=0.003
    ACCELERATION_SCALING=0.003
    ;;
  0.010|0.005|0.002)
    MAX_WRIST_TARGET_DISTANCE=0.02
    MAX_JOINT_DELTA_RAD=0.05
    VELOCITY_SCALING=0.001
    ACCELERATION_SCALING=0.001
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"

CARTESIAN_EEF_STEP=0.001
CARTESIAN_JUMP_THRESHOLD="${CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD}"
CARTESIAN_MIN_FRACTION="${CONTACT_BOX_CARTESIAN_MIN_FRACTION}"
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
if contact_box_config_value_changed CONTACT_BOX_MAX_JOINT_DELTA_RAD; then
  MAX_JOINT_DELTA_RAD="${CONTACT_BOX_MAX_JOINT_DELTA_RAD}"
fi

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if [[ "${EXECUTE_MOTION}" == "true" ]]; then
  echo "WARNING: execution requested. Both execute_motion and allow_standoff_execution will be true."
else
  echo "Plan-only mode. Execution gates remain false."
fi

echo "Contact-box Cartesian approach standoff: ${STANDOFF_DISTANCE} m"
echo "Approach launch scaling: velocity=${VELOCITY_SCALING}, acceleration=${ACCELERATION_SCALING}"
echo "Approach Cartesian safety: eef_step=${CARTESIAN_EEF_STEP}, jump_threshold=${CARTESIAN_JUMP_THRESHOLD}, min_fraction=${CARTESIAN_MIN_FRACTION}, max_wrist_target_distance=${MAX_WRIST_TARGET_DISTANCE}, max_joint_delta_rad=${MAX_JOINT_DELTA_RAD}"

ros2 launch moveit_move contact_box_plan_debug.launch.py \
  planning_mode:=cartesian \
  lock_contact_pose:=true \
  locked_contact_pose_source:=manual_params \
  locked_contact_x:=0.3323 \
  locked_contact_y:=-0.7376 \
  locked_contact_z:=0.6131 \
  use_tool_offset:=true \
  tool_offset_x:=0.0 \
  tool_offset_y:=0.0 \
  tool_offset_z:=0.263 \
  use_fixed_orientation:=true \
  target_qx:=0.500 \
  target_qy:=0.490 \
  target_qz:=0.011 \
  target_qw:=0.714 \
  plan_once:=true \
  use_tool_standoff:=true \
  standoff_distance:="${STANDOFF_DISTANCE}" \
  cartesian_eef_step:="${CARTESIAN_EEF_STEP}" \
  cartesian_jump_threshold:="${CARTESIAN_JUMP_THRESHOLD}" \
  cartesian_min_fraction:="${CARTESIAN_MIN_FRACTION}" \
  max_wrist_target_distance:="${MAX_WRIST_TARGET_DISTANCE}" \
  max_joint_delta_rad:="${MAX_JOINT_DELTA_RAD}" \
  velocity_scaling:="${VELOCITY_SCALING}" \
  acceleration_scaling:="${ACCELERATION_SCALING}" \
  exit_after_completion:=true \
  execute_motion:="${EXECUTE_MOTION}" \
  allow_standoff_execution:="${ALLOW_STANDOFF_EXECUTION}"
