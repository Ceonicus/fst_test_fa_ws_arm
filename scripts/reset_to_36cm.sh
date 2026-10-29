#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "Plan-only by default. --execute passes both reset execution gates."
}

EXECUTE_MOTION=false
ALLOW_RESET_EXECUTION=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE_MOTION=true
      ALLOW_RESET_EXECUTION=true
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if [[ "${EXECUTE_MOTION}" == "true" ]]; then
  echo "WARNING: reset execution requested. Both execute_motion and allow_reset_execution will be true."
else
  echo "Plan-only reset. Execution gates remain false."
fi
echo "Contact-box reset max joint delta: ${CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD} rad"

ros2 launch moveit_move return_to_saved_joint_pose.launch.py \
  velocity_scaling:="${CONTACT_BOX_RESET_VELOCITY_SCALING}" \
  acceleration_scaling:="${CONTACT_BOX_RESET_ACCELERATION_SCALING}" \
  max_joint_delta_rad:="${CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD}" \
  execute_motion:="${EXECUTE_MOTION}" \
  allow_reset_execution:="${ALLOW_RESET_EXECUTION}"
