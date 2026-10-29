#!/usr/bin/env bash
set -euo pipefail

# Development-only staging/recovery pose for the contact-box workflow.
# This currently delegates to the old reset_to_36cm.sh saved pose.
# The old name is misleading after correcting tool_offset_z from 0.180 to 0.263:
# this pose is not a true 36 cm contact-box standoff anymore.
# It is not a calibrated deployment pose and is not guaranteed safe from every
# possible robot configuration; use it only when MoveIt can plan a valid safe
# transition to it.

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "Plan-only by default. --execute passes both reset execution gates."
}

ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      ARGS+=(--execute)
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

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"

echo "Resetting to DEVELOPMENT contact-box staging pose"
echo "Note: this currently uses the old reset_to_36cm saved pose; the old name is misleading after TCP correction."
echo "Contact-box reset max joint delta: ${CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD} rad"

"${SCRIPT_DIR}/reset_to_36cm.sh" "${ARGS[@]}"
