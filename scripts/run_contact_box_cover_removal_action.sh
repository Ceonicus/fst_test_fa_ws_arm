#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --dry-run"
  echo "  $0 --plan-only"
  echo "  $0 --execute"
  echo "  $0 --execute --no-press"
  echo
  echo "Developer wrapper for /contact_box_cover_removal."
  echo "Dry-run by default: sends execute=false, dry_run=true for a fast action plumbing check."
  echo "--plan-only sends execute=false, dry_run=false for full non-motion planning validation."
  echo "--execute sends execute=true, dry_run=false and can run the full hardware sequence."
}

EXECUTE=false
DRY_RUN=true
PRESS=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      EXECUTE=false
      DRY_RUN=true
      ;;
    --plan-only)
      EXECUTE=false
      DRY_RUN=false
      ;;
    --execute)
      EXECUTE=true
      DRY_RUN=false
      ;;
    --press)
      PRESS=true
      ;;
    --no-press)
      PRESS=false
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

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if [[ "${EXECUTE}" == "true" ]]; then
  echo "WARNING: sending a real contact-box cover-removal execute goal."
  echo "The action server will command robot motion and suction if all safety checks pass."
elif [[ "${DRY_RUN}" == "true" ]]; then
  echo "Sending contact-box cover-removal dry-run goal."
  echo "This is a fast action plumbing check only; no robot motion or suction will be commanded."
else
  echo "Sending contact-box cover-removal plan-only goal."
  echo "The action server will load/use config and poses, validate stages, and plan where possible."
  echo "No robot motion or suction will be commanded."
fi

ros2 action send_goal \
  /contact_box_cover_removal \
  moveit_move/action/ContactBoxCoverRemoval \
  "{execute: ${EXECUTE}, press: ${PRESS}, dry_run: ${DRY_RUN}}" \
  --feedback
