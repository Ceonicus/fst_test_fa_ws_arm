#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "Dry-run by default. The old repeated suction cycle is disabled."
  echo "OFF now uses EVS08 full-state MoveGripper release, but repeated cycling still requires manual validation."
}

EXECUTE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=true
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

echo "No real suction/grip feedback is currently available; only service success and software state are reported."
echo
echo "Checking suction command service before cycle."
"${SCRIPT_DIR}/check_suction_interface.sh"

if [[ "${EXECUTE}" != "true" ]]; then
  echo "DRY RUN: old B_OFF -> B_ON -> B_OFF cycle is disabled."
  echo "Manual validation sequence is: suction_init.sh --execute, suction_on.sh --execute, then suction_off.sh --execute."
  echo "No suction command was sent. --execute is disabled for this cycle helper."
  exit 0
fi

echo "ERROR: repeated suction cycle execution is disabled." >&2
echo "Use the single-command suction helpers for manually approved EVS08 validation." >&2
exit 2
