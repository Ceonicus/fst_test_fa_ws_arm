#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "Dry-run by default. --execute sends EVS08 stop-all/break-vacuum through /fairino_suction_command."
  echo "Commanded EVS08 state: MoveGripper(1,0,70,60,6000,1,1,0,70,60)."
  echo "It intentionally avoids ActGripper(1,0) for normal release."
}

EXECUTE=false

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

if [[ "${EXECUTE}" != "true" ]]; then
  echo "DRY RUN: would release suction using EVS08 stop-all/break-vacuum."
  echo "Would force suction A/top cups OFF and suction B/bottom cups OFF."
  echo "Would send MoveGripper(1,0,70,60,6000,1,1,0,70,60) through command 6."
  echo "No suction command was sent. Re-run with --execute to command hardware."
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_ROOT}"
ROS_SETUP_START_MS="$(now_ms)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
record_timing "source ROS workspace setup" "$(( $(now_ms) - ROS_SETUP_START_MS ))"

echo "Executing suction OFF / EVS08 stop-all command 6 through /fairino_suction_command."
echo "This does not command robot motion."
echo "This sends MoveGripper(1,0,70,60,6000,1,1,0,70,60) and avoids ActGripper(1,0)."
OUTPUT_FILE="$(mktemp /tmp/fairino_suction_off.XXXXXX)"
trap 'rm -f "${OUTPUT_FILE}"' EXIT

SERVICE_CALL_START_MS="$(now_ms)"
set +e
ros2 service call /fairino_suction_command fairino_msgs/srv/FairinoSuctionCommand "{command: 6}" | tee "${OUTPUT_FILE}"
SERVICE_CALL_STATUS=${PIPESTATUS[0]}
set -e
record_timing "suction OFF service call" "$(( $(now_ms) - SERVICE_CALL_START_MS ))"
if [[ "${SERVICE_CALL_STATUS}" -ne 0 ]]; then
  echo "ERROR: ros2 service call failed for suction OFF / EVS08 stop-all." >&2
  echo "WARNING: stop suction testing and recover/release manually if suction may still be ON." >&2
  print_timing_summary
  exit 1
fi

if grep -Eq "success[=:][[:space:]]*(False|false)" "${OUTPUT_FILE}"; then
  echo "ERROR: suction OFF / EVS08 stop-all service returned success=false." >&2
  echo "WARNING: stop suction testing and recover/release manually; software/hardware suction state may be ambiguous." >&2
  print_timing_summary
  exit 1
fi

if ! grep -Eq "success[=:][[:space:]]*(True|true)" "${OUTPUT_FILE}"; then
  echo "ERROR: could not confirm success=true in suction OFF / EVS08 stop-all service response." >&2
  echo "WARNING: stop suction testing and recover/release manually if suction may still be ON." >&2
  print_timing_summary
  exit 1
fi

print_timing_summary
