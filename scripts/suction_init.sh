#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "Dry-run by default. --execute publishes /initialize_suction and polls read-only suction STATUS."
  echo "This prepares the EVS08 peripheral only; it does not turn suction ON."
}

EXECUTE=false
STATUS_TIMEOUT_SEC=25
STATUS_POLL_INTERVAL_SEC=0.25

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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${EXECUTE}" != "true" ]]; then
  echo "DRY RUN: would publish /initialize_suction std_msgs/msg/Bool '{data: true}'."
  echo "Then would poll /fairino_suction_command command 0 STATUS until suction is prepared."
  echo "No suction setup command was sent. Re-run with --execute to prepare hardware."
  exit 0
fi

cd "${WORKSPACE_ROOT}"
ROS_SETUP_START_MS="$(now_ms)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
record_timing "source ROS workspace setup" "$(( $(now_ms) - ROS_SETUP_START_MS ))"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available. Source ROS 2 Humble and this workspace first." >&2
  print_timing_summary
  exit 1
fi

echo "Checking /initialize_suction subscriber..."
CHECK_START_MS="$(now_ms)"
TOPIC_INFO="$(ros2 topic info /initialize_suction 2>/dev/null || true)"
record_timing "check initialize_suction subscriber" "$(( $(now_ms) - CHECK_START_MS ))"
if grep -Eq "Subscription count:[[:space:]]*[1-9][0-9]*" <<< "${TOPIC_INFO}"; then
  echo "Found /initialize_suction subscriber."
else
  echo "WARNING: /initialize_suction has no visible subscriber. The robot stack may not be ready." >&2
fi

echo "Publishing suction setup request on /initialize_suction."
PUBLISH_START_MS="$(now_ms)"
ros2 topic pub --once /initialize_suction std_msgs/msg/Bool "{data: true}"
record_timing "publish suction setup request" "$(( $(now_ms) - PUBLISH_START_MS ))"

SERVICE_CHECK_START_MS="$(now_ms)"
if ! ros2 service list 2>/dev/null | grep -Fxq "/fairino_suction_command"; then
  record_timing "check suction command service" "$(( $(now_ms) - SERVICE_CHECK_START_MS ))"
  echo "WARNING: /fairino_suction_command is not available, so setup cannot be confirmed." >&2
  echo "Check the robot stack logs before running ./scripts/suction_on.sh --execute." >&2
  print_timing_summary
  exit 1
fi
record_timing "check suction command service" "$(( $(now_ms) - SERVICE_CHECK_START_MS ))"

echo "Polling read-only suction STATUS for up to ${STATUS_TIMEOUT_SEC}s every ${STATUS_POLL_INTERVAL_SEC}s..."
OUTPUT_FILE="$(mktemp /tmp/fairino_suction_status.XXXXXX)"
trap 'rm -f "${OUTPUT_FILE}"' EXIT

POLL_START_MS="$(now_ms)"
POLL_ATTEMPTS=0
deadline=$((SECONDS + STATUS_TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  POLL_ATTEMPTS=$((POLL_ATTEMPTS + 1))
  if timeout 5s ros2 service call /fairino_suction_command fairino_msgs/srv/FairinoSuctionCommand "{command: 0}" > "${OUTPUT_FILE}" 2>&1; then
    if grep -Eq "success[=:][[:space:]]*(True|true)" "${OUTPUT_FILE}"; then
      record_timing "poll suction STATUS (${POLL_ATTEMPTS} attempts)" "$(( $(now_ms) - POLL_START_MS ))"
      cat "${OUTPUT_FILE}"
      echo "Suction setup complete. You may now run ./scripts/suction_on.sh --execute and ./scripts/suction_off.sh --execute."
      print_timing_summary
      exit 0
    fi
  fi
  sleep "${STATUS_POLL_INTERVAL_SEC}"
done
record_timing "poll suction STATUS (${POLL_ATTEMPTS} attempts)" "$(( $(now_ms) - POLL_START_MS ))"

echo "WARNING: suction setup was requested but could not be confirmed by STATUS within ${STATUS_TIMEOUT_SEC}s." >&2
echo "Last STATUS response:" >&2
cat "${OUTPUT_FILE}" >&2 || true
echo "Check the robot stack logs before running ./scripts/suction_on.sh --execute." >&2
print_timing_summary
exit 1
