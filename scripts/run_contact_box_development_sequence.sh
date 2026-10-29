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
  echo "Plan-only by default. --execute runs the full development motion sequence."
  echo "--fast is forwarded only to the direct contact approach and is valid only with --execute."
  echo "--press and --press-depth-m are forwarded only to the final direct contact approach."
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
ENV_FILE=/tmp/contact_box_locked_pose.env
EXECUTE_ARGS=()
DIRECT_ARGS=()

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

run_timed_step() {
  local label="$1"
  local failure_message="$2"
  shift 2
  local start_ms
  local duration_ms
  local status

  start_ms="$(now_ms)"
  set +e
  "$@"
  status=$?
  set -e
  duration_ms="$(( $(now_ms) - start_ms ))"
  record_timing "${label}" "${duration_ms}"

  if [[ "${status}" -eq 0 ]]; then
    echo "TIMING: ${label}: $(format_duration_ms "${duration_ms}")"
    return 0
  fi

  echo "ERROR: ${failure_message}" >&2
  echo "TIMING: ${label} failed after $(format_duration_ms "${duration_ms}")" >&2
  print_timing_summary
  exit "${status}"
}

CONFIG_LOAD_START_MS="$(now_ms)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"
record_timing "load config" "$(( $(now_ms) - CONFIG_LOAD_START_MS ))"

if [[ "${EXECUTE}" == "true" ]]; then
  EXECUTE_ARGS+=(--execute)
  DIRECT_ARGS+=(--execute)
fi
if [[ "${FAST}" == "true" ]]; then
  DIRECT_ARGS+=(--fast)
fi
if [[ "${PRESS}" == "true" ]]; then
  if [[ -n "${PRESS_DEPTH_M}" ]]; then
    DIRECT_ARGS+=(--press-depth-m "${PRESS_DEPTH_M}")
  else
    DIRECT_ARGS+=(--press)
  fi
fi

STALE_CLEAR_START_MS="$(now_ms)"
echo "Clearing stale locked contact pose env: ${ENV_FILE}"
rm -f -- "${ENV_FILE}"
record_timing "clear stale locked pose" "$(( $(now_ms) - STALE_CLEAR_START_MS ))"

echo
echo "========== Step 1: Development contact-box staging pose =========="
run_timed_step \
  "move to staging pose" \
  "Failed to reach contact-box staging pose. Refusing to continue." \
  "${SCRIPT_DIR}/reset_to_contact_box_staging_pose.sh" "${EXECUTE_ARGS[@]}"

echo
echo "========== Step 2: True 30 cm contact-box ready pose =========="
run_timed_step \
  "move to true 30 cm ready pose" \
  "Failed to reach true 30 cm contact-box ready pose. Refusing final approach." \
  "${SCRIPT_DIR}/reset_to_30cm.sh" "${EXECUTE_ARGS[@]}"

echo
echo "========== Step 3: Direct one-shot keypoint-to-contact approach =========="
echo "Fresh contact-box pose acquisition happens now, from the true 30 cm ready pose."
STALE_CLEAR_START_MS="$(now_ms)"
rm -f -- "${ENV_FILE}"
record_timing "clear pre-approach locked pose" "$(( $(now_ms) - STALE_CLEAR_START_MS ))"
run_timed_step \
  "fresh pose validation and final approach" \
  "Direct one-shot keypoint-to-contact approach failed." \
  "${SCRIPT_DIR}/verify_keypoint_and_touch_contact_box_direct.sh" "${DIRECT_ARGS[@]}"

echo
echo "Contact-box development sequence complete."
print_timing_summary
