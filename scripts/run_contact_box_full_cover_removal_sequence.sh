#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --execute"
  echo
  echo "Dry-run by default: prints the full stage order and commands only."
  echo "--execute runs the complete cover-removal workflow."
  echo
  echo "Successful final state:"
  echo "  cover released at shifted cover_mount_place"
  echo "  robot retreated to shifted cover_mount_approach"
  echo "  suction is OFF"
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
LOG_DIR="${WORKSPACE_ROOT}/logs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/contact_box_full_cover_removal_${TIMESTAMP}.log"

mkdir -p "${LOG_DIR}"

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

log_info() {
  printf '%s INFO: %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "${LOG_FILE}"
}

log_error() {
  printf '%s ERROR: %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "${LOG_FILE}" >&2
}

format_command() {
  local part
  for part in "$@"; do
    printf '%q ' "${part}"
  done
}

print_timing_summary() {
  local sequence_elapsed_ms
  local i

  sequence_elapsed_ms="$(( $(now_ms) - SEQUENCE_START_MS ))"

  log_info "Timing summary:"
  for i in "${!TIMING_LABELS[@]}"; do
    log_info "- ${TIMING_LABELS[$i]}: $(format_duration_ms "${TIMING_DURATIONS_MS[$i]}")"
  done
  log_info "- total: $(format_duration_ms "${sequence_elapsed_ms}")"
}

fail_stage() {
  local stage="$1"
  local exit_code="$2"
  log_error "Stage ${stage} failed with exit code ${exit_code}."
  log_error "FULL COVER REMOVAL FAILED at stage: ${stage}"
  print_timing_summary
  log_error "Log file: ${LOG_FILE}"
  exit "${exit_code}"
}

run_stage() {
  local stage="$1"
  shift
  local -a cmd=("$@")
  local start_ms
  local duration_ms
  local status

  log_info "STAGE START: ${stage}"
  log_info "COMMAND: $(format_command "${cmd[@]}")"
  start_ms="$(now_ms)"
  set +e
  "${cmd[@]}" 2>&1 | tee -a "${LOG_FILE}"
  status=${PIPESTATUS[0]}
  set -e
  duration_ms="$(( $(now_ms) - start_ms ))"
  record_timing "${stage}" "${duration_ms}"

  if [[ "${status}" -eq 0 ]]; then
    log_info "STAGE SUCCESS: ${stage} exit_code=${status} duration=$(format_duration_ms "${duration_ms}")"
    return 0
  fi

  log_error "STAGE FAILURE: ${stage} exit_code=${status} duration=$(format_duration_ms "${duration_ms}")"
  fail_stage "${stage}" "${status}"
}

run_stage_with_retry() {
  local stage="$1"
  local max_attempts="$2"
  shift 2
  local -a cmd=("$@")
  local attempt
  local attempt_start_ms
  local attempt_duration_ms
  local stage_start_ms
  local stage_duration_ms
  local status=1

  stage_start_ms="$(now_ms)"
  for ((attempt = 1; attempt <= max_attempts; attempt += 1)); do
    log_info "STAGE START: ${stage} attempt ${attempt}/${max_attempts}"
    log_info "COMMAND: $(format_command "${cmd[@]}")"
    attempt_start_ms="$(now_ms)"
    set +e
    "${cmd[@]}" 2>&1 | tee -a "${LOG_FILE}"
    status=${PIPESTATUS[0]}
    set -e
    attempt_duration_ms="$(( $(now_ms) - attempt_start_ms ))"
    record_timing "${stage} attempt ${attempt}" "${attempt_duration_ms}"

    if [[ "${status}" -eq 0 ]]; then
      stage_duration_ms="$(( $(now_ms) - stage_start_ms ))"
      record_timing "${stage} total" "${stage_duration_ms}"
      log_info "STAGE SUCCESS: ${stage} attempt=${attempt} exit_code=${status} duration=$(format_duration_ms "${stage_duration_ms}")"
      return 0
    fi

    log_error "STAGE FAILURE: ${stage} attempt=${attempt} exit_code=${status} duration=$(format_duration_ms "${attempt_duration_ms}")"
    if [[ "${attempt}" -lt "${max_attempts}" ]]; then
      log_info "Retrying ${stage}; next attempt $((attempt + 1))/${max_attempts}."
    fi
  done

  stage_duration_ms="$(( $(now_ms) - stage_start_ms ))"
  record_timing "${stage} total" "${stage_duration_ms}"
  fail_stage "${stage}" "${status}"
}

print_plan_stage() {
  local stage="$1"
  shift
  local -a cmd=("$@")
  log_info "PLAN STAGE: ${stage}"
  log_info "COMMAND: $(format_command "${cmd[@]}")"
}

require_script() {
  local script_path="$1"
  if [[ ! -x "${script_path}" ]]; then
    log_error "Required script is missing or not executable: ${script_path}"
    exit 2
  fi
}

SUCTION_INIT="${SCRIPT_DIR}/suction_init.sh"
CONTACT_SEQUENCE="${SCRIPT_DIR}/run_contact_box_development_sequence.sh"
SUCTION_ON="${SCRIPT_DIR}/suction_on.sh"
PICKOUT="${SCRIPT_DIR}/run_contact_box_cover_pickout_sequence.sh"
MOVE_TO_MOUNT="${SCRIPT_DIR}/run_contact_box_cover_to_mount_sequence.sh"
SUCTION_OFF="${SCRIPT_DIR}/suction_off.sh"
MOUNT_RETREAT="${SCRIPT_DIR}/run_contact_box_cover_mount_retreat_sequence.sh"

for required_script in \
  "${SUCTION_INIT}" \
  "${CONTACT_SEQUENCE}" \
  "${SUCTION_ON}" \
  "${PICKOUT}" \
  "${MOVE_TO_MOUNT}" \
  "${SUCTION_OFF}" \
  "${MOUNT_RETREAT}"
do
  require_script "${required_script}"
done

log_info "Contact-box full cover-removal log file: ${LOG_FILE}"
log_info "Workspace: ${WORKSPACE_ROOT}"

CONFIG_LOAD_START_MS="$(now_ms)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh" 2>&1 | tee -a "${LOG_FILE}"
record_timing "load config" "$(( $(now_ms) - CONFIG_LOAD_START_MS ))"

if [[ "${EXECUTE}" != "true" ]]; then
  log_info "DRY RUN: no child script will be executed."
  log_info "Run with --execute to run the real hardware workflow."
  log_info "Planned stage order:"
  print_plan_stage "SUCTION_INIT attempt 1/3" "${SUCTION_INIT}" --execute
  print_plan_stage "SUCTION_INIT attempt 2/3 if needed" "${SUCTION_INIT}" --execute
  print_plan_stage "SUCTION_INIT attempt 3/3 if needed" "${SUCTION_INIT}" --execute
  print_plan_stage "CONTACT_APPROACH_PRESS" "${CONTACT_SEQUENCE}" --execute --press
  print_plan_stage "SUCTION_ON" "${SUCTION_ON}" --execute
  print_plan_stage "PICKOUT" "${PICKOUT}" --execute
  print_plan_stage "MOVE_TO_MOUNT" "${MOVE_TO_MOUNT}" --execute
  print_plan_stage "SUCTION_OFF" "${SUCTION_OFF}" --execute
  print_plan_stage "MOUNT_RETREAT" "${MOUNT_RETREAT}" --execute
  log_info "DRY RUN COMPLETE. No robot motion or suction command was sent."
  print_timing_summary
  exit 0
fi

log_info "WARNING: FULL COVER REMOVAL EXECUTION ENABLED."
log_info "Each stage will run synchronously and must succeed before the next stage starts."
log_info "Suction ON will not run until contact approach/press succeeds."
log_info "Suction OFF will not run until the cover reaches shifted cover_mount_place."
log_info "Mount retreat will not run unless suction OFF succeeds."

cd "${WORKSPACE_ROOT}"

run_stage_with_retry "SUCTION_INIT" 3 "${SUCTION_INIT}" --execute
run_stage "CONTACT_APPROACH_PRESS" "${CONTACT_SEQUENCE}" --execute --press
run_stage "SUCTION_ON" "${SUCTION_ON}" --execute
run_stage "PICKOUT" "${PICKOUT}" --execute
run_stage "MOVE_TO_MOUNT" "${MOVE_TO_MOUNT}" --execute
run_stage "SUCTION_OFF" "${SUCTION_OFF}" --execute
run_stage "MOUNT_RETREAT" "${MOUNT_RETREAT}" --execute

log_info "FULL COVER REMOVAL COMPLETE."
log_info "Final state: cover released at shifted cover_mount_place; robot retreated to shifted cover_mount_approach."
log_info "Suction B was not requested by this wrapper; suction scripts remain A/top-cups-only."
print_timing_summary
log_info "Log file: ${LOG_FILE}"
