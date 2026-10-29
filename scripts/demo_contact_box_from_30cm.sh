#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--reset-first] [--execute]"
  echo
  echo "Plan-only by default. --execute is forwarded to reset/demo stages."
}

RESET_FIRST=false
EXECUTE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --reset-first)
      RESET_FIRST=true
      ;;
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
ARGS=()
if [[ "${EXECUTE}" == "true" ]]; then
  ARGS+=(--execute)
fi

if [[ "${RESET_FIRST}" == "true" ]]; then
  "${SCRIPT_DIR}/reset_to_30cm.sh" "${ARGS[@]}"
fi

"${SCRIPT_DIR}/verify_keypoint_and_touch_contact_box.sh" "${ARGS[@]}"
