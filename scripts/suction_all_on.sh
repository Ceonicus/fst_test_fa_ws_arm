#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--execute]"
  echo
  echo "ALL_ON is disabled for this project workflow."
  echo "Use ./scripts/suction_on.sh for suction A / top two cups only."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
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

echo "REFUSED: suction_all_on.sh is disabled."
echo "This project workflow never turns on suction B / bottom cups."
echo "Use ./scripts/suction_on.sh --execute for suction A / top two cups only."
exit 1
