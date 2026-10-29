#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "suction_all_off.sh is a backward-compatible safe release alias for suction_off.sh."
echo "It sends EVS08 stop-all/break-vacuum and avoids ActGripper(1,0)."
exec "${SCRIPT_DIR}/suction_off.sh" "$@"
