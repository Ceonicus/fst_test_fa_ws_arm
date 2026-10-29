#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "suction_a_off.sh is a backward-compatible alias for suction_off.sh."
echo "Project workflow release uses EVS08 stop-all/break-vacuum."
exec "${SCRIPT_DIR}/suction_off.sh" "$@"
