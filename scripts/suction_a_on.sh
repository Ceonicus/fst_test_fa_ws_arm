#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "suction_a_on.sh is a backward-compatible alias for suction_on.sh."
echo "Project workflow uses suction A / top two cups only."
exec "${SCRIPT_DIR}/suction_on.sh" "$@"
