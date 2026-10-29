#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FAILURES=0
EXPECTED_SERVICE_TYPE="fairino_msgs/srv/FairinoSuctionCommand"

cd "${WORKSPACE_ROOT}"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

pass() {
  echo "PASS: $1"
}

fail() {
  echo "FAIL: $1" >&2
  FAILURES=$((FAILURES + 1))
}

echo "Checking FAIRINO integrated suction interface."
echo "This script does not move the robot and does not command suction/gripper hardware."
echo "/initialize_suction is explicit suction preparation, not suction ON."
echo

if ros2 topic list 2>/dev/null | grep -Fxq "/initialize_suction"; then
  pass "/initialize_suction topic is present"
else
  fail "/initialize_suction topic is missing"
fi

if ros2 service list 2>/dev/null | grep -Fxq "/fairino_suction_command"; then
  pass "/fairino_suction_command service is present"
else
  fail "/fairino_suction_command service is missing"
fi

SERVICE_TYPE="$(ros2 service type /fairino_suction_command 2>/dev/null || true)"
if [[ "${SERVICE_TYPE}" == "${EXPECTED_SERVICE_TYPE}" ]]; then
  pass "/fairino_suction_command service type is ${EXPECTED_SERVICE_TYPE}"
else
  fail "/fairino_suction_command service type mismatch: expected ${EXPECTED_SERVICE_TYPE}, got '${SERVICE_TYPE:-none}'"
fi

echo
if [[ "${FAILURES}" -eq 0 ]]; then
  echo "SUCTION INTERFACE CHECK PASS"
  exit 0
fi

echo "SUCTION INTERFACE CHECK FAIL: ${FAILURES} issue(s)" >&2
exit 1
