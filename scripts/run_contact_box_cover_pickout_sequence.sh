#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --execute"
  echo
  echo "Plan-only by default. --execute performs only the cover pickup-out motion."
  echo "Preconditions:"
  echo "  1. Robot is already at the contact-box press/contact pose."
  echo "  2. Suction A/top cups are already ON from ./scripts/suction_on.sh --execute."
  echo "Postcondition: cover is lifted/pulled clear, suction remains ON."
  echo "This script does not move to cover_mount_approach/place and does not release suction."
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
CONFIG_FILE="${WORKSPACE_ROOT}/config/contact_box_motion.env"

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

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "ERROR: missing contact-box motion config: ${CONFIG_FILE}" >&2
  exit 2
fi

CONFIG_LOAD_START_MS="$(now_ms)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"
record_timing "load config" "$(( $(now_ms) - CONFIG_LOAD_START_MS ))"

validate_pickout_config() {
  python3 - \
    "${CONTACT_BOX_COVER_LIFT_Z_M}" \
    "${CONTACT_BOX_COVER_PULL_OUT_AXIS}" \
    "${CONTACT_BOX_COVER_PULL_OUT_M}" <<'PY'
import math
import sys

lift_raw, axis, pull_raw = sys.argv[1:4]

try:
    lift = float(lift_raw)
except ValueError:
    print(f"ERROR: CONTACT_BOX_COVER_LIFT_Z_M must be numeric, got {lift_raw!r}", file=sys.stderr)
    sys.exit(2)
if not math.isfinite(lift) or lift <= 0.0 or lift > 0.050:
    print("ERROR: CONTACT_BOX_COVER_LIFT_Z_M must be > 0 and <= 0.050 m.", file=sys.stderr)
    sys.exit(2)

if axis != "opposite_tool_axis":
    print(
        "ERROR: CONTACT_BOX_COVER_PULL_OUT_AXIS must be 'opposite_tool_axis'. "
        "Pull-out direction is not known; refusing to move.",
        file=sys.stderr,
    )
    sys.exit(2)

if pull_raw == "":
    print(
        "ERROR: CONTACT_BOX_COVER_PULL_OUT_M is unset. Calibrate the pull-out "
        "distance in config/contact_box_motion.env before planning or executing pickup-out.",
        file=sys.stderr,
    )
    sys.exit(2)
try:
    pull = float(pull_raw)
except ValueError:
    print(f"ERROR: CONTACT_BOX_COVER_PULL_OUT_M must be numeric, got {pull_raw!r}", file=sys.stderr)
    sys.exit(2)
if not math.isfinite(pull) or pull <= 0.0 or pull > 0.250:
    print("ERROR: CONTACT_BOX_COVER_PULL_OUT_M must be > 0 and <= 0.250 m.", file=sys.stderr)
    sys.exit(2)
PY
}

VALIDATE_CONFIG_START_MS="$(now_ms)"
validate_pickout_config
record_timing "validate pickup-out config" "$(( $(now_ms) - VALIDATE_CONFIG_START_MS ))"

cd "${WORKSPACE_ROOT}"
ROS_SETUP_START_MS="$(now_ms)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
record_timing "source ROS workspace setup" "$(( $(now_ms) - ROS_SETUP_START_MS ))"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available after sourcing ROS/workspace setup." >&2
  exit 1
fi

EXECUTE_MOTION=false
ALLOW_STANDOFF_EXECUTION=false
if [[ "${EXECUTE}" == "true" ]]; then
  EXECUTE_MOTION=true
  ALLOW_STANDOFF_EXECUTION=true
  echo "WARNING: COVER PICKUP-OUT EXECUTION ENABLED."
else
  echo "COVER PICKUP-OUT: plan-only mode. No robot motion will be commanded."
fi

echo "Precondition: robot is already at the contact-box press/contact pose."
echo "Precondition: suction A/top cups are already ON from ./scripts/suction_on.sh --execute."
echo "This script will not command suction, will not release suction, and will not move to cover_mount_approach/place."
echo "Lift in base_link +Z: ${CONTACT_BOX_COVER_LIFT_Z_M} m"
echo "Pull-out axis: ${CONTACT_BOX_COVER_PULL_OUT_AXIS}"
echo "Pull-out distance: ${CONTACT_BOX_COVER_PULL_OUT_M} m"

TARGET_COMPUTE_START_MS="$(now_ms)"
TARGET_ENV="$(
  python3 - \
    "${CONTACT_BOX_COVER_LIFT_Z_M}" \
    "${CONTACT_BOX_COVER_PULL_OUT_M}" <<'PY'
import math
import sys

import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener


def normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise RuntimeError("invalid wrist3_link quaternion from TF")
    return tuple(v / norm for v in q)


def rotate_tool_z(q):
    qx, qy, qz, qw = q
    vx, vy, vz = 0.0, 0.0, 1.0
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def wait_for_transform(node, buffer):
    deadline = node.get_clock().now() + Duration(seconds=5.0)
    last_error = None
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            return buffer.lookup_transform("base_link", "wrist3_link", rclpy.time.Time())
        except TransformException as exc:
            last_error = exc
    raise RuntimeError(f"failed to lookup TF base_link -> wrist3_link: {last_error}")


lift = float(sys.argv[1])
pull = float(sys.argv[2])

rclpy.init()
node = rclpy.create_node("contact_box_cover_pickout_target")
buffer = Buffer()
TransformListener(buffer, node)

try:
    transform = wait_for_transform(node, buffer)
    t = transform.transform.translation
    r = transform.transform.rotation
    q = normalize((r.x, r.y, r.z, r.w))
    tool_axis = rotate_tool_z(q)

    current = (t.x, t.y, t.z)
    lift_target = (current[0], current[1], current[2] + lift)
    pull_target = (
        lift_target[0] - tool_axis[0] * pull,
        lift_target[1] - tool_axis[1] * pull,
        lift_target[2] - tool_axis[2] * pull,
    )
    pull_total = math.sqrt(
        (pull_target[0] - current[0]) ** 2
        + (pull_target[1] - current[1]) ** 2
        + (pull_target[2] - current[2]) ** 2
    )

    values = {
        "CURRENT_WRIST_X": current[0],
        "CURRENT_WRIST_Y": current[1],
        "CURRENT_WRIST_Z": current[2],
        "CURRENT_WRIST_QX": q[0],
        "CURRENT_WRIST_QY": q[1],
        "CURRENT_WRIST_QZ": q[2],
        "CURRENT_WRIST_QW": q[3],
        "TOOL_AXIS_X": tool_axis[0],
        "TOOL_AXIS_Y": tool_axis[1],
        "TOOL_AXIS_Z": tool_axis[2],
        "LIFT_TARGET_X": lift_target[0],
        "LIFT_TARGET_Y": lift_target[1],
        "LIFT_TARGET_Z": lift_target[2],
        "PULL_TARGET_X": pull_target[0],
        "PULL_TARGET_Y": pull_target[1],
        "PULL_TARGET_Z": pull_target[2],
        "LIFT_MAX_WRIST_TARGET_DISTANCE": lift + 0.010,
        "PULL_MAX_WRIST_TARGET_DISTANCE": pull_total + 0.010,
    }
    for key, value in values.items():
        print(f"{key}={value:.9f}")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
)"
record_timing "compute pickup-out targets" "$(( $(now_ms) - TARGET_COMPUTE_START_MS ))"

eval "${TARGET_ENV}"

echo "Current wrist3_link:"
echo "  position=(${CURRENT_WRIST_X}, ${CURRENT_WRIST_Y}, ${CURRENT_WRIST_Z})"
echo "  qxyzw=(${CURRENT_WRIST_QX}, ${CURRENT_WRIST_QY}, ${CURRENT_WRIST_QZ}, ${CURRENT_WRIST_QW})"
echo "Current tool +Z / contact approach direction:"
echo "  (${TOOL_AXIS_X}, ${TOOL_AXIS_Y}, ${TOOL_AXIS_Z})"
echo "Pickup-out targets:"
echo "  lift target=(${LIFT_TARGET_X}, ${LIFT_TARGET_Y}, ${LIFT_TARGET_Z})"
echo "  pull target=(${PULL_TARGET_X}, ${PULL_TARGET_Y}, ${PULL_TARGET_Z})"

run_cartesian_wrist_target() {
  local label="$1"
  local target_x="$2"
  local target_y="$3"
  local target_z="$4"
  local max_wrist_target_distance="$5"

  echo
  echo "========== ${label} =========="
  echo "Target wrist3_link position: (${target_x}, ${target_y}, ${target_z})"
  echo "Max wrist target distance: ${max_wrist_target_distance} m"

  local start_ms
  local duration_ms
  local status
  start_ms="$(now_ms)"
  set +e
  ros2 launch moveit_move contact_box_plan_debug.launch.py \
    planning_mode:=cartesian \
    lock_contact_pose:=true \
    locked_contact_pose_source:=manual_params \
    locked_contact_x:="${target_x}" \
    locked_contact_y:="${target_y}" \
    locked_contact_z:="${target_z}" \
    use_tool_offset:=false \
    tool_offset_x:=0.0 \
    tool_offset_y:=0.0 \
    tool_offset_z:=0.0 \
    use_fixed_orientation:=true \
    target_qx:="${CURRENT_WRIST_QX}" \
    target_qy:="${CURRENT_WRIST_QY}" \
    target_qz:="${CURRENT_WRIST_QZ}" \
    target_qw:="${CURRENT_WRIST_QW}" \
    plan_once:=true \
    use_tool_standoff:=false \
    standoff_distance:=0.0 \
    cartesian_eef_step:="${CONTACT_BOX_CARTESIAN_EEF_STEP}" \
    cartesian_jump_threshold:="${CONTACT_BOX_CARTESIAN_JUMP_THRESHOLD}" \
    cartesian_min_fraction:="${CONTACT_BOX_CARTESIAN_MIN_FRACTION}" \
    max_wrist_target_distance:="${max_wrist_target_distance}" \
    max_joint_delta_rad:="${CONTACT_BOX_APPROACH_MAX_JOINT_DELTA_RAD}" \
    velocity_scaling:="${CONTACT_BOX_APPROACH_VELOCITY_SCALING}" \
    acceleration_scaling:="${CONTACT_BOX_APPROACH_ACCELERATION_SCALING}" \
    exit_after_completion:=true \
    execute_motion:="${EXECUTE_MOTION}" \
    allow_standoff_execution:="${ALLOW_STANDOFF_EXECUTION}"
  status=$?
  set -e
  duration_ms="$(( $(now_ms) - start_ms ))"
  record_timing "${label}" "${duration_ms}"
  echo "TIMING: ${label}: $(format_duration_ms "${duration_ms}")"
  return "${status}"
}

if ! run_cartesian_wrist_target \
  "Step 1: lift cover in base_link +Z" \
  "${LIFT_TARGET_X}" "${LIFT_TARGET_Y}" "${LIFT_TARGET_Z}" \
  "${LIFT_MAX_WRIST_TARGET_DISTANCE}"
then
  echo "ERROR: cover lift step failed. Refusing to continue to pull-out." >&2
  print_timing_summary
  exit 1
fi

if ! run_cartesian_wrist_target \
  "Step 2: pull cover out opposite contact-box approach direction" \
  "${PULL_TARGET_X}" "${PULL_TARGET_Y}" "${PULL_TARGET_Z}" \
  "${PULL_MAX_WRIST_TARGET_DISTANCE}"
then
  echo "ERROR: cover pull-out step failed." >&2
  print_timing_summary
  exit 1
fi

echo
echo "COVER PICKUP-OUT SEQUENCE COMPLETE."
echo "Postcondition: cover should be lifted and pulled clear of the contact box."
echo "Suction is still ON. No suction release was commanded."
echo "This script did not move to cover_mount_approach or cover_mount_place."
print_timing_summary
