#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage:"
  echo "  $0"
  echo "  $0 --execute"
  echo
  echo "Plan-only by default. --execute performs the cover-to-mount motion."
  echo "Preconditions:"
  echo "  1. ./scripts/run_contact_box_cover_pickout_sequence.sh --execute has completed."
  echo "  2. The cover is lifted/pulled clear of the contact box."
  echo "  3. Suction A/top cups are still ON and the robot is holding the cover."
  echo "Postcondition: cover is at cover_mount_place, suction remains ON."
  echo "This script does not call suction_off.sh and does not release suction."
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
RECORDED_POSES_FILE="${WORKSPACE_ROOT}/config/recorded_robot_poses.json"

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

if [[ ! -f "${RECORDED_POSES_FILE}" ]]; then
  echo "ERROR: missing recorded poses file: ${RECORDED_POSES_FILE}" >&2
  exit 2
fi

CONFIG_LOAD_START_MS="$(now_ms)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_contact_box_motion_config.sh"
record_timing "load config" "$(( $(now_ms) - CONFIG_LOAD_START_MS ))"

POSE_LOAD_START_MS="$(now_ms)"
POSE_ENV="$(
  python3 - \
    "${RECORDED_POSES_FILE}" \
    "${CONTACT_BOX_COVER_MOUNT_OFFSET_X_M}" \
    "${CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M}" \
    "${CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M}" <<'PY'
import json
import math
import sys


def fail(message):
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(2)


def require_finite(value, path):
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        fail(f"{path} must be a finite numeric value")
    return float(value)


def require_pose(data, name):
    poses = data.get("poses")
    if not isinstance(poses, dict):
        fail("recorded_robot_poses.json must contain object field 'poses'")
    pose = poses.get(name)
    if not isinstance(pose, dict):
        fail(f"missing recorded pose '{name}'")
    return pose


def validate_joint_state(pose, name):
    joint_state = pose.get("joint_state")
    if not isinstance(joint_state, dict):
        fail(f"{name}.joint_state must be an object")
    names = joint_state.get("names")
    positions = joint_state.get("positions")
    if not isinstance(names, list) or not isinstance(positions, list):
        fail(f"{name}.joint_state.names and positions must be lists")
    if len(names) != len(positions):
        fail(f"{name}.joint_state.names and positions lengths differ")
    by_name = {}
    for index, joint_name in enumerate(names):
        if not isinstance(joint_name, str):
            fail(f"{name}.joint_state.names[{index}] must be a string")
        by_name[joint_name] = require_finite(
            positions[index], f"{name}.joint_state.positions[{index}]"
        )
    missing = [joint for joint in ("j1", "j2", "j3", "j4", "j5", "j6") if joint not in by_name]
    if missing:
        fail(f"{name}.joint_state is missing joints: {', '.join(missing)}")
    return by_name


def validate_tool_pose(pose, name):
    tool_pose = pose.get("tool_pose")
    if not isinstance(tool_pose, dict):
        fail(f"{name}.tool_pose must be an object")
    parent = tool_pose.get("parent_frame")
    child = tool_pose.get("child_frame")
    if parent != "base_link" or child != "wrist3_link":
        fail(
            f"{name}.tool_pose must be base_link -> wrist3_link, got "
            f"{parent!r} -> {child!r}"
        )
    translation = tool_pose.get("translation_m")
    rotation = tool_pose.get("rotation_xyzw")
    if not isinstance(translation, dict) or not isinstance(rotation, dict):
        fail(f"{name}.tool_pose translation_m and rotation_xyzw must be objects")
    xyz = tuple(
        require_finite(translation.get(axis), f"{name}.tool_pose.translation_m.{axis}")
        for axis in ("x", "y", "z")
    )
    q = tuple(
        require_finite(rotation.get(axis), f"{name}.tool_pose.rotation_xyzw.{axis}")
        for axis in ("x", "y", "z", "w")
    )
    norm = math.sqrt(sum(component * component for component in q))
    if norm < 0.95 or norm > 1.05:
        fail(f"{name}.tool_pose.rotation_xyzw norm must be near 1.0, got {norm:.9f}")
    return xyz, q, norm


def parse_offset(raw, name):
    try:
        value = float(raw)
    except ValueError:
        fail(f"{name} must be numeric, got {raw!r}")
    if not math.isfinite(value) or abs(value) > 0.050:
        fail(f"{name} must be finite and within +/-0.050 m, got {value}")
    return value


path = sys.argv[1]
mount_offset = (
    parse_offset(sys.argv[2], "CONTACT_BOX_COVER_MOUNT_OFFSET_X_M"),
    parse_offset(sys.argv[3], "CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M"),
    parse_offset(sys.argv[4], "CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M"),
)
try:
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
except OSError as exc:
    fail(f"failed to read {path}: {exc}")
except json.JSONDecodeError as exc:
    fail(f"invalid JSON in {path}: {exc}")

approach = require_pose(data, "cover_mount_approach")
place = require_pose(data, "cover_mount_place")
approach_joints = validate_joint_state(approach, "cover_mount_approach")
validate_joint_state(place, "cover_mount_place")
approach_xyz, approach_q, approach_q_norm = validate_tool_pose(
    approach, "cover_mount_approach"
)
place_xyz, place_q, place_q_norm = validate_tool_pose(place, "cover_mount_place")

shifted_approach_xyz = tuple(approach_xyz[i] + mount_offset[i] for i in range(3))
shifted_place_xyz = tuple(place_xyz[i] + mount_offset[i] for i in range(3))

approach_offset_distance = math.sqrt(sum(component * component for component in mount_offset))
delta = tuple(shifted_place_xyz[i] - shifted_approach_xyz[i] for i in range(3))
distance = math.sqrt(sum(component * component for component in delta))

aq = tuple(component / approach_q_norm for component in approach_q)
pq = tuple(component / place_q_norm for component in place_q)
dot = abs(sum(aq[i] * pq[i] for i in range(4)))
dot = max(-1.0, min(1.0, dot))
orientation_angle = 2.0 * math.acos(dot)

values = {
    "APPROACH_J1": approach_joints["j1"],
    "APPROACH_J2": approach_joints["j2"],
    "APPROACH_J3": approach_joints["j3"],
    "APPROACH_J4": approach_joints["j4"],
    "APPROACH_J5": approach_joints["j5"],
    "APPROACH_J6": approach_joints["j6"],
    "APPROACH_X": approach_xyz[0],
    "APPROACH_Y": approach_xyz[1],
    "APPROACH_Z": approach_xyz[2],
    "APPROACH_QX": approach_q[0],
    "APPROACH_QY": approach_q[1],
    "APPROACH_QZ": approach_q[2],
    "APPROACH_QW": approach_q[3],
    "SHIFTED_APPROACH_X": shifted_approach_xyz[0],
    "SHIFTED_APPROACH_Y": shifted_approach_xyz[1],
    "SHIFTED_APPROACH_Z": shifted_approach_xyz[2],
    "PLACE_X": place_xyz[0],
    "PLACE_Y": place_xyz[1],
    "PLACE_Z": place_xyz[2],
    "SHIFTED_PLACE_X": shifted_place_xyz[0],
    "SHIFTED_PLACE_Y": shifted_place_xyz[1],
    "SHIFTED_PLACE_Z": shifted_place_xyz[2],
    "PLACE_QX": place_q[0],
    "PLACE_QY": place_q[1],
    "PLACE_QZ": place_q[2],
    "PLACE_QW": place_q[3],
    "PLACE_DELTA_X": delta[0],
    "PLACE_DELTA_Y": delta[1],
    "PLACE_DELTA_Z": delta[2],
    "PLACE_DISTANCE_M": distance,
    "PLACE_ORIENTATION_DIFF_RAD": orientation_angle,
    "APPROACH_OFFSET_DISTANCE_M": approach_offset_distance,
    "APPROACH_OFFSET_MAX_WRIST_TARGET_DISTANCE": max(approach_offset_distance + 0.010, 0.010),
    "PLACE_MAX_WRIST_TARGET_DISTANCE": max(distance + 0.010, 0.010),
}

for key, value in values.items():
    print(f"{key}={value:.12f}")

print(f"PLACE_DISTANCE_NEAR_ZERO={'true' if distance < 0.001 else 'false'}")
print(f"PLACE_ORIENTATION_DIFF_RISK={'true' if orientation_angle > 0.010 else 'false'}")
PY
)"
record_timing "load and validate mount poses" "$(( $(now_ms) - POSE_LOAD_START_MS ))"

eval "${POSE_ENV}"

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
ALLOW_RESET_EXECUTION=false
ALLOW_STANDOFF_EXECUTION=false
if [[ "${EXECUTE}" == "true" ]]; then
  EXECUTE_MOTION=true
  ALLOW_RESET_EXECUTION=true
  ALLOW_STANDOFF_EXECUTION=true
  echo "WARNING: COVER TO MOUNT EXECUTION ENABLED."
else
  echo "COVER TO MOUNT: plan-only mode. No robot motion will be commanded."
  echo "Note: the Cartesian place plan is only a true approach-to-place plan when the robot is already at cover_mount_approach."
fi

echo "Precondition: ./scripts/run_contact_box_cover_pickout_sequence.sh --execute has completed."
echo "Precondition: suction A/top cups are still ON; this script will not release suction."
echo "Recorded poses file: ${RECORDED_POSES_FILE}"
echo "Mount offset applied to both recorded poses:"
echo "  dx=${CONTACT_BOX_COVER_MOUNT_OFFSET_X_M}, dy=${CONTACT_BOX_COVER_MOUNT_OFFSET_Y_M}, dz=${CONTACT_BOX_COVER_MOUNT_OFFSET_Z_M} m"
echo "Step 1 target recorded pose: cover_mount_approach"
echo "Step 2 target shifted pose: cover_mount_approach + mount offset"
echo "Step 3 target shifted pose: cover_mount_place + mount offset"
echo
echo "cover_mount_approach joints:"
echo "  j1=${APPROACH_J1}, j2=${APPROACH_J2}, j3=${APPROACH_J3}, j4=${APPROACH_J4}, j5=${APPROACH_J5}, j6=${APPROACH_J6}"
echo "cover_mount_approach original wrist3_link pose:"
echo "  position=(${APPROACH_X}, ${APPROACH_Y}, ${APPROACH_Z})"
echo "  qxyzw=(${APPROACH_QX}, ${APPROACH_QY}, ${APPROACH_QZ}, ${APPROACH_QW})"
echo "cover_mount_approach shifted wrist3_link pose:"
echo "  position=(${SHIFTED_APPROACH_X}, ${SHIFTED_APPROACH_Y}, ${SHIFTED_APPROACH_Z})"
echo "  qxyzw=(${APPROACH_QX}, ${APPROACH_QY}, ${APPROACH_QZ}, ${APPROACH_QW})"
echo "cover_mount_place original wrist3_link pose:"
echo "  position=(${PLACE_X}, ${PLACE_Y}, ${PLACE_Z})"
echo "  qxyzw=(${PLACE_QX}, ${PLACE_QY}, ${PLACE_QZ}, ${PLACE_QW})"
echo "cover_mount_place shifted wrist3_link pose:"
echo "  position=(${SHIFTED_PLACE_X}, ${SHIFTED_PLACE_Y}, ${SHIFTED_PLACE_Z})"
echo "  qxyzw=(${PLACE_QX}, ${PLACE_QY}, ${PLACE_QZ}, ${PLACE_QW})"
echo "shifted approach -> shifted place delta:"
echo "  dxyz=(${PLACE_DELTA_X}, ${PLACE_DELTA_Y}, ${PLACE_DELTA_Z}) m"
echo "  distance=${PLACE_DISTANCE_M} m"
echo "  orientation_diff=${PLACE_ORIENTATION_DIFF_RAD} rad"
echo "Cartesian fraction threshold: ${CONTACT_BOX_CARTESIAN_MIN_FRACTION}"

if [[ "${PLACE_DISTANCE_NEAR_ZERO}" == "true" ]]; then
  echo "WARNING: cover_mount_approach and cover_mount_place positions are nearly identical."
  echo "WARNING: Step 3 may be a zero-distance/no-op; re-record poses if a vertical placement move is intended."
fi

if [[ "${PLACE_ORIENTATION_DIFF_RISK}" == "true" ]]; then
  echo "WARNING: cover_mount_approach and cover_mount_place orientations differ."
  echo "WARNING: Step 3 uses the recorded cover_mount_place orientation; verify this before execution."
fi

run_cartesian_wrist_target() {
  local label="$1"
  local target_x="$2"
  local target_y="$3"
  local target_z="$4"
  local target_qx="$5"
  local target_qy="$6"
  local target_qz="$7"
  local target_qw="$8"
  local max_wrist_target_distance="$9"

  echo
  echo "========== ${label} =========="
  echo "This step is Cartesian only. There is no joint-space fallback."
  echo "Target wrist3_link position: (${target_x}, ${target_y}, ${target_z})"
  echo "Target wrist3_link qxyzw: (${target_qx}, ${target_qy}, ${target_qz}, ${target_qw})"
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
    target_qx:="${target_qx}" \
    target_qy:="${target_qy}" \
    target_qz:="${target_qz}" \
    target_qw:="${target_qw}" \
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

offset_nudge_required() {
  python3 - "${APPROACH_OFFSET_DISTANCE_M}" <<'PY'
import sys

distance = float(sys.argv[1])
sys.exit(0 if distance > 0.0005 else 1)
PY
}

echo
echo "========== Step 1: move to recorded cover_mount_approach =========="
echo "Using saved recorded-pose joint movement with reset safety gates."
STEP_START_MS="$(now_ms)"
set +e
ros2 launch moveit_move return_to_saved_joint_pose.launch.py \
  j1:="${APPROACH_J1}" \
  j2:="${APPROACH_J2}" \
  j3:="${APPROACH_J3}" \
  j4:="${APPROACH_J4}" \
  j5:="${APPROACH_J5}" \
  j6:="${APPROACH_J6}" \
  velocity_scaling:="${CONTACT_BOX_RESET_VELOCITY_SCALING}" \
  acceleration_scaling:="${CONTACT_BOX_RESET_ACCELERATION_SCALING}" \
  max_joint_delta_rad:="${CONTACT_BOX_RESET_MAX_JOINT_DELTA_RAD}" \
  execute_motion:="${EXECUTE_MOTION}" \
  allow_reset_execution:="${ALLOW_RESET_EXECUTION}"
STEP_STATUS=$?
set -e
record_timing "Step 1: move to recorded cover_mount_approach" "$(( $(now_ms) - STEP_START_MS ))"
if [[ "${STEP_STATUS}" -ne 0 ]]
then
  echo "ERROR: failed to reach cover_mount_approach. Refusing to continue to cover_mount_place." >&2
  print_timing_summary
  exit 1
fi

if offset_nudge_required; then
  if ! run_cartesian_wrist_target \
    "Step 2: Cartesian nudge to shifted cover_mount_approach" \
    "${SHIFTED_APPROACH_X}" "${SHIFTED_APPROACH_Y}" "${SHIFTED_APPROACH_Z}" \
    "${APPROACH_QX}" "${APPROACH_QY}" "${APPROACH_QZ}" "${APPROACH_QW}" \
    "${APPROACH_OFFSET_MAX_WRIST_TARGET_DISTANCE}"
  then
    echo "ERROR: Cartesian move from recorded cover_mount_approach to shifted cover_mount_approach failed." >&2
    print_timing_summary
    exit 1
  fi
else
  echo
  echo "========== Step 2: Cartesian nudge to shifted cover_mount_approach =========="
  echo "Skipping zero-distance mount-offset nudge; runtime mount offset is ${APPROACH_OFFSET_DISTANCE_M} m."
  record_timing "Step 2: skipped zero-distance mount-offset nudge" 0
fi

if ! run_cartesian_wrist_target \
  "Step 3: Cartesian move from shifted cover_mount_approach to shifted cover_mount_place" \
  "${SHIFTED_PLACE_X}" "${SHIFTED_PLACE_Y}" "${SHIFTED_PLACE_Z}" \
  "${PLACE_QX}" "${PLACE_QY}" "${PLACE_QZ}" "${PLACE_QW}" \
  "${PLACE_MAX_WRIST_TARGET_DISTANCE}"
then
  echo "ERROR: Cartesian move from shifted cover_mount_approach to shifted cover_mount_place failed." >&2
  print_timing_summary
  exit 1
fi

echo
echo "COVER TO MOUNT SEQUENCE COMPLETE."
echo "Cover is at shifted cover_mount_place. Suction is still ON. Release has not been commanded."
echo "This script did not call suction_off.sh and did not retreat from cover_mount_place."
print_timing_summary
