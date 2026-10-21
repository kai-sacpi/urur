#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENVIRONMENT_ID="new_board_4cm_rod33365"
INDEX_PATH="${REPOSITORY_ROOT}/runs/up50_recordings.latest.csv"
HOME_JOINTS=(
  -2.264215638612
  -1.362229481182
  -1.056971395008
  -2.253045531399
  1.573239787748
  1.570796326795
)
TRAJECTORIES=(
  bafe_continuous_wave_3d_zplus26mm_up50mm
  bafe_continuous_helix_3d_zplus26mm_up50mm
  bafe_continuous_rounded_square_3d_zplus26mm_up50mm
  bafe_continuous_figure8_3d_zplus26mm_up50mm
  bafe_continuous_lissajous_3d_zplus26mm_up50mm
  bafe_continuous_trefoil_knot_3d_zplus26mm_up50mm
  bafe_continuous_heart_3d_zplus26mm_up50mm
  bafe_continuous_spirograph_3d_zplus26mm_up50mm
  bafe_continuous_spherical_spiral_3d_zplus26mm_up50mm
  bafe_continuous_celtic_knot_3d_zplus26mm_up50mm
)

fail() {
  echo "replay_10_up50.sh: $*" >&2
  exit 1
}

action_server_count() {
  local action_name="$1"
  ros2 action info "${action_name}" | awk '/^Action servers:/ {print $3}'
}

if (($#)); then
  [[ "$1" == "--index" && $# == 2 ]] || \
    fail "usage: scripts/replay_10_up50.sh [--index FILE]"
  INDEX_PATH="$2"
fi

[[ -r "${INDEX_PATH}" ]] || fail "recording index is missing: ${INDEX_PATH}"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "${REPOSITORY_ROOT}/install/setup.bash" ]] || fail "workspace is not built"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${REPOSITORY_ROOT}/install/setup.bash"
set -u

[[ "$(action_server_count /move_action)" == "1" ]] || \
  fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || \
  fail "/hx_waypoint/play_recording must have exactly one server"

CONTROLLER_STATE="$({ timeout 15 ros2 control list_controllers; } | \
  awk '$1 == "joint_trajectory_controller" {print $3}')"
if [[ "${CONTROLLER_STATE}" != "active" ]]; then
  timeout 15 ros2 control switch_controllers \
    --activate joint_trajectory_controller \
    --strict
fi

RECORDINGS=()
line_index=0
while IFS=, read -r trajectory recording; do
  if ((line_index == 0)); then
    [[ "${trajectory},${recording}" == "trajectory_id,recording_id" ]] || \
      fail "invalid recording index header"
    line_index=$((line_index + 1))
    continue
  fi
  expected_index=$((line_index - 1))
  ((expected_index < ${#TRAJECTORIES[@]})) || fail "recording index has extra rows"
  [[ "${trajectory}" == "${TRAJECTORIES[$expected_index]}" ]] || \
    fail "recording index order/content mismatch at ${trajectory}"
  [[ "${recording}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe recording id"
  [[ -d "${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/recordings/${recording}" ]] || \
    fail "missing recording: ${recording}"
  RECORDINGS+=("${recording}")
  line_index=$((line_index + 1))
done < "${INDEX_PATH}"
[[ "${#RECORDINGS[@]}" == "${#TRAJECTORIES[@]}" ]] || \
  fail "recording index must contain exactly ${#TRAJECTORIES[@]} recordings"

mkdir -p -- "${REPOSITORY_ROOT}/runs"
LOG_PATH="${REPOSITORY_ROOT}/runs/up50_replay_10__$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "${LOG_PATH}") 2>&1

on_exit() {
  local status=$?
  if ((status == 0)); then
    echo "All 10 recordings completed. Log: ${LOG_PATH}"
  else
    echo "Batch stopped with status ${status}. Log: ${LOG_PATH}" >&2
  fi
}
trap on_exit EXIT

cd -- "${REPOSITORY_ROOT}"
for index in "${!RECORDINGS[@]}"; do
  recording="${RECORDINGS[$index]}"
  recording_root="${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/recordings/${recording}"
  echo "===== [$((index + 1))/${#RECORDINGS[@]}] ${recording} ====="

  python3 "${SCRIPT_DIR}/moveit_joint_prepare.py" \
    --workspace "${REPOSITORY_ROOT}" \
    --target "${HOME_JOINTS[@]}" \
    --execute

  mapfile -t START_JOINTS < <(
    awk -F, 'NR == 2 {for (column = 2; column <= 7; ++column) print $column; exit}' \
      "${recording_root}/joint_trajectory.csv"
  )
  [[ "${#START_JOINTS[@]}" == "6" ]] || \
    fail "cannot read the six sealed start joints for ${recording}"

  python3 "${SCRIPT_DIR}/moveit_joint_prepare.py" \
    --workspace "${REPOSITORY_ROOT}" \
    --target "${START_JOINTS[@]}" \
    --execute

  ros2 run hx_waypoint_tools tipdraw \
    --workspace "${REPOSITORY_ROOT}" \
    play \
    --env "${ENVIRONMENT_ID}" \
    --recording "${recording}" \
    --externally-prepared-start
done
