#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENVIRONMENT_ID="new_board_4cm_rod33365"
RECORDINGS_ROOT="${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/recordings"
HOME_JOINTS=(
  -2.264215638612
  -1.362229481182
  -1.056971395008
  -2.253045531399
  1.573239787748
  1.570796326795
)
RECORDINGS=(
  bafe_continuous_wave_3d_zplus26mm__20260805_193239
  bafe_continuous_helix_3d_zplus26mm__20260805_193550
  bafe_continuous_rounded_square_3d_zplus26mm__20260805_193924
  bafe_continuous_figure8_3d_zplus26mm__20260805_194242
  bafe_continuous_lissajous_3d_zplus26mm__20260805_194623
  bafe_continuous_trefoil_knot_3d_zplus26mm__20260805_194941
  bafe_continuous_heart_3d_zplus26mm__20260805_195300
  bafe_continuous_spirograph_3d_zplus26mm__20260805_195623
  bafe_continuous_spherical_spiral_3d_zplus26mm__20260805_200049
  bafe_continuous_celtic_knot_3d_zplus26mm__20260805_200413
)

fail() {
  echo "replay_10_for_video.sh: $*" >&2
  exit 1
}

action_server_count() {
  local action_name="$1"
  ros2 action info "${action_name}" | awk '/^Action servers:/ {print $3}'
}

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
[[ "${CONTROLLER_STATE}" == "active" ]] || \
  fail "joint_trajectory_controller is not active"

for recording in "${RECORDINGS[@]}"; do
  [[ -d "${RECORDINGS_ROOT}/${recording}" ]] || \
    fail "missing authoritative recording: ${recording}"
done

mkdir -p -- "${REPOSITORY_ROOT}/runs"
LOG_PATH="${REPOSITORY_ROOT}/runs/video_replay_10__$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "${LOG_PATH}") 2>&1

on_exit() {
  local status=$?
  if ((status == 0)); then
    echo "All 10 recordings completed. Log: ${LOG_PATH}"
  else
    echo "Batch stopped with status ${status}. Robot received no later recordings. Log: ${LOG_PATH}" >&2
  fi
}
trap on_exit EXIT

cd -- "${REPOSITORY_ROOT}"
for index in "${!RECORDINGS[@]}"; do
  recording="${RECORDINGS[$index]}"
  echo "===== [$((index + 1))/${#RECORDINGS[@]}] ${recording} ====="

  python3 "${SCRIPT_DIR}/moveit_joint_prepare.py" \
    --workspace "${REPOSITORY_ROOT}" \
    --target "${HOME_JOINTS[@]}" \
    --execute

  mapfile -t START_JOINTS < <(
    awk -F, 'NR == 2 {for (column = 2; column <= 7; ++column) print $column; exit}' \
      "${RECORDINGS_ROOT}/${recording}/joint_trajectory.csv"
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
