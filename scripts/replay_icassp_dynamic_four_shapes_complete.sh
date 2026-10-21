#!/usr/bin/env bash
set -Eeuo pipefail

# Replay the complete 4-shape × 5-height simulation set on a connected UR5.
# The runtime's mla start-state policy performs a collision-checked
# home_then_recording_start prepare before each sealed recording.  The
# recording payload itself is never scaled, retimed, clipped, or shortened.

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
INDEX_PATH="${WORKSPACE}/environments/${ENVIRONMENT_ID}/trajectories/icassp_dynamic_four_shapes_complete_index.csv"

fail() {
  echo "replay_icassp_dynamic_four_shapes_complete.sh: $*" >&2
  exit 1
}

if (($#)); then
  [[ "$#" == 2 && "$1" == "--index" ]] || fail "usage: $0 [--index FILE]"
  INDEX_PATH="$2"
fi

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "${WORKSPACE}/install/setup.bash" ]] || fail "workspace is not built"
[[ -r "${INDEX_PATH}" ]] || fail "complete recording index is missing: ${INDEX_PATH}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${WORKSPACE}/install/setup.bash"
set -u

EXPECTED_TRAJECTORIES=()
for shape in circle rounded_square figure8; do
  for height in 07 09 11 13 15; do
    EXPECTED_TRAJECTORIES+=("icassp_dynamic_${shape}_z${height}")
  done
done
for height in 07 09 11 13 15; do
  EXPECTED_TRAJECTORIES+=("icassp_dynamic_depth_loop_safe10mm_z${height}")
done

RECORDINGS=()
line=0
while IFS=, read -r shape height trajectory authority_file authority_sha recording_id recording_path recording_manifest_sha status; do
  if ((line == 0)); then
    [[ "${shape},${height},${trajectory},${authority_file},${authority_sha},${recording_id},${recording_path},${recording_manifest_sha},${status}" == \
      "shape,height_cm,trajectory_id,authority_file,authority_sha256,recording_id,recording_relative_path,recording_manifest_sha256,status" ]] ||
      fail "invalid complete index header"
    line=1
    continue
  fi
  expected_index=$((line - 1))
  ((expected_index < ${#EXPECTED_TRAJECTORIES[@]})) || fail "complete index has extra rows"
  [[ "${trajectory}" == "${EXPECTED_TRAJECTORIES[$expected_index]}" ]] ||
    fail "complete index order mismatch at row ${line}: ${trajectory}"
  [[ "${recording_id}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe recording id: ${recording_id}"
  [[ "${status}" == "validated" ]] || fail "recording is not validated: ${recording_id}"
  [[ -d "${WORKSPACE}/environments/${ENVIRONMENT_ID}/recordings/${recording_id}" ]] ||
    fail "missing recording: ${recording_id}"
  RECORDINGS+=("${recording_id}")
  line=$((line + 1))
done < "${INDEX_PATH}"
[[ "${#RECORDINGS[@]}" == "${#EXPECTED_TRAJECTORIES[@]}" ]] ||
  fail "complete index must contain exactly ${#EXPECTED_TRAJECTORIES[@]} rows"

action_server_count() {
  ros2 action info "$1" | awk '/^Action servers:/ {print $3}'
}

[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] ||
  fail "/hx_waypoint/play_recording must have exactly one server"

CONTROLLER_STATE="$({ timeout 15 ros2 control list_controllers; } 2>/dev/null | \
  awk '$1 == "scaled_joint_trajectory_controller" {print $3}')"
if [[ "${CONTROLLER_STATE}" != "active" ]]; then
  timeout 15 ros2 control switch_controllers \
    --activate scaled_joint_trajectory_controller \
    --deactivate joint_trajectory_controller \
    --strict || fail "could not activate scaled_joint_trajectory_controller"
fi

mkdir -p -- "${WORKSPACE}/runs"
LOG_PATH="${WORKSPACE}/runs/icassp_dynamic_four_shapes_complete_replay__$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "${LOG_PATH}") 2>&1

on_exit() {
  local status=$?
  if ((status == 0)); then
    echo "All ${#RECORDINGS[@]} complete-set recordings completed. Log: ${LOG_PATH}"
  else
    echo "Replay stopped with status ${status}. No later recording was sent. Log: ${LOG_PATH}" >&2
  fi
}
trap on_exit EXIT

cd -- "${WORKSPACE}"
for index in "${!RECORDINGS[@]}"; do
  recording="${RECORDINGS[$index]}"
  echo "===== [$((index + 1))/${#RECORDINGS[@]}] ${recording} ====="
  # mla/recording.yaml fixes this to home_then_recording_start; --auto-prepare
  # asks the runtime to execute that collision-checked preparation each time.
  ros2 run hx_waypoint_tools tipdraw \
    --workspace "${WORKSPACE}" \
    play \
    --env "${ENVIRONMENT_ID}" \
    --recording "${recording}" \
    --auto-prepare
done
