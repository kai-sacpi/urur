#!/usr/bin/env bash
set -Eeuo pipefail

# Replay the validated 13-route F64 authority set on a connected UR5.
# mla/recording.yaml enforces the collision-checked home_then_recording_start
# preparation.  The sealed recording payload is sent unchanged.

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
ID_PREFIX="${ICASSP_F64_ID_PREFIX:-icassp_f64_mj_feasible_v2}"
INDEX_PATH="${WORKSPACE}/environments/${ENVIRONMENT_ID}/trajectories/${ID_PREFIX}_13_validated_recordings_index.csv"

fail() { echo "replay_icassp_f64_13.sh: $*" >&2; exit 1; }

INDEX_OVERRIDE=0
while (($#)); do
  case "$1" in
    --index)
      [[ "$#" -ge 2 ]] || fail "--index requires FILE"
      INDEX_PATH="$2"
      INDEX_OVERRIDE=1
      shift 2
      ;;
    --id-prefix)
      [[ "$#" -ge 2 ]] || fail "--id-prefix requires PREFIX"
      ID_PREFIX="$2"
      shift 2
      ;;
    *)
      fail "usage: $0 [--id-prefix PREFIX] [--index FILE]"
      ;;
  esac
done
[[ "${ID_PREFIX}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe id prefix: ${ID_PREFIX}"
if ((INDEX_OVERRIDE == 0)); then
  INDEX_PATH="${WORKSPACE}/environments/${ENVIRONMENT_ID}/trajectories/${ID_PREFIX}_13_validated_recordings_index.csv"
fi

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "${WORKSPACE}/install/setup.bash" ]] || fail "workspace is not built"
[[ -r "${INDEX_PATH}" ]] || fail "recording index is missing: ${INDEX_PATH}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

ROUTES=(
  line_x_vertical_z09 line_x_tilt90_z09
  line_y_vertical_z09 line_y_tilt90_z09
  line_x_vertical_z13 line_x_tilt90_z13
  line_y_vertical_z13 line_y_tilt90_z13
  line_x_vertical_z15 line_x_tilt90_z15
  line_y_vertical_z15 line_y_tilt90_z15
  eight_tilt90_z13
)
EXPECTED=()
for route in "${ROUTES[@]}"; do
  EXPECTED+=("${ID_PREFIX}_${route}")
done

RECORDINGS=()
row=0
while IFS=, read -r order trajectory source_route authority_file authority_sha recording_id recording_path recording_manifest_sha duration source_count point_count status; do
  if ((row == 0)); then
    [[ "${order},${trajectory},${source_route},${authority_file},${authority_sha},${recording_id},${recording_path},${recording_manifest_sha},${duration},${source_count},${point_count},${status}" == \
      "order,trajectory_id,source_route,authority_file,authority_sha256,recording_id,recording_relative_path,recording_manifest_sha256,duration_s,source_waypoint_count,recording_point_count,status" ]] || fail "invalid recording index header"
    row=1
    continue
  fi
  expected_index=$((row - 1))
  ((expected_index < ${#EXPECTED[@]})) || fail "recording index has extra rows"
  [[ "${trajectory}" == "${EXPECTED[$expected_index]}" ]] || fail "recording index order mismatch at row ${row}: ${trajectory}"
  [[ "${recording_id}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe recording id"
  [[ "${status}" == "validated" ]] || fail "recording is not validated: ${recording_id}"
  [[ -d "${WORKSPACE}/environments/${ENVIRONMENT_ID}/recordings/${recording_id}" ]] || fail "missing recording: ${recording_id}"
  RECORDINGS+=("${recording_id}")
  row=$((row + 1))
done < "${INDEX_PATH}"
[[ "${#RECORDINGS[@]}" == "${#EXPECTED[@]}" ]] || fail "recording index must contain exactly ${#EXPECTED[@]} entries"

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || fail "/hx_waypoint/play_recording must have exactly one server"

controller_state="$({ timeout 15 ros2 control list_controllers; } 2>/dev/null | awk '$1 == "scaled_joint_trajectory_controller" {print $3}')"
if [[ "${controller_state}" != "active" ]]; then
  timeout 15 ros2 control switch_controllers \
    --activate scaled_joint_trajectory_controller \
    --deactivate joint_trajectory_controller \
    --strict || fail "could not activate scaled_joint_trajectory_controller"
fi

mkdir -p "${WORKSPACE}/runs"
LOG_PATH="${WORKSPACE}/runs/${ID_PREFIX}_13_replay__$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "${LOG_PATH}") 2>&1
trap 'rc=$?; if ((rc == 0)); then echo "All ${#RECORDINGS[@]} F64 recordings completed (prefix ${ID_PREFIX}). Log: ${LOG_PATH}"; else echo "Replay stopped at first failure (status ${rc}). Log: ${LOG_PATH}" >&2; fi' EXIT

cd "${WORKSPACE}"
for index in "${!RECORDINGS[@]}"; do
  recording="${RECORDINGS[$index]}"
  echo "===== [$((index + 1))/${#RECORDINGS[@]}] ${recording} ====="
  ros2 run hx_waypoint_tools tipdraw \
    --workspace "${WORKSPACE}" play \
    --env "${ENVIRONMENT_ID}" \
    --recording "${recording}" \
    --auto-prepare
done
