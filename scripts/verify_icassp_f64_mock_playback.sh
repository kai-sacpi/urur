#!/usr/bin/env bash

# Replay the sealed 13-route F64 bundle in a fresh, isolated mock runtime.
# This is verification evidence only: it never connects to a real controller
# and never modifies, retimes, scales, clips, or drops a recording point.

set -Eeuo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
ID_PREFIX="${ICASSP_F64_ID_PREFIX:-icassp_f64_mj_feasible_v2}"
ROS_DOMAIN_ID="${ICASSP_F64_MOCK_DOMAIN_ID:-232}"
INDEX_PATH=""

fail() {
  echo "verify_icassp_f64_mock_playback.sh: $*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --id-prefix)
      [[ "$#" -ge 2 ]] || fail "--id-prefix requires PREFIX"
      ID_PREFIX="$2"
      shift 2
      ;;
    --index)
      [[ "$#" -ge 2 ]] || fail "--index requires FILE"
      INDEX_PATH="$2"
      shift 2
      ;;
    --domain-id)
      [[ "$#" -ge 2 ]] || fail "--domain-id requires INTEGER"
      ROS_DOMAIN_ID="$2"
      shift 2
      ;;
    *)
      fail "usage: $0 [--id-prefix PREFIX] [--index FILE] [--domain-id INTEGER]"
      ;;
  esac
done

[[ "$ID_PREFIX" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe id prefix: $ID_PREFIX"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || fail "domain id must be an integer"
((ROS_DOMAIN_ID <= 232)) || fail "domain id must be in [0, 232]"
if [[ -z "$INDEX_PATH" ]]; then
  INDEX_PATH="$WORKSPACE/environments/$ENVIRONMENT_ID/trajectories/${ID_PREFIX}_13_validated_recordings_index.csv"
fi

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "$WORKSPACE/install/setup.bash" ]] || fail "workspace is not built"
[[ -r "$INDEX_PATH" ]] || fail "recording index is missing: $INDEX_PATH"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$WORKSPACE/.venv/bin/activate"
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
set -u

CUROBO_COMMIT="$(awk -F': *' '$1 == "commit" {print $2; exit}' "$WORKSPACE/curobo.lock")"
CUROBO_SOURCE="$WORKSPACE/.deps/curobo-${CUROBO_COMMIT}"
[[ -f "$CUROBO_SOURCE/curobo/__init__.py" ]] || fail "locked cuRobo runtime is unavailable"
VENV_SITE_PACKAGES=""
for candidate in "$WORKSPACE"/.venv/lib/python*/site-packages; do
  [[ -d "$candidate" ]] || continue
  [[ -z "$VENV_SITE_PACKAGES" ]] || fail "ambiguous venv site-packages directories"
  VENV_SITE_PACKAGES="$candidate"
done
[[ -n "$VENV_SITE_PACKAGES" ]] || fail "venv site-packages is unavailable"

export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS2CLI_NO_DAEMON=1
export PYTHONNOUSERSITE=1
export PYTHONPATH="$CUROBO_SOURCE:$VENV_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"

ROUTES=(
  line_x_vertical_z09 line_x_tilt90_z09
  line_y_vertical_z09 line_y_tilt90_z09
  line_x_vertical_z13 line_x_tilt90_z13
  line_y_vertical_z13 line_y_tilt90_z13
  line_x_vertical_z15 line_x_tilt90_z15
  line_y_vertical_z15 line_y_tilt90_z15
  eight_tilt90_z13
)
RECORDINGS=()
TRAJECTORIES=()
row=0
while IFS=, read -r order trajectory source_route authority_file authority_sha recording_id recording_path recording_manifest_sha duration source_count point_count status; do
  if ((row == 0)); then
    [[ "${order},${trajectory},${source_route},${authority_file},${authority_sha},${recording_id},${recording_path},${recording_manifest_sha},${duration},${source_count},${point_count},${status}" == \
      "order,trajectory_id,source_route,authority_file,authority_sha256,recording_id,recording_relative_path,recording_manifest_sha256,duration_s,source_waypoint_count,recording_point_count,status" ]] || fail "invalid recording index header"
    row=1
    continue
  fi
  expected_index=$((row - 1))
  ((expected_index < ${#ROUTES[@]})) || fail "recording index has extra rows"
  expected_trajectory="${ID_PREFIX}_${ROUTES[$expected_index]}"
  [[ "$order" == "$expected_index" ]] || fail "recording order mismatch at row $row"
  [[ "$trajectory" == "$expected_trajectory" ]] || fail "trajectory mismatch at row $row: $trajectory"
  [[ "$recording_id" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe recording id"
  [[ "$status" == "validated" ]] || fail "recording is not validated: $recording_id"
  [[ -d "$WORKSPACE/environments/$ENVIRONMENT_ID/recordings/$recording_id" ]] || fail "recording is missing: $recording_id"
  TRAJECTORIES+=("$trajectory")
  RECORDINGS+=("$recording_id")
  row=$((row + 1))
done < "$INDEX_PATH"
[[ "${#RECORDINGS[@]}" == "${#ROUTES[@]}" ]] || fail "recording index must contain exactly 13 entries"

RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/${ID_PREFIX}_independent_mock_playback_${RUN_STAMP}"
mkdir -p "$RUN_DIR/logs"
printf 'order,trajectory_id,recording_id,play_status,play_log\n' > "$RUN_DIR/summary.csv"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: %s\n' "$ENVIRONMENT_ID"
  printf 'id_prefix: %s\n' "$ID_PREFIX"
  printf 'ros_domain_id: %s\n' "$ROS_DOMAIN_ID"
  printf 'recording_index: %s\n' "$INDEX_PATH"
  printf 'route_count: %s\n' "${#RECORDINGS[@]}"
  printf 'policy: sealed playback without scaling, retiming, clipping, or point dropping\n'
} > "$RUN_DIR/run_manifest.yaml"

sim_pid=""
cleanup() {
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill -INT -- "-$sim_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$sim_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:="$ENVIRONMENT_ID" workspace:="$WORKSPACE" \
  launch_rviz:=false > "$RUN_DIR/simulation.launch.log" 2>&1 &
sim_pid=$!

ready=0
for _ in $(seq 1 180); do
  kill -0 "$sim_pid" 2>/dev/null || fail "simulation exited before readiness"
  nodes="$(ros2 node list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"
  if grep -qx '/hx_waypoint_runtime' <<<"$nodes" \
    && grep -qx '/joint_trajectory_controller' <<<"$nodes" \
    && grep -qx '/hx_waypoint/play_recording' <<<"$actions"; then
    ready=1
    break
  fi
  sleep 1
done
[[ "$ready" == 1 ]] || fail "simulation readiness timeout"

overall=0
for index in "${!RECORDINGS[@]}"; do
  recording="${RECORDINGS[$index]}"
  trajectory="${TRAJECTORIES[$index]}"
  play_log="$RUN_DIR/logs/${trajectory}.play.log"
  echo "[playback] [$((index + 1))/${#RECORDINGS[@]}] $recording"
  set +e
  timeout 240 ros2 run hx_waypoint_tools tipdraw \
    --workspace "$WORKSPACE" play --env "$ENVIRONMENT_ID" \
    --recording "$recording" --auto-prepare > "$play_log" 2>&1
  play_status=$?
  set -e
  printf '%s,%s,%s,%s,%s\n' \
    "$index" "$trajectory" "$recording" "$play_status" "$play_log" >> "$RUN_DIR/summary.csv"
  if ((play_status == 0)); then
    echo "[playback] PASS $trajectory"
  else
    echo "[playback] FAIL status=$play_status $trajectory" >&2
    overall=1
  fi
done

echo "[playback] summary=$RUN_DIR/summary.csv"
exit "$overall"
