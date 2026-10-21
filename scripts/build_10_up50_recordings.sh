#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENVIRONMENT_ID="new_board_4cm_rod33365"
INDEX_PATH="${REPOSITORY_ROOT}/runs/up50_recordings.latest.csv"
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
  echo "build_10_up50_recordings.sh: $*" >&2
  exit 1
}

action_server_count() {
  local action_name="$1"
  ROS2CLI_NO_DAEMON=1 ros2 action info "${action_name}" | \
    awk '/^Action servers:/ {print $3}'
}

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "${REPOSITORY_ROOT}/install/setup.bash" ]] || fail "workspace is not built"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -r "${REPOSITORY_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1090
  source "${REPOSITORY_ROOT}/.venv/bin/activate"
fi
# shellcheck disable=SC1090
source "${REPOSITORY_ROOT}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

for trajectory in "${TRAJECTORIES[@]}"; do
  [[ -f "${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/trajectories/${trajectory}.csv" ]] || \
    fail "missing authoritative trajectory: ${trajectory}"
done

mkdir -p -- "${REPOSITORY_ROOT}/runs"
PARTIAL_INDEX="${INDEX_PATH}.partial.$(date -u +%Y%m%dT%H%M%SZ)"
SIMULATION_LOG="${REPOSITORY_ROOT}/runs/up50_build_simulation__$(date -u +%Y%m%dT%H%M%SZ).log"
printf '%s\n' "trajectory_id,recording_id" > "${PARTIAL_INDEX}"

cd -- "${REPOSITORY_ROOT}"
setsid ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 \
  environment_id:="${ENVIRONMENT_ID}" \
  workspace:="${REPOSITORY_ROOT}" \
  launch_rviz:=false \
  > "${SIMULATION_LOG}" 2>&1 &
SIMULATION_PID=$!

stop_simulation() {
  if kill -0 "${SIMULATION_PID}" 2>/dev/null; then
    kill -INT -- "-${SIMULATION_PID}" 2>/dev/null || true
    wait "${SIMULATION_PID}" 2>/dev/null || true
  fi
}
trap stop_simulation EXIT
trap 'exit 130' INT TERM

server_ready=0
for _ in $(seq 1 120); do
  if ! kill -0 "${SIMULATION_PID}" 2>/dev/null; then
    tail -n 80 "${SIMULATION_LOG}" >&2 || true
    fail "isolated simulation exited before BuildRecording became ready"
  fi
  if [[ "$(action_server_count /hx_waypoint/build_recording 2>/dev/null || true)" == "1" ]]; then
    server_ready=1
    break
  fi
  sleep 1
done
((server_ready == 1)) || fail "isolated simulation did not become ready in 120 s"

for index in "${!TRAJECTORIES[@]}"; do
  trajectory="${TRAJECTORIES[$index]}"
  echo "===== [$((index + 1))/${#TRAJECTORIES[@]}] ${trajectory} ====="
  output="$(
    ros2 run hx_waypoint_tools tipdraw \
      --workspace "${REPOSITORY_ROOT}" \
      build-recording \
      --env "${ENVIRONMENT_ID}" \
      --trajectory "${trajectory}" \
      --external-runtime
  )"
  recording_path="$(printf '%s\n' "${output}" | tail -n 1)"
  recording_id="$(basename -- "${recording_path}")"
  [[ -d "${recording_path}" ]] || fail "build did not return a recording directory"
  [[ "${recording_id}" == "${trajectory}__"* ]] || \
    fail "unexpected recording id: ${recording_id}"
  printf '%s,%s\n' "${trajectory}" "${recording_id}" >> "${PARTIAL_INDEX}"
  echo "recording: ${recording_id}"
done

mv -- "${PARTIAL_INDEX}" "${INDEX_PATH}"
echo "Recording index: ${INDEX_PATH}"
