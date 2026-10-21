#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
INDEX="$WORKSPACE/environments/mla/trajectories/icassp_f64_mj_height_sweep_validated_recordings_index.csv"
NUCLEO_PORT="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00"
REUSE_EXISTING_SYNC=0
START_AT=1

fail() { echo "run_mla_height_sweep_real_nucleo.sh: $*" >&2; exit 1; }

while (($#)); do
  case "$1" in
    --port) (($# >= 2)) || fail "--port requires DEVICE"; NUCLEO_PORT="$2"; shift 2 ;;
    --index) (($# >= 2)) || fail "--index requires FILE"; INDEX="$2"; shift 2 ;;
    --reuse-existing-sync) REUSE_EXISTING_SYNC=1; shift ;;
    --start-at) (($# >= 2)) || fail "--start-at requires N"; START_AT="$2"; shift 2 ;;
    *) fail "unknown argument: $1" ;;
  esac
done

[[ "$START_AT" =~ ^[0-9]+$ ]] || fail "--start-at must be an integer"
((START_AT >= 1 && START_AT <= 20)) || fail "--start-at must be in [1,20]"
[[ -r "$INDEX" ]] || fail "missing recording index: $INDEX"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || fail "invalid NUCLEO port: $NUCLEO_PORT"
[[ -r /opt/ros/jazzy/setup.bash && -r "$WORKSPACE/install/setup.bash" ]] || fail "ROS workspace is not built"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || fail "/hx_waypoint/play_recording must have one server"

controller_state="$({ timeout 15 ros2 control list_controllers; } 2>/dev/null | awk '$1 == "joint_trajectory_controller" {print $3}')"
if [[ "$controller_state" != "active" ]]; then
  timeout 15 ros2 control switch_controllers \
    --activate joint_trajectory_controller \
    --deactivate scaled_joint_trajectory_controller \
    --strict || fail "could not activate joint_trajectory_controller"
fi

TRAJECTORIES=()
RECORDINGS=()
while IFS=, read -r order kind axis height trajectory authority_file authority_sha recording rest; do
  [[ "$order" == "order" ]] && continue
  [[ "$trajectory" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe trajectory id: $trajectory"
  [[ "$recording" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe recording id: $recording"
  [[ -d "$WORKSPACE/environments/$ENVIRONMENT_ID/recordings/$recording" ]] || fail "missing recording: $recording"
  TRAJECTORIES+=("$trajectory")
  RECORDINGS+=("$recording")
done < "$INDEX"
[[ "${#RECORDINGS[@]}" == "20" ]] || fail "recording index must contain 20 entries"

RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/mla_height_sweep_real_nucleo_$RUN_STAMP"
mkdir -p "$RUN_DIR/logs"
SUMMARY="$RUN_DIR/summary.csv"
printf 'order,trajectory_id,recording_id,play_status,nucleo_telemetry,nucleo_manifest,play_log\n' > "$SUMMARY"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: mla\n'
  printf 'recording_index: %s\n' "$INDEX"
  printf 'nucleo_port: %s\n' "$NUCLEO_PORT"
  printf 'ros_domain_id: %s\n' "${ROS_DOMAIN_ID:-0}"
  printf 'start_at: %s\n' "$START_AT"
  printf 'reuse_existing_sync: %s\n' "$REUSE_EXISTING_SYNC"
  printf 'route_count: 20\n'
} > "$RUN_DIR/run_manifest.yaml"

exec > >(tee -a "$RUN_DIR/batch.log") 2>&1
echo "run_dir: $RUN_DIR"

for index in "${!RECORDINGS[@]}"; do
  sequence=$((index + 1))
  ((sequence >= START_AT)) || continue
  trajectory="${TRAJECTORIES[$index]}"
  recording="${RECORDINGS[$index]}"
  play_log="$RUN_DIR/logs/${sequence}_${trajectory}.log"
  reuse_args=()
  if ((sequence > START_AT || REUSE_EXISTING_SYNC == 1)); then
    reuse_args+=(--nucleo-reuse-sync)
  fi

  echo "===== [$sequence/20] $trajectory ====="
  set +e
  ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" play \
    --env "$ENVIRONMENT_ID" --recording "$recording" --auto-prepare \
    --record-nucleo --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 \
    --nucleo-max-incomplete-groups 10 --nucleo-max-channel-sequence-drops 10 \
    "${reuse_args[@]}" 2>&1 | tee "$play_log"
  play_status=${PIPESTATUS[0]}
  set -e

  nucleo_telemetry="$(sed -n 's/^NUCLEO telemetry: //p' "$play_log" | tail -1)"
  nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1)"
  printf '%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$trajectory" "$recording" "$play_status" \
    "$nucleo_telemetry" "$nucleo_manifest" "$play_log" >> "$SUMMARY"
  ((play_status == 0)) || fail "trajectory $sequence failed with status $play_status"
done

echo "All 20 real robot + NUCLEO recordings completed. Summary: $SUMMARY"
