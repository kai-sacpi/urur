#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
ENVIRONMENT_PATH="$REPOSITORY_ROOT/environments/$ENVIRONMENT_ID"
NUCLEO_PORT="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00"
RUN_ROOT=""
START_SEQUENCE=1
PERFORM_INITIAL_SYNC=0
MAX_INCOMPLETE=10
MAX_CHANNEL_DROPS=10
MAX_DUPLICATE_PACKETS=10
MAX_REORDERED_PACKETS=10

fail() { echo "run_mla_d16_c16_paired_real_nucleo.sh: $*" >&2; exit 1; }

while (($#)); do
  case "$1" in
    --port) (($# >= 2)) || fail "--port requires a device"; NUCLEO_PORT="$2"; shift 2 ;;
    --run-root) (($# >= 2)) || fail "--run-root requires a directory"; RUN_ROOT="$2"; shift 2 ;;
    --start-sequence) (($# >= 2)) || fail "--start-sequence requires an integer"; START_SEQUENCE="$2"; shift 2 ;;
    --perform-initial-sync) PERFORM_INITIAL_SYNC=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--port DEVICE] [--run-root DIR] [--start-sequence N] [--perform-initial-sync]"
      exit 0
      ;;
    *) fail "unknown argument: $1" ;;
  esac
done

[[ "$START_SEQUENCE" =~ ^([1-9]|1[0-2])$ ]] || fail "--start-sequence must be in 1..12"
if ((START_SEQUENCE > 1 && PERFORM_INITIAL_SYNC == 1)); then
  fail "--perform-initial-sync is only valid when starting at sequence 1"
fi

[[ -d "$ENVIRONMENT_PATH" ]] || fail "missing environment: $ENVIRONMENT_PATH"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || \
  fail "NUCLEO port is not readable/writable: $NUCLEO_PORT"
[[ -r /opt/ros/jazzy/setup.bash && -r "$REPOSITORY_ROOT/install/setup.bash" ]] || \
  fail "ROS Jazzy workspace is not built"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPOSITORY_ROOT/install/setup.bash"
set -u

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || \
  fail "/hx_waypoint/play_recording must have exactly one server"
controller_state="$(timeout 15 ros2 control list_controllers | awk '$1=="joint_trajectory_controller" {print $3}')"
[[ "$controller_state" == "active" ]] || fail "joint_trajectory_controller is not active: $controller_state"
speed_value="$(timeout 5 ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once | awk '/^data:/ {print $2; exit}')"
python3 - "$speed_value" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except (IndexError, ValueError):
    raise SystemExit("speed scaling unavailable")
if not math.isfinite(value) or abs(value - 100.0) > 1e-9:
    raise SystemExit(f"speed scaling must be 100.0, got {value}")
PY

read -r HOME1 HOME2 HOME3 HOME4 HOME5 HOME6 < <(python3 - "$ENVIRONMENT_PATH/robot/robot_model.yaml" <<'PY'
import sys
import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
values = data.get("home_joint_positions")
if values is None:
    values = data.get("robot", {}).get("home_joint_positions")
if not isinstance(values, list) or len(values) != 6:
    raise SystemExit("six home joints required")
print(" ".join(f"{float(value):.12f}" for value in values))
PY
)

SCHEDULE=(
  $'z070_d16\tdifficult\tmla_z070_d16_xp050_ym050__20260905_142320'
  $'z070_d16\tvertical_control\tmla_grid_z070_xp050_ym050__20260904_231410'
  $'z070_c16\tdifficult\tmla_z070_c16_xm050_yp050__20260905_142353'
  $'z070_c16\tvertical_control\tmla_grid_z070_xm050_yp050__20260904_231617'
  $'z100_d16\tdifficult\tmla_z100_d16_xp050_ym050__20260905_142836'
  $'z100_d16\tvertical_control\tmla_grid_z100_xp050_ym050__20260904_233114'
  $'z100_c16\tdifficult\tmla_z100_c16_xp050_yp050__20260905_142526'
  $'z100_c16\tvertical_control\tmla_grid_z100_xp050_yp050__20260904_232750'
  $'z120_d16\tdifficult\tmla_z120_d16_xp050_ym050__20260905_142631'
  $'z120_d16\tvertical_control\tmla_grid_z120_xp050_ym050__20260904_234051'
  $'z120_c16_sym_xp050_ym050\tdifficult\tmla_z120_c16_sym_xp050_ym050__20260905_151457'
  $'z120_c16_sym_xp050_ym050\tvertical_control\tmla_grid_z120_xp050_ym050__20260904_234051'
)

for row in "${SCHEDULE[@]}"; do
  IFS=$'\t' read -r _ _ recording_id <<<"$row"
  recording_root="$ENVIRONMENT_PATH/recordings/$recording_id"
  [[ -r "$recording_root/manifest.yaml" && -r "$recording_root/joint_trajectory.csv" ]] || \
    fail "missing sealed recording: $recording_id"
  python3 - "$recording_root/manifest.yaml" "$recording_id" <<'PY'
import sys
import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
if not isinstance(data, dict) or data.get("status") != "validated":
    raise SystemExit(f"recording is not validated: {sys.argv[2]}")
if data.get("environment_id") != "mla" or data.get("recording_id") != sys.argv[2]:
    raise SystemExit(f"recording authority mismatch: {sys.argv[2]}")
PY
done

if [[ -z "$RUN_ROOT" ]]; then
  RUN_ROOT="/data/ur-waypoint/mla_d16_c16_paired_real/$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "$RUN_ROOT/artifacts" "$RUN_ROOT/logs"
exec 9>"/data/ur-waypoint/mla_d16_c16_paired_real/.batch.lock"
flock -n 9 || fail "another paired MLA real batch is already running"
LOG_PATH="$RUN_ROOT/batch.log"
exec > >(tee -a "$LOG_PATH") 2>&1

RESULTS_PATH="$RUN_ROOT/results.csv"
if ((START_SEQUENCE == 1)); then
  printf 'sequence,pair_id,role,recording_id,start_utc,end_utc,play_exit_code,decision,robot_telemetry,nucleo_manifest,durable_artifact\n' > "$RESULTS_PATH"
else
  [[ -s "$RESULTS_PATH" ]] || fail "resume requires existing results: $RESULTS_PATH"
fi

move_home() {
  python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" \
    --target "$HOME1" "$HOME2" "$HOME3" "$HOME4" "$HOME5" "$HOME6" --execute
}

bounded_sensor_loss_ok() {
  local manifest_path="$1"
  [[ -n "$manifest_path" && -f "$manifest_path" ]] || return 1
  python3 - "$manifest_path" "$MAX_INCOMPLETE" "$MAX_CHANNEL_DROPS" "$MAX_DUPLICATE_PACKETS" "$MAX_REORDERED_PACKETS" <<'PY'
import sys
import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
capture = data.get("capture") if isinstance(data, dict) else None
mapping = data.get("mapping") if isinstance(data, dict) else None
if data.get("status") != "incomplete" or not isinstance(capture, dict) or capture.get("capture_error") is not None:
    raise SystemExit(1)
if not isinstance(mapping, dict) or not isinstance(mapping.get("parser_integrity_deltas"), dict):
    raise SystemExit(1)
deltas = mapping["parser_integrity_deltas"]
limits = {
    "parser_incomplete_groups": int(sys.argv[2]),
    "parser_channel_sequence_drops": int(sys.argv[3]),
    "parser_duplicate_packets": int(sys.argv[4]),
    "parser_reordered_packets": int(sys.argv[5]),
}
for key, value in deltas.items():
    value = int(value)
    if value < 0 or (key in limits and value > limits[key]) or (key not in limits and value != 0):
        raise SystemExit(1)
if int(mapping.get("offset_ns", {}).get("maximum_absolute", 0)) > 50_000_000:
    raise SystemExit(1)
PY
}

echo "batch_root: $RUN_ROOT"
echo "environment: $ENVIRONMENT_ID"
echo "capture_count: ${#SCHEDULE[@]}"
if ((PERFORM_INITIAL_SYNC == 1)); then
  echo "nucleo_sync: perform_once_then_reuse"
else
  echo "nucleo_sync: reuse_existing_stream"
fi

previous_pair=""
sequence=0
for row in "${SCHEDULE[@]}"; do
  IFS=$'\t' read -r pair_id role recording_id <<<"$row"
  sequence=$((sequence + 1))
  ((sequence < START_SEQUENCE)) && continue
  if [[ "$pair_id" != "$previous_pair" ]]; then
    move_home
    previous_pair="$pair_id"
  fi

  echo "===== [$sequence/${#SCHEDULE[@]}] $pair_id $role ====="
  recording_root="$ENVIRONMENT_PATH/recordings/$recording_id"
  start_values="$(awk -F, 'NR==2 {for (i=2;i<=7;++i) printf "%s%s",$i,(i<7?" ":""); exit}' "$recording_root/joint_trajectory.csv")"
  read -r START1 START2 START3 START4 START5 START6 <<<"$start_values"
  [[ -n "$START6" ]] || fail "cannot read sealed start joints: $recording_id"
  python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" \
    --target "$START1" "$START2" "$START3" "$START4" "$START5" "$START6" --execute

  start_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
  play_log="$RUN_ROOT/logs/$(printf '%02d' "$sequence")__${pair_id}__${role}.log"
  nucleo_sync_args=(--nucleo-reuse-sync)
  if ((sequence == 1 && PERFORM_INITIAL_SYNC == 1)); then
    nucleo_sync_args=()
  fi
  set +e
  set -o pipefail
  ros2 run hx_waypoint_tools tipdraw --workspace "$REPOSITORY_ROOT" play \
    --env "$ENVIRONMENT_ID" --recording "$recording_id" \
    --externally-prepared-start --record-nucleo "${nucleo_sync_args[@]}" \
    --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 \
    --nucleo-max-incomplete-groups "$MAX_INCOMPLETE" \
    --nucleo-max-channel-sequence-drops "$MAX_CHANNEL_DROPS" 2>&1 | tee "$play_log"
  play_rc=$?
  set +o pipefail
  set -e
  end_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"

  robot_telemetry="$(grep -E '^/[^ ]+/execution_telemetry\.csv$' "$play_log" | tail -1 || true)"
  nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1 || true)"
  if [[ -z "$robot_telemetry" && -f "$nucleo_manifest" ]]; then
    robot_telemetry="$(python3 - "$nucleo_manifest" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
print(data.get("artifacts", {}).get("robot_telemetry_csv", {}).get("path", ""))
PY
)"
  fi
  decision="COMPLETE"
  manifest_status="$(python3 - "$nucleo_manifest" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
print(data.get("status", ""))
PY
)"
  if [[ "$manifest_status" == "incomplete" ]] && bounded_sensor_loss_ok "$nucleo_manifest"; then
    decision="ACCEPTED_BOUNDED_SENSOR_LOSS"
  elif ((play_rc != 0)) || [[ "$manifest_status" != "complete" ]]; then
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$pair_id" "$role" "$recording_id" "$start_utc" "$end_utc" "$play_rc" "REJECTED" "$robot_telemetry" "$nucleo_manifest" "" >> "$RESULTS_PATH"
    fail "unaccepted playback/capture failure at sequence $sequence; see $play_log"
  fi

  durable_artifact=""
  if [[ -n "$robot_telemetry" && -f "$robot_telemetry" ]]; then
    source_run="$(dirname -- "$(dirname -- "$robot_telemetry")")"
    durable_artifact="$RUN_ROOT/artifacts/$(printf '%02d' "$sequence")__${pair_id}__${role}"
    mkdir -p "$durable_artifact"
    rsync -a "$source_run/" "$durable_artifact/"
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$pair_id" "$role" "$recording_id" "$start_utc" "$end_utc" "$play_rc" "$decision" "$robot_telemetry" "$nucleo_manifest" "$durable_artifact" >> "$RESULTS_PATH"
done

move_home
(
  cd "$(dirname -- "$RUN_ROOT")"
  archive="mla_d16_c16_paired_real_$(basename -- "$RUN_ROOT").tar.gz"
  tar -czf "$archive" "$(basename -- "$RUN_ROOT")"
  sha256sum "$archive" > "$archive.sha256"
)

echo "All ${#SCHEDULE[@]} paired real robot + NUCLEO captures completed."
echo "results: $RESULTS_PATH"
echo "log: $LOG_PATH"
