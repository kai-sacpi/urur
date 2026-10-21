#!/usr/bin/env bash
# Own one complete mla_short stack. Never kill or replace an existing driver.
set -Eeo pipefail
set +u

PROJECT_ROOT=/home/huaxin/Codes/ur-waypoint
ROS_SETUP=/opt/ros/jazzy/setup.bash
ROBOT_IP=192.168.1.99
LAUNCH_RVIZ=true
MODE=start

usage() {
  printf '%s\n' \
    'Usage: bash workstation_magtrack/ops/start_mla_short.sh [OPTIONS]' \
    '  --robot-ip ADDRESS   UR5 address (default 192.168.1.99).' \
    '  --launch-rviz BOOL   true or false (default true).' \
    '  --dry-run            Show command only; no files sourced or hardware accessed.' \
    '  --env-report         Source and verify dependencies only; no ROS nodes started.' \
    '  --project-root PATH  Explicit installation root.' \
    '  --ros-setup FILE     Explicit ROS setup file.' \
    'Never stops/restarts an existing stack, plays the pendant, or opens NUCLEO.' \
    'After startup, press Run / External Control on the UR5 pendant.'
}

fail() { printf 'start_mla_short: %s\n' "$*" >&2; exit 2; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --dry-run|--env-report)
      [[ "$MODE" == start ]] || fail 'choose only one mode'
      MODE="${1#--}"; shift ;;
    --project-root|--ros-setup|--robot-ip|--launch-rviz)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "$1 requires a value"
      case "$1" in
        --project-root) PROJECT_ROOT="$2" ;;
        --ros-setup) ROS_SETUP="$2" ;;
        --robot-ip) ROBOT_IP="$2" ;;
        --launch-rviz) LAUNCH_RVIZ="$2" ;;
      esac
      shift 2 ;;
    *) fail "unknown option: $1" ;;
  esac
done
[[ "$PROJECT_ROOT" == /* && "$ROS_SETUP" == /* ]] || fail 'setup paths must be absolute'
[[ "$LAUNCH_RVIZ" == true || "$LAUNCH_RVIZ" == false ]] || fail '--launch-rviz must be true or false'
[[ "$ROBOT_IP" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || fail '--robot-ip must be an IPv4 address'
IFS=. read -r -a octets <<< "$ROBOT_IP"
for octet in "${octets[@]}"; do
  (( 10#$octet <= 255 )) || fail '--robot-ip octet exceeds 255'
done
WORKSPACE="$PROJECT_ROOT/workstation_magtrack"
COMMAND=(ros2 launch hx_waypoint_bringup real.launch.py
  "robot_ip:=$ROBOT_IP" "workspace:=$WORKSPACE"
  environment_id:=mla_short "launch_rviz:=$LAUNCH_RVIZ")
print_command() { printf 'command:'; printf ' %q' "${COMMAND[@]}"; printf '\n'; }
if [[ "$MODE" == dry-run ]]; then
  printf 'mode=dry-run (no source, ROS, GPU, serial, or motion)\n'
  printf 'shared_environment=%q\n' "$PROJECT_ROOT/workstation_magtrack/ops/ros_environment.sh"
  print_command
  exit 0
fi

HELPER="$PROJECT_ROOT/workstation_magtrack/ops/ros_environment.sh"
[[ -f "$HELPER" && -r "$HELPER" ]] || fail "shared environment helper missing: $HELPER"
PHASE='load shared ROS / locked planner environment'
trap 'status=$?; printf "start_mla_short: %s failed (exit %s); no automatic restart.\n" "$PHASE" "$status" >&2; exit "$status"' ERR
source "$HELPER"
wm_load_environment "$PROJECT_ROOT" "$ROS_SETUP"
PHASE='verify locked planner imports (no warmup or motion)'
wm_verify_planner_imports
cd "$PROJECT_ROOT"
if [[ "$MODE" == env-report ]]; then
  printf 'mode=env-report (dependencies verified; no ROS nodes started)\n'
  for name in VIRTUAL_ENV PYTHONPATH RMW_IMPLEMENTATION ROS_DOMAIN_ID \
    ROS_AUTOMATIC_DISCOVERY_RANGE FASTRTPS_DEFAULT_PROFILES_FILE; do
    printf '%s=%q\n' "$name" "${!name}"
  done
  print_command
  exit 0
fi

PHASE='refuse duplicate driver / runtime'
command -v flock >/dev/null || fail 'flock is required'
command -v timeout >/dev/null || fail 'timeout is required'
# DDS discovery cannot see a driver started with another domain/profile.
# Inspect concrete local executable names as well; do not match shell command
# text, which can contain these names without running a robot process.
PROCESSES=$(ps -eo pid=,comm=) || fail 'could not inspect local stack processes'
while read -r pid process; do
  case "$process" in
    ros2_control_node|ros2_control_no|ur_ros2_control_node|ur_ros2_control|runtime_node|curobo_server|move_group)
      fail "existing local stack process detected ($process, pid $pid); not starting a second driver" ;;
  esac
done <<< "$PROCESSES"
umask 077
exec 9>"$WORKSPACE/.mla_stack.lock"
flock -n 9 || fail 'another start_mla_short process owns the stack lock'
NODES=$(timeout 12 ros2 node list --no-daemon) || fail 'could not verify that ROS is free; existing stack was not touched'
while IFS= read -r node; do
  case "${node##*/}" in
    controller_manager|hx_waypoint_runtime|hx_waypoint_curobo_server|move_group)
      fail "existing ROS stack detected ($node); not starting a second driver" ;;
  esac
done <<< "$NODES"
printf 'Dependencies verified. Start External Control / press Run on the UR5 pendant after the driver connects.\n'
print_command
PHASE='run the complete mla_short stack'
exec "${COMMAND[@]}"
