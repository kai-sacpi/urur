#!/usr/bin/env bash
# One collection process, using the existing mla_short stack. No retry or SYNC
# policy is invented here: collection.run receives the operator's exact flags.
set -Eeo pipefail
set +u  # ROS setup files are not safe under nounset.

PROJECT_ROOT=/home/huaxin/Codes/ur-waypoint
ROS_SETUP=/opt/ros/jazzy/setup.bash
MODE=collect

usage() {
  printf '%s\n' \
    'Usage: bash workstation_magtrack/ops/collect_reliable.sh [launcher options] [--] COLLECT_ARGS...' \
    '' \
    'COLLECT_ARGS are passed unchanged to collection.run collect, e.g.:' \
    '  --manifest FILE --output NEW_RUN --boot-id BOOT --port DEVICE --initial-background' \
    '  Add --reuse-nucleo-stream only for an already initialized hardware boot.' \
    '' \
    'Launcher options (place before COLLECT_ARGS):' \
    '  --dry-run             Print planned environment/command; no source, checks, or execution.' \
    '  --env-report          Check/source the environment and print it; never run collection.' \
    '  --dependency-preflight  CPU-only imports using installed planner shebang; no collection or GPU warmup.' \
    '  --project-root PATH   Explicit installation path (default /home/huaxin/Codes/ur-waypoint).' \
    '  --ros-setup FILE      Explicit ROS setup (default /opt/ros/jazzy/setup.bash).' \
    '  --help                Show this help without sourcing or executing anything.' \
    '' \
    'Never starts/restarts ROS, drivers, or controllers; never retries a failed run.'
}

fail() { printf 'collect_reliable: %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --dry-run|--env-report|--dependency-preflight)
      [[ "$MODE" == collect ]] || fail 'choose only one report/preflight mode'
      MODE="${1#--}"
      shift ;;
    --project-root|--ros-setup)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "$1 requires a path"
      case "$1" in
        --project-root) PROJECT_ROOT="$2" ;;
        --ros-setup) ROS_SETUP="$2" ;;
      esac
      shift 2 ;;
    --) shift; break ;;
    *) break ;;
  esac
done

[[ "$PROJECT_ROOT" == /* ]] || fail '--project-root must be absolute'
[[ "$ROS_SETUP" == /* ]] || fail '--ros-setup must be absolute'
WORKSPACE="$PROJECT_ROOT/workstation_magtrack"
PYTHON="$PROJECT_ROOT/.venv/bin/python"
PROFILE="$WORKSPACE/ops/fastdds_loopback_udp.xml"
COLLECTOR=("$PYTHON" -m collection.run --workspace "$WORKSPACE" collect "$@")
SOURCE_PYTHONPATH="$WORKSPACE:$PROJECT_ROOT/src/hx_waypoint_curobo:$PROJECT_ROOT/src/hx_waypoint_tools:$PROJECT_ROOT/src/hx_waypoint_nucleo:$PROJECT_ROOT/.deps/curobo-<commit from workspace/curobo.lock>:$PROJECT_ROOT/.venv/lib/python<locked ABI>/site-packages"

print_command() {
  printf 'command:'
  printf ' %q' "${COLLECTOR[@]}"
  printf '\n'
}

if [[ "$MODE" == dry-run ]]; then
  printf 'mode=dry-run (planned only; no files sourced or hardware opened)\n'
  printf 'source=%q\n' "$ROS_SETUP" "$PROJECT_ROOT/install/setup.bash" "$PROJECT_ROOT/.venv/bin/activate"
  printf 'cwd=%q\n' "$WORKSPACE"
  printf 'PYTHONPATH_prefix=%q\n' "$SOURCE_PYTHONPATH"
  printf 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp\nROS_DOMAIN_ID=0\nROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT\n'
  printf 'FASTRTPS_DEFAULT_PROFILES_FILE=%q\n' "$PROFILE"
  printf 'unset=ROS_LOCALHOST_ONLY,FASTDDS_BUILTIN_TRANSPORTS\n'
  print_command
  exit 0
fi

PHASE='load the shared locked ROS/Python environment'
trap 'status=$?; printf "collect_reliable: %s failed (exit %s); collector not retried.\n" "$PHASE" "$status" >&2; exit "$status"' ERR
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros_environment.sh"
wm_load_environment "$PROJECT_ROOT" "$ROS_SETUP"
PHASE="enter workspace: $WORKSPACE"
cd "$WORKSPACE"

if [[ "$MODE" == env-report ]]; then
  printf 'mode=env-report (collection not executed)\n'
  wm_report_environment
  printf 'cwd=%q\n' "$PWD"
  printf 'unset=ROS_LOCALHOST_ONLY,FASTDDS_BUILTIN_TRANSPORTS\n'
  print_command
  exit 0
fi

if [[ "$MODE" == dependency-preflight ]]; then
  wm_verify_planner_imports
  exit 0
fi

PHASE='execute collection.run using the checked project interpreter'
exec "${COLLECTOR[@]}"
