#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RUN_TESTS=0
SKIP_ASSETS=0
ENVIRONMENT_ID="lab_wall_a"

usage() {
  cat <<'EOF'
Usage: scripts/build_workspace.sh [--test] [--skip-assets] [--env ID]

Build the ROS 2 Jazzy workspace without accessing the network.

  --test         run colcon tests and repository pytest after building
  --skip-assets  do not regenerate the controlled URDF/provenance assets
  --env ID       environment used for asset generation (default: lab_wall_a)
EOF
}

fail() {
  echo "build_workspace.sh: $*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --test)
      RUN_TESTS=1
      shift
      ;;
    --skip-assets)
      SKIP_ASSETS=1
      shift
      ;;
    --env)
      (($# >= 2)) || fail "--env requires an identifier"
      ENVIRONMENT_ID="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

[[ "${ENVIRONMENT_ID}" =~ ^[A-Za-z0-9_-]+$ ]] || fail "unsafe environment id"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "/opt/ros/jazzy/setup.bash is missing"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  fail "ROS_DISTRO=${ROS_DISTRO}; start a clean shell before sourcing ROS 2 Jazzy"
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == "jazzy" ]] || fail "failed to activate ROS 2 Jazzy"

[[ -f "${REPOSITORY_ROOT}/.venv/pyvenv.cfg" ]] || \
  fail "missing .venv; run scripts/setup_ubuntu24.sh first"
set +u
# shellcheck disable=SC1090
source "${REPOSITORY_ROOT}/.venv/bin/activate"
set -u
export PYTHONNOUSERSITE=1

command -v python >/dev/null 2>&1 || fail "venv Python is unavailable"
python -c 'import colcon_core.command' >/dev/null 2>&1 || \
  fail "colcon_core is unavailable through the --system-site-packages venv"
COLCON=(
  python
  -c
  'import sys; from colcon_core.command import main; sys.argv[0] = "colcon"; raise SystemExit(main())'
)

URDF_PATH="${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/robot/assets/ur5.urdf"
PROVENANCE_PATH="${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/robot/asset_provenance.yaml"
if ((SKIP_ASSETS == 1)); then
  [[ -s "${URDF_PATH}" ]] || fail "missing controlled URDF asset: ${URDF_PATH}"
  [[ -s "${PROVENANCE_PATH}" ]] || \
    fail "missing controlled asset provenance: ${PROVENANCE_PATH}"
else
  "${SCRIPT_DIR}/generate_ur_assets.sh" --env "${ENVIRONMENT_ID}" --ur-type ur5
fi

cd "${REPOSITORY_ROOT}"
"${COLCON[@]}" build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

set +u
# shellcheck disable=SC1090
source "${REPOSITORY_ROOT}/install/setup.bash"
set -u

if ((RUN_TESTS == 1)); then
  # The Python packages are covered by the repository-level pytest suite below.
  # Running colcon's pytest extension in those package roots finds no tests and
  # returns pytest exit code 5, so colcon is reserved for the CMake/GTest package.
  "${COLCON[@]}" test \
    --packages-select hx_waypoint_runtime \
    --event-handlers console_direct+
  "${COLCON[@]}" test-result --verbose
  # ROS Jazzy exposes apt-managed pytest plugins through the system-site venv.
  # They are not used by this repository suite and older launch_testing hooks
  # are incompatible with newer venv pytest releases, so do not auto-load them.
  PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    python -m pytest -p no:cacheprovider tests -q
fi

cat <<EOF
Build completed. In a new terminal run:
  source /opt/ros/jazzy/setup.bash
  source ${REPOSITORY_ROOT}/.venv/bin/activate
  export PYTHONNOUSERSITE=1
  source ${REPOSITORY_ROOT}/install/setup.bash
EOF
