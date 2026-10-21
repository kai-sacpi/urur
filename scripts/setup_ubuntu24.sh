#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_PATH="${REPOSITORY_ROOT}/.venv"
ASSUME_YES=0

usage() {
  cat <<'EOF'
Usage: scripts/setup_ubuntu24.sh [--yes] [--venv PATH]

Create a Python 3.12 --system-site-packages venv and install the locked
Python/cuRobo dependencies for Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.8.

  --yes        acknowledge the displayed network operations non-interactively
  --venv PATH  use a venv path other than <repository>/.venv

This script never installs, upgrades, or reloads the NVIDIA driver.
EOF
}

fail() {
  echo "setup_ubuntu24.sh: $*" >&2
  exit 1
}

note() {
  echo "[setup] $*"
}

warn() {
  echo "[setup][WARNING] $*" >&2
}

while (($#)); do
  case "$1" in
    --yes)
      ASSUME_YES=1
      shift
      ;;
    --venv)
      (($# >= 2)) || fail "--venv requires a path"
      VENV_PATH="$2"
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

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "/opt/ros/jazzy/setup.bash is missing"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  fail "ROS_DISTRO=${ROS_DISTRO}; start a clean shell before sourcing ROS 2 Jazzy"
fi
# This is deliberately the only ROS distribution sourced by this script.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == "jazzy" ]] || fail "failed to activate ROS 2 Jazzy"

command -v python3.12 >/dev/null 2>&1 || fail "python3.12 is required"
command -v git >/dev/null 2>&1 || fail "git is required for the locked cuRobo checkout"
command -v dpkg >/dev/null 2>&1 || fail "dpkg is required for driver version comparison"
command -v rosdep >/dev/null 2>&1 || fail "rosdep is required for ROS dependency installation"

# A venv created with --system-site-packages is required to see ROS Jazzy's
# apt-managed Python modules, but user-site/Conda injection would make the
# locked CUDA environment ambiguous.  Keep only the venv, ROS and OS paths.
export PYTHONNOUSERSITE=1
# The CUDA/cuRobo dependency closure contains several multi-megabyte wheels.
# Remote installations may traverse a slow proxy, so use bounded retries and a
# longer per-read timeout instead of failing halfway through an otherwise
# reproducible setup.  Callers can still override either value explicitly.
export PIP_DEFAULT_TIMEOUT="${PIP_DEFAULT_TIMEOUT:-180}"
export PIP_RETRIES="${PIP_RETRIES:-20}"
export PIP_DISABLE_PIP_VERSION_CHECK=1

LOCK_FILE="${REPOSITORY_ROOT}/curobo.lock"
PYTHON_LOCK="${REPOSITORY_ROOT}/src/hx_waypoint_curobo/requirements.lock"
[[ -f "${LOCK_FILE}" ]] || fail "missing ${LOCK_FILE}"
[[ -f "${PYTHON_LOCK}" ]] || fail "missing ${PYTHON_LOCK}"

lock_value() {
  local key="$1"
  awk -v wanted="${key}" '
    $0 ~ "^" wanted ":[[:space:]]*" {
      value = $0
      sub("^[^:]+:[[:space:]]*", "", value)
      gsub(/^\"|\"$/, "", value)
      print value
      exit
    }
  ' "${LOCK_FILE}"
}

CUROBO_REPOSITORY="$(lock_value repository)"
CUROBO_COMMIT="$(lock_value commit)"
CUROBO_SOURCE_TREE="$(lock_value source_tree)"
PYTHON_ABI="$(lock_value python_abi)"
CUDA_TOOLKIT="$(lock_value cuda_toolkit)"
PYTORCH_INDEX="$(lock_value pytorch_index)"
PYTORCH_VERSION="$(lock_value pytorch_version)"
MINIMUM_DRIVER="$(lock_value minimum_nvidia_driver)"

[[ "${CUROBO_REPOSITORY}" == "https://github.com/NVlabs/curobo.git" ]] || \
  fail "unexpected cuRobo repository in lock: ${CUROBO_REPOSITORY}"
[[ "${CUROBO_COMMIT}" =~ ^[0-9a-f]{40}$ ]] || fail "curobo.lock commit is not immutable"
[[ "${CUROBO_SOURCE_TREE}" =~ ^[0-9a-f]{40}$ ]] || \
  fail "curobo.lock source_tree is invalid"
[[ -n "${PYTORCH_INDEX}" ]] || fail "curobo.lock is missing pytorch_index"
[[ "${PYTORCH_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\+cu[0-9]+$ ]] || \
  fail "curobo.lock pytorch_version is not an exact CUDA wheel version"
EXPECTED_PYTORCH_CUDA_TAG="cu${CUDA_TOOLKIT//./}"
[[ "${PYTORCH_VERSION}" == *"+${EXPECTED_PYTORCH_CUDA_TAG}" ]] || \
  fail "PyTorch ${PYTORCH_VERSION} does not match CUDA toolkit ${CUDA_TOOLKIT}"
[[ -n "${MINIMUM_DRIVER}" ]] || fail "curobo.lock is missing minimum_nvidia_driver"

CURRENT_PYTHON_ABI="$(python3.12 -c 'import sys; print(f"cp{sys.version_info.major}{sys.version_info.minor}")')"
[[ "${CURRENT_PYTHON_ABI}" == "${PYTHON_ABI}" ]] || \
  fail "Python ABI ${CURRENT_PYTHON_ABI} does not match lock ${PYTHON_ABI}"

EXPECTED_CUDA_HOME="/usr/local/cuda"
if [[ -n "${CUDA_HOME:-}" && "${CUDA_HOME}" != "${EXPECTED_CUDA_HOME}" ]]; then
  fail "CUDA_HOME must be ${EXPECTED_CUDA_HOME}, got ${CUDA_HOME}"
fi
export CUDA_HOME="${EXPECTED_CUDA_HOME}"
[[ -x "${CUDA_HOME}/bin/nvcc" ]] || fail "${CUDA_HOME}/bin/nvcc is missing or not executable"
NVCC_RELEASE="$("${CUDA_HOME}/bin/nvcc" --version | sed -n 's/.*release \([0-9][0-9.]*\),.*/\1/p' | tail -n 1)"
[[ "${NVCC_RELEASE}" == "${CUDA_TOOLKIT}" ]] || \
  fail "CUDA toolkit ${NVCC_RELEASE:-unknown} does not match lock ${CUDA_TOOLKIT}"

DRIVER_READY=1
if command -v nvidia-smi >/dev/null 2>&1; then
  mapfile -t DRIVER_VERSIONS < <(
    nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | sed '/^[[:space:]]*$/d'
  )
  if ((${#DRIVER_VERSIONS[@]} == 0)); then
    DRIVER_READY=0
    warn "nvidia-smi returned no GPU driver version"
  else
    for driver_version in "${DRIVER_VERSIONS[@]}"; do
      driver_version="${driver_version//[[:space:]]/}"
      if dpkg --compare-versions "${driver_version}" lt "${MINIMUM_DRIVER}"; then
        DRIVER_READY=0
        warn "NVIDIA driver ${driver_version} is below cuRobo minimum ${MINIMUM_DRIVER}"
      else
        note "NVIDIA driver ${driver_version} satisfies minimum ${MINIMUM_DRIVER}"
      fi
    done
  fi
else
  DRIVER_READY=0
  warn "nvidia-smi is unavailable; GPU readiness cannot be proven"
fi

if [[ -e "${VENV_PATH}" && ! -f "${VENV_PATH}/pyvenv.cfg" ]]; then
  fail "venv target exists but is not a Python venv: ${VENV_PATH}"
fi
if [[ ! -f "${VENV_PATH}/pyvenv.cfg" ]]; then
  note "creating Python 3.12 venv with --system-site-packages at ${VENV_PATH}"
  python3.12 -m venv --system-site-packages "${VENV_PATH}"
fi
if ! grep -Eiq '^include-system-site-packages[[:space:]]*=[[:space:]]*true$' \
  "${VENV_PATH}/pyvenv.cfg"; then
  fail "existing venv was not created with --system-site-packages: ${VENV_PATH}"
fi
set +u
# shellcheck disable=SC1090
source "${VENV_PATH}/bin/activate"
set -u
[[ "$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == "3.12" ]] || \
  fail "activated venv is not Python 3.12"
VENV_PATH_ENV="${VENV_PATH}" python -c '
import os
import sys
from pathlib import Path

allowed = (
    Path(os.environ["VENV_PATH_ENV"]).resolve(),
    Path("/opt/ros/jazzy").resolve(),
    Path("/usr/lib").resolve(),
    Path("/usr/local/lib").resolve(),
)
foreign = []
for raw in sys.path:
    if not raw or "site-packages" not in raw:
        continue
    path = Path(raw).resolve()
    if not any(path == root or root in path.parents for root in allowed):
        foreign.append(str(path))
if foreign:
    raise SystemExit(
        "venv contains foreign site-packages path injection; remove the responsible .pth: "
        + ", ".join(sorted(set(foreign)))
    )
'

cat <<EOF

[setup][NETWORK REVIEW]
The next step will access the network and install Python packages into:
  ${VENV_PATH}

Network sources:
  - Ubuntu/ROS package repositories selected by rosdep
  - Python package indexes used by pip
  - PyTorch CUDA index: ${PYTORCH_INDEX}
  - Exact PyTorch wheel: torch==${PYTORCH_VERSION}
  - Locked cuRobo Git repository: ${CUROBO_REPOSITORY}

Locked cuRobo identity:
  commit: ${CUROBO_COMMIT}
  source tree: ${CUROBO_SOURCE_TREE}

The source step fetches only the locked commit, checks out detached HEAD, and
verifies both HEAD and HEAD^{tree}. GitHub-generated source archives are not used.

rosdep may invoke the OS package manager for dependencies declared by package.xml.
This script does not request an NVIDIA driver or CUDA toolkit package, reload
kernel modules, or reboot the host.
EOF

if ((ASSUME_YES == 0)); then
  if [[ ! -t 0 ]]; then
    fail "network confirmation requires an interactive terminal or --yes"
  fi
  read -r -p "Type 'yes' to continue with the network installation: " answer
  [[ "${answer}" == "yes" ]] || fail "network installation was not approved"
fi

note "installing ROS/package.xml dependencies through rosdep"
rosdep install \
  --from-paths "${REPOSITORY_ROOT}/src" \
  --ignore-src \
  --rosdistro jazzy \
  --skip-keys ament_python \
  -r \
  -y

note "installing project-owned locked Python dependencies"
python -m pip install --requirement "${PYTHON_LOCK}"
python -m pip install --requirement "${REPOSITORY_ROOT}/requirements-dev.txt"

note "installing PyTorch from the locked CUDA index"
python -m pip install --index-url "${PYTORCH_INDEX}" "torch==${PYTORCH_VERSION}"

DEPENDENCY_DIR="${REPOSITORY_ROOT}/.deps"
SOURCE_DIR="${DEPENDENCY_DIR}/curobo-${CUROBO_COMMIT}"
mkdir -p "${DEPENDENCY_DIR}"

TEMP_SOURCE=""
cleanup() {
  [[ -z "${TEMP_SOURCE}" || ! -e "${TEMP_SOURCE}" ]] || rm -rf -- "${TEMP_SOURCE}"
}
trap cleanup EXIT

verify_curobo_checkout() {
  local source_dir="$1"
  local source_real git_top origin_url actual_head actual_tree status_output

  [[ -d "${source_dir}" && ! -L "${source_dir}" ]] || \
    fail "cuRobo source must be a real directory: ${source_dir}"
  [[ -d "${source_dir}/.git" ]] || \
    fail "cuRobo source is not a standalone Git checkout: ${source_dir}"
  source_real="$(cd -- "${source_dir}" && pwd -P)"
  git_top="$(git -C "${source_dir}" rev-parse --show-toplevel 2>/dev/null)" || \
    fail "cannot inspect cuRobo Git checkout: ${source_dir}"
  [[ "${git_top}" == "${source_real}" ]] || \
    fail "cuRobo Git top-level does not match source directory"

  origin_url="$(git -C "${source_dir}" remote get-url origin 2>/dev/null)" || \
    fail "cuRobo checkout has no verifiable origin remote"
  [[ "${origin_url}" == "${CUROBO_REPOSITORY}" ]] || \
    fail "cuRobo origin ${origin_url} does not match lock ${CUROBO_REPOSITORY}"
  actual_head="$(git -C "${source_dir}" rev-parse --verify HEAD 2>/dev/null)" || \
    fail "cannot resolve cuRobo checkout HEAD"
  [[ "${actual_head}" == "${CUROBO_COMMIT}" ]] || \
    fail "cuRobo checkout HEAD ${actual_head} does not match lock ${CUROBO_COMMIT}"
  actual_tree="$(git -C "${source_dir}" rev-parse --verify 'HEAD^{tree}' 2>/dev/null)" || \
    fail "cannot resolve cuRobo checkout source tree"
  [[ "${actual_tree}" == "${CUROBO_SOURCE_TREE}" ]] || \
    fail "cuRobo source tree ${actual_tree} does not match lock ${CUROBO_SOURCE_TREE}"

  [[ -f "${source_dir}/.ur-waypoint-curobo-commit" ]] || \
    fail "cuRobo checkout has no commit marker"
  [[ "$(<"${source_dir}/.ur-waypoint-curobo-commit")" == "${CUROBO_COMMIT}" ]] || \
    fail "cuRobo commit marker does not match lock"
  [[ -f "${source_dir}/.ur-waypoint-curobo-tree" ]] || \
    fail "cuRobo checkout has no source-tree marker"
  [[ "$(<"${source_dir}/.ur-waypoint-curobo-tree")" == "${CUROBO_SOURCE_TREE}" ]] || \
    fail "cuRobo source-tree marker does not match lock"

  status_output="$(git -C "${source_dir}" status --porcelain=v1 --untracked-files=all)" || \
    fail "cannot verify cuRobo checkout cleanliness"
  [[ -z "${status_output}" ]] || \
    fail "cuRobo checkout is dirty; remove local changes before installation: ${source_dir}"
}

if [[ -e "${SOURCE_DIR}" || -L "${SOURCE_DIR}" ]]; then
  verify_curobo_checkout "${SOURCE_DIR}"
  note "using existing verified clean cuRobo Git checkout"
else
  TEMP_SOURCE="$(mktemp -d "${DEPENDENCY_DIR}/.curobo-source.XXXXXX")"
  git -C "${TEMP_SOURCE}" init --quiet
  git -C "${TEMP_SOURCE}" remote add origin "${CUROBO_REPOSITORY}"
  note "fetching locked cuRobo commit ${CUROBO_COMMIT}"
  git -C "${TEMP_SOURCE}" fetch --depth 1 --no-tags origin "${CUROBO_COMMIT}"
  git -C "${TEMP_SOURCE}" checkout --quiet --detach "${CUROBO_COMMIT}"
  cat >> "${TEMP_SOURCE}/.git/info/exclude" <<'EOF'

# ur-waypoint local source identity markers
/.ur-waypoint-curobo-commit
/.ur-waypoint-curobo-tree
EOF
  printf '%s\n' "${CUROBO_COMMIT}" > "${TEMP_SOURCE}/.ur-waypoint-curobo-commit"
  printf '%s\n' "${CUROBO_SOURCE_TREE}" > "${TEMP_SOURCE}/.ur-waypoint-curobo-tree"
  verify_curobo_checkout "${TEMP_SOURCE}"
  mv -- "${TEMP_SOURCE}" "${SOURCE_DIR}"
  TEMP_SOURCE=""
  verify_curobo_checkout "${SOURCE_DIR}"
fi

note "installing verified cuRobo source ${CUROBO_COMMIT} with CUDA 12/PyTorch extras"
python -m pip install --no-build-isolation --editable "${SOURCE_DIR}[cu12-torch]"
verify_curobo_checkout "${SOURCE_DIR}"
python -c '
from importlib.metadata import PackageNotFoundError, requires, version

from packaging.requirements import Requirement

missing = []
incompatible = []
for raw in requires("nvidia-curobo") or ():
    requirement = Requirement(raw)
    environments = ({"extra": ""}, {"extra": "cu12-torch"})
    if requirement.marker is not None and not any(
        requirement.marker.evaluate(environment) for environment in environments
    ):
        continue
    try:
        installed = version(requirement.name)
    except PackageNotFoundError:
        missing.append(requirement.name)
        continue
    if requirement.specifier and installed not in requirement.specifier:
        incompatible.append(f"{requirement.name}=={installed} not in {requirement.specifier}")
if missing or incompatible:
    raise SystemExit(
        "locked cuRobo dependency verification failed: missing="
        + repr(sorted(set(missing)))
        + ", incompatible="
        + repr(sorted(set(incompatible)))
    )
'
if ((DRIVER_READY == 1)); then
  CUROBO_SOURCE_DIR="${SOURCE_DIR}" \
    CUDA_TOOLKIT="${CUDA_TOOLKIT}" \
    PYTORCH_VERSION="${PYTORCH_VERSION}" \
    python -c '
import os
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import curobo
import torch

expected = Path(os.environ["CUROBO_SOURCE_DIR"]).resolve()
expected_torch_version = os.environ["PYTORCH_VERSION"]
expected_cuda_toolkit = os.environ["CUDA_TOOLKIT"]
module = Path(curobo.__file__).resolve()
if expected != module and expected not in module.parents:
    raise SystemExit(f"cuRobo import is not the verified source: {module} != {expected}")
if str(torch.__version__) != expected_torch_version:
    raise SystemExit(
        f"PyTorch version {torch.__version__!r} does not match "
        f"lock {expected_torch_version}"
    )
if str(torch.version.cuda or "") != expected_cuda_toolkit:
    raise SystemExit(
        f"PyTorch CUDA runtime {torch.version.cuda!r} does not match "
        f"toolkit {expected_cuda_toolkit}"
    )
if not torch.cuda.is_available():
    raise SystemExit("PyTorch cannot access the CUDA GPU")
try:
    cuda_core_version = version("cuda-core")
except PackageNotFoundError as error:
    raise SystemExit("cuRobo cu12-torch extra did not install cuda-core") from error
import cuda.core  # noqa: E402  # Verify the installed distribution is importable.
print(
    f"cuRobo import OK from {module}; torch={torch.__version__}; "
    f"torch CUDA={torch.version.cuda}; cuda-core={cuda_core_version}"
)
'
else
  note "skipping cuRobo/torch runtime import while the NVIDIA driver gate is blocked"
fi

note "dependency setup completed"
if ((DRIVER_READY == 0)); then
  cat >&2 <<EOF
[setup][BLOCKED] Python/cuRobo installation completed, but GPU readiness is NOT proven.
Do not run cuRobo warmup or build-recording. Upgrade the NVIDIA driver to
>= ${MINIMUM_DRIVER} through the system maintenance process, reboot if the
driver changes, and re-check nvidia-smi first.
EOF
else
  note "GPU version gate passed; run warmup separately after building the workspace"
fi
