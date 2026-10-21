#!/usr/bin/env bash
# Shared by the collection client and the existing-stack launcher. Sourcing
# this file only defines functions; it never starts ROS, serial, or a GPU job.

wm_environment_error() {
  printf 'mla_short environment: %s\n' "$*" >&2
  return 2
}

wm_lock_value() {
  # curobo.lock is a flat project-owned YAML mapping. Require exactly one
  # scalar occurrence; never evaluate any value as shell input.
  awk -v wanted="$1" '
    $0 ~ "^" wanted ":[[:space:]]*" {
      count++
      value = $0
      sub("^[^:]+:[[:space:]]*", "", value)
      gsub(/^\"|\"$/, "", value)
    }
    END { if (count == 1) print value; else exit 2 }
  ' "$2"
}

wm_load_environment() {
  [[ $# -eq 2 ]] || { wm_environment_error 'wm_load_environment requires PROJECT_ROOT ROS_SETUP'; return 2; }
  PROJECT_ROOT="$1"
  ROS_SETUP="$2"
  [[ "$PROJECT_ROOT" == /* && "$ROS_SETUP" == /* ]] || {
    wm_environment_error 'project root and ROS setup must be absolute paths'; return 2;
  }
  WORKSPACE="$PROJECT_ROOT/workstation_magtrack"
  PYTHON="$PROJECT_ROOT/.venv/bin/python"
  PROFILE="$WORKSPACE/ops/fastdds_loopback_udp.xml"
  WM_LOCK="$WORKSPACE/curobo.lock"

  local directory file
  for directory in "$PROJECT_ROOT" "$WORKSPACE" \
    "$PROJECT_ROOT/src/hx_waypoint_curobo" "$PROJECT_ROOT/src/hx_waypoint_tools" \
    "$PROJECT_ROOT/src/hx_waypoint_nucleo"; do
    [[ -d "$directory" ]] || { wm_environment_error "required directory missing: $directory"; return 2; }
  done
  for file in "$ROS_SETUP" "$PROJECT_ROOT/install/setup.bash" \
    "$PROJECT_ROOT/.venv/bin/activate" "$PROFILE" "$WORKSPACE/collection/run.py" \
    "$WM_LOCK" "$PROJECT_ROOT/curobo.lock"; do
    [[ -f "$file" && -r "$file" ]] || {
      wm_environment_error "required file missing or unreadable: $file"; return 2;
    }
  done
  [[ -x "$PYTHON" ]] || {
    wm_environment_error "required Python interpreter missing or not executable: $PYTHON"; return 2;
  }
  cmp -s "$WM_LOCK" "$PROJECT_ROOT/curobo.lock" || {
    wm_environment_error 'workspace and project curobo.lock differ; no dependency fallback'; return 2;
  }
  WM_CUROBO_COMMIT="$(wm_lock_value commit "$WM_LOCK")" || {
    wm_environment_error 'curobo.lock must contain exactly one commit'; return 2;
  }
  WM_CUROBO_TREE="$(wm_lock_value source_tree "$WM_LOCK")" || {
    wm_environment_error 'curobo.lock must contain exactly one source_tree'; return 2;
  }
  WM_PYTHON_ABI="$(wm_lock_value python_abi "$WM_LOCK")" || {
    wm_environment_error 'curobo.lock must contain exactly one python_abi'; return 2;
  }
  [[ "$WM_CUROBO_COMMIT" =~ ^[0-9a-f]{40}$ && "$WM_CUROBO_TREE" =~ ^[0-9a-f]{40}$ ]] || {
    wm_environment_error 'curobo.lock must pin immutable commit and source_tree values'; return 2;
  }
  [[ "$WM_PYTHON_ABI" =~ ^cp([0-9])([0-9]+)$ ]] || {
    wm_environment_error "unsupported locked Python ABI: $WM_PYTHON_ABI"; return 2;
  }
  WM_VENV_SITE="$PROJECT_ROOT/.venv/lib/python${BASH_REMATCH[1]}.${BASH_REMATCH[2]}/site-packages"
  WM_CUROBO_SOURCE="$PROJECT_ROOT/.deps/curobo-$WM_CUROBO_COMMIT"
  [[ -d "$WM_VENV_SITE" ]] || {
    wm_environment_error "locked ABI venv site-packages unavailable: $WM_VENV_SITE"; return 2;
  }
  for file in "$WM_CUROBO_SOURCE/curobo/__init__.py" \
    "$WM_CUROBO_SOURCE/.ur-waypoint-curobo-commit" "$WM_CUROBO_SOURCE/.ur-waypoint-curobo-tree"; do
    [[ -f "$file" && -r "$file" ]] || {
      wm_environment_error "locked cuRobo dependency file missing or unreadable: $file"; return 2;
    }
  done
  [[ "$(<"$WM_CUROBO_SOURCE/.ur-waypoint-curobo-commit")" == "$WM_CUROBO_COMMIT" && \
     "$(<"$WM_CUROBO_SOURCE/.ur-waypoint-curobo-tree")" == "$WM_CUROBO_TREE" ]] || {
    wm_environment_error "cuRobo source identity markers differ from lock: $WM_CUROBO_SOURCE"; return 2;
  }
  # AdapterV2 resolves .deps relative to its workspace lock. Require that
  # location to identify the same installed checkout instead of making a link.
  [[ "$WORKSPACE/.deps/curobo-$WM_CUROBO_COMMIT" -ef "$WM_CUROBO_SOURCE" ]] || {
    wm_environment_error "workspace .deps does not identify the locked project source: $WORKSPACE/.deps"; return 2;
  }

  # All existence/identity checks above precede setup scripts and the collector.
  # Explicit site-packages is essential for /usr/bin/python3 ROS console entry
  # points; activating a venv does not change their build-time shebang.
  PHASE="source ROS setup: $ROS_SETUP"
  source "$ROS_SETUP"
  PHASE="source project overlay: $PROJECT_ROOT/install/setup.bash"
  source "$PROJECT_ROOT/install/setup.bash"
  PHASE="activate project virtual environment: $PROJECT_ROOT/.venv/bin/activate"
  source "$PROJECT_ROOT/.venv/bin/activate"
  export PYTHONPATH="$WORKSPACE:$PROJECT_ROOT/src/hx_waypoint_curobo:$PROJECT_ROOT/src/hx_waypoint_tools:$PROJECT_ROOT/src/hx_waypoint_nucleo:$WM_CUROBO_SOURCE:$WM_VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
  unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
}

wm_report_environment() {
  local name
  for name in VIRTUAL_ENV PYTHONPATH PYTHONNOUSERSITE PYTHONUNBUFFERED \
    RMW_IMPLEMENTATION ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE FASTRTPS_DEFAULT_PROFILES_FILE \
    WM_CUROBO_SOURCE WM_VENV_SITE WM_PYTHON_ABI; do
    printf '%s=%q\n' "$name" "${!name}"
  done
  printf 'unset=ROS_LOCALHOST_ONLY,FASTDDS_BUILTIN_TRANSPORTS\n'
}

wm_verify_planner_imports() {
  # Verify the actual generated ROS server's interpreter, not merely the
  # activated venv. No CUDA availability call, warmup, solver, or ROS import.
  local server="$PROJECT_ROOT/install/hx_waypoint_curobo/lib/hx_waypoint_curobo/curobo_server"
  local firstline
  [[ -f "$server" && -r "$server" ]] || {
    wm_environment_error "installed planner console entry point unavailable: $server"; return 2;
  }
  IFS= read -r firstline < "$server" || {
    wm_environment_error "cannot read planner console shebang: $server"; return 2;
  }
  [[ "$firstline" =~ ^#!/[^[:space:]]+$ ]] || {
    wm_environment_error "planner console shebang is not a single absolute interpreter: $server"; return 2;
  }
  WM_PLANNER_PYTHON="${firstline#\#!}"
  [[ -x "$WM_PLANNER_PYTHON" ]] || {
    wm_environment_error "planner shebang interpreter unavailable: $WM_PLANNER_PYTHON"; return 2;
  }
  PHASE="CPU-only locked dependency imports using planner interpreter: $WM_PLANNER_PYTHON"
  PYTHONDONTWRITEBYTECODE=1 "$WM_PLANNER_PYTHON" - "$WM_LOCK" "$WM_CUROBO_SOURCE" "$WM_VENV_SITE" <<'PY'
import importlib
import json
from pathlib import Path
import sys

try:
    import yaml
    lock_path, expected_source, expected_site = map(Path, sys.argv[1:])
    lock = yaml.safe_load(lock_path.read_text())
    abi = f"cp{sys.version_info.major}{sys.version_info.minor}"
    if abi != lock["python_abi"]:
        raise RuntimeError(f"planner Python ABI {abi} differs from lock {lock['python_abi']}")
    expected_source, expected_site = expected_source.resolve(), expected_site.resolve()
    curobo = importlib.import_module("curobo")
    source = Path(curobo.__file__).resolve()
    if not source.is_relative_to(expected_source):
        raise RuntimeError(f"cuRobo import is not the locked source: {source} != {expected_source}")
    adapter_source = (lock_path.parent / ".deps" / f"curobo-{lock['commit']}").resolve()
    if adapter_source != expected_source:
        raise RuntimeError("adapter-relative cuRobo source differs from locked installed source")
    for key, marker in (("commit", ".ur-waypoint-curobo-commit"),
                        ("source_tree", ".ur-waypoint-curobo-tree")):
        if (expected_source / marker).read_text().strip() != lock[key]:
            raise RuntimeError(f"cuRobo {key} marker does not match lock")
    torch = importlib.import_module("torch")
    torch_source = Path(torch.__file__).resolve()
    if not torch_source.is_relative_to(expected_site):
        raise RuntimeError(f"PyTorch is not from the locked venv site-packages: {torch_source}")
    if str(torch.__version__) != lock["pytorch_version"]:
        raise RuntimeError(f"PyTorch version {torch.__version__} differs from lock {lock['pytorch_version']}")
    if str(torch.version.cuda or "") != lock["cuda_toolkit"]:
        raise RuntimeError(f"PyTorch CUDA build {torch.version.cuda} differs from lock {lock['cuda_toolkit']}")
    required = {
        "curobo.inverse_kinematics": ("InverseKinematics", "InverseKinematicsCfg"),
        "curobo.collision_checking": ("RobotCollisionChecker", "RobotCollisionCheckerCfg"),
        "curobo.motion_planner": ("MotionPlanner", "MotionPlannerCfg"),
        "curobo.trajectory_optimizer": ("TrajectoryOptimizer", "TrajectoryOptimizerCfg"),
        "curobo.types": ("GoalToolPose", "JointState", "Pose"),
    }
    for module_name, symbols in required.items():
        module = importlib.import_module(module_name)
        for name in symbols:
            getattr(module, name)
    print(json.dumps({
        "scope": "locked_planner_imports_only_not_live_server_or_gpu_readiness",
        "passed": True, "interpreter": sys.executable, "python_abi": abi,
        "curobo_source": str(source), "torch_source": str(torch_source),
        "torch_version": str(torch.__version__), "torch_cuda_build": str(torch.version.cuda),
        "imported_modules": list(required), "gpu_warmup_performed": False,
        "robot_commanded": False, "serial_opened": False,
    }, sort_keys=True))
except Exception as error:
    print(f"locked planner dependency preflight failed: {type(error).__name__}: {error}", file=sys.stderr)
    raise SystemExit(2)
PY
}
