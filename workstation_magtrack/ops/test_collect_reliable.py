"""Launcher tests use only stub source files and a fake Python; no hardware I/O."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).with_name("collect_reliable.sh")
HELPER = SCRIPT.with_name("ros_environment.sh")
COMMIT = "b" * 40
TREE = "c" * 40
ABI = f"cp{sys.version_info.major}{sys.version_info.minor}"


@pytest.fixture
def installation(tmp_path: Path) -> tuple[list[str], dict[str, str], Path, Path]:
    root = tmp_path / "project with spaces"
    workspace = root / "workstation_magtrack"
    for path in (workspace / "ops", workspace / "collection", root / "install",
                 root / ".venv/bin", root / "src/hx_waypoint_curobo",
                 root / "src/hx_waypoint_tools", root / "src/hx_waypoint_nucleo"):
        path.mkdir(parents=True, exist_ok=True)
    lock = (f"schema_version: 1\ncommit: {COMMIT}\nsource_tree: {TREE}\npython_abi: \"{ABI}\"\n"
            'pytorch_version: "0.1.0+cu999"\ncuda_toolkit: "99.9"\n')
    (root / "curobo.lock").write_text(lock)
    (workspace / "curobo.lock").write_text(lock)
    source = root / f".deps/curobo-{COMMIT}"
    curobo = source / "curobo"
    curobo.mkdir(parents=True)
    (curobo / "__init__.py").write_text("# Locked synthetic cuRobo, never an external dependency.\n")
    (source / ".ur-waypoint-curobo-commit").write_text(COMMIT + "\n")
    (source / ".ur-waypoint-curobo-tree").write_text(TREE + "\n")
    (workspace / ".deps").symlink_to("../.deps", target_is_directory=True)
    modules = {
        "inverse_kinematics": ["InverseKinematics", "InverseKinematicsCfg"],
        "collision_checking": ["RobotCollisionChecker", "RobotCollisionCheckerCfg"],
        "motion_planner": ["MotionPlanner", "MotionPlannerCfg"],
        "trajectory_optimizer": ["TrajectoryOptimizer", "TrajectoryOptimizerCfg"],
        "types": ["GoalToolPose", "JointState", "Pose"],
    }
    for module, names in modules.items():
        (curobo / f"{module}.py").write_text("\n".join(
            f'class {name}:\n    def __init__(self):\n        raise RuntimeError("solver must never initialize")'
            for name in names) + "\n")
    site = root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    (site / "torch").mkdir(parents=True)
    (site / "torch/__init__.py").write_text(
        'from types import SimpleNamespace\n__version__ = "0.1.0+cu999"\n'
        'version = SimpleNamespace(cuda="99.9")\n'
        'class ForbiddenCuda:\n'
        '    def __getattr__(self, name):\n        raise RuntimeError("CUDA must not be accessed")\n'
        'cuda = ForbiddenCuda()\n')
    (site / "yaml.py").write_text(
        'def safe_load(text):\n'
        '    return {key: value.strip().strip(chr(34)) for key, value in '
        '(line.split(":", 1) for line in text.splitlines() if ":" in line)}\n')
    server = root / "install/hx_waypoint_curobo/lib/hx_waypoint_curobo/curobo_server"
    server.parent.mkdir(parents=True)
    server.write_text(f'#!{sys.executable}\nraise RuntimeError("server must never start")\n')
    ros_setup = tmp_path / "ROS setup.bash"
    # These trusted stubs stand in for setup files, not ROS processes. The
    # unset variable also exercises the intentional absence of set -u.
    ros_setup.write_text('printf "ros %s\\n" "$ROS_UNSET_TEST" >> "$SOURCE_LOG"\n'
                         'export PYTHONPATH=ros_stub\n')
    (root / "install/setup.bash").write_text(
        'printf "overlay\\n" >> "$SOURCE_LOG"\nexport PYTHONPATH=overlay_stub:$PYTHONPATH\n'
        'export RMW_IMPLEMENTATION=wrong_vendor ROS_DOMAIN_ID=77\n'
        'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\n')
    (root / ".venv/bin/activate").write_text(
        f'export VIRTUAL_ENV={shlex.quote(str(root / ".venv"))}\n'
        'printf "venv\\n" >> "$SOURCE_LOG"\n')
    (workspace / "ops/fastdds_loopback_udp.xml").write_text("<profiles/>\n")
    (workspace / "collection/run.py").write_text("# Never imported by the fake interpreter.\n")
    python = root / ".venv/bin/python"
    python.write_text(f"#!{sys.executable}\n" + '''
import json
import os
from pathlib import Path
import sys
keys = ["VIRTUAL_ENV", "PYTHONPATH", "PYTHONNOUSERSITE", "PYTHONUNBUFFERED",
        "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE",
        "FASTRTPS_DEFAULT_PROFILES_FILE", "ROS_LOCALHOST_ONLY", "FASTDDS_BUILTIN_TRANSPORTS"]
with Path(os.environ["CALL_LOG"]).open("a") as stream:
    stream.write(json.dumps({"argv": sys.argv[1:], "cwd": os.getcwd(),
                             "env": {key: os.environ.get(key) for key in keys}}) + "\\n")
raise SystemExit(int(os.environ.get("FAKE_EXIT", "0")))
''')
    python.chmod(0o755)
    call_log = tmp_path / "calls.jsonl"
    env = {**os.environ, "CALL_LOG": str(call_log), "SOURCE_LOG": str(tmp_path / "sources.log"),
           "ROS_LOCALHOST_ONLY": "1", "FASTDDS_BUILTIN_TRANSPORTS": "SHM",
           "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_DOMAIN_ID": "88"}
    command = ["/bin/bash", str(SCRIPT), "--project-root", str(root), "--ros-setup", str(ros_setup)]
    return command, env, root, call_log


def test_shell_syntax() -> None:
    subprocess.run(["/bin/bash", "-n", str(SCRIPT)], check=True)
    subprocess.run(["/bin/bash", "-n", str(HELPER)], check=True)


def test_help_and_default_dry_run_need_no_ros_or_python(tmp_path: Path) -> None:
    for flag in ("--help", "--dry-run"):
        result = subprocess.run(["/bin/bash", str(SCRIPT), flag], capture_output=True, text=True,
                                cwd=tmp_path, check=False)
        assert result.returncode == 0, result.stderr
        assert "/home/huaxin/Codes/ur-waypoint" in result.stdout
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in result.stdout
    assert "/opt/ros/jazzy/setup.bash" in result.stdout
    assert "collection.run" in result.stdout


def test_sources_in_order_and_executes_exact_arguments(installation) -> None:
    command, env, root, call_log = installation
    malicious = "manifest $(touch NEVER_CREATED);'*?.json"
    args = ["--manifest", malicious, "--output", "runs/new run", "--boot-id", "boot id",
            "--port", "/dev/serial/by-id/test", "--initial-background", "--reuse-nucleo-stream",
            "--prior-background-provenance", "reports/prior background.json"]
    result = subprocess.run([*command, "--", *args], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    calls = [json.loads(line) for line in call_log.read_text().splitlines()]
    assert len(calls) == 1
    call = calls[0]
    workspace = root / "workstation_magtrack"
    assert call["argv"] == ["-m", "collection.run", "--workspace", str(workspace), "collect", *args]
    assert call["cwd"] == str(workspace)
    assert not (workspace / "NEVER_CREATED").exists()
    assert Path(env["SOURCE_LOG"]).read_text().splitlines() == ["ros ", "overlay", "venv"]
    actual = call["env"]
    assert actual["PYTHONPATH"] == ":".join(map(str, [workspace, root / "src/hx_waypoint_curobo",
        root / "src/hx_waypoint_tools", root / "src/hx_waypoint_nucleo", root / f".deps/curobo-{COMMIT}",
        root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"])) + ":overlay_stub:ros_stub"
    assert actual["VIRTUAL_ENV"] == str(root / ".venv")
    assert actual["PYTHONNOUSERSITE"] == actual["PYTHONUNBUFFERED"] == "1"
    assert actual["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert actual["ROS_DOMAIN_ID"] == "0"
    assert actual["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "SYSTEM_DEFAULT"
    assert actual["FASTRTPS_DEFAULT_PROFILES_FILE"] == str(workspace / "ops/fastdds_loopback_udp.xml")
    assert actual["ROS_LOCALHOST_ONLY"] is actual["FASTDDS_BUILTIN_TRANSPORTS"] is None


def test_environment_report_never_executes_python(installation) -> None:
    command, env, root, call_log = installation
    result = subprocess.run([*command, "--env-report"], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not call_log.exists()
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in result.stdout
    assert "ROS_DOMAIN_ID=0" in result.stdout
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT" in result.stdout
    assert Path(env["SOURCE_LOG"]).read_text().splitlines() == ["ros ", "overlay", "venv"]


def test_dry_run_never_sources_even_existing_setup_files(installation) -> None:
    command, env, root, call_log = installation
    result = subprocess.run([*command, "--dry-run", "--manifest", "x y.json"],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()
    tokens = shlex.split(next(line for line in result.stdout.splitlines() if line.startswith("command:")))[1:]
    assert tokens == [str(root / ".venv/bin/python"), "-m", "collection.run", "--workspace",
                      str(root / "workstation_magtrack"), "collect", "--manifest", "x y.json"]


@pytest.mark.parametrize("missing", ["install/setup.bash", ".venv/bin/activate", ".venv/bin/python",
    "workstation_magtrack/ops/fastdds_loopback_udp.xml", "workstation_magtrack/collection/run.py",
    "src/hx_waypoint_curobo", "src/hx_waypoint_tools", "src/hx_waypoint_nucleo"])
def test_missing_paths_fail_before_sourcing_or_collecting(installation, missing: str) -> None:
    command, env, root, call_log = installation
    path = root / missing
    path.rmdir() if path.is_dir() else path.unlink()
    result = subprocess.run([*command, "--manifest", "example.json"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert str(path) in result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


def test_setup_failure_does_not_launch_collector(installation) -> None:
    command, env, root, call_log = installation
    (root / "install/setup.bash").write_text("return 19\n")
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 19
    assert "source project overlay:" in result.stderr
    assert "failed (exit 19); collector not retried" in result.stderr
    assert not call_log.exists()


def test_missing_ros_setup_fails_before_sources_or_python(installation) -> None:
    command, env, root, call_log = installation
    ros_setup = Path(command[-1])
    ros_setup.unlink()
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert str(ros_setup) in result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


def test_nonexecutable_interpreter_is_not_replaced_by_system_python(installation) -> None:
    command, env, root, call_log = installation
    (root / ".venv/bin/python").chmod(0o644)
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "required Python interpreter missing or not executable" in result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


def test_collector_failure_is_not_retried(installation) -> None:
    command, env, root, call_log = installation
    result = subprocess.run(command, env={**env, "FAKE_EXIT": "23"}, capture_output=True, text=True)
    assert result.returncode == 23
    assert len(call_log.read_text().splitlines()) == 1


def test_optional_planner_imports_use_exact_console_interpreter_without_cuda_or_ros(installation) -> None:
    command, env, root, call_log = installation
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    proof = json.loads(result.stdout)
    assert proof["passed"] is True
    assert proof["interpreter"] == sys.executable
    assert proof["python_abi"] == ABI
    assert proof["torch_version"] == "0.1.0+cu999"
    assert proof["gpu_warmup_performed"] is proof["serial_opened"] is proof["robot_commanded"] is False
    assert len(proof["imported_modules"]) == 5
    assert not call_log.exists()  # The fake collector interpreter was never used.


@pytest.mark.parametrize("file, content, expected", [
    ("motion_planner.py", 'raise ImportError("synthetic nested dependency absent")\n', "synthetic nested dependency absent"),
    ("types.py", "GoalToolPose = JointState = object\n", "Pose"),
])
def test_deep_import_preflight_exposes_actual_failure_and_never_collects(installation, file, content, expected) -> None:
    command, env, root, call_log = installation
    (root / f".deps/curobo-{COMMIT}/curobo/{file}").write_text(content)
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert expected in result.stderr
    assert not call_log.exists()


@pytest.mark.parametrize("relative", ["curobo.lock", "workstation_magtrack/curobo.lock",
    f".deps/curobo-{COMMIT}/curobo/__init__.py", f".deps/curobo-{COMMIT}/.ur-waypoint-curobo-commit",
    f".deps/curobo-{COMMIT}/.ur-waypoint-curobo-tree"])
def test_required_locked_dependency_files_fail_before_sources(installation, relative: str) -> None:
    command, env, root, call_log = installation
    path = root / relative
    path.unlink()
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert str(path) in result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


def test_missing_locked_abi_site_is_not_replaced_by_other_version(installation) -> None:
    command, env, root, call_log = installation
    site = root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    site.rename(site.with_name("not-site-packages"))
    (root / ".venv/lib/python0.0/site-packages").mkdir(parents=True)
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "locked ABI venv site-packages unavailable" in result.stderr
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


@pytest.mark.parametrize("mutate", ["lock", "commit_marker", "tree_marker", "workspace_symlink"])
def test_dependency_identity_mismatch_is_never_repaired_or_collected(installation, mutate: str) -> None:
    command, env, root, call_log = installation
    if mutate == "lock":
        (root / "curobo.lock").write_text("wrong root lock\n")
    elif mutate == "workspace_symlink":
        (root / "workstation_magtrack/.deps").unlink()
    else:
        marker = "commit" if mutate == "commit_marker" else "tree"
        (root / f".deps/curobo-{COMMIT}/.ur-waypoint-curobo-{marker}").write_text("wrong identity\n")
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert not call_log.exists()
    assert not Path(env["SOURCE_LOG"]).exists()


def test_import_preflight_rejects_wrong_torch_without_gpu_call(installation) -> None:
    command, env, root, call_log = installation
    site = root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    torch = site / "torch/__init__.py"
    torch.write_text(torch.read_text().replace("0.1.0+cu999", "wrong-version"))
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "PyTorch version wrong-version differs from lock" in result.stderr
    assert not call_log.exists()


@pytest.mark.parametrize("shebang", ["#!/usr/bin/env python3", "not-a-shebang", "#!/missing/python"])
def test_preflight_never_substitutes_for_unverifiable_console_interpreter(installation, shebang: str) -> None:
    command, env, root, call_log = installation
    server = root / "install/hx_waypoint_curobo/lib/hx_waypoint_curobo/curobo_server"
    server.write_text(shebang + '\nraise RuntimeError("must never execute")\n')
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "planner" in result.stderr
    assert not call_log.exists()


def test_preflight_rejects_mismatched_cuda_build_without_accessing_cuda(installation) -> None:
    command, env, root, call_log = installation
    site = root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    torch = site / "torch/__init__.py"
    torch.write_text(torch.read_text().replace('cuda="99.9"', 'cuda="wrong-build"'))
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "PyTorch CUDA build wrong-build differs from lock" in result.stderr
    assert not call_log.exists()


def test_env_report_exposes_locked_paths_without_importing_them(installation) -> None:
    command, env, root, call_log = installation
    (root / f".deps/curobo-{COMMIT}/curobo/__init__.py").write_text(
        'raise RuntimeError("environment report must not import cuRobo")\n')
    result = subprocess.run([*command, "--env-report"], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = dict(shlex.split(line, posix=True)[0].split("=", 1) for line in result.stdout.splitlines()
                  if "=" in line and not line.startswith("mode="))
    assert report["WM_CUROBO_SOURCE"] == str(root / f".deps/curobo-{COMMIT}")
    assert report["WM_PYTHON_ABI"] == ABI
    assert report["WM_VENV_SITE"] in report["PYTHONPATH"].split(":")
    assert not call_log.exists()


def test_preflight_refuses_console_interpreter_with_wrong_locked_abi(installation) -> None:
    command, env, root, call_log = installation
    for lock in (root / "curobo.lock", root / "workstation_magtrack/curobo.lock"):
        lock.write_text(lock.read_text().replace(ABI, "cp999"))
    (root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}").rename(
        root / ".venv/lib/python9.99")
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert f"planner Python ABI {ABI} differs from lock cp999" in result.stderr
    assert not call_log.exists()


@pytest.mark.parametrize("package", ["curobo", "torch"])
def test_preflight_refuses_imported_module_outside_locked_source(installation, package: str) -> None:
    command, env, root, call_log = installation
    original = (root / f".deps/curobo-{COMMIT}/curobo/__init__.py" if package == "curobo" else
                root / f".venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/torch/__init__.py")
    foreign = root / f"foreign_{package}.py"
    original.rename(foreign)
    original.symlink_to(foreign)
    result = subprocess.run([*command, "--dependency-preflight"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    expected = "cuRobo import is not the locked source" if package == "curobo" else "PyTorch is not from the locked venv"
    assert expected in result.stderr
    assert not call_log.exists()


@pytest.mark.parametrize("args", [["--project-root"], ["--ros-setup"], ["--project-root", "relative"],
                                  ["--ros-setup", "relative"], ["--dry-run", "--env-report"]])
def test_invalid_launcher_arguments_fail(args: list[str]) -> None:
    result = subprocess.run(["/bin/bash", str(SCRIPT), *args], capture_output=True, text=True)
    assert result.returncode == 2
