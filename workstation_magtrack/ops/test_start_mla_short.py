"""No ROS, serial, GPU, or real installation is used by these shell tests."""
from pathlib import Path
import os
import subprocess

import pytest


SCRIPT = Path(__file__).with_name("start_mla_short.sh")


def invoke(*args, env=None):
    return subprocess.run(["bash", str(SCRIPT), *args], text=True, capture_output=True,
                          env={**os.environ, **(env or {})}, timeout=8)


@pytest.fixture
def install(tmp_path):
    root = tmp_path / "project with spaces"
    ops = root / "workstation_magtrack/ops"
    ops.mkdir(parents=True)
    bin_dir = root / "fakebin"
    bin_dir.mkdir()
    calls = root / "calls"
    helper = ops / "ros_environment.sh"
    helper.write_text('''wm_load_environment() {
  printf 'load\\n' >> "$CALLS"
  WORKSPACE="$1/workstation_magtrack"
  export PATH="$FAKEBIN:$PATH"
  export PYTHONPATH=locked_source:venv_packages
  export VIRTUAL_ENV="$1/.venv" RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTRTPS_DEFAULT_PROFILES_FILE=fake_profile
}
wm_verify_planner_imports() {
  printf 'verify\\n' >> "$CALLS"
  return "${IMPORT_STATUS:-0}"
}
''')
    ros2 = bin_dir / "ros2"
    ros2.write_text('''#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$CALLS"
if [[ "$1" == node ]]; then
  [[ "${DISCOVERY_STATUS:-0}" == 0 ]] || exit "$DISCOVERY_STATUS"
  printf '%s\\n' "${EXISTING_NODES:-}"
fi
''')
    ros2.chmod(0o755)
    # Use the host's flock when available; macOS does not ship it.
    flock = bin_dir / "flock"
    flock.write_text('#!/usr/bin/env bash\nexit "${LOCK_STATUS:-0}"\n')
    flock.chmod(0o755)
    timeout = bin_dir / "timeout"
    timeout.write_text('#!/usr/bin/env bash\nshift\nexec "$@"\n')
    timeout.chmod(0o755)
    ps = bin_dir / "ps"
    ps.write_text('''#!/usr/bin/env bash
[[ "${PS_STATUS:-0}" == 0 ]] || exit "$PS_STATUS"
printf '%s\\n' "${LOCAL_PROCESSES:-}"
''')
    ps.chmod(0o755)
    env = {"CALLS": str(calls), "FAKEBIN": str(bin_dir)}
    return root, calls, env


def test_shell_syntax():
    assert subprocess.run(["bash", "-n", str(SCRIPT)]).returncode == 0


def test_dry_run_has_no_side_effects(install):
    root, calls, env = install
    result = invoke("--project-root", str(root), "--dry-run", env=env)
    assert result.returncode == 0, result.stderr
    assert not calls.exists()
    assert not (root / "workstation_magtrack/.mla_stack.lock").exists()
    assert "environment_id:=mla_short" in result.stdout
    assert "192.168.1.99" in result.stdout


def test_start_uses_verified_environment_and_exact_workspace(install):
    root, calls, env = install
    result = invoke("--project-root", str(root), "--launch-rviz", "false", env=env)
    assert result.returncode == 0, result.stderr
    lines = calls.read_text().splitlines()
    assert lines[:3] == ["load", "verify", "node list --no-daemon"]
    assert lines[3] == ("launch hx_waypoint_bringup real.launch.py robot_ip:=192.168.1.99 "
                        f"workspace:={root}/workstation_magtrack environment_id:=mla_short launch_rviz:=false")


def test_env_report_checks_dependencies_but_never_starts_ros(install):
    root, calls, env = install
    result = invoke("--project-root", str(root), "--env-report", env=env)
    assert result.returncode == 0, result.stderr
    assert calls.read_text().splitlines() == ["load", "verify"]
    assert "locked_source:venv_packages" in result.stdout


@pytest.mark.parametrize("node", ["/controller_manager", "/hx_waypoint_runtime",
                                  "/hx_waypoint_curobo_server", "/move_group",
                                  "/ur5/controller_manager", "/ur5/hx_waypoint_runtime",
                                  "/another/domain/hx_waypoint_curobo_server", "/ur/move_group"])
def test_existing_stack_refused_without_restarting(install, node):
    root, calls, env = install
    result = invoke("--project-root", str(root), env={**env, "EXISTING_NODES": node})
    assert result.returncode == 2
    assert "existing ROS stack detected" in result.stderr
    assert not any(line.startswith("launch ") for line in calls.read_text().splitlines())


@pytest.mark.parametrize("process", ["ros2_control_node", "ros2_control_no", "ur_ros2_control",
                                     "runtime_node", "curobo_server", "move_group"])
def test_local_stack_refused_even_with_empty_ros_graph(install, process):
    root, calls, env = install
    result = invoke("--project-root", str(root),
                    env={**env, "LOCAL_PROCESSES": f"12345 {process}", "EXISTING_NODES": ""})
    assert result.returncode == 2
    assert "existing local stack process detected" in result.stderr
    assert calls.read_text().splitlines() == ["load", "verify"]


@pytest.mark.parametrize("failure", [{"IMPORT_STATUS": "7"}, {"LOCK_STATUS": "1"},
                                    {"DISCOVERY_STATUS": "1"}, {"PS_STATUS": "1"}])
def test_fail_closed_before_driver_start(install, failure):
    root, calls, env = install
    result = invoke("--project-root", str(root), env={**env, **failure})
    assert result.returncode != 0
    assert not any(line.startswith("launch ") for line in calls.read_text().splitlines())


@pytest.mark.parametrize("args", [["--robot-ip", "192.168.1.999"],
                                   ["--robot-ip", "$(touch /tmp/never)"],
                                   ["--launch-rviz", "yes"], ["--unknown"],
                                   ["--project-root", "relative"],
                                   ["--robot-ip"], ["--dry-run", "--env-report"]])
def test_invalid_options_do_not_source_or_execute(install, args):
    root, calls, env = install
    result = invoke("--project-root", str(root), *args, env=env)
    assert result.returncode == 2
    assert not calls.exists()
