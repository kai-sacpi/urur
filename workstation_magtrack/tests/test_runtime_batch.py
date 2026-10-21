"""Pure audit tests; no ROS initialization, robot, sensor or GPU required."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import yaml


SOURCE = Path(__file__).parents[1] / "runtime" / "batch_recordings.py"
SPEC = importlib.util.spec_from_file_location("wm_batch", SOURCE)
batch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(batch)


def test_domains_never_allow_default_or_user_robot_domains():
    assert batch.checked_domain(219) == 219
    assert batch.checked_domain(220) == 220
    for value in (0, 1, 42, 218, 221):
        with pytest.raises(ValueError):
            batch.checked_domain(value)


def test_source_time_order_is_authoritative(tmp_path):
    source = tmp_path / "source.csv"
    source.write_text("time_s,tip_x_m\n0,0\n0.05,1\n0.05,2\n")
    with pytest.raises(ValueError, match="strictly increasing"):
        batch.source_info(source)
    source.write_text("time_s,tip_x_m\n0,0\n0.05,1\n0.10,2\n")
    assert batch.source_info(source)["duration_s"] == 0.1


def test_manifest_and_plain_ids_equivalent(tmp_path):
    text = tmp_path / "ids.txt"
    manifest = tmp_path / "ids.json"
    text.write_text("# trajectories\na\nb\n")
    manifest.write_text(json.dumps([{"trajectory_id": "a"}, {"trajectory_id": "b"}]))
    assert batch.read_ids(text) == batch.read_ids(manifest) == ["a", "b"]
    text.write_text("a\na\n")
    with pytest.raises(ValueError, match="Duplicate"):
        batch.read_ids(text)


def compiled_fixture(tmp_path):
    coordinator = batch.Coordinator.__new__(batch.Coordinator)
    coordinator.root = tmp_path
    coordinator.args = type("Args", (), {"env": "mla_short", "force": False})()
    coordinator.state = batch.State(tmp_path, "test", ["example"], "mla_short")
    compiled = tmp_path / "runs" / "example__compiled__123"
    compiled.mkdir()
    source = compiled / "source_tip_trajectory.csv"
    source.write_text("time_s,tip_x_m\n0,0\n1,0.01\n")
    joints = compiled / "joint_trajectory.csv"
    joints.write_text("example joint contents")
    info = batch.source_info(source)
    coordinator.state.data["trajectories"]["example"]["source"] = info
    report = {"schema_version": 2, "compiler_contract_revision": "fixed_time_waypoint_v2",
              "environment_id": "mla_short", "trajectory_id": "example",
              "source_trajectory_sha256": info["sha256"], "joint_trajectory_sha256": batch.digest(joints),
              "uniform_time_scale": 1.0, "source_waypoint_count": 2, "matched_source_waypoint_count": 2,
              "source_duration_s": 1.0, "duration_s": 1.0, "max_source_time_error_s": 0.0,
              "max_joint_velocity_ratio": 0.1, "max_joint_acceleration_ratio": 0.1,
              "max_joint_jerk_ratio": 0.1}
    (compiled / "compile_report.yaml").write_text(yaml.safe_dump(report))
    return coordinator, compiled, report


@pytest.mark.parametrize("key,value", [("uniform_time_scale", 1.01),
                                      ("matched_source_waypoint_count", 1),
                                      ("duration_s", 1.1),
                                      ("max_joint_jerk_ratio", 1.1),
                                      ("compiler_contract_revision", "old")])
def test_compile_audit_rejects_contract_changes(tmp_path, key, value):
    coordinator, compiled, report = compiled_fixture(tmp_path)
    assert coordinator.verify_compiled("example", compiled)["report"]["uniform_time_scale"] == 1.0
    report[key] = value
    (compiled / "compile_report.yaml").write_text(yaml.safe_dump(report))
    with pytest.raises(RuntimeError):
        coordinator.verify_compiled("example", compiled)


def test_resume_refuses_changed_artifact(tmp_path):
    coordinator, compiled, _ = compiled_fixture(tmp_path)
    details = coordinator.verify_compiled("example", compiled)
    coordinator.state.begin("example", "compile")
    coordinator.state.finish("example", "compile", "passed", **details)
    assert coordinator.valid_cached("example", "compile")
    (compiled / "joint_trajectory.csv").write_text("mutated")
    with pytest.raises(RuntimeError, match="changed"):
        coordinator.valid_cached("example", "compile")


def test_offline_audit_requires_coherent_witness_even_for_successful_legacy_action(tmp_path):
    coordinator, _, _ = compiled_fixture(tmp_path)
    coordinator.environment = tmp_path / "environments" / "mla_short"
    log = tmp_path / "legacy-play.log"
    log.write_text(json.dumps({"result": {"action_status": 4, "success": True,
                                         "telemetry_path": str(tmp_path / "legacy.csv"),
                                         "runtime": {"domain": 220, "execution_mode": "simulation",
                                                     "telemetry_root": str(tmp_path / "runs")}}}) + "\n")
    coordinator.state.data["trajectories"]["example"]["play"] = {"status": "passed", "log": str(log)}
    coordinator.audit_existing_play("example")
    item = coordinator.state.data["trajectories"]["example"]
    assert item["play"]["status"] == "failed"
    assert "rerunplayrequired" in item["play"]["error"]
    assert item["attempts"]["play"][0]["status"] == "passed"


def test_resume_does_not_reuse_legacy_play_without_coherent_witness(tmp_path):
    coordinator, compiled, _ = compiled_fixture(tmp_path)
    path = compiled / "joint_trajectory.csv"
    coordinator.state.data["trajectories"]["example"]["play"] = {
        "status": "passed", "artifact_hashes": {str(path): batch.digest(path)}}
    assert not coordinator.valid_cached("example", "play")


def test_cancelled_compile_queue_never_spawns_more_commands(tmp_path, monkeypatch):
    spawned = []
    monkeypatch.setattr(batch.subprocess, "Popen", lambda *a, **kw: spawned.append(a))
    batch.STOP_REQUESTED.clear()
    def worker(name):
        if name == "first":
            time.sleep(.02)  # Ensure the remaining jobs are already submitted.
            batch.STOP_REQUESTED.set()
            raise KeyboardInterrupt
        batch.run_command(["must-never-spawn"], tmp_path / f"{name}.log", {}, 1)
    try:
        with pytest.raises(KeyboardInterrupt):
            batch.compile_many(worker, ["first", *[f"queued_{i}" for i in range(30)]], 1)
        assert spawned == []
    finally:
        batch.STOP_REQUESTED.clear()


def test_stop_process_kills_stubborn_child_after_leader_exits():
    child_code = ("import os,signal,time; "
                  "signal.signal(signal.SIGINT,signal.SIG_IGN); "
                  "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                  "print(os.getpid(),flush=True); time.sleep(60)")
    leader_code = ("import subprocess,sys,time; "
                   f"subprocess.Popen([sys.executable,'-c',{child_code!r}]); time.sleep(60)")
    process = subprocess.Popen([sys.executable, "-c", leader_code], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, start_new_session=True, text=True)
    child = int(process.stdout.readline().strip())
    try:
        batch.stop_process(process, grace_seconds=(.15, .15, .5))
        assert process.poll() is not None
        status = subprocess.run(["ps", "-o", "stat=", "-p", str(child)],
                                capture_output=True, text=True).stdout.strip()
        # A killed orphan can briefly await init's reap as a zombie; it cannot
        # execute or retain a GPU/runtime resource, and is never a survivor.
        assert not status or status.startswith("Z")
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2)
        process.stdout.close()


@pytest.mark.parametrize("upstream,invalidated", [("compile", ("build", "inspect", "play")),
                                                 ("build", ("inspect", "play")),
                                                 ("inspect", ("play",))])
def test_upstream_rerun_invalidates_and_archives_downstream(tmp_path, upstream, invalidated):
    state = batch.State(tmp_path, "invalidation", ["example"], "mla_short")
    entry = state.data["trajectories"]["example"]
    for phase in batch.PHASES:
        entry[phase] = {"status": "passed", "recording_id": "R1"}
    state.begin("example", upstream)
    assert entry[upstream]["status"] == "running"
    for phase in invalidated:
        assert phase not in entry
        historical = entry["attempts"][phase][-1]
        assert historical["recording_id"] == "R1"
        assert historical["invalidated_by_phase"] == upstream
