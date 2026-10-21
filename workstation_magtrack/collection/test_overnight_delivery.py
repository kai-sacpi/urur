"""Pure temporary-fixture tests for the immutable overnight handoff.

These tests neither inspect production recordings nor start ROS, GPUs, serial
ports, remote processes, or robot motion.
"""

from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from types import SimpleNamespace

import pytest

from workstation_magtrack.ops.export_overnight_delivery import (
    immutable_bundle,
    verify_campaign_summary,
    verified_live_pids,
)
from workstation_magtrack.ops import export_overnight_delivery as delivery
from workstation_magtrack.continuous_augmented.replay import export_index as replay_index
from workstation_magtrack.continuous_augmented.replay.tests.test_ops import complete_evidence, put
from workstation_magtrack.ops.export_replay_index import check_seal


PHASES = ("compile", "build", "inspect", "play")
AUGMENTED_BATCH = "ca_open_elastic_batch120_20261016_v3"


def authority_summary() -> dict:
    return {
        "batch": "authority_v4",
        "state": "finished",
        "trajectory_count": 96,
        "complete_passed": 96,
        "phase_counts": {phase: {"passed": 96} for phase in PHASES},
        "phase_exit_codes": {
            phase: 0 for phase in (
                "build", "inspect", "play", "inventory", "independent_source_audit",
            )
        },
        "workflow_passed": True,
        "real_robot_used": False,
        "sensor_used": False,
    }


def augmented_summary() -> dict:
    return {
        "source_batch_id": AUGMENTED_BATCH,
        "state": "complete",
        "expected_trajectory_count": 120,
        "observed_trajectory_count": 120,
        "complete_four_stage_passed": 120,
        "ready_on_this_host": 120,
        "phase_counts": {phase: {"passed": 120} for phase in PHASES},
        "command_attempts": [
            {"label": label, "phase": phase, "exit_code": 0}
            for label in ("canary", "remaining117") for phase in PHASES
        ],
        "workflow_passed": True,
        "robot_used": False,
        "sensor_used": False,
        "simulation_only": True,
        "real_execution_authorized": False,
    }


def test_immutable_bundle_publishes_all_exact_bytes_including_nested_files(tmp_path: Path) -> None:
    destination = tmp_path / "delivery"
    outputs = {"README.md": "轨迹回放\n".encode(), "nested/index.json": b'{"ready":336}\n'}
    before = copy.deepcopy(outputs)

    assert immutable_bundle(destination, outputs) is None

    assert outputs == before
    assert (destination / "README.md").read_bytes() == outputs["README.md"]
    assert (destination / "nested/index.json").read_bytes() == outputs["nested/index.json"]
    assert sorted(path.name for path in tmp_path.iterdir()) == ["delivery"]


def test_immutable_bundle_exact_retry_is_idempotent_and_preserves_existing_files(tmp_path: Path) -> None:
    destination = tmp_path / "delivery"
    outputs = {"index.json": b"index\n", "nested/report.md": b"report\n"}
    immutable_bundle(destination, outputs)
    inode_and_mtime = {
        name: ((destination / name).stat().st_ino, (destination / name).stat().st_mtime_ns)
        for name in outputs
    }

    assert immutable_bundle(destination, dict(outputs)) is None

    assert inode_and_mtime == {
        name: ((destination / name).stat().st_ino, (destination / name).stat().st_mtime_ns)
        for name in outputs
    }


@pytest.mark.parametrize("change", ("different_bytes", "missing_file", "extra_file"))
def test_immutable_bundle_conflict_does_not_repair_or_overwrite_existing_delivery(
    tmp_path: Path, change: str,
) -> None:
    destination = tmp_path / "delivery"
    outputs = {"index.json": b"index\n", "report.md": b"report\n"}
    immutable_bundle(destination, outputs)
    if change == "different_bytes":
        (destination / "report.md").write_bytes(b"original must survive\n")
    elif change == "missing_file":
        (destination / "report.md").unlink()
    else:
        (destination / "extra.txt").write_bytes(b"original extra\n")
    before = {path.relative_to(destination).as_posix(): path.read_bytes()
              for path in destination.rglob("*") if path.is_file()}

    with pytest.raises(ValueError):
        immutable_bundle(destination, outputs)

    assert before == {path.relative_to(destination).as_posix(): path.read_bytes()
                      for path in destination.rglob("*") if path.is_file()}
    assert sorted(path.name for path in tmp_path.iterdir()) == ["delivery"]


@pytest.mark.parametrize("unsafe_name", ("../outside.txt", "nested/../../outside.txt", "ABSOLUTE", "", "."))
def test_immutable_bundle_rejects_unsafe_relative_output_names_before_publication(
    tmp_path: Path, unsafe_name: str,
) -> None:
    destination = tmp_path / "delivery"
    if unsafe_name == "ABSOLUTE":
        unsafe_name = str(tmp_path / "outside.txt")

    with pytest.raises(ValueError):
        immutable_bundle(destination, {"valid.txt": b"valid", unsafe_name: b"unsafe"})

    assert not destination.exists()
    assert not (tmp_path / "outside.txt").exists()


def test_immutable_bundle_rejects_destination_that_is_a_file(tmp_path: Path) -> None:
    destination = tmp_path / "delivery"
    destination.write_bytes(b"original")

    with pytest.raises(ValueError):
        immutable_bundle(destination, {"index.json": b"data"})

    assert destination.read_bytes() == b"original"


def test_immutable_bundle_does_not_accept_same_bytes_through_symlink(tmp_path: Path) -> None:
    destination = tmp_path / "delivery"
    destination.mkdir()
    outside = tmp_path / "outside.txt"
    outside.write_bytes(b"same bytes")
    (destination / "index.json").symlink_to(outside)

    with pytest.raises(ValueError):
        immutable_bundle(destination, {"index.json": b"same bytes"})

    assert (destination / "index.json").is_symlink()
    assert outside.read_bytes() == b"same bytes"


def test_immutable_bundle_rejects_extra_empty_directory_without_removing_it(tmp_path: Path) -> None:
    destination = tmp_path / "delivery"
    immutable_bundle(destination, {"index.json": b"index"})
    extra = destination / "unrequested-empty-directory"
    extra.mkdir()
    inode = extra.stat().st_ino

    with pytest.raises(ValueError):
        immutable_bundle(destination, {"index.json": b"index"})

    assert extra.stat().st_ino == inode
    assert (destination / "index.json").read_bytes() == b"index"


def test_immutable_bundle_exclusive_publication_cannot_replace_racing_empty_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "delivery"
    native_rename = delivery._rename_directory_exclusive
    inode: list[int] = []

    def create_competing_directory_then_rename(source: Path, target: Path) -> None:
        assert target == destination
        assert not target.exists()
        target.mkdir()
        inode.append(target.stat().st_ino)
        native_rename(source, target)

    monkeypatch.setattr(delivery, "_rename_directory_exclusive", create_competing_directory_then_rename)

    with pytest.raises(ValueError):
        immutable_bundle(destination, {"index.json": b"new content"})

    assert destination.stat().st_ino == inode[0]
    assert list(destination.iterdir()) == []
    assert list(tmp_path.iterdir()) == [destination]


def test_immutable_bundle_failed_native_publish_removes_only_own_staging_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "delivery"
    unrelated = tmp_path / ".delivery.staging-other-owner"
    unrelated.mkdir()
    (unrelated / "owner.txt").write_bytes(b"must survive")

    def fail_publish(source: Path, target: Path) -> None:
        assert not target.exists()
        assert (source / "index.json").read_bytes() == b"new content"
        raise OSError("fixture native publication failure")

    monkeypatch.setattr(delivery, "_rename_directory_exclusive", fail_publish)

    with pytest.raises(OSError, match="fixture native publication failure"):
        immutable_bundle(destination, {"index.json": b"new content"})

    assert not destination.exists()
    assert list(tmp_path.iterdir()) == [unrelated]
    assert (unrelated / "owner.txt").read_bytes() == b"must survive"


def test_exact_authority_summary_is_accepted_without_mutation() -> None:
    summary = authority_summary()
    before = copy.deepcopy(summary)

    assert verify_campaign_summary(summary, "authority_v4", 96) is None

    assert summary == before


@pytest.mark.parametrize("key,value", (
    ("batch", "another_batch"),
    ("state", "play_finished"),
    ("state", "complete"),
    ("workflow_passed", False),
    ("workflow_passed", 1),
    ("trajectory_count", 95),
    ("trajectory_count", 96.0),
    ("complete_passed", 95),
    ("complete_passed", True),
))
def test_authority_summary_rejects_incomplete_or_wrong_identity_or_loose_types(
    key: str, value: object,
) -> None:
    summary = authority_summary()
    summary[key] = value

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, "authority_v4", 96)


@pytest.mark.parametrize("phase", PHASES)
@pytest.mark.parametrize("counts", ({"passed": 95}, {"passed": 96, "failed": 0}, {"passed": 96, "pending": 1}, {}))
def test_authority_requires_every_phase_exactly_complete(phase: str, counts: dict) -> None:
    summary = authority_summary()
    summary["phase_counts"][phase] = counts

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, "authority_v4", 96)


@pytest.mark.parametrize("phase", ("build", "inspect", "play", "inventory", "independent_source_audit"))
@pytest.mark.parametrize("failure", ("missing", "failed", "boolean_zero"))
def test_authority_requires_every_postcheck_and_exact_zero_exit_code(phase: str, failure: str) -> None:
    summary = authority_summary()
    if failure == "missing":
        del summary["phase_exit_codes"][phase]
    else:
        summary["phase_exit_codes"][phase] = 2 if failure == "failed" else False

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, "authority_v4", 96)


def test_authority_does_not_hide_nonzero_unexpected_stage() -> None:
    summary = authority_summary()
    summary["phase_exit_codes"]["later_postcheck"] = 2

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, "authority_v4", 96)


def test_exact_augmented_summary_is_accepted_without_mutation() -> None:
    summary = augmented_summary()
    before = copy.deepcopy(summary)

    assert verify_campaign_summary(summary, AUGMENTED_BATCH, 120) is None

    assert summary == before


@pytest.mark.parametrize("key,value", (
    ("source_batch_id", "ca_open_elastic_batch120_20261016_v2"),
    ("state", "remaining117_play_finished"),
    ("state", "finished"),
    ("workflow_passed", False),
    ("workflow_passed", 1),
    ("expected_trajectory_count", 119),
    ("observed_trajectory_count", 121),
    ("complete_four_stage_passed", 119),
    ("ready_on_this_host", 3),
    ("ready_on_this_host", 120.0),
))
def test_augmented_requires_exact_terminal_counts_and_final_inventory_count(
    key: str, value: object,
) -> None:
    summary = augmented_summary()
    summary[key] = value

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, AUGMENTED_BATCH, 120)


@pytest.mark.parametrize("phase", PHASES)
def test_augmented_cannot_hide_pending_phase_under_passed_counter(phase: str) -> None:
    summary = augmented_summary()
    summary["phase_counts"][phase] = {"passed": 120, "running": 1}

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, AUGMENTED_BATCH, 120)


@pytest.mark.parametrize("attempt_index", range(8))
def test_augmented_rejects_each_failed_child_attempt_even_with_all_green_counts(attempt_index: int) -> None:
    summary = augmented_summary()
    summary["command_attempts"][attempt_index]["exit_code"] = 2

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, AUGMENTED_BATCH, 120)


@pytest.mark.parametrize("defect", ("none", "one_missing", "duplicate_phase", "extra", "wrong_label", "boolean_zero"))
def test_augmented_requires_exact_eight_distinct_canary_and_remaining_phases(defect: str) -> None:
    summary = augmented_summary()
    attempts = summary["command_attempts"]
    if defect == "none":
        attempts.clear()
    elif defect == "one_missing":
        attempts.pop()
    elif defect == "duplicate_phase":
        attempts[-1] = copy.deepcopy(attempts[-2])
    elif defect == "extra":
        attempts.append(copy.deepcopy(attempts[0]))
    elif defect == "wrong_label":
        attempts[-1]["label"] = "remaining116"
    else:
        attempts[-1]["exit_code"] = False

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, AUGMENTED_BATCH, 120)


@pytest.mark.parametrize("kind", ("authority", "augmented"))
@pytest.mark.parametrize("phase", PHASES)
def test_missing_entire_phase_is_rejected(kind: str, phase: str) -> None:
    summary = authority_summary() if kind == "authority" else augmented_summary()
    del summary["phase_counts"][phase]

    with pytest.raises(ValueError):
        verify_campaign_summary(summary, "authority_v4" if kind == "authority" else AUGMENTED_BATCH,
                                96 if kind == "authority" else 120)


@pytest.fixture
def original_campaign_fixture(tmp_path: Path) -> tuple[Path, delivery.Campaign]:
    """A miniature original bank with actual hashed compile/seal/audit bytes.

    Only the authority count is reduced; the real evidence verification code is
    used without replacing its seal, artifact, dynamics, or witness checks.
    """
    root = tmp_path / "workstation_magtrack"
    authority, state, origin = complete_evidence(root)
    authority["source_file"] = "trajectories/fake_open.csv"
    put(root / authority["source_file"],
        (root / "runs/fake_open__compiled__fixture/source_tip_trajectory.csv").read_bytes())
    proof = replay_index.verify_complete(root, authority, state, origin, check_seal)
    # Exercise the original authority's different count-key convention. The
    # source state and proof still use waypoint_count, as the runtime does.
    original_authority = copy.deepcopy(authority)
    original_authority["source_pose_count"] = original_authority.pop("waypoint_count")
    summary = authority_summary()
    summary.update(trajectory_count=1, complete_passed=1,
                   phase_counts={phase: {"passed": 1} for phase in PHASES})
    campaign = delivery.Campaign("authority_v4", "authority_v4", 1,
                                 "planning/authority_index_v4.json", "reports/replay_index.json",
                                 "continuous_augmented.replay")
    put(root / "runs/authority_v4/campaign_summary.json", summary)
    put(root / "runs/authority_v4/status.json", {
        "workspace": origin, "environment_id": "mla_short",
        "trajectories": {authority["trajectory_id"]: state},
    })
    put(root / campaign.authority_index, {
        "environment_id": "mla_short", "trajectory_count": 1,
        "trajectories": [original_authority],
    })
    final_row = {
        **authority, **proof, **{phase: "passed" for phase in PHASES},
        "ready_for_simulation_replay_on_this_host": True,
        "real_execution_authorized": False,
    }
    # The original bank's persisted inventory uses this older witness field.
    final_row["coherent_playback_witness"] = final_row.pop("coherent_controller_witness")
    put(root / campaign.final_inventory, {
        "environment_id": "mla_short", "trajectory_count": 1, "ready_on_this_host": 1,
        "simulation_only": True, "recordings": [final_row],
    })
    checker_source = Path(delivery.__file__).with_name("export_replay_index.py")
    put(root / "ops/export_replay_index.py", checker_source.read_bytes())
    return root, campaign


def test_original_campaign_rechecks_actual_proofs_and_adapts_source_pose_count(
    original_campaign_fixture: tuple[Path, delivery.Campaign],
) -> None:
    root, campaign = original_campaign_fixture

    rows, fingerprints = delivery.verify_campaign(root, campaign)

    assert len(rows) == 1
    assert rows[0]["trajectory_id"] == "fake_open"
    assert rows[0]["waypoint_count"] == 2
    assert rows[0]["coherent_controller_witness"] == "passed"
    assert rows[0]["artifact_fingerprints_verified"] is True
    assert rows[0]["real_execution_authorized"] is False
    assert set(fingerprints) == {
        "runs/authority_v4/campaign_summary.json", "runs/authority_v4/status.json",
        campaign.authority_index, campaign.final_inventory,
    }
    assert all(len(value) == 64 for value in fingerprints.values())


@pytest.mark.parametrize("key,value", (
    ("source_file", "trajectories/different.csv"),
    ("source_sha256", "0" * 64),
    ("duration_s", 2.0),
    ("recording_id", "another_recording"),
    ("recording_directory", "environments/mla_short/recordings/another_recording"),
    ("coherent_playback_witness", "pending"),
    ("play", "pending"),
    ("ready_for_simulation_replay_on_this_host", False),
    ("real_execution_authorized", True),
))
def test_original_campaign_rejects_final_inventory_disagreeing_with_actual_proof(
    original_campaign_fixture: tuple[Path, delivery.Campaign], key: str, value: object,
) -> None:
    root, campaign = original_campaign_fixture
    path = root / campaign.final_inventory
    inventory = json.loads(path.read_text())
    inventory["recordings"][0][key] = value
    put(path, inventory)

    with pytest.raises(ValueError):
        delivery.verify_campaign(root, campaign)


@pytest.mark.parametrize("defect", ("source_bytes", "seal_bytes", "witness_bytes", "source_count", "missing_state"))
def test_original_campaign_cannot_replace_byte_proof_with_green_status_flags(
    original_campaign_fixture: tuple[Path, delivery.Campaign], defect: str,
) -> None:
    root, campaign = original_campaign_fixture
    if defect == "source_bytes":
        put(root / "trajectories/fake_open.csv", "different authoritative input\n")
    elif defect == "seal_bytes":
        put(root / "environments/mla_short/recordings/fake_open__sealed/sim_actual_tip.csv", "changed raw tip\n")
    elif defect == "witness_bytes":
        put(root / "runs/batch/controller.csv", "changed raw controller witness\n")
    elif defect == "source_count":
        path = root / campaign.authority_index
        authority_index = json.loads(path.read_text())
        authority_index["trajectories"][0]["source_pose_count"] = 3
        put(path, authority_index)
    else:
        path = root / "runs/authority_v4/status.json"
        state = json.loads(path.read_text())
        state["trajectories"] = {}
        put(path, state)

    with pytest.raises(ValueError):
        delivery.verify_campaign(root, campaign)


@pytest.mark.parametrize("bad_alias", ("coherent_controller_witness", "coherent_playback_witness"))
def test_original_campaign_cannot_hide_failing_present_witness_alias(
    original_campaign_fixture: tuple[Path, delivery.Campaign], bad_alias: str,
) -> None:
    root, campaign = original_campaign_fixture
    path = root / campaign.final_inventory
    inventory = json.loads(path.read_text())
    row = inventory["recordings"][0]
    row.update(coherent_controller_witness="passed", coherent_playback_witness="passed")
    row[bad_alias] = "failed"
    put(path, inventory)

    with pytest.raises(ValueError):
        delivery.verify_campaign(root, campaign)


def live_process_result(lines: list[str]) -> SimpleNamespace:
    return SimpleNamespace(stdout="\n".join(lines), returncode=0)


@pytest.mark.parametrize("invocation", (
    "absolute_script", "module", "batch_equals", "module_batch_equals", "module_workspace_equals",
))
def test_live_pid_recognizes_absolute_and_module_campaign_invocations(
    tmp_path: Path, invocation: str,
) -> None:
    root = tmp_path / "path with spaces" / "workstation_magtrack"
    batch = "ca_batch120_20261016_v3"
    if invocation.startswith("module"):
        tokens = ["/usr/bin/python3", "-u", "-m", "continuous_augmented.replay_v3.run_campaign"]
    else:
        tokens = ["/usr/bin/python3", "-u", str(root / "continuous_augmented/replay_v3/run_campaign.py")]
    tokens += [f"--workspace={root}"] if invocation == "module_workspace_equals" else ["--workspace", str(root)]
    tokens += [f"--batch={batch}"] if invocation in {"batch_equals", "module_batch_equals"} else ["--batch", batch]
    seen: list[tuple[list[str], dict]] = []

    def runner(command: list[str], **kwargs: object) -> SimpleNamespace:
        seen.append((command, kwargs))
        return live_process_result([f"987654321 Ssl {shlex.join(tokens)}"])

    assert verified_live_pids(root, {batch}, runner) == [987654321]
    assert seen == [(["ps", "-eo", "pid=,stat=,args="],
                     {"check": True, "capture_output": True, "text": True})]


@pytest.mark.parametrize("script", (
    "collection/validate_parking.py", "collection/mock_parking.py",
    "collection.validate_parking", "collection.mock_parking",
))
def test_live_pid_recognizes_gpu_and_independent_mock_parking_job(
    tmp_path: Path, script: str,
) -> None:
    root = tmp_path / "workstation_magtrack"
    tokens = ["python3", str(root / script)] if script.endswith(".py") else ["python3", "-m", script]
    role = "gpu" if "validate_parking" in script else "mock"
    certificate = root / (delivery.PARKING_VALIDATION if role == "gpu" else delivery.PARKING_MOCK)
    tokens += ["--workspace", str(root), "--output", str(certificate.parent)]
    if role == "gpu":
        tokens.append("--gpu")

    assert verified_live_pids(root, set(), lambda *a, **kw: live_process_result([
        f"987654322 Sl {shlex.join(tokens)}",
    ])) == [987654322]


@pytest.mark.parametrize("defect", (
    "zombie", "own_pid", "exporter", "export_shell", "different_root", "root_prefix_only",
    "different_batch", "no_pid", "invalid_shell_quote", "not_campaign",
))
def test_live_pid_ignores_nonlive_or_unrelated_process_records(tmp_path: Path, defect: str) -> None:
    root = tmp_path / "workstation_magtrack"
    batch = "ca_batch120_20261016_v3"
    pid, status = 987654323, "Sl"
    actual_root = root
    if defect == "different_root":
        actual_root = tmp_path / "elsewhere/workstation_magtrack"
    elif defect == "root_prefix_only":
        actual_root = tmp_path / "workstation_magtrack_copy"
    script = "continuous_augmented/replay_v3/run_campaign.py"
    if defect == "exporter":
        script = "ops/export_overnight_delivery.py"
    elif defect == "export_shell":
        script = "ops/overnight_delivery.sh"
    elif defect == "not_campaign":
        script = "tools/check_environment.py"
    if defect == "own_pid":
        pid = os.getpid()
    if defect == "zombie":
        status = "Z+"
    selected_batch = "unrelated_batch" if defect == "different_batch" else batch
    command = shlex.join(["python3", str(actual_root / script), "--workspace", str(actual_root),
                          "--batch", selected_batch])
    line = f"{pid} {status} {command}"
    if defect == "no_pid":
        line = f"Sl {command}"
    elif defect == "invalid_shell_quote":
        line += " 'unterminated"

    assert verified_live_pids(root, {batch}, lambda *a, **kw: live_process_result([line])) == []


def test_live_pid_does_not_treat_stale_lock_state_or_declared_pid_as_running(tmp_path: Path) -> None:
    root = tmp_path / "workstation_magtrack"
    root.mkdir()
    (root / ".campaign.lock").write_text("987654324\n")
    (root / "status.json").write_text('{"pid":987654324,"state":"running"}')

    assert verified_live_pids(root, {"ca_batch120_20261016_v3"},
                              lambda *a, **kw: live_process_result([])) == []


def test_live_pid_observation_failure_is_not_silently_reported_as_terminal(tmp_path: Path) -> None:
    def failed_runner(*args: object, **kwargs: object) -> SimpleNamespace:
        raise subprocess.CalledProcessError(1, ["ps"])

    with pytest.raises(subprocess.CalledProcessError):
        verified_live_pids(tmp_path / "workstation_magtrack", set(), failed_runner)


def test_live_pid_results_are_sorted_and_unique(tmp_path: Path) -> None:
    root = tmp_path / "workstation_magtrack"
    batch = "ca_batch120_20261016_v3"
    command = shlex.join(["python3", str(root / "runtime/batch_recordings.py"),
                          "--workspace", str(root), "--batch", batch])
    lines = [f"987654326 S {command}", f"987654325 S {command}", f"987654326 S {command}"]

    assert verified_live_pids(root, {batch}, lambda *a, **kw: live_process_result(lines)) == [987654325, 987654326]


@pytest.mark.parametrize("wrapper", (
    "tmux", "bash", "sh", "env", "echo", "printf", "python_code", "python_other_module", "python_other_script",
))
def test_live_pid_rejects_retained_or_nested_launch_command_not_actual_python_producer(
    tmp_path: Path, wrapper: str,
) -> None:
    root = tmp_path / "workstation_magtrack"
    batch = "ca_batch120_20261016_v3"
    producer = ["python3", "-u", str(root / "continuous_augmented/replay_v3/run_campaign.py"),
                "--workspace", str(root), "--batch", batch]
    wrappers = {
        "tmux": ["tmux", "new-session", "-d", "-s", "retained-launch"],
        "bash": ["/bin/bash", "-c"],
        "sh": ["/bin/sh", "-c"],
        "env": ["/usr/bin/env"],
        "echo": ["/bin/echo"],
        "printf": ["/usr/bin/printf", "%s"],
        "python_code": ["python3", "-c", "print('not a campaign')"],
        "python_other_module": ["python3", "-m", "unrelated.application"],
        "python_other_script": ["python3", str(tmp_path / "unrelated.py")],
    }
    command = shlex.join(wrappers[wrapper] + producer)

    assert verified_live_pids(root, {batch}, lambda *a, **kw: live_process_result([
        f"987654327 Sl {command}",
    ])) == []


@pytest.mark.parametrize("stage", ("gpu", "mock", "bundle"))
def test_live_pid_only_accepts_exact_output_producer_for_requested_missing_parking_stage(
    tmp_path: Path, stage: str,
) -> None:
    root = tmp_path / "workstation_magtrack"
    paths = {"gpu": root / delivery.PARKING_VALIDATION,
             "mock": root / delivery.PARKING_MOCK,
             "bundle": root / delivery.PARKING_BUNDLE}
    modules = {"gpu": "collection.validate_parking", "mock": "collection.mock_parking",
               "bundle": "collection.prepared_parking"}
    tokens = ["python3", "-u", "-m", modules[stage]]
    if stage == "bundle":
        tokens.append("create")
    tokens += ["--workspace", str(root), "--output", str(paths[stage] if stage == "bundle" else paths[stage].parent)]
    if stage == "gpu":
        tokens.append("--gpu")
    runner = lambda *a, **kw: live_process_result([f"987654328 Sl {shlex.join(tokens)}"])

    assert verified_live_pids(root, set(), runner, parking_producers={stage: paths[stage]}) == [987654328]
    assert verified_live_pids(root, set(), runner, parking_producers={}) == []


@pytest.mark.parametrize("defect", ("wrong_output", "referenced_as_input", "wrong_stage", "verify_only"))
def test_live_pid_does_not_borrow_unrelated_parking_process_to_wait_for_missing_artifact(
    tmp_path: Path, defect: str,
) -> None:
    root = tmp_path / "workstation_magtrack"
    needed = root / delivery.PARKING_MOCK
    output = needed.parent if defect != "wrong_output" else root / "runs/other_mock_run"
    module_name = "collection.mock_parking" if defect != "wrong_stage" else "collection.validate_parking"
    tokens = ["python3", "-m", module_name, "--workspace", str(root), "--output", str(output)]
    if defect == "referenced_as_input":
        tokens = ["python3", "-m", "collection.mock_parking", "--workspace", str(root),
                  "--output", str(root / "runs/other_mock_run"), "--validation", str(needed)]
    if defect == "verify_only":
        tokens.append("--verify-only")

    assert verified_live_pids(root, set(), lambda *a, **kw: live_process_result([
        f"987654329 Sl {shlex.join(tokens)}",
    ]), parking_producers={"mock": needed}) == []


def test_live_pid_cpu_precheck_is_not_a_producer_of_required_gpu_certificate(tmp_path: Path) -> None:
    root = tmp_path / "workstation_magtrack"
    needed = root / delivery.PARKING_VALIDATION
    tokens = ["python3", "-m", "collection.validate_parking", "--workspace", str(root),
              "--output", str(needed.parent)]

    assert verified_live_pids(root, set(), lambda *a, **kw: live_process_result([
        f"987654330 Sl {shlex.join(tokens)}",
    ]), parking_producers={"gpu": needed}) == []


@pytest.mark.parametrize("missing", ("gpu", "mock", "bundle"))
def test_wait_main_only_observes_current_missing_producers_not_completed_campaigns(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, missing: str,
) -> None:
    root = tmp_path / "workstation_magtrack"
    state = {
        "accepted": False, "prerequisites_ready_for_full_byte_audit": False, "terminal_failure": False,
        "campaigns": [{"batch_id": campaign.source_batch_id, "accepted_terminal_workflow": True}
                      for campaign in delivery.CAMPAIGNS],
        "parking": {"gpu_validation": "passed", "independent_mock": "passed", "prepared_bundle": "present"},
    }
    state["parking"][{"gpu": "gpu_validation", "mock": "independent_mock", "bundle": "prepared_bundle"}[missing]] = "missing"
    observed: list[tuple[set[str], dict]] = []
    monkeypatch.setattr(delivery, "probe", lambda *a, **kw: copy.deepcopy(state))

    def no_live_processes(workspace: Path, batches: set[str], **kwargs: object) -> list[int]:
        assert workspace == root
        observed.append((batches, kwargs))
        return []

    monkeypatch.setattr(delivery, "verified_live_pids", no_live_processes)
    # The real CLI deliberately runs only on Ubuntu; this is a mocked,
    # non-publishing control-flow fixture, not a Mac delivery export.
    monkeypatch.setattr(delivery.sys, "platform", "linux")
    monkeypatch.setattr(sys, "argv", ["export_overnight_delivery.py", "--workspace", str(root), "--wait"])

    assert delivery.main() == 2

    expected_path = root / {"gpu": delivery.PARKING_VALIDATION, "mock": delivery.PARKING_MOCK,
                            "bundle": delivery.PARKING_BUNDLE}[missing]
    assert observed == [(set(), {"parking_producers": {missing: expected_path}})]


@pytest.mark.parametrize("unfinished_index", range(3))
def test_wait_main_does_not_borrow_completed_campaign_to_wait_for_other_campaign(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, unfinished_index: int,
) -> None:
    root = tmp_path / "workstation_magtrack"
    rows = [{"batch_id": campaign.source_batch_id, "accepted_terminal_workflow": True}
            for campaign in delivery.CAMPAIGNS]
    rows[unfinished_index]["accepted_terminal_workflow"] = False
    state = {"accepted": False, "prerequisites_ready_for_full_byte_audit": False,
             "terminal_failure": False, "campaigns": rows,
             "parking": {"gpu_validation": "passed", "independent_mock": "passed", "prepared_bundle": "present"}}
    observed: list[tuple[set[str], dict]] = []
    monkeypatch.setattr(delivery, "probe", lambda *a, **kw: copy.deepcopy(state))

    def no_live_processes(workspace: Path, batches: set[str], **kwargs: object) -> list[int]:
        observed.append((batches, kwargs))
        return []

    monkeypatch.setattr(delivery, "verified_live_pids", no_live_processes)
    monkeypatch.setattr(delivery.sys, "platform", "linux")
    monkeypatch.setattr(sys, "argv", ["export_overnight_delivery.py", "--workspace", str(root), "--wait"])

    assert delivery.main() == 2

    assert observed == [({delivery.CAMPAIGNS[unfinished_index].run_batch_id}, {"parking_producers": {}})]


def test_wrapper_imports_all_offline_evidence_dependencies_in_clean_fresh_environment(tmp_path: Path) -> None:
    """Exercise the actual wrapper's PYTHONPATH, with no inherited source path.

    Its temporary Python shim performs imports only: it intentionally never
    calls the exporter, an audit, ROS, a GPU operation, or a serial session.
    """
    project, source_project = tmp_path / "project", Path(delivery.__file__).resolve().parents[2]
    workspace = project / "workstation_magtrack"
    wrapper = workspace / "ops/overnight_delivery.sh"
    wrapper.parent.mkdir(parents=True)
    wrapper.write_bytes((Path(delivery.__file__).with_name("overnight_delivery.sh")).read_bytes())
    (workspace / "collection").symlink_to(source_project / "workstation_magtrack/collection", target_is_directory=True)
    (project / "src").mkdir()
    for name in ("hx_waypoint_curobo", "hx_waypoint_tools", "hx_waypoint_nucleo"):
        (project / "src" / name).symlink_to(source_project / "src" / name, target_is_directory=True)
    interpreter = project / ".venv/bin/python"
    interpreter.parent.mkdir(parents=True)
    interpreter.write_text(f"#!{sys.executable}\n" + """import sys
from collection.mock_parking import verify_scene_snapshot
import collection.prepared_parking
from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition
assert callable(verify_scene_snapshot) and callable(load_planning_scene_definition)
assert 'rclpy' not in sys.modules
assert 'torch' not in sys.modules
print('OFFLINE_DEPENDENCIES_IMPORTED')
""")
    interpreter.chmod(0o755)
    environment = {"PATH": os.defpath, "HOME": str(tmp_path / "empty-home"), "PYTHONNOUSERSITE": "1"}

    result = subprocess.run(["bash", str(wrapper), "--once"], cwd=tmp_path, env=environment,
                            check=False, capture_output=True, text=True, timeout=20)

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "OFFLINE_DEPENDENCIES_IMPORTED"
