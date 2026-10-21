"""Fast supervisor contract tests: no ROS, GPU, SSH, or sensor access."""
from __future__ import annotations

from copy import deepcopy
import fcntl
from pathlib import Path
import signal
from types import SimpleNamespace
import threading
from unittest.mock import Mock

import pytest

from workstation_magtrack.continuous_augmented.replay import run_campaign as campaign


def authority_index(count: int = 120) -> dict:
    """Synthetic, unique authority rows; these tests do not stage source CSVs."""
    return {
        "source_batch_id": campaign.BATCH,
        "trajectory_count": count,
        "trajectories": [
            {"trajectory_id": f"route_{number:03d}", "source_sha256": f"{number:064x}",
             "duration_s": 10.0 + number, "waypoint_count": 201 + number,
             "group_id": "full_height_one_way" if number in (1, 2) else "fixed_height",
             "profile": {"lower_m": .07, "upper_m": .20 if number in (1, 2) else .07,
                         "direction": {1: "ascending_height", 2: "descending_height"}.get(number, "constant")}}
            for number in range(count)
        ],
    }


def state_for(root: Path, index: dict, completed: int = 0) -> dict:
    entries = {}
    for row in index["trajectories"][:completed]:
        entries[row["trajectory_id"]] = {
            "source": campaign.source_identity(row),
            **{phase: {"status": "passed"} for phase in campaign.PHASES},
        }
    return {"workspace": str(root), "environment_id": "mla_short", "trajectories": entries}


@pytest.fixture
def supervisor(tmp_path: Path) -> campaign.Campaign:
    """Bypass only expensive immutable-source validation, not tested methods."""
    instance = campaign.Campaign.__new__(campaign.Campaign)
    instance.root = tmp_path / "workstation_magtrack"
    instance.args = SimpleNamespace(batch="unit_campaign", jobs=2, compile_timeout=10.0,
                                    canary=[], wait_batch=[])
    instance.directory = instance.root / "runs" / instance.args.batch
    instance.directory.mkdir(parents=True)
    (instance.root / "continuous_augmented/replay").mkdir(parents=True)
    instance.status_path = instance.directory / "status.json"
    instance.summary_path = instance.directory / "campaign_summary.json"
    instance.current = None
    instance.stop = threading.Event()
    instance.attempts = []
    instance.index = authority_index()
    return instance


@pytest.mark.parametrize("phase,prerequisite", [
    ("build", "compile"), ("inspect", "build"), ("play", "inspect"),
])
def test_phase_complete_requires_terminal_only_for_eligible_sources(
    phase: str, prerequisite: str,
) -> None:
    names = ["passed_upstream", "failed_upstream", "missing_upstream"]
    status = {"trajectories": {
        "passed_upstream": {prerequisite: {"status": "passed"}, phase: {"status": "running"}},
        "failed_upstream": {prerequisite: {"status": "failed"}},
    }}
    assert campaign.phase_complete(status, names, phase) is False
    status["trajectories"]["passed_upstream"][phase]["status"] = "passed"
    assert campaign.phase_complete(status, names, phase) is True
    status["trajectories"]["passed_upstream"][phase]["status"] = "failed"
    assert campaign.phase_complete(status, names, phase) is True


def test_compile_requires_every_requested_source_terminal() -> None:
    status = {"trajectories": {"a": {"compile": {"status": "passed"}}}}
    assert campaign.phase_complete(status, ["a", "b"], "compile") is False
    status["trajectories"]["b"] = {"compile": {"status": "failed"}}
    assert campaign.phase_complete(status, ["a", "b"], "compile") is True


def test_state_accepts_partial_state_bound_to_exact_authority(supervisor: campaign.Campaign) -> None:
    state = state_for(supervisor.root, supervisor.index, completed=1)
    campaign.verify_state(state, supervisor.index, supervisor.root)


@pytest.mark.parametrize("key,replacement", [
    ("sha256", "f" * 64), ("duration_s", 999.0), ("waypoint_count", 999),
])
def test_state_rejects_different_source_of_same_id(
    supervisor: campaign.Campaign, key: str, replacement: object,
) -> None:
    state = state_for(supervisor.root, supervisor.index, completed=1)
    state["trajectories"]["route_000"]["source"][key] = replacement
    with pytest.raises(ValueError, match="different source"):
        campaign.verify_state(state, supervisor.index, supervisor.root)


def test_state_rejects_foreign_batch_ids(supervisor: campaign.Campaign) -> None:
    state = state_for(supervisor.root, supervisor.index)
    state["trajectories"]["previous_authority_v4_route"] = {
        "source": campaign.source_identity(supervisor.index["trajectories"][0]),
        **{phase: {"status": "passed"} for phase in campaign.PHASES},
    }
    with pytest.raises(ValueError, match="nonauthority"):
        campaign.verify_state(state, supervisor.index, supervisor.root)


@pytest.mark.parametrize("key,replacement", [
    ("workspace", "/other/workstation_magtrack"), ("environment_id", "other_environment"),
])
def test_state_rejects_foreign_workspace_or_environment(
    supervisor: campaign.Campaign, key: str, replacement: str,
) -> None:
    state = state_for(supervisor.root, supervisor.index, completed=120)
    state[key] = replacement
    with pytest.raises(ValueError, match="different workspace/environment"):
        campaign.verify_state(state, supervisor.index, supervisor.root)


def test_canary_success_is_not_full_120_readiness(supervisor: campaign.Campaign) -> None:
    state = state_for(supervisor.root, supervisor.index, completed=1)
    result = campaign.summary(state, supervisor.index, "complete", [{"exit_code": 0}],
                              {"ready_on_this_host": 1})
    assert result["expected_trajectory_count"] == 120
    assert result["complete_four_stage_passed"] == 1
    assert result["phase_counts"]["play"] == {"passed": 1, "pending": 119}
    assert result["workflow_passed"] is False
    assert result["real_execution_authorized"] is False


def test_accidental_canary_only_index_cannot_be_reported_as_complete(
    supervisor: campaign.Campaign,
) -> None:
    canary_index = authority_index(count=1)
    result = campaign.summary(state_for(supervisor.root, canary_index, completed=1),
                              canary_index, "complete", [{"exit_code": 0}], {"ready_on_this_host": 1})
    assert result["workflow_passed"] is False


@pytest.mark.parametrize("attempts", [[], [{"exit_code": 0}, {"exit_code": 2}]])
def test_passed_counters_cannot_hide_missing_or_failed_command(
    supervisor: campaign.Campaign, attempts: list[dict],
) -> None:
    result = campaign.summary(state_for(supervisor.root, supervisor.index, completed=120),
                              supervisor.index, "complete", attempts, {"ready_on_this_host": 120})
    assert result["complete_four_stage_passed"] == 120
    assert result["workflow_passed"] is False


@pytest.mark.parametrize("inventory", [None, {"ready_on_this_host": 119}, {"ready_on_this_host": 121}])
def test_full_status_requires_exact_full_verified_inventory(
    supervisor: campaign.Campaign, inventory: dict | None,
) -> None:
    result = campaign.summary(state_for(supervisor.root, supervisor.index, completed=120),
                              supervisor.index, "complete", [{"exit_code": 0}], inventory)
    assert result["workflow_passed"] is False


def test_only_completed_workflow_with_exact_authority_is_ready(supervisor: campaign.Campaign) -> None:
    state = state_for(supervisor.root, supervisor.index, completed=120)
    arguments = (state, supervisor.index, "complete", [{"exit_code": 0}], {"ready_on_this_host": 120})
    assert campaign.summary(*arguments)["workflow_passed"] is True
    assert campaign.summary(state, supervisor.index, "canary_inventory_exported", arguments[3],
                            arguments[4])["workflow_passed"] is False
    state["trajectories"]["foreign_route"] = deepcopy(state["trajectories"]["route_000"])
    assert campaign.summary(*arguments)["workflow_passed"] is False


def test_interruption_signals_only_owned_live_coordinator(
    supervisor: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    child = Mock()
    child.poll.return_value = None
    supervisor.current = child
    kill = Mock(side_effect=AssertionError("must not signal arbitrary process IDs"))
    killpg = Mock(side_effect=AssertionError("must not signal foreign process groups"))
    monkeypatch.setattr(campaign.os, "kill", kill)
    monkeypatch.setattr(campaign.os, "killpg", killpg)
    supervisor.interrupted(signal.SIGTERM, None)
    assert supervisor.stop.is_set()
    child.send_signal.assert_called_once_with(signal.SIGINT)
    kill.assert_not_called()
    killpg.assert_not_called()


@pytest.mark.parametrize("with_finished_child", [False, True])
def test_interruption_does_not_signal_absent_or_finished_child(
    supervisor: campaign.Campaign, with_finished_child: bool,
) -> None:
    child = Mock()
    child.poll.return_value = 0
    supervisor.current = child if with_finished_child else None
    supervisor.interrupted(signal.SIGINT, None)
    assert supervisor.stop.is_set()
    child.send_signal.assert_not_called()


@pytest.mark.parametrize("scope", ["same_batch", "any_batch"])
def test_second_supervisor_cannot_overwrite_active_summary(
    supervisor: campaign.Campaign, monkeypatch: pytest.MonkeyPatch, scope: str,
) -> None:
    sentinel = b"active owner summary\n"
    supervisor.summary_path.write_bytes(sentinel)
    run_locked, save_summary = Mock(), Mock()
    monkeypatch.setattr(supervisor, "run_locked", run_locked)
    monkeypatch.setattr(supervisor, "save_summary", save_summary)
    lock_path = (supervisor.directory / ".campaign.lock" if scope == "same_batch"
                 else supervisor.root / "continuous_augmented/replay/.campaign.lock")
    with lock_path.open("a+") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            supervisor.run()
    run_locked.assert_not_called()
    save_summary.assert_not_called()
    assert supervisor.summary_path.read_bytes() == sentinel


def test_default_three_canaries_finish_before_remaining117_and_final_readiness(
    supervisor: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    index_path = supervisor.root / "continuous_augmented/replay/planning/execution_index.json"
    index_path.parent.mkdir(parents=True)
    index_path.write_text("{}\n")
    state = state_for(supervisor.root, supervisor.index)
    calls: list[tuple[str, list[str], str]] = []
    reports: list[dict] = []

    def fake_phase(names: list[str], label: str, phase: str) -> None:
        calls.append((label, names[:], phase))
        supervisor.attempts.append({"label": label, "phase": phase, "exit_code": 0})
        for name in names:
            state["trajectories"].setdefault(name, {})[phase] = {"status": "passed"}

    def fake_summary(label: str, inventory: dict | None = None) -> dict:
        report = campaign.summary(state, supervisor.index, label, supervisor.attempts, inventory)
        reports.append(report)
        return report

    monkeypatch.setattr(supervisor, "run_phase", fake_phase)
    monkeypatch.setattr(supervisor, "save_summary", fake_summary)
    monkeypatch.setattr(supervisor, "wait_existing", Mock())
    monkeypatch.setattr(campaign.signal, "signal", Mock())
    monkeypatch.setattr(campaign, "export", Mock(side_effect=[
        {"ready_on_this_host": 3}, {"ready_on_this_host": 120}, {"ready_on_this_host": 120},
    ]))
    assert supervisor.run_locked() == 0
    assert [(label, phase) for label, _, phase in calls] == [
        (label, phase) for label in ("canary", "remaining117") for phase in campaign.PHASES
    ]
    assert all(names == ["route_000", "route_001", "route_002"] for _, names, _ in calls[:4])
    assert all(len(names) == 117 and not {"route_000", "route_001", "route_002"}.intersection(names)
               for _, names, _ in calls[4:])
    assert reports[1]["complete_four_stage_passed"] == 3
    assert reports[1]["workflow_passed"] is False
    assert reports[-1]["complete_four_stage_passed"] == 120
    assert reports[-1]["workflow_passed"] is True


def test_signal_during_popen_interrupts_just_spawned_owned_coordinator(
    supervisor: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    child = Mock()
    child.returncode = None
    child.poll.side_effect = lambda: child.returncode

    def finish_on_interrupt(signum: int) -> None:
        assert signum == signal.SIGINT
        child.returncode = -signal.SIGINT

    child.send_signal.side_effect = finish_on_interrupt

    def spawn_with_signal(*args: object, **kwargs: object) -> Mock:
        assert supervisor.current is None
        supervisor.interrupted(signal.SIGTERM, None)
        return child

    popen = Mock(side_effect=spawn_with_signal)
    save_summary = Mock()
    read_status = Mock(side_effect=AssertionError("interrupted dispatch must not continue validation"))
    monkeypatch.setattr(campaign.subprocess, "Popen", popen)
    monkeypatch.setattr(supervisor, "save_summary", save_summary)
    monkeypatch.setattr(supervisor, "read_status", read_status)
    monkeypatch.setattr(campaign.time, "sleep", Mock(side_effect=AssertionError("mock child already stopped")))
    with pytest.raises(KeyboardInterrupt):
        supervisor.run_phase(["route_000"], "canary", "compile")
    assert supervisor.stop.is_set()
    assert supervisor.current is None
    child.send_signal.assert_called_once_with(signal.SIGINT)
    read_status.assert_not_called()
    assert supervisor.attempts[0]["exit_code"] == -signal.SIGINT
    assert popen.call_count == 1
    assert popen.call_args.kwargs["start_new_session"] is True
    save_summary.assert_called_once_with("canary_compile_finished")
