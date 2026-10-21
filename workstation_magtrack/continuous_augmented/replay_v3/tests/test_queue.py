"""Campaign queue isolation tests using local locks and instantaneous waits only."""
from __future__ import annotations

import fcntl
import json
from pathlib import Path
import signal
from types import SimpleNamespace
import threading
from unittest.mock import Mock

import pytest

from workstation_magtrack.continuous_augmented.replay_v3 import run_campaign as campaign


@pytest.fixture
def queued(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> campaign.Campaign:
    instance = campaign.Campaign.__new__(campaign.Campaign)
    instance.root = tmp_path / "workstation_magtrack"
    instance.args = SimpleNamespace(batch="v3_unit_queue", jobs=2, compile_timeout=10.0,
                                    canary=[], wait_batch=[], wait_for_campaign=True)
    instance.directory = instance.root / "runs" / instance.args.batch
    instance.directory.mkdir(parents=True)
    (instance.root / "continuous_augmented/replay").mkdir(parents=True)
    (instance.root / "continuous_augmented/replay_v3").mkdir(parents=True)
    instance.status_path = instance.directory / "status.json"
    instance.summary_path = instance.directory / "campaign_summary.json"
    instance.index = {"trajectories": [{"trajectory_id": f"route_{i:03d}"} for i in range(120)]}
    instance.current = None
    instance.stop = threading.Event()
    instance.attempts = []
    monkeypatch.setattr(instance, "run_phase", Mock(side_effect=AssertionError("no phase may start in queue")))
    monkeypatch.setattr(instance, "validate_authority", Mock())
    monkeypatch.setattr(campaign.signal, "signal", Mock())
    monkeypatch.setattr(campaign.subprocess, "Popen", Mock(side_effect=AssertionError("no subprocess may start")))
    monkeypatch.setattr(campaign.subprocess, "run", Mock(side_effect=AssertionError("no subprocess may start")))
    return instance


def assert_locked(path: Path) -> None:
    with path.open("a+") as contender:
        with pytest.raises(BlockingIOError):
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)


@pytest.mark.parametrize("release_before_cancel", [False, True])
def test_v3_waits_on_v2_shared_lock_holds_own_batch_and_cancel_is_safe(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch, release_before_cancel: bool,
) -> None:
    global_path = queued.root / "continuous_augmented/replay/.campaign.lock"
    run_locked = Mock(side_effect=AssertionError("busy/cancelled queue must not dispatch"))
    monkeypatch.setattr(queued, "run_locked", run_locked)
    with global_path.open("a+") as previous_owner:
        fcntl.flock(previous_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)

        def cancel_wait(seconds: float) -> bool:
            assert seconds == 5
            assert_locked(queued.directory / ".campaign.lock")
            run_locked.assert_not_called()
            queued.run_phase.assert_not_called()
            campaign.subprocess.Popen.assert_not_called()
            if release_before_cancel:
                fcntl.flock(previous_owner, fcntl.LOCK_UN)
            queued.interrupted(signal.SIGINT, None)
            return True

        wait = Mock(side_effect=cancel_wait)
        monkeypatch.setattr(queued.stop, "wait", wait)
        assert queued.run() == 130
    wait.assert_called_once_with(5)
    run_locked.assert_not_called()
    queued.run_phase.assert_not_called()
    campaign.subprocess.Popen.assert_not_called()
    campaign.subprocess.run.assert_not_called()
    report = json.loads(queued.summary_path.read_text())
    assert report["state"] == "interrupted"
    assert report["workflow_passed"] is False
    assert report["command_attempts"] == []
    assert report["robot_used"] is False
    assert report["sensor_used"] is False
    assert not (queued.root / "continuous_augmented/replay_v3/.campaign.lock").exists()


def test_released_shared_lock_dispatches_once_while_both_locks_are_held(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    global_path = queued.root / "continuous_augmented/replay/.campaign.lock"

    def dispatch() -> int:
        assert not queued.stop.is_set()
        assert_locked(global_path)
        assert_locked(queued.directory / ".campaign.lock")
        return 0

    run_locked = Mock(side_effect=dispatch)
    monkeypatch.setattr(queued, "run_locked", run_locked)
    with global_path.open("a+") as previous_owner:
        fcntl.flock(previous_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)

        def release_wait(seconds: float) -> bool:
            assert seconds == 5
            assert_locked(queued.directory / ".campaign.lock")
            run_locked.assert_not_called()
            fcntl.flock(previous_owner, fcntl.LOCK_UN)
            return False

        wait = Mock(side_effect=release_wait)
        monkeypatch.setattr(queued.stop, "wait", wait)
        assert queued.run() == 0
    wait.assert_called_once_with(5)
    run_locked.assert_called_once_with()
    queued.run_phase.assert_not_called()
    campaign.subprocess.Popen.assert_not_called()
    campaign.subprocess.run.assert_not_called()
    for path in (global_path, queued.directory / ".campaign.lock"):
        with path.open("a+") as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)


def test_nonwaiting_main_reports_busy_failure_without_dispatch(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    queued.args.wait_for_campaign = False
    run_locked = Mock(side_effect=AssertionError("nonwaiting busy queue must not dispatch"))
    wait = Mock(side_effect=AssertionError("nonwaiting mode must not wait"))
    monkeypatch.setattr(queued, "run_locked", run_locked)
    monkeypatch.setattr(queued.stop, "wait", wait)
    monkeypatch.setattr(campaign, "Campaign", Mock(return_value=queued))
    monkeypatch.setattr(campaign.sys, "argv", ["run_campaign.py", "--workspace", str(queued.root)])
    global_path = queued.root / "continuous_augmented/replay/.campaign.lock"
    with global_path.open("a+") as previous_owner:
        fcntl.flock(previous_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert campaign.main() == 2
    wait.assert_not_called()
    run_locked.assert_not_called()
    campaign.subprocess.Popen.assert_not_called()
    report = json.loads(queued.summary_path.read_text())
    assert report["state"] == "failed"
    assert report["workflow_passed"] is False
    assert report["command_attempts"] == []
    assert "--wait-for-campaign" in report["error"]


def test_same_batch_queue_owner_is_rejected_without_wait_or_summary_write(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    sentinel = b"queued owner summary\n"
    queued.summary_path.write_bytes(sentinel)
    run_locked = Mock(side_effect=AssertionError("duplicate batch must not dispatch"))
    wait = Mock(side_effect=AssertionError("duplicate batch must not wait"))
    monkeypatch.setattr(queued, "run_locked", run_locked)
    monkeypatch.setattr(queued.stop, "wait", wait)
    monkeypatch.setattr(campaign, "Campaign", Mock(return_value=queued))
    monkeypatch.setattr(campaign.sys, "argv", ["run_campaign.py", "--wait-for-campaign"])
    with (queued.directory / ".campaign.lock").open("a+") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert campaign.main() == 2
    wait.assert_not_called()
    run_locked.assert_not_called()
    campaign.subprocess.Popen.assert_not_called()
    assert queued.summary_path.read_bytes() == sentinel


def test_cli_exposes_opt_in_wait_for_campaign_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    constructed = Mock(return_value=SimpleNamespace(run=lambda: 0))
    monkeypatch.setattr(campaign, "Campaign", constructed)
    monkeypatch.setattr(campaign.sys, "argv", ["run_campaign.py", "--wait-for-campaign"])
    assert campaign.main() == 0
    assert constructed.call_args.args[0].wait_for_campaign is True


def test_authority_change_while_queued_is_rejected_after_lock_before_dispatch(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    global_path = queued.root / "continuous_augmented/replay/.campaign.lock"
    run_locked = Mock(side_effect=AssertionError("changed authorities must never dispatch"))
    fresh_check = Mock(side_effect=ValueError("queued source authority changed"))
    monkeypatch.setattr(queued, "run_locked", run_locked)
    monkeypatch.setattr(queued, "validate_authority", fresh_check)
    with global_path.open("a+") as previous_owner:
        fcntl.flock(previous_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)

        def release_after_change(seconds: float) -> bool:
            fresh_check.assert_not_called()
            run_locked.assert_not_called()
            fcntl.flock(previous_owner, fcntl.LOCK_UN)
            return False

        monkeypatch.setattr(queued.stop, "wait", Mock(side_effect=release_after_change))
        assert queued.run() == 2
    fresh_check.assert_called_once_with()
    run_locked.assert_not_called()
    campaign.subprocess.Popen.assert_not_called()
    report = json.loads(queued.summary_path.read_text())
    assert report["state"] == "failed"
    assert "queued source authority changed" in report["error"]


def test_fresh_authority_check_rejects_index_bytes_changed_since_construction(
    queued: campaign.Campaign, monkeypatch: pytest.MonkeyPatch,
) -> None:
    planning = queued.root / "continuous_augmented/replay_v3/planning"
    planning.mkdir(parents=True)
    (planning / "execution_index.json").write_text(json.dumps({"trajectories": []}))
    queued.protected_snapshot = {"sha256": {}}
    (planning / "protected_inputs.json").write_text(json.dumps(queued.protected_snapshot))
    monkeypatch.setattr(campaign, "validate_sources", Mock(return_value=queued.index))
    monkeypatch.setattr(campaign, "protected", Mock(return_value={}))
    with pytest.raises(ValueError, match="frozen source index"):
        campaign.Campaign.validate_authority(queued)
