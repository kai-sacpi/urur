"""Supervisor must not trust stale passed counters after a failed operation."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from workstation_magtrack.ops import finish_campaign as campaign


@pytest.fixture
def prepared(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    root = tmp_path / "workstation_magtrack"
    directory = root / "runs/authority_v4"
    directory.mkdir(parents=True)
    index = root / "planning/authority_index_v4.json"
    index.parent.mkdir()
    index.write_text(json.dumps({"trajectories": [{"trajectory_id": "a"}]}))
    (directory / "status.json").write_text(json.dumps({
        "trajectories": {"a": {phase: {"status": "passed"}
                                  for phase in ("compile", "build", "inspect", "play")}},
    }))
    monkeypatch.setattr(campaign, "ROOT", root)
    monkeypatch.setattr(campaign, "DIRECTORY", directory)
    monkeypatch.setattr(campaign, "INDEX", index)
    return directory


def test_stale_passed_counters_cannot_hide_operation_failure(prepared: Path) -> None:
    report = campaign.summarize("finished", {"build": 2, "inspect": 2, "play": 2})
    assert report["complete_passed"] == 1
    assert report["workflow_passed"] is False
    assert campaign.summarize("waiting", {})["workflow_passed"] is False
    assert campaign.summarize("finished", {"build": 0, "inspect": 0, "play": 0})["workflow_passed"] is True


def test_failed_child_operations_exit_failure_despite_old_pass_state(
    prepared: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(campaign.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=2))
    assert campaign.main() == 2
    report = json.loads((prepared / "campaign_summary.json").read_text())
    assert report["workflow_passed"] is False
    assert report["phase_exit_codes"] == {
        "build": 2, "inspect": 2, "play": 2, "inventory": 2, "independent_source_audit": 2,
    }
