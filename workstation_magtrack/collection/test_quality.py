"""Offline bounded-loss policy tests; never open serial or dispatch robot goals."""
from __future__ import annotations

import copy
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import pytest

from hx_waypoint_nucleo.capture import (
    NucleoCaptureConfig, NucleoCaptureSummary, RAW_CAPTURE_COLUMNS, finalize_nucleo_capture,
)
from hx_waypoint_nucleo.protocol import ParserStats

from .quality import BOUNDED_POLICY, STRICT_POLICY, evaluate_window


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_csv(path: Path, columns: tuple[str, ...] | list[str], rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, columns)
        writer.writeheader()
        writer.writerows(rows)


@pytest.fixture
def sample(tmp_path: Path):
    def create(*, missing: tuple[int, ...] = (100,), kind: str = "trajectory",
               missing_channels: int = 1, timed_out: bool = False):
        workspace = tmp_path / f"workspace_{len(list(tmp_path.iterdir()))}"
        root = workspace / "runs" / "test"
        root.mkdir(parents=True)
        config = NucleoCaptureConfig(port="offline://no_serial", perform_sync=False)
        rows = []
        count = 0
        losses = 0
        start = 10_000_000_000
        for n in range(3001 if kind == "trajectory" else 3000):
            if n in missing:
                losses += 1
                continue
            row = {name: "0" for name in RAW_CAPTURE_COLUMNS}
            row.update(sensor_rx_sync_time_ns=str(start + n * 10_000_000),
                       sensor_rx_ros_time_ns=str(1_000_000_000_000 + n * 10_000_000),
                       parser_host_monotonic_ns=str(start + n * 10_000_000),
                       logical_seq=str(count), src_seq=str(n % 16),
                       source_time_ms_u32=str(n * 10), channel_ids="1;2;3;4",
                       parser_hub_packets=str(4 * (n + 1) - losses * missing_channels),
                       parser_completed_groups=str(count + 1),
                       parser_incomplete_groups=str(losses if missing_channels < 4 else 0),
                       parser_source_sequence_drops=str(losses if missing_channels == 4 else 0),
                       parser_timeout_groups=str(losses if timed_out else 0),
                       parser_channel_sequence_drops=str(losses * missing_channels))
            for channel in range(1, 5):
                row[f"channel_{channel}_seq"] = str(n)
                row[f"channel_{channel}_nucleo_time_ms_u32"] = str(n * 10)
                row[f"channel_{channel}_source_time_ms_u32"] = str(n * 10)
            rows.append(row)
            count += 1
        raw = root / "nucleo_raw_telemetry.csv"
        _write_csv(raw, RAW_CAPTURE_COLUMNS, rows)
        counter_names = [name for name in RAW_CAPTURE_COLUMNS if name.startswith("parser_")
                         and name != "parser_host_monotonic_ns"]
        baseline = {name: int(rows[0][name]) if kind == "trajectory" else 0 for name in counter_names}
        deltas = {name: int(rows[-1][name]) - baseline[name] for name in counter_names}
        identity = {"session_id": "test_session", "epoch_id": "test_epoch",
                    "hardware_boot_id": "physical_boot", "host_boot_id": "host_boot"}
        readiness = {"status": "observed", "error": None, "unavailable_frames": 0,
                     "time_mapping": "host_monotonic_receive_time_not_device_time",
                     "timestamps_modified": False, "sensor_values_modified": False,
                     "absolute_clock_synchronization_calibrated": False,
                     "startup_baseline": {"required": True, "ready": True, "status": "ready"}}
        drift = root / "drift.json"
        drift.write_text(json.dumps({**readiness, **identity}), encoding="utf-8")
        result = {**identity, "kind": kind, "status": "incomplete",
                  "raw_csv": str(raw), "raw_sha256": _sha(raw),
                  "raw_rows_including_baseline": len(rows),
                  "capture_config": asdict(config),
                  "parser_baseline": baseline, "parser_counter_deltas": deltas,
                  "time_mapping": readiness["time_mapping"],
                  "clock_drift": {"path": str(drift), "sha256": _sha(drift),
                                  "error": None, "status": "observed"}}
        if kind == "background":
            reasons = [f"{name}={value}" for name, value in deltas.items()
                       if value and name not in {"parser_hub_packets", "parser_completed_groups"}]
            result.update(frames=len(rows), start_monotonic_ns=start,
                          end_monotonic_ns=start + 30_000_000_000, requested_duration_s=30.,
                          quality={"accepted": False, "reasons": reasons})
        else:
            robot = root / "execution_telemetry.csv"
            robot_rows = [{"sync_time_ns": str(start + n * 5_000_000),
                           "trajectory_time_s": str(n * .005), "trajectory_time_ns": str(n * 5_000_000),
                           "ros_time_ns": str(1_000_000_000_000 + n * 5_000_000)} for n in range(1, 6001)]
            _write_csv(robot, ["sync_time_ns", "trajectory_time_s", "trajectory_time_ns", "ros_time_ns"], robot_rows)
            temporary = root / "nucleo_capture__test.staging.csv"
            temporary.write_bytes(raw.read_bytes())
            finalized = finalize_nucleo_capture(NucleoCaptureSummary(
                temporary, config, workspace, frames_written=len(rows),
                parser_stats=ParserStats(completed_groups=len(rows),
                    incomplete_groups=losses if missing_channels < 4 else 0,
                    source_sequence_drops=losses if missing_channels == 4 else 0,
                    channel_sequence_drops=losses * missing_channels)), robot)
            assert finalized.status == "incomplete"
            result.update(frames=len(rows) - 1,
                          robot_telemetry=str(robot), robot_telemetry_sha256=_sha(robot),
                          session_sensor_guard_error=None,
                          quality={"accepted": False, "reasons": [finalized.message]},
                          mapping={"status": finalized.status, "message": finalized.message,
                                   "path": str(finalized.sensor_telemetry_path),
                                   "manifest": str(finalized.manifest_path)})
        return result, readiness
    return create


@pytest.mark.parametrize("kind", ["trajectory", "background"])
def test_bounded_policy_preserves_strict_evidence(sample, kind):
    result, ready = sample(kind=kind)
    original = copy.deepcopy(result)
    raw = Path(result["raw_csv"]).read_bytes()
    strict_manifest = Path(result["mapping"]["manifest"]).read_bytes() if kind == "trajectory" else None
    answer = evaluate_window(result, BOUNDED_POLICY, readiness=ready)
    assert answer["status"] == "accepted_with_recorded_group_loss", answer
    assert answer["accepted"] is True
    assert answer["missing_groups"] == 1
    assert result == original and Path(result["raw_csv"]).read_bytes() == raw
    if strict_manifest:
        assert Path(result["mapping"]["manifest"]).read_bytes() == strict_manifest
    assert answer["absolute_clock_synchronization_calibrated"] is False
    assert answer["position_bias_applied"] is False


def test_default_policy_remains_strict(sample):
    result, ready = sample()
    answer = evaluate_window(result, readiness=ready)
    assert answer["policy_id"] == STRICT_POLICY
    assert answer["accepted"] is False


@pytest.mark.parametrize("kind", ["trajectory", "background"])
@pytest.mark.parametrize("missing_channels", [2, 3, 4])
def test_partial_and_whole_lost_groups_not_counted_as_channel_groups(sample, kind, missing_channels):
    result, ready = sample(kind=kind, missing_channels=missing_channels)
    answer = evaluate_window(result, BOUNDED_POLICY, readiness=ready)
    assert answer["accepted"], answer
    assert answer["missing_groups"] == 1
    assert answer["parser_counter_deltas"]["parser_channel_sequence_drops"] == missing_channels


@pytest.mark.parametrize("kind", ["trajectory", "background"])
def test_timeout_is_a_subset_of_incomplete_not_an_extra_loss(sample, kind):
    result, ready = sample(kind=kind, timed_out=True)
    answer = evaluate_window(result, BOUNDED_POLICY, readiness=ready)
    assert answer["accepted"], answer
    assert answer["missing_groups"] == 1


def test_unknown_policy_never_accepts():
    assert evaluate_window({"status": "complete", "quality": {"accepted": True}}, "unknown")["accepted"] is False


def test_strict_success_does_not_need_loss_exception():
    result = {"status": "complete", "quality": {"accepted": True}}
    assert evaluate_window(result)["status"] == "accepted_strict"


@pytest.mark.parametrize("kind", ["trajectory", "background"])
def test_ten_lost_groups_accepted_but_eleven_rejected(sample, kind):
    result, ready = sample(kind=kind, missing=tuple(range(100, 1100, 100)))
    answer = evaluate_window(result, BOUNDED_POLICY, readiness=ready)
    assert answer["accepted"], answer
    result, ready = sample(kind=kind, missing=tuple(range(100, 1200, 100)))
    assert not evaluate_window(result, BOUNDED_POLICY, readiness=ready)["accepted"]


@pytest.mark.parametrize("problem", ["guard", "drift", "epoch", "raw_hash", "readiness", "counter", "unknown_reason"])
def test_nonloss_failures_never_waived(sample, problem):
    result, ready = sample()
    if problem == "guard":
        result["session_sensor_guard_error"] = "stale or capture_error"
    elif problem == "drift":
        result["clock_drift"]["error"] = "source phase budget exceeded"
    elif problem == "epoch":
        result["epoch_id"] = "other_epoch"
    elif problem == "raw_hash":
        result["raw_sha256"] = "0" * 64
    elif problem == "readiness":
        ready["startup_baseline"]["ready"] = False
    elif problem == "counter":
        result["parser_counter_deltas"]["parser_malformed_packets"] = 1
    else:
        result["quality"]["reasons"].append("unhandled hard failure")
    assert not evaluate_window(result, BOUNDED_POLICY, readiness=ready)["accepted"]


@pytest.mark.parametrize("field", ["sync_offset_ns", "sync_time_ns", "robot_row_index", "sensor_00_x_ut",
                                   "trajectory_time_s", "trajectory_time_ns", "robot_ros_time_ns"])
def test_mapping_changes_rejected_even_with_updated_manifest_sha(sample, field):
    result, ready = sample()
    path = Path(result["mapping"]["path"])
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        columns, rows = reader.fieldnames, list(reader)
    rows[50][field] = str(float(rows[50][field]) + 100_000_000)
    _write_csv(path, columns, rows)
    manifest_path = Path(result["mapping"]["manifest"])
    manifest = json.loads(manifest_path.read_text())
    manifest["artifacts"]["sensor_telemetry_csv"]["sha256"] = _sha(path)
    manifest_path.write_text(json.dumps(manifest))
    assert not evaluate_window(result, BOUNDED_POLICY, readiness=ready)["accepted"]


@pytest.mark.parametrize("field", ["source_time_ms_u32", "channel_2_nucleo_time_ms_u32", "sensor_rx_sync_time_ns",
                                   "channel_1_seq", "src_seq", "parser_incomplete_groups"])
def test_regression_or_unexplained_loss_never_waived(sample, field):
    result, ready = sample(kind="background")
    path = Path(result["raw_csv"])
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    rows[150][field] = "0"
    _write_csv(path, list(RAW_CAPTURE_COLUMNS), rows)
    result["raw_sha256"] = _sha(path)
    assert not evaluate_window(result, BOUNDED_POLICY, readiness=ready)["accepted"]


def test_missing_dispatch_readiness_rejected(sample):
    result, _ = sample()
    assert not evaluate_window(result, BOUNDED_POLICY)["accepted"]


def test_background_coverage_not_waived(sample):
    result, ready = sample(kind="background")
    result["requested_duration_s"] = 60.
    assert not evaluate_window(result, BOUNDED_POLICY, readiness=ready)["accepted"]
