"""Independent, opt-in RAW acquisition acceptance; never rewrite strict evidence.

Only a prospective release may select the bounded-loss policy.  It does not
calibrate position, device clocks, or absolute USB latency, fill missing groups,
or change the existing drift, liveness, and robot/sensor mapping gates.
"""
from __future__ import annotations

from bisect import bisect_left
import csv
import hashlib
import io
import json
import math
from pathlib import Path
from typing import Any, Mapping

STRICT_POLICY = "strict_v1"
BOUNDED_POLICY = "bounded_group_loss_10_per_3000_v1"
POLICY_IDS = (STRICT_POLICY, BOUNDED_POLICY)
_LOSSES = {"parser_incomplete_groups", "parser_timeout_groups",
           "parser_channel_sequence_drops", "parser_source_sequence_drops"}
_LABELS = {"parser_incomplete_groups": "incomplete groups",
           "parser_channel_sequence_drops": "channel sequence drops",
           "parser_source_sequence_drops": "source sequence drops"}
_TIME_MAPPING = "host_monotonic_receive_time_not_device_time"
_COUNTERS = {
    "parser_hub_packets", "parser_completed_groups", "parser_duplicate_or_reordered_groups",
    "parser_incomplete_groups", "parser_timeout_groups", "parser_duplicate_packets",
    "parser_reordered_packets", "parser_channel_sequence_drops", "parser_source_sequence_drops",
    "parser_bytes_discarded", "parser_malformed_packets", "parser_unexpected_channel_packets",
    "parser_pending_evictions",
}


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"invalid integer: {label}")
    text = str(value)
    number = int(text)
    if str(number) != text or number < 0:
        raise ValueError(f"invalid nonnegative integer: {label}")
    return number


def _bytes(path: str | Path, digest: str | None = None) -> bytes:
    source = Path(path)
    if source.is_symlink() or not source.is_file():
        raise ValueError(f"evidence is not a regular non-symlink file: {source}")
    content = source.read_bytes()
    if digest is not None and hashlib.sha256(content).hexdigest() != digest:
        raise ValueError(f"evidence SHA256 mismatch: {source}")
    return content


def _csv(content: bytes) -> list[dict[str, str]]:
    reader = csv.DictReader(io.StringIO(content.decode("utf-8")))
    columns = reader.fieldnames or []
    if not columns or len(columns) != len(set(columns)):
        raise ValueError("missing or duplicate CSV columns")
    rows = list(reader)
    if not rows or any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError("empty or malformed CSV evidence")
    return rows


def _clock(report: Mapping[str, Any], *, ready: bool) -> None:
    if (report.get("error") or report.get("status") not in {"observed", "insufficient_observation"}
            or report.get("unavailable_frames") != 0
            or report.get("time_mapping") != _TIME_MAPPING
            or report.get("timestamps_modified") is not False
            or report.get("sensor_values_modified") is not False):
        raise ValueError("missing or failed clock/epoch diagnostics")
    if ready:
        baseline = report.get("startup_baseline", {})
        if (baseline.get("required") is not True or baseline.get("ready") is not True
                or baseline.get("status") != "ready"):
            raise ValueError("stable-start readiness not established")


def _raw(result: Mapping[str, Any]) -> tuple[list[dict[str, str]], dict[str, int], int]:
    rows = _csv(_bytes(result["raw_csv"], result["raw_sha256"]))
    counters = result["parser_counter_deltas"]
    baseline = result["parser_baseline"]
    names = {name for name in rows[0] if name.startswith("parser_")
             and name != "parser_host_monotonic_ns"}
    if names != _COUNTERS or set(counters) != names or set(baseline) != names:
        raise ValueError("parser counter columns/baseline incomplete")
    required = {"sensor_rx_sync_time_ns", "sensor_rx_ros_time_ns", "parser_host_monotonic_ns",
                "logical_seq", "src_seq", "source_time_ms_u32", "channel_ids"}
    required.update(f"sensor_{sensor:02d}_{axis}_ut" for sensor in range(64) for axis in "xyz")
    required.update(f"channel_{channel}_{suffix}" for channel in range(1, 5)
                    for suffix in ("seq", "nucleo_time_ms_u32", "source_time_ms_u32"))
    if not required.issubset(rows[0]):
        raise ValueError("raw authority columns incomplete")
    deltas = {name: _integer(counters[name], name) for name in names}
    for name, count in deltas.items():
        if count and name not in _LOSSES | {"parser_hub_packets", "parser_completed_groups"}:
            raise ValueError(f"non-loss parser integrity failure: {name}")
        values = [_integer(row[name], name) for row in rows]
        if any(b < a for a, b in zip(values, values[1:])):
            raise ValueError(f"parser counter regressed: {name}")
        if values[-1] - _integer(baseline[name], name) != count:
            raise ValueError(f"parser delta does not match evidence: {name}")
        # Without a raw pre-window row we cannot justify first-sample losses.
        if values[0] != _integer(baseline[name], name) and name not in {
                "parser_hub_packets", "parser_completed_groups"}:
            raise ValueError("unprovable first-sample loss without raw baseline")
    channels = result.get("capture_config", {}).get("channels")
    if channels != [1, 2, 3, 4] and channels != (1, 2, 3, 4):
        raise ValueError("bounded policy requires the complete four-channel board")
    missing = 0
    previous: dict[str, str] | None = None
    for row in rows:
        if row.get("channel_ids") != "1;2;3;4":
            raise ValueError("raw row is missing complete channels")
        src = _integer(row["src_seq"], "src_seq")
        if src >= 16:
            raise ValueError("source sequence outside mod16")
        for name in ["source_time_ms_u32", *[
                f"channel_{channel}_{kind}_time_ms_u32"
                for channel in channels for kind in ("nucleo", "source")]]:
            if _integer(row[name], name) > 0xFFFFFFFF:
                raise ValueError("device clock outside uint32")
            if previous is not None and ((_integer(row[name], name) - _integer(previous[name], name))
                                         & 0xFFFFFFFF) >= 0x80000000:
                raise ValueError("device clock regressed or hardware epoch changed")
        if previous is not None:
            for name in ("sensor_rx_sync_time_ns", "parser_host_monotonic_ns"):
                if _integer(row[name], name) < _integer(previous[name], name):
                    raise ValueError("host receive clock regressed")
            step = (src - _integer(previous["src_seq"], "src_seq")) % 16
            if not 1 <= step < 8:
                raise ValueError("ambiguous, duplicate or reordered source groups")
            if _integer(row["logical_seq"], "logical_seq") != _integer(previous["logical_seq"], "logical_seq") + 1:
                raise ValueError("complete-frame sequence changed unexpectedly")
            for channel in channels:
                name = f"channel_{channel}_seq"
                if ((_integer(row[name], name) - _integer(previous[name], name)) & 0xFFFFFFFF) != step:
                    raise ValueError("channel/source sequence gap mismatch")
            missing += step - 1
        previous = row
    incomplete = deltas["parser_incomplete_groups"]
    source = deltas["parser_source_sequence_drops"]
    drops = deltas["parser_channel_sequence_drops"]
    if (missing != incomplete + source or deltas["parser_timeout_groups"] > incomplete
            or not incomplete + 4 * source <= drops <= 3 * incomplete + 4 * source):
        raise ValueError("parser losses are not explained by missing source groups")
    return rows, deltas, missing


def _trajectory(result: Mapping[str, Any], rows: list[dict[str, str]],
                deltas: Mapping[str, int]) -> tuple[int, float]:
    mapping = result["mapping"]
    manifest = json.loads(_bytes(mapping["manifest"]))
    expected = "; ".join(f"parser observed {deltas[name]} {label} during playback"
                         for name, label in _LABELS.items() if deltas[name])
    if (mapping.get("status") != "incomplete" or manifest.get("status") != "incomplete"
            or not expected or mapping.get("message") != expected
            or manifest.get("message") != expected
            or result["quality"]["reasons"] != [expected]
            or manifest["capture"].get("capture_error")):
        raise ValueError("strict finalizer has a failure other than bounded parser loss")
    for name, count in manifest["mapping"]["parser_integrity_deltas"].items():
        if count != deltas.get(name):
            raise ValueError("finalizer/parser delta mismatch")
    artifacts = manifest["artifacts"]
    robot_bytes = _bytes(result["robot_telemetry"], result["robot_telemetry_sha256"])
    if hashlib.sha256(robot_bytes).hexdigest() != artifacts["robot_telemetry_csv"]["sha256"]:
        raise ValueError("finalizer robot evidence differs")
    robot = _csv(robot_bytes)
    times = [_integer(row["sync_time_ns"], "robot sync_time_ns") for row in robot]
    if len(times) < 2 or any(b < a for a, b in zip(times, times[1:])) or times[-1] <= times[0]:
        raise ValueError("robot time window is empty or regressed")
    sensor_artifact = artifacts["sensor_telemetry_csv"]
    if Path(sensor_artifact["path"]).resolve() != Path(mapping["path"]).resolve():
        raise ValueError("mapped artifact path mismatch")
    mapped = _csv(_bytes(mapping["path"], sensor_artifact["sha256"]))
    inside = [row for row in rows if times[0] <= int(row["sensor_rx_sync_time_ns"]) <= times[-1]]
    if (len(inside) != len(mapped) or len(inside) != result["frames"]
            or manifest["counts"]["raw_sensor_frames"] != len(rows)
            or manifest["counts"]["frames_reported_by_capture"] != len(rows)
            or manifest["counts"]["mapped_sensor_frames"] != len(inside)
            or manifest["counts"]["robot_samples"] != len(robot)):
        raise ValueError("raw/robot/mapped count mismatch")
    config = result["capture_config"]
    maximum_offset = float(config["max_mapping_offset_s"]) * 1e9
    liveness = float(config["liveness_timeout_s"]) * 1e9
    if not 0 < maximum_offset <= 50_000_000 or not 0 < liveness <= 500_000_000:
        raise ValueError("original mapping/liveness thresholds were loosened")
    for raw, selected in zip(inside, mapped, strict=True):
        stamp = int(raw["sensor_rx_sync_time_ns"])
        index = bisect_left(times, stamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
        nearest = min(candidates, key=lambda i: (abs(times[i] - stamp), i))
        robot_columns = {"trajectory_time_s": "trajectory_time_s"}
        if "trajectory_time_ns" in robot[nearest]:
            robot_columns["trajectory_time_ns"] = "trajectory_time_ns"
        if "ros_time_ns" in robot[nearest]:
            robot_columns["robot_ros_time_ns"] = "ros_time_ns"
        if (any(selected.get(name) != value for name, value in raw.items())
                or any(selected.get(target) != robot[nearest][source]
                       for target, source in robot_columns.items())
                or selected["sync_time_ns"] != robot[nearest]["sync_time_ns"]
                or int(selected["robot_row_index"]) != nearest
                or int(selected["sync_offset_ns"]) != stamp - times[nearest]
                or abs(stamp - times[nearest]) > maximum_offset):
            raise ValueError("robot/sensor mapping missing, altered, or offset exceeded")
    stamps = [int(row["sensor_rx_sync_time_ns"]) for row in inside]
    if not stamps or max(stamps[0] - times[0], times[-1] - stamps[-1],
                          max((b - a for a, b in zip(stamps, stamps[1:])), default=0)) > liveness:
        raise ValueError("original sensor coverage/liveness gate failed")
    return len(inside), (times[-1] - times[0]) / 1e9


def _background(result: Mapping[str, Any], rows: list[dict[str, str]],
                deltas: Mapping[str, int]) -> tuple[int, float]:
    expected = {f"{name}={count}" for name, count in deltas.items() if count and name in _LOSSES}
    if set(result["quality"]["reasons"]) != expected or not expected:
        raise ValueError("background has a failure other than bounded parser loss")
    start, end = result["start_monotonic_ns"], result["end_monotonic_ns"]
    duration = float(result["requested_duration_s"])
    stamps = [_integer(row["sensor_rx_sync_time_ns"], "receive time") for row in rows]
    coverage = (stamps[-1] - stamps[0]) / 1e9
    if (not math.isfinite(duration) or duration <= 0 or end - start != round(duration * 1e9)
            or not all(start <= stamp < end for stamp in stamps)
            or len(rows) != result["frames"] or len(rows) < math.ceil(duration * 99.5)
            or coverage < max(0.0, duration - .1)
            or max((b - a for a, b in zip(stamps, stamps[1:])), default=0) > 500_000_000):
        raise ValueError("original background count/coverage/liveness gate failed")
    return len(rows), duration


def evaluate_window(result: Mapping[str, Any], policy_id: str = STRICT_POLICY, *,
                    readiness: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return separate operational acceptance, leaving ``result`` untouched.

    Readiness is the actual ``check_ready()`` result frozen by the caller before
    dispatch. A prospective release binds the selected policy; this function
    does not rewrite, relabel, or recertify any historical run.
    """
    strict = result.get("status") == "complete" and result.get("quality", {}).get("accepted") is True
    output: dict[str, Any] = {"schema": "raw_operational_acceptance_v1", "policy_id": policy_id,
        "accepted": strict, "status": "accepted_strict" if strict else "rejected",
        "strict_status": result.get("status"), "strict_accepted": strict,
        "strict_evidence_modified": False, "raw_values_modified": False,
        "timestamps_modified": False, "absolute_latency_calibrated": False,
        "absolute_clock_synchronization_calibrated": False,
        "position_bias_applied": False, "missing_groups_filled": False,
        "time_mapping": _TIME_MAPPING, "reasons": []}
    if policy_id not in POLICY_IDS:
        output.update(accepted=False, status="rejected", reasons=["unknown quality policy"])
        return output
    if strict or policy_id == STRICT_POLICY:
        if not strict:
            output["reasons"] = ["strict sensor quality did not pass"]
        return output
    try:
        if (result.get("status") != "incomplete" or result.get("session_sensor_guard_error")
                or result.get("time_mapping") != _TIME_MAPPING):
            raise ValueError("sensor guard, epoch, or strict status failed")
        if readiness is None:
            raise ValueError("dispatch stable-start readiness evidence missing")
        _clock(readiness, ready=True)
        drift_evidence = result["clock_drift"]
        if drift_evidence.get("error"):
            raise ValueError("window drift guard failed")
        drift = json.loads(_bytes(drift_evidence["path"], drift_evidence["sha256"]))
        _clock(drift, ready=False)
        for name in ("session_id", "epoch_id", "host_boot_id", "hardware_boot_id"):
            if not result.get(name) or drift.get(name) != result[name]:
                raise ValueError(f"missing or mismatched hardware/host epoch: {name}")
        rows, deltas, missing = _raw(result)
        if result.get("kind") == "trajectory":
            frames, duration = _trajectory(result, rows, deltas)
        elif result.get("kind") == "background":
            frames, duration = _background(result, rows, deltas)
        else:
            raise ValueError("unsupported window kind")
        if deltas["parser_completed_groups"] != frames:
            raise ValueError("completed-group counter differs from window row count")
        if missing <= 0 or missing * 300 > frames + missing:
            raise ValueError("missing groups exceed 10 per 3000 expected groups")
        output.update(accepted=True, status="accepted_with_recorded_group_loss",
                      missing_groups=missing, complete_groups=frames,
                      expected_groups=frames + missing, duration_s=duration,
                      loss_fraction=missing / (frames + missing), parser_counter_deltas=dict(deltas),
                      raw_sha256=result["raw_sha256"],
                      acceptance_scope="raw_integrity_only_not_physical_calibration_or_stationarity")
    except (KeyError, TypeError, ValueError, OSError, OverflowError, AttributeError, csv.Error) as exc:
        output.update(accepted=False, status="rejected", reasons=[str(exc)])
    return output
