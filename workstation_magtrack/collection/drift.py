"""Receive-clock-relative diagnostics; never rewrite raw sensor timestamps.

NUCLEO and source clocks are not synchronised to the host by this estimator.
USB buffering and oscillator drift are deliberately not claimed separable.
Sustained source-to-receive or interchannel changes trigger the operating
budget. Common NUCLEO-to-receive clock-rate differences are diagnostic only:
the time mapping uses host receive time, never the NUCLEO timestamp. A new
hardware epoch is required after ANY device clock regression.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from statistics import median
from typing import Any, Iterable, Mapping


TIME_MAPPING = "host_monotonic_receive_time_not_device_time"
INTEGRITY_COUNTERS = (
    "duplicate_or_reordered_groups", "incomplete_groups", "timeout_groups",
    "duplicate_packets", "reordered_packets", "channel_sequence_drops",
    "source_sequence_drops", "bytes_discarded", "malformed_packets",
    "unexpected_channel_packets", "pending_evictions",
)


@dataclass(frozen=True)
class DriftConfig:
    warmup_s: float = 1.0
    bin_s: float = 1.0
    minimum_observation_s: float = 5.0
    phase_change_budget_ms: float = 10.0
    consecutive_bins: int = 3
    retained_bins: int = 120
    maximum_samples_per_bin: int = 512
    startup_stable_bins: int = 5
    startup_phase_range_ms: float = 2.0
    startup_timeout_s: float = 30.0
    startup_minimum_frames_per_bin: int = 95
    startup_minimum_coverage_s: float = .90
    startup_maximum_frame_gap_s: float = .5

    def __post_init__(self) -> None:
        for name in ("warmup_s", "bin_s", "minimum_observation_s", "phase_change_budget_ms",
                     "startup_phase_range_ms", "startup_timeout_s", "startup_minimum_coverage_s",
                     "startup_maximum_frame_gap_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("consecutive_bins", "retained_bins", "maximum_samples_per_bin",
                     "startup_stable_bins", "startup_minimum_frames_per_bin"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 2:
                raise ValueError(f"{name} must be an integer >= 2")


class _Clock:
    def __init__(self, config: DriftConfig, *, phase_budget_enforced: bool = True,
                 automatic_baseline: bool = True) -> None:
        self.config = config
        self.phase_budget_enforced = phase_budget_enforced
        self.automatic_baseline = automatic_baseline
        self.count = 0
        self.previous_u32: int | None = None
        self.first_host_ns: int | None = None
        self.last_host_ns: int | None = None
        self.elapsed_ms = 0
        self.wraps = 0
        self.bin_id: int | None = None
        self.samples: deque[tuple[float, float]] = deque(maxlen=config.maximum_samples_per_bin)
        self.bins: deque[tuple[float, float]] = deque(maxlen=config.retained_bins)
        self.baseline: float | None = None
        self.phase_ms: float | None = None
        self.maximum_phase_ms = 0.0
        self.over_budget_bins = 0
        self.error: str | None = None
        self.epoch_error = False
        self.phase_budget_crossing: str | None = None

    def add(self, value: int, host_ns: int) -> None:
        if self.epoch_error:
            return
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
            self.error = "device timestamp is not uint32 milliseconds"
            self.epoch_error = True
            return
        if self.last_host_ns is not None and host_ns < self.last_host_ns:
            self.error = "host monotonic receive clock regressed"
            self.epoch_error = True
            return
        if self.previous_u32 is None:
            self.first_host_ns = host_ns
        else:
            delta = (value - self.previous_u32) & 0xFFFFFFFF
            if delta >= 0x80000000:
                self.error = "device clock regressed: reset/reorder requires a new explicit epoch"
                self.epoch_error = True
                return
            self.elapsed_ms += delta
            self.wraps += int(value < self.previous_u32)
        self.previous_u32 = value
        self.last_host_ns = host_ns
        self.count += 1
        assert self.first_host_ns is not None
        elapsed_s = (host_ns - self.first_host_ns) / 1e9
        if elapsed_s < self.config.warmup_s:
            return
        bin_id = int((elapsed_s - self.config.warmup_s) / self.config.bin_s)
        if self.bin_id is not None and bin_id != self.bin_id:
            self._finish_bin()
        self.bin_id = bin_id
        self.samples.append((elapsed_s, self.elapsed_ms - elapsed_s * 1000.0))

    def _finish_bin(self) -> None:
        if not self.samples:
            return
        elapsed_s = median(sample[0] for sample in self.samples)
        phase = median(sample[1] for sample in self.samples)
        self.samples.clear()
        self.bins.append((elapsed_s, phase))
        if self.baseline is None:
            if not self.automatic_baseline:
                return
            self.baseline = phase
        self.phase_ms = phase - self.baseline
        self.maximum_phase_ms = max(self.maximum_phase_ms, abs(self.phase_ms))
        exceeded = abs(self.phase_ms) > self.config.phase_change_budget_ms
        self.over_budget_bins = self.over_budget_bins + 1 if exceeded else 0
        if (self.phase_budget_crossing is None and elapsed_s >= self.config.minimum_observation_s and
                self.over_budget_bins >= self.config.consecutive_bins):
            self.phase_budget_crossing = (f"sustained receive-relative clock phase change {self.phase_ms:.3f} ms "
                                          f"exceeds {self.config.phase_change_budget_ms:g} ms operating budget")
            if self.phase_budget_enforced and self.error is None:
                self.error = self.phase_budget_crossing

    def report(self) -> dict[str, Any]:
        duration = ((self.last_host_ns - self.first_host_ns) / 1e9
                    if self.last_host_ns is not None and self.first_host_ns is not None else 0.0)
        fit = len(self.bins) >= 2 and self.bins[-1][0] - self.bins[0][0] >= self.config.minimum_observation_s
        ppm = None
        if fit:
            mean_x = sum(row[0] for row in self.bins) / len(self.bins)
            mean_y = sum(row[1] for row in self.bins) / len(self.bins)
            denominator = sum((row[0] - mean_x) ** 2 for row in self.bins)
            ppm = 1000.0 * sum((x - mean_x) * (y - mean_y) for x, y in self.bins) / denominator
        status = "exceeded" if self.error else "observed" if fit else "insufficient_observation"
        return {"status": status, "samples": self.count, "observation_s": duration,
                "uint32_wraps": self.wraps, "completed_phase_bins": len(self.bins),
                "drift_ppm_receive_clock_relative": ppm,
                "phase_change_from_baseline_ms": self.phase_ms,
                "max_abs_phase_change_from_baseline_ms": self.maximum_phase_ms,
                "receive_phase_budget_enforced": self.phase_budget_enforced,
                "receive_phase_baseline_established": self.baseline is not None,
                "sustained_receive_phase_budget_crossed": self.phase_budget_crossing is not None,
                "receive_phase_budget_crossing": self.phase_budget_crossing,
                "consecutive_over_budget_bins": self.over_budget_bins, "error": self.error}


class DeviceClockDrift:
    """Bounded, O(channels) online diagnostics, one tracker per hardware boot."""

    def __init__(self, channels: Iterable[int] = (1, 2, 3, 4), *,
                 config: DriftConfig | None = None, require_stable_start: bool = False) -> None:
        self.config = config or DriftConfig()
        self.channels = tuple(channels)
        if not self.channels or len(set(self.channels)) != len(self.channels):
            raise ValueError("drift channels must be nonempty and unique")
        if not isinstance(require_stable_start, bool):
            raise ValueError("require_stable_start must be boolean")
        self.require_stable_start = require_stable_start
        self.clocks = {f"channel_{channel}_{kind}": _Clock(
                           self.config, phase_budget_enforced=kind == "source",
                           automatic_baseline=not require_stable_start)
                       for channel in self.channels for kind in ("nucleo", "source")}
        self.frames = 0
        self.unavailable_frames = 0
        self._relative_baseline: dict[str, float] = {}
        self._relative_baseline_samples: dict[str, deque[int]] = {}
        self._relative_baseline_bin: int | None = None
        self.relative_phase_ms: dict[str, float] = {}
        self.maximum_relative_phase_ms: dict[str, float] = {}
        self._relative_exceeded_since: dict[str, int] = {}
        self.error: str | None = None
        self._startup_ready = not require_stable_start
        self._startup_first_ns: int | None = None
        self._startup_previous_ns: int | None = None
        self._startup_previous_counters: dict[str, int] | None = None
        self._startup_current: dict[str, Any] | None = None
        self._startup_bins: deque[dict[str, Any]] = deque(maxlen=self.config.startup_stable_bins)
        self._startup_frozen: dict[str, Any] | None = None
        self._startup_rejection: str | None = None
        self._startup_failed = False

    def update(self, host_ns: int, values: Mapping[int, tuple[int, int]], *,
               parser_counters: Mapping[str, int] | None = None) -> None:
        self.frames += 1
        if any(channel not in values for channel in self.channels):
            self.unavailable_frames += 1
            if self.require_stable_start and not self._startup_ready:
                self._reject_startup("missing complete channel timestamps")
            return
        for channel in self.channels:
            for index, kind in enumerate(("nucleo", "source")):
                key = f"channel_{channel}_{kind}"
                clock = self.clocks[key]
                clock.add(values[channel][index], host_ns)
                if clock.error and self.error is None:
                    self.error = f"{key}: {clock.error}"
        if self.require_stable_start and not self._startup_ready:
            self._observe_startup(host_ns, parser_counters)
            if not self._startup_ready:
                return
        reference = self.channels[0]
        reference_clock = self.clocks[f"channel_{reference}_nucleo"]
        if reference_clock.first_host_ns is None:
            return
        elapsed_s = (host_ns - reference_clock.first_host_ns) / 1e9
        if elapsed_s < self.config.warmup_s:
            return
        relative_bin = int((elapsed_s - self.config.warmup_s) / self.config.bin_s)
        if self._relative_baseline_bin is None:
            self._relative_baseline_bin = relative_bin
        for channel in self.channels[1:]:
            for kind in ("nucleo", "source"):
                key = f"channel_{channel}_minus_{reference}_{kind}"
                delta = (self.clocks[f"channel_{channel}_{kind}"].elapsed_ms -
                         self.clocks[f"channel_{reference}_{kind}"].elapsed_ms)
                if relative_bin == self._relative_baseline_bin:
                    samples = self._relative_baseline_samples.setdefault(
                        key, deque(maxlen=self.config.maximum_samples_per_bin))
                    samples.append(delta)
                    continue
                if key not in self._relative_baseline:
                    self._relative_baseline[key] = median(self._relative_baseline_samples.pop(key))
                phase = delta - self._relative_baseline[key]
                self.relative_phase_ms[key] = phase
                self.maximum_relative_phase_ms[key] = max(self.maximum_relative_phase_ms.get(key, 0), abs(phase))
                if abs(phase) > self.config.phase_change_budget_ms:
                    since = self._relative_exceeded_since.setdefault(key, host_ns)
                    reference_clock = self.clocks[f"channel_{reference}_{kind}"]
                    start = reference_clock.first_host_ns
                    if (self.error is None and start is not None and
                            host_ns - start >= self.config.minimum_observation_s * 1e9 and
                            host_ns - since >= self.config.consecutive_bins * self.config.bin_s * 1e9):
                        self.error = (f"{key}: sustained interchannel relative phase change {phase} ms "
                                      f"exceeds {self.config.phase_change_budget_ms:g} ms operating budget")
                else:
                    self._relative_exceeded_since.pop(key, None)

    def _reject_startup(self, reason: str) -> None:
        self._startup_rejection = reason
        self._startup_bins.clear()
        if self._startup_current is not None:
            self._startup_current["valid"] = False

    def _observe_startup(self, host_ns: int, counters: Mapping[str, int] | None) -> None:
        if self._startup_first_ns is None:
            self._startup_first_ns = host_ns
        elapsed_s = (host_ns - self._startup_first_ns) / 1e9
        if self.error:
            self._startup_failed = True
            return
        if elapsed_s >= self.config.startup_timeout_s:
            self.error = "stable startup baseline not observed within configured timeout"
            self._startup_failed = True
            return
        reason = None
        if counters is None or any(
                key not in counters or isinstance(counters[key], bool) or
                not isinstance(counters[key], int) or counters[key] < 0 for key in INTEGRITY_COUNTERS):
            reason = "missing or invalid parser integrity counters"
            self._startup_previous_counters = None
        else:
            current_counters = {key: counters[key] for key in INTEGRITY_COUNTERS}
            if self._startup_previous_counters is not None:
                deltas = {key: current_counters[key] - self._startup_previous_counters[key]
                          for key in INTEGRITY_COUNTERS}
                if any(value < 0 for value in deltas.values()):
                    self.error = "parser integrity counter epoch regressed during startup"
                    self._startup_failed = True
                    return
                if any(deltas.values()):
                    reason = "parser integrity counters incremented"
            self._startup_previous_counters = current_counters
        if (self._startup_previous_ns is not None and
                host_ns - self._startup_previous_ns > self.config.startup_maximum_frame_gap_s * 1e9):
            reason = "complete-frame receive gap exceeds startup budget"
        self._startup_previous_ns = host_ns
        if reason:
            self._reject_startup(reason)
        if elapsed_s < self.config.warmup_s:
            return
        bin_id = int((elapsed_s - self.config.warmup_s) / self.config.bin_s)
        previous = self._startup_current
        if previous is not None and bin_id != previous["bin_id"]:
            if bin_id != previous["bin_id"] + 1:
                self._reject_startup("startup bins are not contiguous")
            self._finish_startup_bin(host_ns)
            self._startup_current = None
        if self._startup_ready:
            return
        if self._startup_current is None:
            self._startup_current = {"bin_id": bin_id, "first_ns": host_ns,
                                     "last_ns": host_ns, "frames": 0,
                                     "valid": reason is None,
                                     "phases": {key: deque(maxlen=self.config.maximum_samples_per_bin)
                                                for key in self.clocks},
                                     "relative": {}}
        current = self._startup_current
        current["last_ns"] = host_ns
        current["frames"] += 1
        for key, clock in self.clocks.items():
            assert clock.first_host_ns is not None
            current["phases"][key].append(clock.elapsed_ms - (host_ns - clock.first_host_ns) / 1e6)
        reference = self.channels[0]
        for channel in self.channels[1:]:
            for kind in ("nucleo", "source"):
                key = f"channel_{channel}_minus_{reference}_{kind}"
                samples = current["relative"].setdefault(
                    key, deque(maxlen=self.config.maximum_samples_per_bin))
                samples.append(self.clocks[f"channel_{channel}_{kind}"].elapsed_ms -
                               self.clocks[f"channel_{reference}_{kind}"].elapsed_ms)

    def _finish_startup_bin(self, ready_host_ns: int) -> None:
        current = self._startup_current
        assert current is not None
        coverage = (current["last_ns"] - current["first_ns"]) / 1e9
        if (not current["valid"] or current["frames"] < self.config.startup_minimum_frames_per_bin or
                coverage < self.config.startup_minimum_coverage_s):
            self._reject_startup("startup bin lacks clean 100Hz frame count or time coverage")
            return
        complete = {key: current[key] for key in ("bin_id", "first_ns", "last_ns", "frames")}
        complete["coverage_s"] = coverage
        complete["phase_medians_ms"] = {key: median(values) for key, values in current["phases"].items()}
        complete["interchannel_medians_ms"] = {key: median(values) for key, values in current["relative"].items()}
        self._startup_bins.append(complete)
        if len(self._startup_bins) < self.config.startup_stable_bins:
            return
        for channel in self.channels:
            key = f"channel_{channel}_source"
            values = [row["phase_medians_ms"][key] for row in self._startup_bins]
            if max(values) - min(values) > self.config.startup_phase_range_ms:
                self._startup_rejection = "source phase medians have not stabilized"
                return
        clock_baselines = {key: median(row["phase_medians_ms"][key] for row in self._startup_bins)
                           for key in self.clocks}
        relative_baselines = {key: median(row["interchannel_medians_ms"][key] for row in self._startup_bins)
                              for key in complete["interchannel_medians_ms"]}
        for key, clock in self.clocks.items():
            clock.baseline = clock_baselines[key]
            clock.phase_ms = complete["phase_medians_ms"][key] - clock.baseline
            clock.maximum_phase_ms = abs(clock.phase_ms)
        self._relative_baseline = relative_baselines
        self._relative_baseline_bin = -1
        self._startup_ready = True
        self._startup_frozen = {
            "ready_host_monotonic_ns": ready_host_ns,
            "interval_first_sample_monotonic_ns": self._startup_bins[0]["first_ns"],
            "interval_last_sample_monotonic_ns": self._startup_bins[-1]["last_ns"],
            "clock_phase_baselines_ms": clock_baselines,
            "interchannel_phase_baselines_ms": dict(relative_baselines),
            "accepted_bins": list(self._startup_bins),
        }

    def _startup_report(self) -> dict[str, Any]:
        return {"required": self.require_stable_start, "ready": self._startup_ready,
                "status": "not_required" if not self.require_stable_start else "failed" if self._startup_failed
                          else "ready" if self._startup_ready else "warming_up",
                "criteria": {"consecutive_complete_bins": self.config.startup_stable_bins,
                             "bin_s": self.config.bin_s, "warmup_s": self.config.warmup_s,
                             "source_phase_median_range_max_ms": self.config.startup_phase_range_ms,
                             "minimum_frames_per_bin": self.config.startup_minimum_frames_per_bin,
                             "minimum_bin_coverage_s": self.config.startup_minimum_coverage_s,
                             "maximum_complete_frame_gap_s": self.config.startup_maximum_frame_gap_s,
                             "timeout_s": self.config.startup_timeout_s,
                             "zero_increments_required_for_counters": list(INTEGRITY_COUNTERS)},
                "candidate_complete_bins": len(self._startup_bins),
                "last_rejection": self._startup_rejection, "frozen": deepcopy(self._startup_frozen),
                "rebase_permitted": False,
                "error": self.error if self._startup_failed else None}

    def update_frame(self, frame: Any) -> None:
        self.update(int(frame.host_monotonic_ns), {
            int(record.channel_id): (int(record.nucleo_time_ms_u32), int(record.source_time_ms_u32))
            for record in frame.records
        }, parser_counters={key: getattr(frame.stats, key, None) for key in INTEGRITY_COUNTERS}
           if hasattr(frame, "stats") else None)

    def update_row(self, row: Mapping[str, str]) -> None:
        try:
            values = {channel: (int(row[f"channel_{channel}_nucleo_time_ms_u32"]),
                                int(row[f"channel_{channel}_source_time_ms_u32"]))
                      for channel in self.channels}
            host_ns = int(row["sensor_rx_sync_time_ns"])
        except (KeyError, TypeError, ValueError):
            self.frames += 1
            self.unavailable_frames += 1
            if self.require_stable_start and not self._startup_ready:
                self._reject_startup("missing or invalid raw-row channel timestamps")
            return
        try:
            counters = {key: int(row[f"parser_{key}"]) for key in INTEGRITY_COUNTERS}
        except (KeyError, TypeError, ValueError):
            counters = None
        self.update(host_ns, values, parser_counters=counters)

    def report(self) -> dict[str, Any]:
        clocks = {key: value.report() for key, value in self.clocks.items()}
        available = any(row["samples"] for row in clocks.values())
        observed = available and all(row["status"] == "observed" for row in clocks.values())
        return {"schema": "receive_relative_device_clock_drift_v1", "config": asdict(self.config),
                "status": "exceeded" if self.error else "observed" if observed else
                          "insufficient_observation" if available else "unavailable",
                "error": self.error, "frames": self.frames, "unavailable_frames": self.unavailable_frames,
                "startup_baseline": self._startup_report(),
                "time_mapping": TIME_MAPPING, "timestamps_modified": False,
                "sensor_values_modified": False, "absolute_latency_calibrated": False,
                "absolute_clock_synchronization_calibrated": False,
                "interpretation": "device/source elapsed minus host receive elapsed; includes USB buffering jitter",
                "guard_policy": {
                    "policy_id": "host_receive_mapping_source_and_interchannel_guard_v1",
                    "source_to_host_receive_phase": "sustained_budget_enforced",
                    "nucleo_to_host_receive_phase": "diagnostic_only_not_used_for_time_mapping",
                    "interchannel_relative_phase": "sustained_budget_enforced_both_clock_kinds",
                    "clock_regression_or_reset": "hard_failure_both_clock_kinds",
                    "uint32_forward_wrap": "unwrap_without_timestamp_rewrite",
                },
                "nucleo_rate_offset_diagnostic": {
                    "status": "reported_not_a_host_receive_mapping_failure",
                    "channels": {key: {
                        "drift_ppm_receive_clock_relative": row["drift_ppm_receive_clock_relative"],
                        "phase_change_from_baseline_ms": row["phase_change_from_baseline_ms"],
                        "sustained_receive_phase_budget_crossed": row["sustained_receive_phase_budget_crossed"],
                    } for key, row in clocks.items() if key.endswith("_nucleo")},
                },
                "clock_resolution_ms": 1, "partial_final_bin_used": False,
                "interchannel_budget_sustained_s": self.config.consecutive_bins * self.config.bin_s,
                "channels": clocks, "interchannel_relative_phase_change_ms": dict(self.relative_phase_ms),
                "max_abs_interchannel_relative_phase_change_ms": dict(self.maximum_relative_phase_ms)}
