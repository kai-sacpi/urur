"""CPU-only clock diagnostics: no serial, ROS, robot, rewriting or resampling."""
from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from workstation_magtrack.collection.drift import DeviceClockDrift, DriftConfig, INTEGRITY_COUNTERS


def feed(tracker: DeviceClockDrift, duration_s: float, *, ppm: float = 0,
         offset_ms: int = 123456, channel_4_ppm: float = 0,
         jitter: bool = False) -> None:
    for index in range(round(duration_s * 100) + 1):
        elapsed_ms = index * 10
        # Smooth positive delay noise; monotonic 100Hz arrivals are maintained.
        delay_ns = round((math.sin(index * .3) + 1) * 2_000_000) if jitter else 0
        values = {}
        for channel in range(1, 5):
            rate = ppm + (channel_4_ppm if channel == 4 else 0)
            nucleo = (offset_ms + round(elapsed_ms * (1 + rate / 1e6))) & 0xFFFFFFFF
            # Independent absolute source epoch, same rate.  Never compare
            # source absolute values against NUCLEO or another MCU's epoch.
            source = (nucleo + channel * 1_000_000) & 0xFFFFFFFF
            values[channel] = nucleo, source
        tracker.update(10_000_000_000 + index * 10_000_000 + delay_ns, values)


def test_nominal_and_independent_absolute_epochs() -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 15)
    report = tracker.report()
    assert report["status"] == "observed"
    assert report["error"] is None
    assert report["time_mapping"] == "host_monotonic_receive_time_not_device_time"
    assert not report["timestamps_modified"]
    assert not report["sensor_values_modified"]
    assert not report["absolute_clock_synchronization_calibrated"]
    assert all(abs(value["drift_ppm_receive_clock_relative"]) < .0001
               for value in report["channels"].values())
    assert set(report["interchannel_relative_phase_change_ms"].values()) == {0}


def test_linear_drift_ppm_and_accumulated_phase() -> None:
    tracker = DeviceClockDrift(config=DriftConfig(phase_change_budget_ms=100))
    feed(tracker, 50, ppm=500)
    report = tracker.report()
    clock = report["channels"]["channel_1_source"]
    assert report["status"] == "observed"
    assert clock["drift_ppm_receive_clock_relative"] == pytest.approx(500, abs=5)
    assert clock["phase_change_from_baseline_ms"] == pytest.approx(24, abs=1)


def test_measured_common_nucleo_400ppm_is_reported_not_mapping_failure() -> None:
    tracker = DeviceClockDrift()
    for index in range(6001):
        elapsed_ms = index * 10
        nucleo = 100000 + round(elapsed_ms * 1.0004)
        values = {channel: (nucleo, 900000 * channel + elapsed_ms) for channel in range(1, 5)}
        tracker.update(10_000_000_000 + index * 10_000_000, values)
    report = tracker.report()
    assert report["status"] == "observed"
    assert report["error"] is None
    assert report["time_mapping"] == "host_monotonic_receive_time_not_device_time"
    assert report["guard_policy"]["nucleo_to_host_receive_phase"] == "diagnostic_only_not_used_for_time_mapping"
    for channel in range(1, 5):
        nucleo = report["channels"][f"channel_{channel}_nucleo"]
        source = report["channels"][f"channel_{channel}_source"]
        assert nucleo["drift_ppm_receive_clock_relative"] == pytest.approx(400, abs=5)
        assert nucleo["phase_change_from_baseline_ms"] > 10
        assert nucleo["receive_phase_budget_enforced"] is False
        assert nucleo["sustained_receive_phase_budget_crossed"] is True
        assert nucleo["receive_phase_budget_crossing"]
        assert nucleo["error"] is None
        assert source["receive_phase_budget_enforced"] is True
        assert source["phase_change_from_baseline_ms"] == 0
    assert not report["timestamps_modified"]
    assert not report["sensor_values_modified"]


def test_common_source_accumulated_phase_budget_still_fails_closed() -> None:
    tracker = DeviceClockDrift()
    for index in range(6001):
        elapsed_ms = index * 10
        # Source falls progressively behind receive time; no channel differs.
        source = round(elapsed_ms * .9996)
        values = {channel: (100000 + elapsed_ms, channel * 900000 + source) for channel in range(1, 5)}
        tracker.update(10_000_000_000 + index * 10_000_000, values)
    report = tracker.report()
    assert report["status"] == "exceeded"
    assert "source" in report["error"]
    assert "10 ms operating budget" in report["error"]
    assert report["channels"]["channel_1_source"]["drift_ppm_receive_clock_relative"] == pytest.approx(-400, abs=5)


@pytest.mark.parametrize("kind", [0, 1])
def test_nucleo_or_source_interchannel_divergence_still_fails(kind: int) -> None:
    tracker = DeviceClockDrift()
    for index in range(4001):
        elapsed_ms = index * 10
        values = {channel: [100000 + elapsed_ms, channel * 900000 + elapsed_ms] for channel in range(1, 5)}
        values[4][kind] += round(elapsed_ms * .001)
        tracker.update(10_000_000_000 + index * 10_000_000,
                       {channel: tuple(pair) for channel, pair in values.items()})
    report = tracker.report()
    assert report["status"] == "exceeded"
    assert "channel_4" in report["error"]
    if kind == 0:
        assert "interchannel" in report["error"]
        assert report["channels"]["channel_4_nucleo"]["error"] is None


def test_short_observation_never_fits_fake_precision() -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 4.9)
    report = tracker.report()
    assert report["status"] == "insufficient_observation"
    assert all(value["drift_ppm_receive_clock_relative"] is None
               for value in report["channels"].values())


def test_uint32_wrap_is_unwrapped_not_a_reset() -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 12, offset_ms=0xFFFFFFFF - 4000)
    report = tracker.report()
    assert report["status"] == "observed"
    assert report["channels"]["channel_1_nucleo"]["uint32_wraps"] == 1
    assert report["channels"]["channel_1_nucleo"]["phase_change_from_baseline_ms"] == 0


@pytest.mark.parametrize("kind", [0, 1])
def test_nucleo_or_source_reset_is_sticky_and_requires_new_epoch(kind: int) -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 1)
    values = {channel: [124466, 124466 + channel * 1_000_000] for channel in range(1, 5)}
    values[2][kind] = 100
    tracker.update(11_010_000_000, {channel: tuple(pair) for channel, pair in values.items()})
    assert tracker.report()["status"] == "exceeded"
    assert "regressed" in tracker.report()["error"]


def test_usb_jitter_is_not_claimed_separable_from_clock_drift() -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 30, jitter=True)
    report = tracker.report()
    assert report["status"] == "observed"
    assert "USB buffering" in report["interpretation"]
    assert report["absolute_latency_calibrated"] is False
    assert all(abs(row["phase_change_from_baseline_ms"]) < 1
               for row in report["channels"].values())


def test_sustained_phase_budget_refuses_next_dispatch_not_raw_stream() -> None:
    tracker = DeviceClockDrift()
    feed(tracker, 40, channel_4_ppm=1000)
    report = tracker.report()
    assert report["status"] == "exceeded"
    assert "channel_4" in report["error"]
    assert "10 ms operating budget" in report["error"]
    assert report["max_abs_interchannel_relative_phase_change_ms"]["channel_4_minus_1_source"] > 10
    assert report["frames"] == 4001


def test_missing_synthetic_timestamps_are_unavailable_not_zero_drift() -> None:
    tracker = DeviceClockDrift()
    tracker.update_row({"sensor_rx_sync_time_ns": "123"})
    report = tracker.report()
    assert report["status"] == "unavailable"
    assert report["unavailable_frames"] == 1
    assert report["error"] is None


def test_interchannel_drift_budget_not_hidden_by_opposite_individual_drift() -> None:
    tracker = DeviceClockDrift()
    for index in range(2501):
        elapsed_ms = index * 10
        values = {}
        for channel in range(1, 5):
            ppm = 250 if channel == 4 else -250
            device = 100000 + round(elapsed_ms * (1 + ppm / 1e6))
            values[channel] = device, device + channel * 900000
        tracker.update(10_000_000_000 + index * 10_000_000, values)
    report = tracker.report()
    assert report["status"] == "exceeded"
    assert "interchannel" in report["error"]
    assert all(abs(row["phase_change_from_baseline_ms"]) < 10 for row in report["channels"].values())


def test_initial_asynchronous_phase_is_removed_by_warmup_baseline() -> None:
    tracker = DeviceClockDrift()
    for index in range(1501):
        device = 100000 + index * 10
        # A first grouped sample from channel4 is older, but all subsequent
        # timestamps increase.  It is startup phase, not long-term drift.
        values = {channel: (device, device + channel * 100000)
                  for channel in range(1, 5)}
        if index == 0:
            values[4] = (device - 20, device + 400000 - 20)
        tracker.update(10_000_000_000 + index * 10_000_000, values)
    report = tracker.report()
    assert report["status"] == "observed"
    assert report["interchannel_relative_phase_change_ms"]["channel_4_minus_1_source"] == 0


def test_frame_adapter_does_not_mutate_records_or_input_times() -> None:
    records = tuple(SimpleNamespace(channel_id=channel, nucleo_time_ms_u32=4000,
                                    source_time_ms_u32=9000 + channel)
                    for channel in range(1, 5))
    frame = SimpleNamespace(host_monotonic_ns=123456789, records=records)
    before = [(record.nucleo_time_ms_u32, record.source_time_ms_u32) for record in records]
    tracker = DeviceClockDrift()
    tracker.update_frame(frame)
    assert frame.host_monotonic_ns == 123456789
    assert before == [(record.nucleo_time_ms_u32, record.source_time_ms_u32) for record in records]
    assert tracker.report()["frames"] == 1


def test_retained_diagnostics_are_bounded() -> None:
    tracker = DeviceClockDrift(config=DriftConfig(retained_bins=10, maximum_samples_per_bin=101))
    feed(tracker, 40)
    assert all(len(clock.bins) <= 10 and len(clock.samples) <= 101 for clock in tracker.clocks.values())


@pytest.mark.parametrize("config", [dict(phase_change_budget_ms=0), dict(bin_s=float("inf")),
                                    dict(consecutive_bins=1), dict(maximum_samples_per_bin=True)])
def test_invalid_configuration_rejected(config: dict) -> None:
    with pytest.raises(ValueError):
        DriftConfig(**config)


def startup_sample(tracker: DeviceClockDrift, index: int, *, delay_ms: float = 0,
                   source_offset_ms: float = 0, counters: dict | None = None) -> None:
    elapsed_ms = index * 10
    values = {channel: (100000 + round(elapsed_ms * 1.0004),
                        channel * 900000 + elapsed_ms + round(source_offset_ms))
              for channel in range(1, 5)}
    tracker.update(10_000_000_000 + round((elapsed_ms + delay_ms) * 1e6), values,
                   parser_counters=counters)


def clean_counters() -> dict[str, int]:
    return {key: 0 for key in INTEGRITY_COUNTERS}


def test_live_startup_waits_for_stable_bins_after_20ms_backlog_clears() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(1001):
        seconds = index / 100
        delay = 20 * seconds if seconds < 1 else 20 if seconds < 2 else max(0, (4 - seconds) * 10)
        startup_sample(tracker, index, delay_ms=delay, counters=clean_counters())
        if index <= 650:
            assert not tracker.report()["startup_baseline"]["ready"]
        assert tracker.error is None
    report = tracker.report()
    startup = report["startup_baseline"]
    assert startup["required"] is True
    assert startup["ready"] is True
    assert startup["status"] == "ready"
    assert startup["frozen"]["interval_first_sample_monotonic_ns"] >= 14_000_000_000
    assert len(startup["frozen"]["accepted_bins"]) == 5
    assert all(row["frames"] >= 95 and row["coverage_s"] >= .90
               for row in startup["frozen"]["accepted_bins"])
    assert abs(report["channels"]["channel_1_source"]["phase_change_from_baseline_ms"]) < 1


def test_live_startup_fixed_once_and_genuine_source_drift_still_fails() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(1001):
        startup_sample(tracker, index, counters=clean_counters())
    before = tracker.report()["startup_baseline"]["frozen"]
    assert before is not None
    assert tracker.error is None
    for index in range(1001, 3501):
        startup_sample(tracker, index, source_offset_ms=-(index - 1000) * .01,
                       counters=clean_counters())
    report = tracker.report()
    assert "source" in report["error"]
    assert "10 ms operating budget" in report["error"]
    assert report["startup_baseline"]["frozen"] == before
    # The report is a snapshot, not a mutable handle for rebasing internals.
    before["clock_phase_baselines_ms"].clear()
    assert tracker.report()["startup_baseline"]["frozen"]["clock_phase_baselines_ms"]


@pytest.mark.parametrize("kind", [0, 1])
def test_live_startup_clock_reset_fails_even_before_baseline(kind: int) -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    startup_sample(tracker, 0, counters=clean_counters())
    values = {channel: [100010, channel * 900000 + 10] for channel in range(1, 5)}
    values[2][kind] = 0
    tracker.update(10_010_000_000, {channel: tuple(v) for channel, v in values.items()},
                   parser_counters=clean_counters())
    report = tracker.report()
    assert "regressed" in report["error"]
    assert not report["startup_baseline"]["ready"]
    assert report["startup_baseline"]["status"] == "failed"


@pytest.mark.parametrize("mode", ["missing", "invalid", "losses", "low_rate", "gaps", "changing_phase"])
def test_live_startup_never_admits_bad_or_unstable_intervals(mode: str) -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(2001):
        if mode == "low_rate" and index % 2:
            continue
        if mode == "gaps" and index % 300 >= 235:
            continue
        counters = clean_counters()
        if mode == "missing":
            counters = None
        elif mode == "invalid":
            counters["incomplete_groups"] = -1
        elif mode == "losses":
            counters["incomplete_groups"] = index // 150
        delay_ms = index * .02 if mode == "changing_phase" else 0
        # 2ms/s persistent phase change never meets a 2ms/5-bin readiness gate.
        startup_sample(tracker, index, delay_ms=delay_ms, counters=counters)
        assert not tracker.report()["startup_baseline"]["ready"]
    assert tracker.error is None
    assert tracker.report()["startup_baseline"]["status"] == "warming_up"


def test_live_startup_timeout_is_bounded_and_diagnostic() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(3001):
        startup_sample(tracker, index, counters=None)
    report = tracker.report()
    assert report["startup_baseline"]["status"] == "failed"
    assert not report["startup_baseline"]["ready"]
    assert "timeout" in report["error"]


def test_live_update_row_requires_real_parser_counters_and_then_readies() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(1001):
        row = {"sensor_rx_sync_time_ns": str(10_000_000_000 + index * 10_000_000)}
        for channel in range(1, 5):
            row[f"channel_{channel}_nucleo_time_ms_u32"] = str(100000 + index * 10)
            row[f"channel_{channel}_source_time_ms_u32"] = str(channel * 900000 + index * 10)
        if index >= 200:
            row.update({f"parser_{key}": "0" for key in INTEGRITY_COUNTERS})
        tracker.update_row(row)
        if index < 600:
            assert not tracker.report()["startup_baseline"]["ready"]
    assert tracker.report()["startup_baseline"]["ready"]
    assert tracker.report()["startup_baseline"]["frozen"]["interval_first_sample_monotonic_ns"] >= 12_000_000_000


def test_live_update_frame_consumes_parser_stats() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    stats = SimpleNamespace(**clean_counters())
    for index in range(801):
        frame = SimpleNamespace(host_monotonic_ns=10_000_000_000 + index * 10_000_000,
                                stats=stats, records=tuple(SimpleNamespace(
                                    channel_id=channel, nucleo_time_ms_u32=100000 + index * 10,
                                    source_time_ms_u32=channel * 900000 + index * 10)
                                    for channel in range(1, 5)))
        tracker.update_frame(frame)
    assert tracker.report()["startup_baseline"]["ready"]


def test_missing_raw_timestamp_resets_startup_candidate_interval() -> None:
    tracker = DeviceClockDrift(require_stable_start=True)
    for index in range(1201):
        row = {"sensor_rx_sync_time_ns": str(10_000_000_000 + index * 10_000_000),
               **{f"parser_{key}": "0" for key in INTEGRITY_COUNTERS}}
        for channel in range(1, 5):
            row[f"channel_{channel}_nucleo_time_ms_u32"] = str(100000 + index * 10)
            row[f"channel_{channel}_source_time_ms_u32"] = str(channel * 900000 + index * 10)
        if index == 350:
            del row["channel_4_source_time_ms_u32"]
        tracker.update_row(row)
        if index < 900:
            assert not tracker.report()["startup_baseline"]["ready"]
    report = tracker.report()
    assert report["unavailable_frames"] == 1
    assert report["startup_baseline"]["ready"]
    assert report["startup_baseline"]["frozen"]["interval_first_sample_monotonic_ns"] >= 14_000_000_000
