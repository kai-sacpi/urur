"""CPU-only full background pipeline stress tests, without hardware or ROS.

These exercise actual Collector/Sensor/RAW-guard code with the existing fake
append-only sensor and deterministic feedback. They are not physical evidence.
"""
from __future__ import annotations

import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from . import motion as motion_module
from .motion import JointStateSnapshot, MotionController
from .run import Action, Collector
from .stationary_guard import RawStationaryMonitor
from .test_sensor import raw_rows, start_fake
from .test_stationary_guard import ControlledThread, REAL_THREAD


def pipeline(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Use production state checks, but deterministic clocks and no transport."""
    device, factory, clock = start_fake(tmp_path)
    monkeypatch.setattr(motion_module.time, "monotonic_ns", clock)
    monkeypatch.setattr(motion_module.time, "time_ns", clock)
    monkeypatch.setattr(motion_module.threading, "Thread", ControlledThread)
    motion = MotionController.__new__(MotionController)
    motion.mode = "simulation"
    motion.options = {"max_state_age_s": .5, "settle_velocity_rad_s": .01,
                      "endpoint_tolerance_rad": .001}
    motion.environment = SimpleNamespace(joint_names=tuple(f"joint_{i}" for i in range(6)))
    motion.park = (0.,)*6
    motion._last_state_source_ns = motion._last_state_sequence = 0
    motion.verify_runtime_identity = lambda: None
    motion.tip_fk = lambda q: {"plane1_position_m": list(q[:3]),
                               "plane1_quaternion_xyzw": [0., 0., 0., 1.]}
    state = SimpleNamespace(sequence=0, velocity=0., offset=0., active=None,
                            gap_pending=False, freeze_error=None)

    def fresh_state(timeout: float) -> JointStateSnapshot:
        state.sequence += 1
        return JointStateSnapshot(motion.environment.joint_names,
            (state.offset, 0., 0., 0., 0., 0.), (state.velocity, 0., 0., 0., 0., 0.),
            clock(), clock(), state.sequence)

    motion._backend = lambda: SimpleNamespace(fresh_state=fresh_state)

    def verify() -> None:
        assert state.active is not None and not state.active.thread.is_alive()
        if state.freeze_error is not None:
            raise state.freeze_error
        clock.sleep(2.)  # Slow full environment/bundle verification after stop.

    motion._unchanged = verify
    guards = []

    def monitor(path: Path, *, duration_s: float) -> RawStationaryMonitor:
        guard = RawStationaryMonitor(motion, path, duration_s)
        guards.append(guard)
        state.active = guard
        return guard

    motion.stationary_monitor = monitor

    def acquire_sleep(seconds: float) -> None:
        clock.sleep(.6 if state.gap_pending else seconds)
        state.gap_pending = False
        guard = state.active
        if guard is not None and not guard.stop.is_set() and guard.error is None:
            try:
                guard._record(motion._state(target=motion.park))
            except BaseException as error:
                guard.error = error  # Same transfer as inherited reader _loop.

    device._sleep = acquire_sleep
    collector = Collector.__new__(Collector)
    collector.args = SimpleNamespace(command="collect", output=device.run_root)
    collector.sensor, collector.motion = device, motion
    collector.last_park = {"passed": True, "synthetic_cpu_fixture": True}
    collector.operational_quality = lambda result, readiness, output: {
        "accepted": result["quality"]["accepted"], "synthetic_cpu_fixture": True}
    return collector, factory, clock, state, guards


def background(ordinal: int, *, duration_s: float = 30.) -> Action:
    return Action(f"background_{ordinal}", "capture_background", {
        "duration_s": duration_s, "background_ordinal": ordinal,
        "session_id": "synthetic_cpu_session"})


def test_three_consecutive_backgrounds_with_growing_raw_and_long_exports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    collector, factory, clock, state, guards = pipeline(tmp_path, monkeypatch)
    device = collector.sensor
    original_copy = device._copy_window
    snapshots, copied_rows, boundaries = [], [], []
    delays = iter((60., 120., 240.))

    def slow_growing_copy(snap, path, start, end, *, include_baseline):
        guard = state.active
        assert guard.stop.is_set() and not guard.thread.is_alive()
        assert guard.stream.closed
        assert guard._cutoff_ns == end
        snapshots.append(snap.byte_count)
        boundaries.append((start, end, guard._cutoff_ns))
        # This imitates a long GIL-owning export. No robot feedback runs here;
        # the acquisition is already finished and must not be aged or extended.
        clock.sleep(next(delays))
        result = original_copy(snap, path, start, end, include_baseline=include_baseline)
        copied_rows.append(len(raw_rows(path)))
        return result

    device._copy_window = slow_growing_copy
    clock.sleep(60.)  # Existing session history before the first background.
    terminals = []
    try:
        for ordinal in range(1, 4):
            result = collector.perform(background(ordinal))
            guard = guards[-1]
            assert result["accepted"] is True
            assert result["stationary"]["passed"] is True
            assert result["sensor"]["frames"] == 3000
            assert result["sensor"]["end_monotonic_ns"] - result["sensor"]["start_monotonic_ns"] == 30_000_000_000
            assert result["stationary"]["host_window_s"] == pytest.approx(30., abs=1e-12)
            assert result["stationary"]["acquisition_sample_count"] == 601
            assert result["stationary"]["observed_span_s"] == pytest.approx(30., abs=1e-12)
            assert guard.thread.join_calls == 1
            guard.check()  # Even after the long export and bundle verification.
            path = collector.args.output / f"action_background_{ordinal}.background.json"
            terminals.append(path.read_bytes())
            assert json.loads(terminals[-1])["accepted"] is True
        assert copied_rows == [3000, 3000, 3000]
        assert all(left < right for left, right in zip(snapshots, snapshots[1:]))
        assert all(end == cutoff for _, end, cutoff in boundaries)
        assert factory.sessions[0].start_calls == 1
        assert factory.sessions[0].stop_calls == 0
        assert len(factory.sessions) == 1
    finally:
        closed = device.close()
    assert Path(closed["raw_csv"]).is_file()
    assert factory.sessions[0].stop_calls == 1
    for ordinal, saved in enumerate(terminals, 1):
        assert (collector.args.output / f"action_background_{ordinal}.background.json").read_bytes() == saved


@pytest.mark.parametrize("failure", ["gap", "velocity", "joint_position"])
def test_real_within_window_feedback_failure_preserves_raw_and_failed_terminal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure: str,
) -> None:
    collector, factory, clock, state, guards = pipeline(tmp_path, monkeypatch)
    device = collector.sensor
    original_sleep = device._sleep
    sleeps = 0

    def fail_during_acquisition(seconds: float) -> None:
        nonlocal sleeps
        sleeps += 1
        if sleeps == 4:
            if failure == "gap":
                state.gap_pending = True
            elif failure == "velocity":
                state.velocity = .010001
            else:
                state.offset = .001001
        original_sleep(seconds)

    device._sleep = fail_during_acquisition
    try:
        with pytest.raises(Exception, match="background window failed"):
            collector.perform(background(1, duration_s=1.))
        guard = guards[-1]
        terminal = json.loads((collector.args.output / "action_background_1.background.json").read_text())
        sensor_window = json.loads((device.root / "windows/background_0001/window.json").read_text())
        assert terminal["accepted"] is False
        assert terminal["stationary"]["passed"] is False
        assert sensor_window["quality"]["accepted"] is False
        assert Path(sensor_window["raw_csv"]).is_file()
        assert Path(sensor_window["raw_csv"]).stat().st_size > 0
        assert guard.stream.closed and not guard.thread.is_alive()
        expected = {"gap": "gap or regressing clock", "velocity": "not stationary",
                    "joint_position": "left requested stationary pose"}[failure]
        assert expected in str(guard.error)
        assert factory.sessions[0].running is True
        assert factory.sessions[0].start_calls == 1
    finally:
        device.close()


def test_post_acquisition_verification_failure_keeps_valid_raw_but_rejects_action(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    collector, _, _, state, guards = pipeline(tmp_path, monkeypatch)
    device = collector.sensor
    state.freeze_error = ValueError("synthetic frozen bundle changed")
    try:
        with pytest.raises(RuntimeError, match="frozen bundle changed"):
            collector.perform(background(1, duration_s=1.))
        guard = guards[-1]
        terminal = json.loads((collector.args.output / "action_background_1.background.json").read_text())
        sensor_window = json.loads((device.root / "windows/background_0001/window.json").read_text())
        assert terminal["accepted"] is False
        assert terminal["stationary"]["passed"] is False
        assert sensor_window["quality"]["accepted"] is True
        assert len(raw_rows(Path(sensor_window["raw_csv"]))) == 100
        assert "frozen bundle changed" in terminal["stationary"]["error"]
        assert guard.stream.closed and not guard.thread.is_alive()
    finally:
        device.close()


def pending_reader(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, *, outcome: str):
    """A real reader is already waiting in fresh_state at acquisition cutoff."""
    collector, _, clock, _, _ = pipeline(tmp_path, monkeypatch)
    motion = collector.motion
    monkeypatch.setattr(motion_module.threading, "Thread", REAL_THREAD)
    entered, release = threading.Event(), threading.Event()
    calls = 0

    def fresh_state(timeout: float) -> JointStateSnapshot:
        nonlocal calls
        calls += 1
        if calls > 1:
            entered.set()
            if not release.wait(timeout=2.):
                raise TimeoutError("test pending state was not released")
            if outcome == "timeout":
                raise TimeoutError("no new actual JointState before deadline")
        positions, velocities, stamp = (0.,)*6, (0.,)*6, clock()
        if calls > 1 and outcome == "velocity":
            velocities = (.010001, 0., 0., 0., 0., 0.)
        if calls > 1 and outcome == "nonfinite":
            positions = (float("nan"), 0., 0., 0., 0., 0.)
        if calls > 1 and outcome == "queued_drift":
            positions = (.0008, .0008, .0008, 0., 0., 0.)
            stamp -= 300_000_000
        return JointStateSnapshot(motion.environment.joint_names, positions, velocities,
                                  stamp, stamp, calls)

    motion._backend = lambda: SimpleNamespace(fresh_state=fresh_state)
    motion._unchanged = lambda: None
    guard = RawStationaryMonitor(motion, tmp_path / "pending_actual.csv", 1.)
    guard.__enter__()
    assert entered.wait(timeout=1.)
    for sequence in range(2, 7):
        clock.sleep(.2)
        guard._record(JointStateSnapshot(motion.environment.joint_names, (0.,)*6, (0.,)*6,
                                        clock(), clock(), sequence))
    return collector.sensor, guard, clock, release


def test_valid_postcutoff_state_does_not_extend_frozen_acquisition_window(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, guard, clock, release = pending_reader(tmp_path, monkeypatch, outcome="valid")
    original_join = guard.thread.join
    cutoff = clock()

    def release_during_join(timeout: float) -> None:
        assert guard.stop.is_set()
        clock.sleep(.2)
        release.set()
        original_join(timeout)

    monkeypatch.setattr(guard.thread, "join", release_during_join)
    try:
        guard.finish_acquisition()
        guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is True
        assert guard.evidence["acquisition_cutoff_sync_time_ns"] == cutoff
        assert guard.evidence["observed_span_s"] == 1.
        assert guard.evidence["acquisition_sample_count"] == 6
        assert guard.evidence["host_window_s"] == 1.
    finally:
        release.set()
        original_join(timeout=1.)
        device.close()


@pytest.mark.parametrize("outcome,postcutoff_delay_s", [("timeout", .2), ("valid", .6)])
def test_postcutoff_reader_error_must_not_reject_fully_covered_acquisition(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, outcome: str, postcutoff_delay_s: float,
) -> None:
    device, guard, clock, release = pending_reader(tmp_path, monkeypatch, outcome=outcome)
    original_join = guard.thread.join
    cutoff = clock()

    def release_during_join(timeout: float) -> None:
        assert guard.stop.is_set() and guard._cutoff_ns == cutoff
        clock.sleep(postcutoff_delay_s)
        release.set()
        original_join(timeout)

    monkeypatch.setattr(guard.thread, "join", release_during_join)
    try:
        guard.finish_acquisition()
        guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is True
        assert guard.evidence["acquisition_cutoff_sync_time_ns"] == cutoff
        assert guard.evidence["observed_span_s"] == 1.
        assert guard.evidence["acquisition_cutoff_verified"] is True
        assert len(guard.evidence["post_acquisition_reader_diagnostics"]) == 1
        assert guard.evidence["sample_count"] == 6
    finally:
        release.set()
        original_join(timeout=1.)
        device.close()


def test_reader_error_known_before_cutoff_still_rejects_covered_window(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, guard, _, release = pending_reader(tmp_path, monkeypatch, outcome="timeout")
    try:
        release.set()
        guard.thread.join(timeout=1.)
        assert isinstance(guard.error, TimeoutError)
        with pytest.raises(RuntimeError, match="no new actual JointState"):
            guard.finish_acquisition()
        with pytest.raises(RuntimeError, match="no new actual JointState"):
            guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is False
    finally:
        release.set()
        guard.thread.join(timeout=1.)
        device.close()


def test_poststop_timeout_is_not_exempt_when_cutoff_freshness_failed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, guard, clock, release = pending_reader(tmp_path, monkeypatch, outcome="timeout")
    original_join = guard.thread.join
    clock.sleep(.6)

    def release_during_join(timeout: float) -> None:
        clock.sleep(.2)
        release.set()
        original_join(timeout)

    monkeypatch.setattr(guard.thread, "join", release_during_join)
    try:
        with pytest.raises(RuntimeError, match="became stale"):
            guard.finish_acquisition()
        with pytest.raises(RuntimeError, match="became stale"):
            guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is False
        assert guard.evidence["acquisition_cutoff_verified"] is False
        assert guard.evidence["post_acquisition_reader_diagnostics"] == []
    finally:
        release.set()
        original_join(timeout=1.)
        device.close()


@pytest.mark.parametrize("outcome,expected", [
    ("velocity", "not stationary"), ("nonfinite", "nonfinite q/qd")])
def test_untimestamped_state_validation_errors_remain_fatal_even_after_stop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, outcome: str, expected: str,
) -> None:
    device, guard, clock, release = pending_reader(tmp_path, monkeypatch, outcome=outcome)
    original_join = guard.thread.join

    def release_during_join(timeout: float) -> None:
        clock.sleep(.2)
        release.set()
        original_join(timeout)

    monkeypatch.setattr(guard.thread, "join", release_during_join)
    try:
        with pytest.raises(RuntimeError, match=expected):
            guard.finish_acquisition()
        with pytest.raises(RuntimeError, match=expected):
            guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is False
        assert guard.evidence["post_acquisition_reader_diagnostics"] == []
    finally:
        release.set()
        original_join(timeout=1.)
        device.close()


def test_late_returned_state_at_or_before_cutoff_still_has_fk_drift_validated(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, guard, clock, release = pending_reader(tmp_path, monkeypatch, outcome="queued_drift")
    original_join = guard.thread.join
    clock.sleep(.2)  # Last completed feedback remains fresh at this cutoff.

    def release_during_join(timeout: float) -> None:
        clock.sleep(.2)
        release.set()
        original_join(timeout)

    monkeypatch.setattr(guard.thread, "join", release_during_join)
    try:
        with pytest.raises(RuntimeError, match="drift exceeds 1mm"):
            guard.finish_acquisition()
        with pytest.raises(RuntimeError, match="drift exceeds 1mm"):
            guard.__exit__(None, None, None)
        assert guard.evidence["passed"] is False
        assert guard.evidence["post_acquisition_reader_diagnostics"] == []
    finally:
        release.set()
        original_join(timeout=1.)
        device.close()
