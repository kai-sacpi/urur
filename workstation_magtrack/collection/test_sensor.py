"""Persistent sensor contracts exercised entirely with files and a fake clock.

No test opens serial hardware, imports ROS, or sleeps in wall-clock time.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Callable

import pytest

from hx_waypoint_nucleo.capture import (
    NucleoCaptureConfig,
    NucleoCaptureError,
    NucleoCaptureSummary,
    RAW_CAPTURE_COLUMNS,
)
from hx_waypoint_nucleo.protocol import ParserStats


SPEC = importlib.util.spec_from_file_location(
    "collection_sensor_under_test", Path(__file__).with_name("sensor.py")
)
assert SPEC is not None and SPEC.loader is not None
sensor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = sensor
SPEC.loader.exec_module(sensor)


@dataclass
class FakeClock:
    now_ns: int = 10_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def sleep(self, seconds: float) -> None:
        assert seconds >= 0
        self.now_ns += max(1, round(seconds * 1_000_000_000))


class FakeSession:
    """A continuously growing 100 Hz raw capture, without a reader thread."""

    def __init__(
        self,
        workspace: Path,
        config: NucleoCaptureConfig,
        ros_time_factory: Any,
        *,
        clock: FakeClock,
        ordinal: int,
        start_error: Exception | None = None,
        stop_error: Exception | None = None,
        received_bytes: int = 0,
        ready_after_s: float | None = 0.0,
        missing_clock_report: bool = False,
        snapshot_hook: Callable[[FakeSession], None] | None = None,
    ) -> None:
        self.workspace = Path(workspace)
        self.config = config
        self.ros_time_factory = ros_time_factory
        self.clock = clock
        self.start_error = start_error
        self.stop_error = stop_error
        self.received_bytes = received_bytes
        self.ready_after_s = ready_after_s
        self.missing_clock_report = missing_clock_report
        self.snapshot_hook = snapshot_hook
        self.start_calls = 0
        self.stop_calls = 0
        self.capture_error: str | None = None
        self.counter_overrides: dict[str, int] = {}
        self.started_ns = clock()
        self.last_frame_ns: int | None = None
        self.frames_written = 0
        self.running = False
        self.staging_path = (
            self.workspace / "runs" / f"nucleo_capture__fake_{ordinal}.staging.csv"
        )
        self.staging_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.staging_path.open("x", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(
            self.stream, fieldnames=RAW_CAPTURE_COLUMNS, lineterminator="\n"
        )
        self.writer.writeheader()
        self.stream.flush()

    def start(self) -> FakeSession:
        self.start_calls += 1
        if self.start_error is not None:
            raise self.start_error
        self.running = True
        self._emit_until_now()
        return self

    def _emit_until_now(self) -> None:
        if not self.running:
            return
        next_ns = (
            self.started_ns
            if self.last_frame_ns is None
            else self.last_frame_ns + 10_000_000
        )
        while next_ns <= self.clock():
            row: dict[str, str | int] = {name: "0" for name in RAW_CAPTURE_COLUMNS}
            row.update(
                sensor_rx_sync_time_ns=next_ns,
                sensor_rx_ros_time_ns=next_ns + 1_000_000_000,
                parser_host_monotonic_ns=next_ns,
                logical_seq=self.frames_written,
                src_seq=self.frames_written % 16,
                source_time_ms_u32=(next_ns // 1_000_000) & 0xFFFFFFFF,
                channel_ids="1;2;3;4",
                parser_hub_packets=(self.frames_written + 1) * 4,
                parser_completed_groups=self.frames_written + 1,
            )
            for channel in range(1, 5):
                row[f"channel_{channel}_nucleo_time_ms_u32"] = (next_ns // 1_000_000) & 0xFFFFFFFF
                row[f"channel_{channel}_source_time_ms_u32"] = (next_ns // 1_000_000) & 0xFFFFFFFF
            row.update({f"parser_{name}": value for name, value in self.counter_overrides.items()})
            self.writer.writerow(row)
            self.frames_written += 1
            self.last_frame_ns = next_ns
            next_ns += 10_000_000
        self.stream.flush()

    @property
    def parser_stats(self) -> ParserStats:
        return ParserStats(
            bytes_received=self.received_bytes + self.frames_written * 100,
            hub_packets=self.frames_written * 4,
            completed_groups=self.frames_written,
            **self.counter_overrides,
        )

    def snapshot(self) -> Any:
        self._emit_until_now()
        if self.snapshot_hook is not None:
            self.snapshot_hook(self)
        ready = self.ready_after_s is not None and (
            self.clock() - self.started_ns >= round(self.ready_after_s * 1e9)
        )
        report = {
            **sensor.DeviceClockDrift().report(), "status": "observed",
            "frames": self.frames_written,
            "startup_baseline": {"required": True, "ready": ready,
                                 "status": "ready" if ready else "warming_up"},
        }
        return sensor.SensorSnapshot(
            staging_path=self.staging_path,
            byte_count=self.staging_path.stat().st_size,
            frames_written=self.frames_written,
            parser_stats=self.parser_stats,
            last_frame_ns=self.last_frame_ns,
            started_sync_time_ns=self.started_ns,
            capture_error=self.capture_error,
            clock_drift=None if self.missing_clock_report else report,
        )

    def stop(self) -> NucleoCaptureSummary:
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error
        self._emit_until_now()
        self.running = False
        if not self.stream.closed:
            self.stream.close()
        return NucleoCaptureSummary(
            staging_path=self.staging_path,
            workspace=self.workspace,
            config=self.config,
            started_sync_time_ns=self.started_ns,
            stopped_sync_time_ns=self.clock(),
            frames_written=self.frames_written,
            parser_stats=self.parser_stats,
            capture_error=self.capture_error,
            sync_attempts=int(self.config.perform_sync),
        )


class FakeSessionFactory:
    def __init__(
        self,
        clock: FakeClock,
        *,
        passive_error: Exception | None = None,
        passive_received_bytes: int = 0,
        active_error: Exception | None = None,
        passive_stop_error: Exception | None = None,
        active_stop_error: Exception | None = None,
        ready_after_s: float | None = 0.0,
        missing_clock_report: bool = False,
        snapshot_hook: Callable[[FakeSession], None] | None = None,
    ) -> None:
        self.clock = clock
        self.passive_error = passive_error
        self.passive_received_bytes = passive_received_bytes
        self.active_error = active_error
        self.passive_stop_error = passive_stop_error
        self.active_stop_error = active_stop_error
        self.ready_after_s = ready_after_s
        self.missing_clock_report = missing_clock_report
        self.snapshot_hook = snapshot_hook
        self.sessions: list[FakeSession] = []

    def __call__(
        self, workspace: Path, config: NucleoCaptureConfig, ros_time_factory: Any
    ) -> FakeSession:
        active = config.perform_sync
        instance = FakeSession(
            workspace,
            config,
            ros_time_factory,
            clock=self.clock,
            ordinal=len(self.sessions) + 1,
            start_error=self.active_error if active else self.passive_error,
            stop_error=self.active_stop_error if active else self.passive_stop_error,
            received_bytes=0 if active else self.passive_received_bytes,
            ready_after_s=self.ready_after_s,
            missing_clock_report=self.missing_clock_report,
            snapshot_hook=self.snapshot_hook,
        )
        self.sessions.append(instance)
        return instance


def start_fake(
    tmp_path: Path, **factory_kwargs: Any
) -> tuple[Any, FakeSessionFactory, FakeClock]:
    clock = FakeClock()
    factory = FakeSessionFactory(clock, **factory_kwargs)
    device = sensor.PersistentSensor.start_session(
        tmp_path / "workspace" / "runs" / "collection",
        "session_001",
        "/dev/never-opened",
        workspace=tmp_path / "workspace",
        session_factory=factory,
        boot_id_factory=lambda: "host-boot-one",
        clock_ns=clock,
        sleep=clock.sleep,
    )
    return device, factory, clock


def raw_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def robot_telemetry(
    workspace: Path, name: str, times: list[int]
) -> Path:
    path = workspace / "runs" / name / "actual" / "execution_telemetry.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=("sync_time_ns", "trajectory_time_s"),
            lineterminator="\n",
        )
        writer.writeheader()
        for timestamp in times:
            writer.writerow({
                "sync_time_ns": timestamp,
                "trajectory_time_s": (timestamp - times[0]) / 1_000_000_000,
            })
    return path


def test_complete_passive_stream_never_sends_sync(tmp_path: Path) -> None:
    device, factory, _ = start_fake(tmp_path)
    try:
        assert len(factory.sessions) == 1
        assert factory.sessions[0].config.perform_sync is False
        assert factory.sessions[0].start_calls == 1
    finally:
        device.close()


def test_production_default_uses_process_adapter_with_owned_port_lock(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    import fcntl
    import time
    from workstation_magtrack.collection import capture_process
    clock = FakeClock()
    factory = FakeSessionFactory(clock)
    descriptors = []

    def process_factory(workspace, config, wall_clock, *, ownership_fd):
        assert wall_clock is time.time_ns
        fcntl.fcntl(ownership_fd, fcntl.F_GETFD)  # Parent already owns a live lock.
        descriptors.append(ownership_fd)
        session = factory(workspace, config, wall_clock)
        session.process_identity = {"start_method": "spawn", "capture_pid": 321}
        return session

    monkeypatch.setattr(capture_process, "ProcessObservedSession", process_factory)
    monkeypatch.setattr(sensor, "ObservedSession", lambda *a, **kw: pytest.fail("in-process fallback"))
    device = sensor.PersistentSensor.start_session(
        tmp_path / "workspace/runs/collection", "isolated_default", "/dev/never-opened",
        workspace=tmp_path / "workspace", boot_id_factory=lambda: "test-boot",
        clock_ns=clock, sleep=clock.sleep,
    )
    try:
        assert len(descriptors) == 1
        assert device._identity()["capture_process"] == {"start_method": "spawn", "capture_pid": 321}
        assert json.loads((device.root / "epoch.json").read_text())["capture_process"]["capture_pid"] == 321
        assert factory.sessions[0].config.perform_sync is False
    finally:
        device.close()


def test_spawn_failure_never_falls_back_to_thread_or_retries_sync(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    from workstation_magtrack.collection import capture_process
    calls = []

    def failed_process(*args, **kwargs):
        calls.append((args, kwargs))
        raise NucleoCaptureError("spawn could not start capture process")

    monkeypatch.setattr(capture_process, "ProcessObservedSession", failed_process)
    monkeypatch.setattr(sensor, "ObservedSession", lambda *a, **kw: pytest.fail("in-process fallback"))
    clock = FakeClock()
    with pytest.raises(NucleoCaptureError, match="spawn could not start"):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace/runs/collection", "spawn_failed", "/dev/never-opened",
            workspace=tmp_path / "workspace", boot_id_factory=lambda: "test-boot",
            clock_ns=clock, sleep=clock.sleep,
        )
    assert len(calls) == 1
    assert calls[0][0][1].perform_sync is False
    root = tmp_path / "workspace/runs/collection/sensor_sessions/spawn_failed"
    readiness = json.loads((root / "startup_readiness.json").read_text())
    assert readiness["status"] == "failed"
    assert readiness["motion_allowed"] is False


def test_only_silent_preflight_timeout_permits_one_sync(tmp_path: Path) -> None:
    device, factory, _ = start_fake(
        tmp_path,
        passive_error=NucleoCaptureError(
            "Nucleo preflight timed out without a complete grouped frame after 5.000 s"
        ),
    )
    try:
        assert [item.config.perform_sync for item in factory.sessions] == [False, True]
        assert factory.sessions[0].stop_calls == 1
        assert factory.sessions[1].config.sync_retries == 1
        assert factory.sessions[1].start_calls == 1
    finally:
        device.close()


def test_reused_boot_silent_timeout_never_sends_another_sync(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock, passive_error=NucleoCaptureError("Nucleo preflight timed out"))
    with pytest.raises(sensor.SensorError, match="SYNC is forbidden"):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace/runs/no_repeat_sync", "session_001", "/dev/never-opened",
            workspace=tmp_path / "workspace", allow_initial_sync=False,
            session_factory=factory, boot_id_factory=lambda: "host-boot-one",
            clock_ns=clock, sleep=clock.sleep,
        )
    assert len(factory.sessions) == 1
    assert factory.sessions[0].config.perform_sync is False
    assert factory.sessions[0].stop_calls == 1


def test_startup_waits_for_stable_baseline_without_losing_raw_or_reopening(
    tmp_path: Path,
) -> None:
    device, factory, clock = start_fake(tmp_path, ready_after_s=0.12)
    session = factory.sessions[0]
    try:
        assert clock() == 10_150_000_000
        assert len(factory.sessions) == 1
        assert session.start_calls == 1
        assert session.stop_calls == 0
        rows = raw_rows(session.staging_path)
        assert len(rows) == 16
        assert int(rows[0]["sensor_rx_sync_time_ns"]) == 10_000_000_000
        assert int(rows[-1]["sensor_rx_sync_time_ns"]) == 10_150_000_000
        path = device.root / "startup_readiness.json"
        original = path.read_bytes()
        result = json.loads(original)
        assert result["status"] == "ready"
        assert result["motion_allowed"] is True
        assert result["clock_drift"]["startup_baseline"]["required"] is True
        assert result["clock_drift"]["startup_baseline"]["ready"] is True
        assert result["wait_started_monotonic_ns"] == 10_000_000_000
        clock.sleep(0.1)
        device.check_ready()
        assert path.read_bytes() == original
    finally:
        closed = device.close()
    assert raw_rows(Path(closed["raw_csv"]))[:len(rows)] == rows


def test_stable_start_after_one_sync_uses_same_reader_for_entire_wait(tmp_path: Path) -> None:
    device, factory, _ = start_fake(
        tmp_path, ready_after_s=0.12,
        passive_error=NucleoCaptureError("Nucleo preflight timed out"),
    )
    try:
        assert [session.config.perform_sync for session in factory.sessions] == [False, True]
        assert [session.start_calls for session in factory.sessions] == [1, 1]
        assert [session.stop_calls for session in factory.sessions] == [1, 0]
        assert device.check_ready()["startup_baseline"]["ready"] is True
    finally:
        device.close()


@pytest.mark.parametrize(
    "factory_kwargs, expected_error, expected_wait_ns",
    [
        ({"missing_clock_report": True}, "diagnostics are missing", 0),
        ({"ready_after_s": None}, "within 30 seconds", 30_000_000_000),
    ],
)
def test_unready_startup_preserves_all_raw_and_releases_stopped_reader(
    tmp_path: Path, factory_kwargs: dict[str, Any], expected_error: str,
    expected_wait_ns: int,
) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock, **factory_kwargs)
    run = tmp_path / "workspace" / "runs" / "collection"
    with pytest.raises(sensor.SensorError, match=expected_error):
        sensor.PersistentSensor.start_session(
            run, "session_001", "/dev/never-opened", workspace=tmp_path / "workspace",
            session_factory=factory, boot_id_factory=lambda: "host-boot-one",
            clock_ns=clock, sleep=clock.sleep,
        )
    assert clock() == 10_000_000_000 + expected_wait_ns
    assert len(factory.sessions) == 1
    assert factory.sessions[0].stop_calls == 1
    assert not factory.sessions[0].running
    root = run / "sensor_sessions" / "session_001"
    evidence = json.loads((root / "startup_readiness.json").read_text())
    assert evidence["status"] == "failed"
    assert evidence["motion_allowed"] is False
    assert expected_error in evidence["error"]
    manifest = json.loads((root / "startup_session_manifest.json").read_text())
    assert manifest["serial_reader_stopped"] is True
    assert manifest["frames"] == 1 + expected_wait_ns // 10_000_000
    assert len(raw_rows(Path(manifest["raw_csv"]))) == manifest["frames"]
    assert sensor._hash(Path(manifest["raw_csv"])) == manifest["raw_sha256"]
    # A fresh explicit session can obtain the port only because the failed
    # session's existing reader stopped successfully and released ownership.
    factory.ready_after_s = 0
    factory.missing_clock_report = False
    next_device = sensor.PersistentSensor.start_session(
        run, "session_002", "/dev/never-opened", workspace=tmp_path / "workspace",
        session_factory=factory, boot_id_factory=lambda: "host-boot-one",
        clock_ns=clock, sleep=clock.sleep,
    )
    next_device.close()


@pytest.mark.parametrize("failure", ["stale", "reset", "host_epoch"])
def test_stable_start_checks_stream_freshness_reset_and_host_epoch(
    tmp_path: Path, failure: str,
) -> None:
    clock = FakeClock()
    boot = ["host-boot-one"]

    def invalidate(session: FakeSession) -> None:
        if clock() < 10_100_000_000:
            return
        if failure == "stale":
            session.last_frame_ns = clock() - 500_000_001
        elif failure == "reset":
            session.capture_error = "device clock regressed: reset requires a new explicit epoch"
        else:
            boot[0] = "host-boot-two"

    factory = FakeSessionFactory(clock, ready_after_s=10, snapshot_hook=invalidate)
    with pytest.raises(sensor.SensorError, match={
        "stale": "stream is stale", "reset": "device clock regressed",
        "host_epoch": "host boot epoch changed",
    }[failure]):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", "session_001",
            "/dev/never-opened", workspace=tmp_path / "workspace",
            session_factory=factory, boot_id_factory=lambda: boot[0],
            clock_ns=clock, sleep=clock.sleep,
        )
    assert len(factory.sessions) == 1
    assert factory.sessions[0].stop_calls == 1
    root = tmp_path / "workspace" / "runs" / "collection" / "sensor_sessions" / "session_001"
    assert json.loads((root / "startup_readiness.json").read_text())["motion_allowed"] is False
    assert (root / "startup_nucleo_raw_telemetry.csv").is_file()


@pytest.mark.parametrize("baseline", [
    None, {"required": False, "ready": True, "status": "ready"},
    {"required": True, "ready": False, "status": "warming_up"},
    {"required": True, "ready": True, "status": "warming_up"},
])
def test_dispatch_fails_closed_when_ready_report_is_missing_or_invalid(
    tmp_path: Path, baseline: dict[str, Any] | None,
) -> None:
    from dataclasses import replace
    device, factory, _ = start_fake(tmp_path)
    session = factory.sessions[0]
    original = session.snapshot
    report = {**sensor.DeviceClockDrift().report(), "error": None,
              "startup_baseline": baseline}
    session.snapshot = lambda: replace(original(), clock_drift=report)
    try:
        with pytest.raises(sensor.SensorError, match="stable-start"):
            device.check_ready()
        assert len(factory.sessions) == 1
        assert session.start_calls == 1
        assert session.stop_calls == 0
        # The initial readiness outcome is frozen, never silently rebased or
        # changed to make a later dispatch ready.
        evidence = json.loads((device.root / "startup_readiness.json").read_text())
        assert evidence["status"] == "ready"
    finally:
        session.snapshot = original
        device.close()


@pytest.mark.parametrize("received_bytes", [1, 50, 4096])
def test_partial_passive_stream_never_retries_sync(
    tmp_path: Path, received_bytes: int
) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(
        clock,
        passive_error=NucleoCaptureError(
            "Nucleo preflight timed out without a complete grouped frame after 5.000 s"
        ),
        passive_received_bytes=received_bytes,
    )
    with pytest.raises((RuntimeError, ValueError), match="(?i)sync|bytes|stream"):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", "session_001", "/dev/never-opened",
            workspace=tmp_path / "workspace", session_factory=factory,
            boot_id_factory=lambda: "host-boot-one", clock_ns=clock, sleep=clock.sleep,
        )
    assert len(factory.sessions) == 1
    assert factory.sessions[0].config.perform_sync is False
    assert factory.sessions[0].stop_calls == 1


def test_non_timeout_start_error_does_not_send_sync(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock, passive_error=NucleoCaptureError("serial read failed"))
    with pytest.raises(sensor.SensorError, match="refusing another SYNC") as error:
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", "session_001", "/dev/never-opened",
            workspace=tmp_path / "workspace", session_factory=factory,
            boot_id_factory=lambda: "host-boot-one", clock_ns=clock, sleep=clock.sleep,
        )
    assert len(factory.sessions) == 1
    assert factory.sessions[0].config.perform_sync is False
    assert str(error.value.__cause__) == "serial read failed"


def test_close_is_idempotent_and_does_not_reopen_sensor(tmp_path: Path) -> None:
    device, factory, _ = start_fake(tmp_path)
    first = device.close()
    second = device.close()
    assert second == first
    assert factory.sessions[0].stop_calls == 1
    assert len(factory.sessions) == 1
    with pytest.raises((RuntimeError, ValueError), match="(?i)clos|stopp"):
        device.window(
            "after_close", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )


def test_duplicate_session_is_rejected_without_overwriting(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    device.close()
    before = {
        path: path.read_bytes()
        for path in (tmp_path / "workspace" / "runs" / "collection").rglob("*")
        if path.is_file()
    }
    with pytest.raises((FileExistsError, RuntimeError, ValueError)):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", "session_001", "/dev/never-opened",
            workspace=tmp_path / "workspace", session_factory=factory,
            boot_id_factory=lambda: "host-boot-one", clock_ns=clock, sleep=clock.sleep,
        )
    assert len(factory.sessions) == 1
    assert before == {path: path.read_bytes() for path in before}


@pytest.mark.parametrize("name", ["../escape", "/absolute", "", "a/b"])
def test_session_names_cannot_escape_output_root(tmp_path: Path, name: str) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock)
    with pytest.raises((RuntimeError, ValueError)):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", name, "/dev/never-opened",
            workspace=tmp_path / "workspace", session_factory=factory,
            boot_id_factory=lambda: "host-boot-one", clock_ns=clock, sleep=clock.sleep,
        )
    assert not factory.sessions


def test_two_background_windows_share_reader_and_preserve_gap_frames(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    try:
        first = device.window(
            "background_001", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )
        clock.sleep(0.1)  # Represents an intervening robot move, not a capture window.
        second = device.window(
            "background_002", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )
        assert first["quality"]["accepted"] is True
        assert second["quality"]["accepted"] is True
        assert first["quality"]["scope"] == "sensor_integrity_only_not_stationarity"
        assert first["full_stationarity_status"] == "pending_external_monitor_finalization"
        assert first["requires_action_stationarity_terminal_manifest"] is True
        assert first["epoch_id"] == second["epoch_id"]
        assert first["session_id"] == second["session_id"] == "session_001"
        assert len(factory.sessions) == 1
        assert factory.sessions[0].start_calls == 1
        assert factory.sessions[0].stop_calls == 0
        source = raw_rows(factory.sessions[0].staging_path)
        first_rows = raw_rows(Path(first["raw_csv"]))
        second_rows = raw_rows(Path(second["raw_csv"]))
        assert len(first_rows) == len(second_rows) == 20
        assert len(source) > len(first_rows) + len(second_rows)
        assert any(
            first["end_monotonic_ns"] <= int(row["sensor_rx_sync_time_ns"])
            < second["start_monotonic_ns"] for row in source
        )
    finally:
        manifest = device.close()
    assert Path(manifest["raw_csv"]).is_file()
    assert factory.sessions[0].stop_calls == 1
    assert len(raw_rows(Path(manifest["raw_csv"]))) == len(source)


def test_default_background_is_exactly_thirty_seconds_of_fake_time(tmp_path: Path) -> None:
    device, _, clock = start_fake(tmp_path)
    try:
        before = clock()
        result = device.window(
            "thirty_seconds", metadata={"stationary_evidence": {"passed": True}}
        )
        assert clock() - before == 30_000_000_000
        assert result["requested_duration_s"] == 30.0
        assert result["frames"] == 3000
        assert result["status"] == "complete"
    finally:
        device.close()


@pytest.mark.parametrize("metadata", [None, {}, {"stationary_evidence": {"passed": False}}])
def test_background_requires_explicit_passed_stationary_evidence(
    tmp_path: Path, metadata: dict[str, Any] | None
) -> None:
    device, _, clock = start_fake(tmp_path)
    try:
        before = clock()
        with pytest.raises(sensor.SensorError, match="stationary evidence"):
            device.window("not_authorized", duration_s=0.2, metadata=metadata)
        assert clock() == before
        assert not (device.root / "windows" / "not_authorized").exists()
    finally:
        device.close()


def test_stationary_callback_is_checked_during_window(tmp_path: Path) -> None:
    device, _, clock = start_fake(tmp_path)
    checks: list[int] = []
    try:
        device.window(
            "observed", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
            check_stationary=lambda: checks.append(clock()),
        )
        assert checks[0] == 10_000_000_000
        assert checks[-1] == 10_200_000_000
        assert max(right - left for left, right in zip(checks, checks[1:])) <= 50_000_000
    finally:
        device.close()


def test_acquisition_close_precedes_final_snapshot_copy_and_slow_finalization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, factory, clock = start_fake(tmp_path)
    session = factory.sessions[0]
    events: list[tuple[str, int]] = []
    original_snapshot = session.snapshot
    original_copy = device._copy_window

    def snapshot() -> Any:
        events.append(("snapshot", clock()))
        return original_snapshot()

    def copy_window(*args: Any, **kwargs: Any) -> Any:
        events.append(("copy", clock()))
        result = original_copy(*args, **kwargs)
        clock.sleep(60)  # Expensive output work is not live acquisition time.
        events.append(("copy_finished", clock()))
        return result

    monkeypatch.setattr(session, "snapshot", snapshot)
    monkeypatch.setattr(device, "_copy_window", copy_window)
    try:
        result = device.window(
            "closed_before_finalize", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
            check_stationary=lambda: events.append(("stationary", clock())),
            on_acquisition_closed=lambda: events.append(("closed", clock())),
        )
        closed = [i for i, event in enumerate(events) if event[0] == "closed"]
        assert len(closed) == 1
        assert events[closed[0] - 1:closed[0] + 3] == [
            ("stationary", 10_200_000_000), ("closed", 10_200_000_000),
            ("snapshot", 10_200_000_000), ("copy", 10_200_000_000),
        ]
        assert events[-1] == ("copy_finished", 70_200_000_000)
        assert result["end_monotonic_ns"] == 10_200_000_000
        assert result["frames"] == 20
        assert result["quality"]["accepted"] is True
        assert result["acquisition_closed_callback_supplied"] is True
        assert result["acquisition_close_error"] is None
    finally:
        device.close()


def test_acquisition_close_failure_preserves_incomplete_raw_window(
    tmp_path: Path,
) -> None:
    device, factory, clock = start_fake(tmp_path)
    close_calls: list[int] = []
    failure = RuntimeError("stationary reader could not close")

    def close_acquisition() -> None:
        close_calls.append(clock())
        raise failure

    try:
        with pytest.raises(sensor.SensorError, match="background window failed") as raised:
            device.window(
                "close_failed", duration_s=0.2,
                metadata={"stationary_evidence": {"passed": True}},
                on_acquisition_closed=close_acquisition,
            )
        assert raised.value.__cause__ is failure
        assert close_calls == [10_200_000_000]
        evidence = json.loads((device.root / "windows/close_failed/window.json").read_text())
        assert evidence["status"] == "incomplete"
        assert evidence["quality"]["accepted"] is False
        assert evidence["acquisition_close_error"] == "RuntimeError: stationary reader could not close"
        assert evidence["frames"] == 20
        assert Path(evidence["raw_csv"]).is_file()
        assert sensor._hash(Path(evidence["raw_csv"])) == evidence["raw_sha256"]
        assert factory.sessions[0].staging_path.is_file()
    finally:
        device.close()


def test_live_failure_remains_primary_when_acquisition_close_also_fails(
    tmp_path: Path,
) -> None:
    device, _, clock = start_fake(tmp_path)
    primary = RuntimeError("robot began moving")
    secondary = RuntimeError("stationary reader shutdown failed")
    close_calls: list[int] = []

    def check_stationary() -> None:
        if clock() >= 10_100_000_000:
            raise primary

    def close_acquisition() -> None:
        close_calls.append(clock())
        raise secondary

    try:
        with pytest.raises(sensor.SensorError, match="background window failed") as raised:
            device.window(
                "both_failed", duration_s=0.2,
                metadata={"stationary_evidence": {"passed": True}},
                check_stationary=check_stationary,
                on_acquisition_closed=close_acquisition,
            )
        assert raised.value.__cause__ is primary
        assert close_calls == [10_100_000_000]
        evidence = json.loads((device.root / "windows/both_failed/window.json").read_text())
        assert evidence["status"] == "incomplete"
        assert evidence["quality"]["accepted"] is False
        assert evidence["acquisition_close_error"] == "RuntimeError: stationary reader shutdown failed"
        assert any("robot began moving" in reason for reason in evidence["quality"]["reasons"])
        assert Path(evidence["raw_csv"]).is_file()
        assert evidence["end_monotonic_ns"] == 10_100_000_000
    finally:
        device.close()


def test_final_snapshot_failure_happens_after_single_acquisition_close(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    device, factory, _ = start_fake(tmp_path)
    session = factory.sessions[0]
    original_snapshot = session.snapshot
    events: list[str] = []
    closed = False
    failed = False
    failure = RuntimeError("final snapshot RPC unavailable")

    def close_acquisition() -> None:
        nonlocal closed
        closed = True
        events.append("closed")

    def snapshot() -> Any:
        nonlocal failed
        if closed and not failed:
            failed = True
            events.append("final_snapshot_failed")
            raise failure
        if closed:
            events.append("recovery_snapshot")
        return original_snapshot()

    monkeypatch.setattr(session, "snapshot", snapshot)
    try:
        with pytest.raises(sensor.SensorError, match="background window failed") as raised:
            device.window(
                "snapshot_failed", duration_s=0.2,
                metadata={"stationary_evidence": {"passed": True}},
                on_acquisition_closed=close_acquisition,
            )
        assert raised.value.__cause__ is failure
        assert events == ["closed", "final_snapshot_failed", "recovery_snapshot"]
        evidence = json.loads((device.root / "windows/snapshot_failed/window.json").read_text())
        assert evidence["status"] == "incomplete"
        assert evidence["quality"]["accepted"] is False
        assert evidence["acquisition_close_error"] is None
        assert Path(evidence["raw_csv"]).is_file()
        assert factory.sessions[0].staging_path.is_file()
    finally:
        device.close()


def test_stationary_failure_preserves_incomplete_window_evidence(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)

    def reject_motion() -> None:
        if clock() >= 10_100_000_000:
            raise RuntimeError("robot began moving")

    try:
        with pytest.raises(sensor.SensorError, match="background window failed"):
            device.window(
                "interrupted", duration_s=0.2,
                metadata={"stationary_evidence": {"passed": True}},
                check_stationary=reject_motion,
            )
        evidence = json.loads(
            (device.root / "windows" / "interrupted" / "window.json").read_text()
        )
        assert evidence["quality"]["accepted"] is False
        assert evidence["status"] == "incomplete"
        assert any("robot began moving" in reason for reason in evidence["quality"]["reasons"])
        assert Path(evidence["raw_csv"]).is_file()
        assert factory.sessions[0].staging_path.is_file()
    finally:
        device.close()


def test_duplicate_window_never_overwrites_first_export(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    metadata = {"stationary_evidence": {"passed": True}}
    try:
        first = device.window("same_name", duration_s=0.2, metadata=metadata)
        raw = Path(first["raw_csv"])
        manifest = Path(first["manifest"])
        before = (raw.read_bytes(), manifest.read_bytes(), clock())
        with pytest.raises(FileExistsError):
            device.window("same_name", duration_s=0.2, metadata=metadata)
        assert (raw.read_bytes(), manifest.read_bytes(), clock()) == before
        assert len(factory.sessions) == 1
    finally:
        device.close()


def test_robot_finalizer_consumes_only_disposable_copy(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    device, factory, clock = start_fake(tmp_path)
    session = factory.sessions[0]
    original_finalizer = sensor.finalize_nucleo_capture
    observed: list[Path] = []

    def observe_finalizer(summary: NucleoCaptureSummary, robot: Path) -> Any:
        observed.append(summary.staging_path)
        assert summary.staging_path != session.staging_path
        assert summary.staging_path.name.startswith("nucleo_capture__")
        assert summary.staging_path.name.endswith(".staging.csv")
        return original_finalizer(summary, robot)

    monkeypatch.setattr(sensor, "finalize_nucleo_capture", observe_finalizer)
    try:
        clock.sleep(0.2)
        session.snapshot()
        source_before = session.staging_path.read_bytes()
        robot = robot_telemetry(
            session.workspace, "trajectory_one",
            list(range(10_010_000_000, 10_200_000_001, 10_000_000)),
        )
        result = device.finish_robot_window(robot, {"name": "trajectory_one"})
        assert result["status"] == "complete", result
        assert result["quality"]["scope"] == "sensor_robot_time_mapping_only"
        assert len(observed) == 1
        assert not observed[0].exists()  # Standard finalizer intentionally removes its input.
        assert session.staging_path.read_bytes() == source_before
        assert Path(result["raw_csv"]).is_file()
        assert Path(result["raw_csv"]) != observed[0]
        assert Path(result["mapping"]["path"]).is_file()
        assert session.stop_calls == 0
        clock.sleep(0.01)
        background = device.window(
            "after_trajectory", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )
        assert background["quality"]["accepted"] is True
        assert len(factory.sessions) == 1
    finally:
        manifest = device.close()
    assert Path(manifest["raw_csv"]).is_file()
    assert Path(manifest["raw_csv"]).read_bytes().startswith(source_before)


@pytest.mark.parametrize(
    "metadata", [{"session_id": "previous-session"}, {"host_boot_id": "previous-host-boot"}]
)
def test_robot_epoch_metadata_mismatch_rejected_before_finalizer(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, metadata: dict[str, str]
) -> None:
    device, factory, clock = start_fake(tmp_path)
    called: list[object] = []
    monkeypatch.setattr(sensor, "finalize_nucleo_capture", lambda *args: called.append(args))
    try:
        clock.sleep(0.2)
        robot = robot_telemetry(factory.sessions[0].workspace, "wrong_epoch", [10_050_000_000])
        with pytest.raises(sensor.SensorError, match="different epoch"):
            device.finish_robot_window(robot, metadata)
        assert not called
        assert not (robot.parent / "nucleo_telemetry.csv").exists()
    finally:
        device.close()


@pytest.mark.parametrize(
    "times", [[9_999_999_999], [10_200_000_001], [10_100_000_000, 10_050_000_000]]
)
def test_robot_outside_epoch_or_regressing_time_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, times: list[int]
) -> None:
    device, factory, clock = start_fake(tmp_path)
    called: list[object] = []
    monkeypatch.setattr(sensor, "finalize_nucleo_capture", lambda *args: called.append(args))
    try:
        clock.sleep(0.2)
        robot = robot_telemetry(factory.sessions[0].workspace, "invalid_time", times)
        with pytest.raises(sensor.SensorError, match="outside this epoch|clock regressed"):
            device.finish_robot_window(robot, {"name": "invalid_time"})
        assert not called
    finally:
        device.close()


def test_robot_window_cannot_overlap_preceding_background(tmp_path: Path) -> None:
    device, factory, _ = start_fake(tmp_path)
    try:
        device.window(
            "background_first", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )
        robot = robot_telemetry(
            factory.sessions[0].workspace, "overlap", [10_100_000_000, 10_200_000_000]
        )
        with pytest.raises(sensor.SensorError, match="overlaps a previous window"):
            device.finish_robot_window(robot, {"name": "overlap"})
    finally:
        device.close()


def test_changed_host_boot_id_rejects_numerically_overlapping_time(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock)
    boot = ["host-boot-one"]
    device = sensor.PersistentSensor.start_session(
        tmp_path / "workspace" / "runs" / "collection", "session_001", "/dev/never-opened",
        workspace=tmp_path / "workspace", session_factory=factory,
        boot_id_factory=lambda: boot[0], clock_ns=clock, sleep=clock.sleep,
    )
    try:
        clock.sleep(0.2)
        robot = robot_telemetry(factory.sessions[0].workspace, "other_boot", [10_100_000_000])
        boot[0] = "host-boot-two"
        with pytest.raises(sensor.SensorError, match="host boot epoch changed"):
            device.finish_robot_window(robot, {"name": "other_boot"})
        assert factory.sessions[0].staging_path.is_file()
    finally:
        device.close()


def test_window_quality_uses_counter_delta_not_prior_session_losses(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    session = factory.sessions[0]
    try:
        session.counter_overrides["reordered_packets"] = 7
        clock.sleep(0.1)
        first = device.window(
            "after_old_losses", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
        )
        assert first["status"] == "complete"
        assert first["parser_baseline"]["parser_reordered_packets"] == 7
        assert first["parser_counter_deltas"]["parser_reordered_packets"] == 0
        assert all(row["parser_reordered_packets"] == "7"
                   for row in raw_rows(Path(first["raw_csv"])))

        start = clock()

        def one_new_loss() -> None:
            if clock() >= start + 50_000_000:
                session.counter_overrides["reordered_packets"] = 8

        second = device.window(
            "with_new_loss", duration_s=0.2,
            metadata={"stationary_evidence": {"passed": True}},
            check_stationary=one_new_loss,
        )
        assert second["status"] == "incomplete"
        assert second["quality"]["accepted"] is False
        assert second["parser_counter_deltas"]["parser_reordered_packets"] == 1
        assert "parser_reordered_packets=1" in second["quality"]["reasons"]
    finally:
        device.close()


def test_same_port_second_session_fails_before_starting_another_reader(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    try:
        with pytest.raises(BlockingIOError):
            sensor.PersistentSensor.start_session(
                tmp_path / "workspace" / "runs" / "collection", "session_002",
                "/dev/never-opened", workspace=tmp_path / "workspace",
                session_factory=factory, boot_id_factory=lambda: "host-boot-one",
                clock_ns=clock, sleep=clock.sleep,
            )
        assert len(factory.sessions) == 1
        assert factory.sessions[0].stop_calls == 0
        assert factory.sessions[0].running is True
        assert factory.sessions[0].staging_path.is_file()
    finally:
        device.close()


def test_stationary_failure_at_start_still_writes_rejection_evidence(tmp_path: Path) -> None:
    device, factory, _ = start_fake(tmp_path)

    def never_stationary() -> None:
        raise RuntimeError("robot already moving")

    try:
        with pytest.raises(sensor.SensorError, match="background window failed"):
            device.window(
                "rejected_at_start", duration_s=0.2,
                metadata={"stationary_evidence": {"passed": True}},
                check_stationary=never_stationary,
            )
        evidence = json.loads(
            (device.root / "windows" / "rejected_at_start" / "window.json").read_text()
        )
        assert evidence["status"] == "incomplete"
        assert evidence["frames"] == 0
        assert Path(evidence["raw_csv"]).is_file()
        assert factory.sessions[0].running is True
    finally:
        first = device.close()
        assert device.close() == first


def test_failed_single_sync_does_not_start_third_session(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(
        clock,
        passive_error=NucleoCaptureError("Nucleo preflight timed out"),
        active_error=NucleoCaptureError("Nucleo SYNC did not receive a valid matching ACK"),
    )
    with pytest.raises(NucleoCaptureError, match="SYNC did not receive"):
        sensor.PersistentSensor.start_session(
            tmp_path / "workspace" / "runs" / "collection", "session_001",
            "/dev/never-opened", workspace=tmp_path / "workspace",
            session_factory=factory, boot_id_factory=lambda: "host-boot-one",
            clock_ns=clock, sleep=clock.sleep,
        )
    assert [item.config.perform_sync for item in factory.sessions] == [False, True]
    assert all(item.stop_calls == 1 for item in factory.sessions)


@pytest.mark.parametrize("name", ["nucleo_telemetry.csv", "nucleo_capture.yaml"])
def test_existing_robot_mapping_is_never_overwritten(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, name: str
) -> None:
    device, factory, clock = start_fake(tmp_path)
    called: list[object] = []
    monkeypatch.setattr(sensor, "finalize_nucleo_capture", lambda *args: called.append(args))
    try:
        clock.sleep(0.2)
        robot = robot_telemetry(factory.sessions[0].workspace, "existing_mapping", [10_100_000_000])
        sentinel = robot.with_name(name)
        sentinel.write_bytes(b"existing sensor evidence\n")
        with pytest.raises(sensor.SensorError, match="overwrite"):
            device.finish_robot_window(robot, {"name": "existing_mapping"})
        assert sentinel.read_bytes() == b"existing sensor evidence\n"
        assert not called
    finally:
        device.close()


def test_close_never_overwrites_preexisting_raw_output(tmp_path: Path) -> None:
    device, factory, _ = start_fake(tmp_path)
    target = device.root / "nucleo_raw_telemetry.csv"
    target.write_bytes(b"previous raw evidence\n")
    original = factory.sessions[0].staging_path
    try:
        with pytest.raises((FileExistsError, sensor.SensorError), match="exist|overwrite"):
            device.close()
        assert target.read_bytes() == b"previous raw evidence\n"
        assert original.is_file()
    finally:
        # Remove only this test's own sentinel so the normal close can release its lock.
        target.unlink(missing_ok=True)
        device.close()


def release_test_retained_owners(previous: list[Any]) -> None:
    """Release only fake-reader owners created by the current test."""
    for owner in list(sensor.FAILED_SESSION_OWNERS):
        if owner in previous:
            continue
        owner._lock_stream.close()
        fake = owner._session
        if fake is not None and not fake.stream.closed:
            fake.stream.close()
        sensor.FAILED_SESSION_OWNERS.remove(owner)


def test_unjoinable_stable_start_failure_retains_one_active_reader_and_port(
    tmp_path: Path,
) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(
        clock, ready_after_s=None,
        passive_error=NucleoCaptureError("Nucleo preflight timed out"),
        active_stop_error=NucleoCaptureError("capture reader did not stop within join timeout"),
    )
    previous = list(sensor.FAILED_SESSION_OWNERS)
    root = tmp_path / "workspace" / "runs" / "collection"
    kwargs = {"workspace": tmp_path / "workspace", "session_factory": factory,
              "boot_id_factory": lambda: "host-boot-one", "clock_ns": clock,
              "sleep": clock.sleep}
    try:
        with pytest.raises(sensor.SensorError, match="serial ownership retained"):
            sensor.PersistentSensor.start_session(root, "session_001", "/dev/never-opened", **kwargs)
        retained = sensor.FAILED_SESSION_OWNERS[-1]
        assert retained._session is factory.sessions[1]
        assert retained._lock_stream.closed is False
        assert len(factory.sessions) == 2
        assert [session.config.perform_sync for session in factory.sessions] == [False, True]
        assert factory.sessions[1].stop_calls == 1
        assert factory.sessions[1].staging_path.is_file()
        assert len(raw_rows(factory.sessions[1].staging_path)) == 3001
        readiness = json.loads((retained.root / "startup_readiness.json").read_text())
        assert readiness["status"] == "failed"
        assert "within 30 seconds" in readiness["error"]
        assert json.loads((retained.root / "startup_failure.json").read_text())["serial_ownership_retained"]
        with pytest.raises(BlockingIOError):
            sensor.PersistentSensor.start_session(root, "session_002", "/dev/never-opened", **kwargs)
        assert len(factory.sessions) == 2
    finally:
        release_test_retained_owners(previous)


def test_unjoinable_passive_reader_retains_port_lock_and_owner(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(
        clock,
        passive_error=NucleoCaptureError("Nucleo preflight timed out"),
        passive_stop_error=NucleoCaptureError("capture reader did not stop within join timeout"),
    )
    previous = list(sensor.FAILED_SESSION_OWNERS)
    kwargs = {
        "workspace": tmp_path / "workspace", "session_factory": factory,
        "boot_id_factory": lambda: "host-boot-one", "clock_ns": clock,
        "sleep": clock.sleep,
    }
    root = tmp_path / "workspace" / "runs" / "collection"
    try:
        with pytest.raises(sensor.SensorError, match="serial ownership retained"):
            sensor.PersistentSensor.start_session(root, "session_001", "/dev/never-opened", **kwargs)
        retained = sensor.FAILED_SESSION_OWNERS[-1]
        assert retained not in previous
        assert retained._session is factory.sessions[0]
        assert retained._lock_stream.closed is False
        assert factory.sessions[0].stop_calls == 1
        assert factory.sessions[0].config.perform_sync is False
        failure = json.loads((retained.root / "startup_failure.json").read_text())
        assert failure["serial_ownership_retained"] is True
        with pytest.raises(BlockingIOError):
            sensor.PersistentSensor.start_session(root, "session_002", "/dev/never-opened", **kwargs)
        assert len(factory.sessions) == 1
    finally:
        release_test_retained_owners(previous)


def test_unjoinable_close_retains_port_lock_and_owner(tmp_path: Path) -> None:
    device, factory, clock = start_fake(tmp_path)
    previous = list(sensor.FAILED_SESSION_OWNERS)
    factory.sessions[0].stop_error = NucleoCaptureError("capture reader did not stop within join timeout")
    try:
        with pytest.raises(sensor.SensorError, match="serial ownership retained"):
            device.close()
        assert device in sensor.FAILED_SESSION_OWNERS
        assert device._lock_stream.closed is False
        failure = json.loads((device.root / "shutdown_failure.json").read_text())
        assert failure["serial_ownership_retained"] is True
        with pytest.raises(BlockingIOError):
            sensor.PersistentSensor.start_session(
                tmp_path / "workspace" / "runs" / "collection", "session_002",
                "/dev/never-opened", workspace=tmp_path / "workspace",
                session_factory=factory, boot_id_factory=lambda: "host-boot-one",
                clock_ns=clock, sleep=clock.sleep,
            )
        assert len(factory.sessions) == 1
        assert factory.sessions[0].staging_path.is_file()
    finally:
        release_test_retained_owners(previous)


def test_robot_hardware_boot_metadata_must_match_explicit_session_epoch(tmp_path: Path) -> None:
    clock = FakeClock()
    factory = FakeSessionFactory(clock)
    device = sensor.PersistentSensor.start_session(
        tmp_path / "workspace" / "runs" / "collection", "session_001", "/dev/never-opened",
        workspace=tmp_path / "workspace", session_factory=factory,
        hardware_boot_id="board-001", boot_id_factory=lambda: "host-boot-one",
        clock_ns=clock, sleep=clock.sleep,
    )
    try:
        clock.sleep(0.2)
        robot = robot_telemetry(factory.sessions[0].workspace, "previous_board", [10_100_000_000])
        with pytest.raises(sensor.SensorError, match="hardware.*epoch|hardware.*boot"):
            device.finish_robot_window(robot, {"name": "previous_board", "hardware_boot_id": "board-000"})
        assert not (robot.parent / "nucleo_telemetry.csv").exists()
    finally:
        manifest = device.close()
    assert manifest["hardware_boot_id"] == "board-001"


def test_each_window_and_session_keep_hashed_clock_drift_report(tmp_path: Path) -> None:
    device, _, clock = start_fake(tmp_path)
    try:
        result = device.window("background_clock", duration_s=8,
                               metadata={"stationary_evidence": {"passed": True}})
        report_path = Path(result["clock_drift"]["path"])
        report = json.loads(report_path.read_text())
        assert report_path.name == "drift.json"
        assert result["clock_drift"]["sha256"] == sensor._hash(report_path)
        assert report["status"] == "observed"
        assert report["frames"] == result["frames"] == 800
        assert not report["timestamps_modified"]
        assert device.check_ready()["startup_baseline"]["ready"] is True
    finally:
        closed = device.close()
    assert Path(closed["clock_drift"]["path"]).name == "drift.json"
    assert json.loads(Path(closed["clock_drift"]["path"]).read_text())["status"] == "observed"


def test_dispatch_guard_preserves_raw_and_failure_before_raising(tmp_path: Path) -> None:
    from dataclasses import replace
    device, factory, clock = start_fake(tmp_path)
    session = factory.sessions[0]
    snapshot = session.snapshot
    clock.sleep(.1)
    original_bytes = session.snapshot().staging_path.read_bytes()
    report = {**sensor.DeviceClockDrift().report(), "status": "exceeded",
              "error": "channel_4_source: sustained receive-relative clock phase change exceeds budget"}
    session.snapshot = lambda: replace(snapshot(), clock_drift=report)
    try:
        for _ in range(2):
            with pytest.raises(sensor.SensorError, match="clock drift guard"):
                device.check_ready()
        failure = json.loads((device.root / "drift_failure.json").read_text())
        assert failure["status"] == "exceeded"
        assert failure["response"] == "refuse_next_dispatch_preserve_raw_no_blind_robot_motion"
        assert session.staging_path.read_bytes() == original_bytes
        assert session.stop_calls == 0
        assert session.start_calls == 1
        assert session.config.perform_sync is False
    finally:
        closed = device.close()
    assert closed["status"] == "incomplete"
    assert Path(closed["raw_csv"]).read_bytes() == original_bytes


def test_observed_writer_adds_diagnostics_without_changing_raw_bytes(tmp_path: Path) -> None:
    import io
    from hx_waypoint_nucleo.capture import _raw_row
    from hx_waypoint_nucleo.protocol import GroupedFrame, HubRecord, L5Payload
    frames = []
    for index in range(2):
        records = tuple(HubRecord(channel, index, 1000 + index * 10,
                                 L5Payload(index, ((1, -2, 3),) * 16, 3000 + index * 10),
                                 10_000_000_000 + index * 10_000_000)
                        for channel in range(1, 5))
        frames.append(GroupedFrame(records, index, 3000 + index * 10,
                                   10_000_000_000 + index * 10_000_000, index,
                                   {channel: index for channel in range(1, 5)},
                                   {channel: 1000 + index * 10 for channel in range(1, 5)},
                                   ParserStats(completed_groups=index + 1)))
    observed = sensor.ObservedSession(tmp_path, NucleoCaptureConfig(port="never-opened"), lambda: 99)
    observed._staging_path = tmp_path / "test.staging.csv"
    observed._stream = observed._staging_path.open("x", newline="", encoding="utf-8")
    observed._writer = csv.DictWriter(observed._stream, fieldnames=RAW_CAPTURE_COLUMNS, lineterminator="\n")
    observed._writer.writeheader()
    try:
        observed._write_frames(frames)
        snap = observed.snapshot()
        assert snap.clock_drift["frames"] == 2
        assert snap.clock_drift["status"] == "insufficient_observation"
        reference = io.StringIO(newline="")
        writer = csv.DictWriter(reference, fieldnames=RAW_CAPTURE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for frame in frames:
            writer.writerow(_raw_row(frame, sensor_rx_sync_time_ns=frame.host_monotonic_ns,
                                     sensor_rx_ros_time_ns=99))
        assert observed._staging_path.read_bytes() == reference.getvalue().encode()
    finally:
        observed._stream.close()


def test_drift_during_robot_goal_still_exports_incomplete_window(tmp_path: Path) -> None:
    from dataclasses import replace
    device, factory, clock = start_fake(tmp_path)
    session = factory.sessions[0]
    original_snapshot = session.snapshot
    clock.sleep(.2)
    robot = robot_telemetry(session.workspace, "goal_before_drift",
                            list(range(10_010_000_000, 10_200_000_001, 10_000_000)))
    original_raw = original_snapshot().staging_path.read_bytes()
    report = {**sensor.DeviceClockDrift().report(), "status": "exceeded",
              "error": "sustained source clock phase exceeds operating budget"}
    session.snapshot = lambda: replace(original_snapshot(), clock_drift=report)
    try:
        result = device.finish_robot_window(robot, {"name": "drift_during_goal"})
        assert result["status"] == "incomplete"
        assert not result["quality"]["accepted"]
        assert "clock drift guard" in result["session_sensor_guard_error"]
        assert result["frames"] == 20
        assert Path(result["raw_csv"]).is_file()
        assert Path(result["clock_drift"]["path"]).is_file()
        assert Path(result["mapping"]["path"]).is_file()
        assert (device.root / "drift_failure.json").is_file()
        assert session.staging_path.read_bytes() == original_raw
        with pytest.raises(sensor.SensorError, match="clock drift guard"):
            device.check_ready()
    finally:
        closed = device.close()
    assert closed["status"] == "incomplete"
    assert Path(closed["raw_csv"]).read_bytes() == original_raw
