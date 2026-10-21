"""Spawned capture contracts, using files and a fake backend, never hardware.

The fake is deliberately defined at module scope: ``spawn`` must import it in
the child rather than inheriting a patched function or a live reader thread.
"""

from __future__ import annotations

import csv
import fcntl
import multiprocessing
import os
from pathlib import Path
import signal
import struct
import threading
import time
from typing import Any

import pytest

from hx_waypoint_nucleo.capture import (
    NucleoCaptureConfig,
    NucleoCaptureError,
    NucleoCaptureSummary,
)
from hx_waypoint_nucleo.protocol import ParserStats


class UnpickleableBackendError(RuntimeError):
    """The transport must serialize the error text, not the exception object."""

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.unpickleable = lambda: None


class FakeBackend:
    """A real child-local writer thread, but no serial, ROS, or robot access."""

    def __init__(self, workspace: Path, config: NucleoCaptureConfig,
                 ros_time_ns_factory: Any) -> None:
        if config.port == "fake://constructor_error":
            raise UnpickleableBackendError("backend constructor failed before opening serial")
        self.workspace = Path(workspace)
        self.config = config
        self.ros_time_ns_factory = ros_time_ns_factory
        self.mode = config.port.removeprefix("fake://")
        self.staging_path = self.workspace / f"raw_{os.getpid()}.csv"
        self._started_sync_time_ns = time.monotonic_ns()
        self._sync_attempts = int(config.perform_sync)
        self._sync_time_ms_u32 = 12345 if config.perform_sync else None
        self._sync_rtt_ns = 654321 if config.perform_sync else None
        self.frames_written = 0
        self.last_frame_ns: int | None = None
        self.capture_error: str | None = None
        self._snapshot_count = 0
        self._stopped = threading.Event()
        self._rows_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stream = self.staging_path.open("x", encoding="utf-8", newline="")
        self._writer = csv.writer(self._stream, lineterminator="\n")
        self._writer.writerow(("receive_monotonic_ns", "receive_time_ns", "pid"))
        self._stream.flush()

    def start(self) -> FakeBackend:
        with self.workspace.joinpath("starts.txt").open("a", encoding="ascii") as stream:
            stream.write(f"{os.getpid()} {self.config.perform_sync}\n")
        if self.mode == "start_error":
            raise UnpickleableBackendError("preflight timed out; passive bytes=0")
        if self.mode == "hang_start":
            self._stopped.wait(60)
        self._write_one()
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()
        return self

    def _write_one(self) -> None:
        with self._rows_lock:
            now_ns = time.monotonic_ns()
            self._writer.writerow((now_ns, self.ros_time_ns_factory(), os.getpid()))
            self._stream.flush()
            self.frames_written += 1
            self.last_frame_ns = now_ns

    def _write_loop(self) -> None:
        while not self._stopped.wait(0.005):
            if self.mode != "frozen":
                self._write_one()

    def snapshot(self) -> Any:
        from workstation_magtrack.collection.sensor import SensorSnapshot

        self._snapshot_count += 1
        if self.mode == "snapshot_error" and self._snapshot_count > 1:
            raise UnpickleableBackendError("backend snapshot failure with evidence intact")
        if self.mode == "die_snapshot" and self._snapshot_count > 1:
            os._exit(73)
        if self.mode == "hang_snapshot" and self._snapshot_count > 1:
            self._stopped.wait(60)
        with self._rows_lock:
            return SensorSnapshot(
                staging_path=self.staging_path,
                byte_count=self.staging_path.stat().st_size,
                frames_written=self.frames_written,
                parser_stats=ParserStats(bytes_received=self.frames_written * 100,
                                         completed_groups=self.frames_written),
                last_frame_ns=self.last_frame_ns,
                started_sync_time_ns=self._started_sync_time_ns,
                capture_error=self.capture_error,
                clock_drift={"child_pid": os.getpid(),
                             "child_start_method": multiprocessing.get_start_method()},
            )

    def stop(self) -> NucleoCaptureSummary:
        with self.workspace.joinpath("stop_calls.txt").open("a", encoding="ascii") as stream:
            stream.write(f"{os.getpid()}\n")
        if (self.workspace / "serial.lock").exists():
            with (self.workspace / "serial.lock").open("a") as probe:
                try:
                    fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    lock_state = "released_before_stop"
                except BlockingIOError:
                    lock_state = "held_during_stop"
            self.workspace.joinpath("stop_lock_state.txt").write_text(lock_state, encoding="ascii")
        if self.mode == "hang_stop":
            self._stopped.wait(60)
        if self.mode == "delayed_stop":
            time.sleep(0.15)
        self._stopped.set()
        if self._thread is not None:
            self._thread.join(2)
            assert not self._thread.is_alive()
        self._stream.close()
        self.workspace.joinpath("stopped.txt").write_text(str(os.getpid()), encoding="ascii")
        return NucleoCaptureSummary(
            staging_path=self.staging_path, config=self.config, workspace=self.workspace,
            started_sync_time_ns=self._started_sync_time_ns,
            stopped_sync_time_ns=time.monotonic_ns(), frames_written=self.frames_written,
            parser_stats=ParserStats(bytes_received=self.frames_written * 100,
                                     completed_groups=self.frames_written),
            capture_error=self.capture_error, sync_attempts=self._sync_attempts,
            sync_time_ms_u32=self._sync_time_ms_u32, sync_rtt_ns=self._sync_rtt_ns,
        )


def _make_session(tmp_path: Path, mode: str = "normal", **kwargs: Any) -> Any:
    from workstation_magtrack.collection.capture_process import ProcessObservedSession

    return ProcessObservedSession(
        tmp_path, NucleoCaptureConfig(port=f"fake://{mode}", perform_sync=False),
        time.time_ns, _session_factory=FakeBackend, _start_timeout_s=5.0,
        _request_timeout_s=0.5, _stop_timeout_s=0.5, **kwargs,
    )


def _force_cleanup(session: Any) -> None:
    """Test-only cleanup of deliberately dead/hung children; never production."""
    process = getattr(session, "_process", None)
    if process is not None:
        if process.is_alive():
            process.terminate()
        process.join(3)
        assert not process.is_alive()


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_capture_uses_spawned_child_and_child_timestamps(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "frozen")
    try:
        session.start()
        first = session.snapshot()
        time.sleep(0.05)
        second = session.snapshot()
        assert first.last_frame_ns == second.last_frame_ns
        assert first.started_sync_time_ns == second.started_sync_time_ns
        assert second.last_frame_ns < time.monotonic_ns() - 30_000_000
        identity = session.process_identity
        assert identity["parent_pid"] == os.getpid()
        assert identity["capture_pid"] != os.getpid()
        assert identity["start_method"] == "spawn"
        assert second.clock_drift["child_pid"] == identity["capture_pid"]
        assert second.clock_drift["child_start_method"] == "spawn"
        rows = _rows(second.staging_path)
        assert int(rows[-1]["receive_monotonic_ns"]) == second.last_frame_ns
        # The child owns the original wall-clock factory used by production;
        # the parent's RPC receipt time never replaces a sensor timestamp.
        assert int(rows[-1]["receive_time_ns"]) > 1_000_000_000_000_000_000
        assert {int(row["pid"]) for row in rows} == {identity["capture_pid"]}
        summary = session.stop()
        assert summary.frames_written == second.frames_written
        assert summary.staging_path == second.staging_path
        assert session.snapshot().last_frame_ns == second.last_frame_ns
    finally:
        _force_cleanup(session)


def test_unsupported_ros_callback_is_rejected_before_spawn(tmp_path: Path) -> None:
    from workstation_magtrack.collection.capture_process import ProcessObservedSession

    with pytest.raises(NucleoCaptureError, match="requires time.time_ns"):
        ProcessObservedSession(tmp_path, NucleoCaptureConfig(port="fake://normal"),
                               lambda: -1, _session_factory=FakeBackend)
    assert list(tmp_path.iterdir()) == []


def test_sync_metadata_exactly_roundtrips_and_start_is_not_repeatable(tmp_path: Path) -> None:
    from workstation_magtrack.collection.capture_process import ProcessObservedSession

    session = ProcessObservedSession(
        tmp_path, NucleoCaptureConfig(port="fake://normal", perform_sync=True,
                                    sync_retries=1), time.time_ns,
        _session_factory=FakeBackend, _start_timeout_s=5.0,
        _request_timeout_s=0.5, _stop_timeout_s=0.5,
    )
    try:
        session.start()
        assert session._sync_attempts == 1
        assert session._sync_time_ms_u32 == 12345
        assert session._sync_rtt_ns == 654321
        with pytest.raises(NucleoCaptureError, match="already been started"):
            session.start()
        session.snapshot()
        summary = session.stop()
        assert (summary.sync_attempts, summary.sync_time_ms_u32, summary.sync_rtt_ns) == (
            1, 12345, 654321,
        )
        assert len((tmp_path / "starts.txt").read_text().splitlines()) == 1
        assert session.stop() == summary
        assert session.snapshot().frames_written == summary.frames_written
    finally:
        _force_cleanup(session)


def test_child_keeps_appending_while_parent_is_cpu_busy(tmp_path: Path) -> None:
    session = _make_session(tmp_path)
    try:
        session.start()
        first = session.snapshot()
        deadline = time.monotonic() + 0.6
        value = 0
        while time.monotonic() < deadline:
            value = (value * 1664525 + 1013904223) & 0xFFFFFFFF
        second = session.snapshot()
        assert value >= 0
        assert second.frames_written - first.frames_written >= 30
        assert second.byte_count > first.byte_count
        session.stop()
        rows = _rows(second.staging_path)
        times = [int(row["receive_monotonic_ns"]) for row in rows]
        assert max(b - a for a, b in zip(times, times[1:])) < 150_000_000
    finally:
        _force_cleanup(session)


def test_passive_start_failure_preserves_exact_error_and_stop_evidence(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "start_error")
    try:
        with pytest.raises(NucleoCaptureError, match="preflight timed out; passive bytes=0"):
            session.start()
        summary = session.stop()
        assert summary.parser_stats.bytes_received == 0
        assert summary.frames_written == 0
        assert summary.staging_path.is_file()
        assert summary.sync_attempts == 0
        assert (tmp_path / "stopped.txt").is_file()
        starts = (tmp_path / "starts.txt").read_text().splitlines()
        assert len(starts) == 1
        assert starts[0].endswith("False")
    finally:
        _force_cleanup(session)


def test_unpickleable_snapshot_error_is_serialized_without_fallback(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "snapshot_error")
    try:
        session.start()
        with pytest.raises(NucleoCaptureError, match="backend snapshot failure with evidence intact"):
            session.snapshot()
        summary = session.stop()
        assert summary.staging_path.is_file()
        assert len((tmp_path / "starts.txt").read_text().splitlines()) == 1
    finally:
        _force_cleanup(session)


def test_constructor_failure_stop_errors_without_fabricated_summary(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "constructor_error")
    try:
        with pytest.raises(NucleoCaptureError, match="constructor failed before opening serial"):
            session.start()
        with pytest.raises(NucleoCaptureError, match="constructor failed before opening serial"):
            session.stop()
        session._process.join(1)
        assert not session._process.is_alive()
        assert session._process.exitcode == 0
        assert session._summary is None
        assert session.staging_path is None
        assert list(tmp_path.iterdir()) == []
    finally:
        _force_cleanup(session)


def test_late_stop_reply_is_reused_without_second_backend_stop(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "delayed_stop")
    try:
        session.start()
        session._stop_timeout = 0.05
        with pytest.raises(NucleoCaptureError, match="timed out"):
            session.stop()
        time.sleep(0.25)
        summary = session.stop()
        assert summary.capture_error is None
        assert summary.staging_path.is_file()
        assert session._process.exitcode == 0
        assert len((tmp_path / "stop_calls.txt").read_text().splitlines()) == 1
        assert session.snapshot().frames_written == summary.frames_written
        assert session.stop() == summary
    finally:
        _force_cleanup(session)


def test_unexpected_child_death_is_detected_and_never_restarted(tmp_path: Path) -> None:
    session = _make_session(tmp_path, "die_snapshot")
    try:
        session.start()
        started = time.monotonic()
        with pytest.raises(NucleoCaptureError):
            session.snapshot()
        assert time.monotonic() - started < 2
        with pytest.raises(NucleoCaptureError):
            session.snapshot()
        assert len((tmp_path / "starts.txt").read_text().splitlines()) == 1
    finally:
        _force_cleanup(session)


@pytest.mark.parametrize("mode", ["hang_snapshot", "hang_stop"])
def test_unresponsive_rpc_is_bounded_and_retains_serial_lock(tmp_path: Path, mode: str) -> None:
    lock_path = tmp_path / "serial.lock"
    owner = lock_path.open("a")
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = _make_session(tmp_path, mode, ownership_fd=owner.fileno())
    contender = None
    try:
        session.start()
        owner.close()
        contender = lock_path.open("a")
        with pytest.raises(BlockingIOError):
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
        operation = session.snapshot if mode == "hang_snapshot" else session.stop
        started = time.monotonic()
        with pytest.raises(NucleoCaptureError):
            operation()
        assert time.monotonic() - started < 2
        started = time.monotonic()
        with pytest.raises(NucleoCaptureError):
            session.stop()
        assert time.monotonic() - started < 2
        if mode == "hang_stop":
            assert len((tmp_path / "stop_calls.txt").read_text().splitlines()) == 1
        # An RPC timeout must not free ownership while a live child still
        # has the writer/serial backend. No retry can open a second reader.
        if session._process.is_alive():
            with pytest.raises(BlockingIOError):
                fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        owner.close()
        _force_cleanup(session)
        if contender is not None:
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
            contender.close()


def test_serial_lock_survives_parent_fd_close_until_clean_stop(tmp_path: Path) -> None:
    lock_path = tmp_path / "serial.lock"
    owner = lock_path.open("a")
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = _make_session(tmp_path, ownership_fd=owner.fileno())
    try:
        session.start()
        owner.close()
        with lock_path.open("a") as contender:
            with pytest.raises(BlockingIOError):
                fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
            session.stop()
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        owner.close()
        _force_cleanup(session)


def _parent_that_exits(connection: Any, workspace: str) -> None:
    root = Path(workspace)
    owner = (root / "serial.lock").open("a")
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = _make_session(root, ownership_fd=owner.fileno())
    session.start()
    owner.close()
    connection.send(session.process_identity)
    connection.close()
    # Bypass all parent destructors. Child must observe its command-pipe EOF
    # and stop the fake writer just as it must close the real serial reader.
    os._exit(0)


def test_parent_death_eof_stops_child_and_releases_ownership(tmp_path: Path) -> None:
    context = multiprocessing.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    parent = context.Process(target=_parent_that_exits, args=(sender, str(tmp_path)))
    parent.start()
    sender.close()
    identity: dict[str, Any] | None = None
    try:
        assert receiver.poll(8), "nested parent failed to start fake capture"
        identity = receiver.recv()
        assert identity["capture_pid"] != identity["parent_pid"]
        parent.join(3)
        assert parent.exitcode == 0
        deadline = time.monotonic() + 5
        while not (tmp_path / "stopped.txt").exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert (tmp_path / "stopped.txt").is_file(), "orphan capture did not stop on EOF"
        assert (tmp_path / "stop_lock_state.txt").read_text() == "held_during_stop"
        with (tmp_path / "serial.lock").open("a") as contender:
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        receiver.close()
        if parent.is_alive():
            parent.terminate()
        parent.join(3)
        if identity is not None and not (tmp_path / "stopped.txt").exists():
            # Only a deliberately synthetic orphan created by this test can
            # reach here; never kill an application or physical reader.
            try:
                os.kill(identity["capture_pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass


def test_partial_rpc_body_has_a_bounded_deadline() -> None:
    from workstation_magtrack.collection.capture_process import _Channel

    parent, child = multiprocessing.get_context("spawn").Pipe(duplex=True)
    try:
        channel = _Channel(parent)
        os.write(child.fileno(), struct.pack("!I", 100) + b"only a partial payload")
        started = time.monotonic()
        with pytest.raises(TimeoutError):
            channel.receive(time.monotonic() + 0.05)
        assert time.monotonic() - started < 1
    finally:
        parent.close()
        child.close()


def test_blocked_rpc_send_has_a_bounded_deadline() -> None:
    from workstation_magtrack.collection.capture_process import _Channel

    parent, child = multiprocessing.get_context("spawn").Pipe(duplex=True)
    try:
        channel = _Channel(parent)
        started = time.monotonic()
        with pytest.raises(TimeoutError):
            channel.send(b"x" * 1_000_000, time.monotonic() + 0.05)
        assert time.monotonic() - started < 1
    finally:
        parent.close()
        child.close()


@pytest.mark.parametrize("declared_size", [0, 1_048_577])
def test_invalid_rpc_size_is_rejected_without_unbounded_read(declared_size: int) -> None:
    from workstation_magtrack.collection.capture_process import _Channel

    parent, child = multiprocessing.get_context("spawn").Pipe(duplex=True)
    try:
        channel = _Channel(parent)
        os.write(child.fileno(), struct.pack("!I", declared_size))
        with pytest.raises(NucleoCaptureError, match="invalid.*size"):
            channel.receive(time.monotonic() + 0.05)
    finally:
        parent.close()
        child.close()
