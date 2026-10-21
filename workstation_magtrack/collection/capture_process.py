"""Spawn-isolated serial capture; control traffic never carries sensor rows.

The child owns the existing reader, parser, drift guard and append-only writer.
The controller requests lightweight snapshots only.  It never opens serial or
reconstructs receive timestamps, and a failed RPC never permits an in-process
fallback.  Nonblocking framed IPC also bounds partial-message/dead-child waits.
"""

from __future__ import annotations

import multiprocessing
from multiprocessing.connection import Connection
from multiprocessing.reduction import DupFd
from multiprocessing.util import Finalize
import os
from pathlib import Path
import pickle
import select
import struct
import threading
import time
from typing import Any, Callable, TYPE_CHECKING

from hx_waypoint_nucleo.capture import (
    NucleoCaptureConfig, NucleoCaptureError, NucleoCaptureSummary,
)

if TYPE_CHECKING:
    from .sensor import SensorSnapshot


class _Channel:
    """One trusted local peer, one outstanding request, bounded reads/writes."""

    _MAX_PACKET = 1_048_576

    def __init__(self, connection: Connection) -> None:
        self.connection = connection
        self.fd = connection.fileno()
        os.set_blocking(self.fd, False)
        self.incoming = bytearray()
        self.outgoing = b""

    def _wait(self, *, reading: bool, deadline: float) -> None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("capture process IPC timed out")
        select.select([self.fd] if reading else [], [] if reading else [self.fd], [],
                      min(remaining, 0.05))

    def send(self, value: Any, deadline: float) -> None:
        if self.outgoing:
            raise RuntimeError("capture process already has a partial outgoing packet")
        payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload) > self._MAX_PACKET:
            raise NucleoCaptureError("capture process control packet exceeds size bound")
        self.outgoing = struct.pack("!I", len(payload)) + payload
        self.flush(deadline)

    def flush(self, deadline: float) -> None:
        while self.outgoing:
            if time.monotonic() >= deadline:
                raise TimeoutError("capture process IPC send timed out")
            try:
                sent = os.write(self.fd, self.outgoing)
                if sent == 0:
                    raise EOFError("capture process control pipe closed during send")
                self.outgoing = self.outgoing[sent:]
            except BlockingIOError:
                self._wait(reading=False, deadline=deadline)

    def receive(self, deadline: float) -> Any:
        while True:
            if len(self.incoming) >= 4:
                size = struct.unpack("!I", self.incoming[:4])[0]
                if not 0 < size <= self._MAX_PACKET:
                    raise NucleoCaptureError("invalid capture process control packet size")
                if len(self.incoming) >= 4 + size:
                    packet = bytes(self.incoming[4:4 + size])
                    del self.incoming[:4 + size]
                    return pickle.loads(packet)
            if time.monotonic() >= deadline:
                raise TimeoutError("capture process IPC receive timed out")
            try:
                chunk = os.read(self.fd, 65_536)
                if not chunk:
                    raise EOFError("capture process control pipe closed")
                self.incoming.extend(chunk)
            except BlockingIOError:
                self._wait(reading=True, deadline=deadline)


def _metadata(session: Any) -> dict[str, Any]:
    return {
        "staging_path": getattr(session, "staging_path", None),
        "sync_attempts": getattr(session, "_sync_attempts", 0),
        "sync_time_ms_u32": getattr(session, "_sync_time_ms_u32", None),
        "sync_rtt_ns": getattr(session, "_sync_rtt_ns", None),
    }


def _capture_child(
    connection: Connection, workspace: Path, config: NucleoCaptureConfig,
    ownership: Any, factory: Callable[..., Any] | None,
) -> None:
    """Spawn entry: deliberately imports no ROS or motion modules."""
    channel = _Channel(connection)
    ownership_fd = ownership.detach() if ownership is not None else None
    session = None
    constructor_error: str | None = None
    stopped = False
    try:
        while True:
            try:
                request = channel.receive(time.monotonic() + 1.0)
            except TimeoutError:
                continue
            command = request.get("command")
            response: dict[str, Any] = {"id": request.get("id"), "ok": False}
            try:
                if command == "start":
                    if session is not None:
                        raise NucleoCaptureError("capture process has already been started")
                    if factory is None:
                        from .sensor import ObservedSession
                        factory = ObservedSession
                    session = factory(workspace, config, time.time_ns)
                    session.start()
                    response["snapshot"] = session.snapshot()
                elif command == "snapshot":
                    if session is None:
                        raise NucleoCaptureError("capture process session is unavailable")
                    response["snapshot"] = session.snapshot()
                elif command == "stop":
                    if session is None:
                        # ObservedSession construction is resource-free: serial
                        # opens only inside start().  Do not strand an empty
                        # child or fabricate raw evidence/a capture summary.
                        stopped = True
                        raise NucleoCaptureError(constructor_error or "capture session was never constructed")
                    else:
                        response["summary"] = session.stop()
                    stopped = True
                    try:
                        response["snapshot"] = session.snapshot()
                    except Exception as exc:
                        # Setup/preflight failures may have no staging file.
                        # The stopped summary still preserves that failure.
                        response["final_snapshot_error"] = str(exc)
                else:
                    raise NucleoCaptureError("invalid capture process command")
                response["ok"] = True
            except BaseException as exc:
                if command == "start" and session is None:
                    constructor_error = f"{type(exc).__name__}: {exc}"
                response["error"] = str(exc)
                response["error_type"] = type(exc).__name__
            response["metadata"] = _metadata(session)
            channel.send(response, time.monotonic() + 5.0)
            if stopped:
                return
    except (EOFError, BrokenPipeError, OSError, TimeoutError):
        # The parent died, closed control, or ceased consuming replies.  Never
        # keep collecting indefinitely without its owner and never drop a lock
        # while the reader is still alive.
        pass
    finally:
        if session is not None and not stopped:
            while not stopped:
                try:
                    session.stop()
                    stopped = True
                except BaseException:
                    # No forceful termination and no clean-stop claim.  Keep
                    # this process and the shared flock alive until stop works.
                    time.sleep(0.2)
        if ownership_fd is not None:
            os.close(ownership_fd)
        connection.close()


class ProcessObservedSession:
    """Drop-in observed session with a dedicated ``spawn`` capture process.

    Private factory/timeouts exist only for deterministic no-hardware tests.
    The production clock is strictly ``time.time_ns``; a ROS callback cannot
    safely be copied into a child and is therefore rejected, never invoked.
    """

    def __init__(
        self, workspace: str | Path, config: NucleoCaptureConfig,
        ros_time_ns_factory: Callable[[], int], *, ownership_fd: int | None = None,
        _session_factory: Callable[..., Any] | None = None,
        _start_timeout_s: float | None = None, _request_timeout_s: float = 5.0,
        _stop_timeout_s: float = 10.0,
    ) -> None:
        if not isinstance(config, NucleoCaptureConfig):
            raise TypeError("config must be a NucleoCaptureConfig")
        if ros_time_ns_factory is not time.time_ns:
            raise NucleoCaptureError("isolated capture requires time.time_ns; ROS clock callbacks are unsupported")
        self.workspace = Path(workspace).expanduser().resolve()
        self.config = config
        self.ros_time_ns_factory = time.time_ns
        self._ownership_fd = ownership_fd
        self._factory = _session_factory
        self._start_timeout = _start_timeout_s if _start_timeout_s is not None else (
            10.0 + config.settle_time_s + config.sync_timeout_s +
            config.write_timeout_s + config.preflight_timeout_s
        )
        self._request_timeout = _request_timeout_s
        self._stop_timeout = _stop_timeout_s
        for value in (self._start_timeout, self._request_timeout, self._stop_timeout):
            if not isinstance(value, (int, float)) or not 0 < value < float("inf"):
                raise ValueError("capture process timeouts must be finite and positive")
        self._process: Any = None
        self._connection: Connection | None = None
        self._control_finalizer: Any = None
        self._channel: _Channel | None = None
        self._lock = threading.Lock()
        self._started = False
        self._next_id = 0
        self._pending_id: int | None = None
        self._pending_command: str | None = None
        self._summary: NucleoCaptureSummary | None = None
        self._stop_verified = False
        self._last_snapshot: SensorSnapshot | None = None
        self._final_snapshot_error: str | None = None
        self.staging_path: Path | None = None
        self._sync_attempts = 0
        self._sync_time_ms_u32: int | None = None
        self._sync_rtt_ns: int | None = None
        self.process_identity = {
            "parent_pid": os.getpid(), "capture_pid": None, "start_method": "spawn",
            "timestamp_policy": "child_time.time_ns_and_original_receive_monotonic_ns",
            "serial_parser_and_raw_writer_owner": "capture_process",
        }

    def _remember(self, response: dict[str, Any]) -> None:
        metadata = response.get("metadata", {})
        if metadata.get("staging_path") is not None:
            self.staging_path = Path(metadata["staging_path"])
        self._sync_attempts = metadata.get("sync_attempts", self._sync_attempts)
        self._sync_time_ms_u32 = metadata.get("sync_time_ms_u32", self._sync_time_ms_u32)
        self._sync_rtt_ns = metadata.get("sync_rtt_ns", self._sync_rtt_ns)
        if "snapshot" in response:
            self._last_snapshot = response["snapshot"]
        if "summary" in response:
            self._summary = response["summary"]
        self._final_snapshot_error = response.get("final_snapshot_error")

    def _finish_pending(self, deadline: float) -> dict[str, Any]:
        assert self._channel is not None
        self._channel.flush(deadline)
        response = self._channel.receive(deadline)
        if not isinstance(response, dict) or response.get("id") != self._pending_id:
            raise NucleoCaptureError("capture process reply identity mismatch")
        self._pending_id = None
        self._remember(response)
        return response

    def _request(self, command: str, timeout: float) -> dict[str, Any]:
        if self._process is None or self._channel is None:
            raise NucleoCaptureError("capture process has not been started")
        deadline = time.monotonic() + timeout
        try:
            if self._pending_id is not None:
                # A timed-out snapshot may still complete.  Drain its exact
                # reply before stop; never overlap or misattribute requests.
                previous_command = self._pending_command
                response = self._finish_pending(deadline)
                if previous_command == "stop" and command == "stop":
                    if not response.get("ok"):
                        raise NucleoCaptureError(response.get("error") or "capture process stop failed")
                    return response
            self._next_id += 1
            self._pending_id = self._next_id
            self._pending_command = command
            self._channel.send({"id": self._pending_id, "command": command}, deadline)
            response = self._finish_pending(deadline)
        except (EOFError, OSError, TimeoutError) as exc:
            raise NucleoCaptureError(
                f"capture process {command} failed: {exc}; serial ownership unresolved"
            ) from exc
        if not response.get("ok"):
            raise NucleoCaptureError(response.get("error") or "capture process operation failed")
        return response

    def start(self) -> ProcessObservedSession:
        with self._lock:
            if self._started:
                raise NucleoCaptureError("capture session has already been started")
            self._started = True
            context = multiprocessing.get_context("spawn")
            parent, child = context.Pipe(duplex=True)
            self._connection = parent
            # multiprocessing otherwise joins non-daemon children before its
            # own objects are collected.  Close control first on normal parent
            # exit too, so the child's EOF cleanup can stop/join its reader.
            self._control_finalizer = Finalize(self, parent.close, exitpriority=20)
            self._channel = _Channel(parent)
            ownership = DupFd(self._ownership_fd) if self._ownership_fd is not None else None
            self._process = context.Process(
                target=_capture_child,
                args=(child, self.workspace, self.config, ownership, self._factory),
                name="hx-nucleo-capture-process", daemon=False,
            )
            try:
                self._process.start()
            except BaseException as exc:
                parent.close()
                raise NucleoCaptureError(f"capture process could not start: {exc}") from exc
            finally:
                child.close()
            self.process_identity["capture_pid"] = self._process.pid
            self._request("start", self._start_timeout)
            return self

    def snapshot(self) -> SensorSnapshot:
        with self._lock:
            if self._summary is not None:
                if not self._stop_verified:
                    raise NucleoCaptureError("capture process stop has not been joined; serial ownership unresolved")
                if self._final_snapshot_error or self._last_snapshot is None:
                    raise NucleoCaptureError(self._final_snapshot_error or "final capture snapshot unavailable")
                return self._last_snapshot
            response = self._request("snapshot", self._request_timeout)
            return response["snapshot"]

    def stop(self) -> NucleoCaptureSummary:
        with self._lock:
            if not self._started or self._process is None:
                raise NucleoCaptureError("capture session has not been started")
            deadline = time.monotonic() + self._stop_timeout
            if self._summary is None:
                self._request("stop", self._stop_timeout)
            # The stop ACK alone is insufficient: the process must have exited
            # and released its inherited ownership fd before we certify stop.
            self._process.join(timeout=max(0.0, deadline - time.monotonic()))
            if self._process.is_alive():
                raise NucleoCaptureError("capture process did not exit after stop; serial ownership retained")
            if self._process.exitcode != 0 or self._summary is None:
                raise NucleoCaptureError("capture process exited without a verified clean stop; serial ownership unresolved")
            if self._connection is not None:
                self._connection.close()
            self._stop_verified = True
            return self._summary
