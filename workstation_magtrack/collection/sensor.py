"""One NUCLEO reader per explicitly acknowledged hardware epoch.

The controller never owns the serial port: invoke ``tipdraw play`` WITHOUT
``--record-nucleo`` and map its actual execution telemetry afterwards.  Windows
are copies of an append-only raw stream; the standard finalizer may delete only
its disposable input, never the session evidence.  Production capture lives in
a spawned process, not a thread sharing the controller's Python GIL.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
from typing import Any, Callable
import uuid

if __package__:
    from .drift import DeviceClockDrift
else:  # File-loaded test/development harness, not the production module CLI.
    from workstation_magtrack.collection.drift import DeviceClockDrift

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "src" / "hx_waypoint_nucleo"))
from hx_waypoint_nucleo.capture import (  # noqa: E402
    NucleoCaptureConfig, NucleoCaptureError, NucleoCaptureSession,
    NucleoCaptureSummary, RAW_CAPTURE_COLUMNS, _raw_row, finalize_nucleo_capture,
)
from hx_waypoint_nucleo.protocol import ParserStats  # noqa: E402

# A failed startup cannot return its owner to the caller.  Retain a strong
# reference if its reader cannot be joined, so GC cannot release the serial
# lock while that reader might still be alive.  Operator intervention/process
# shutdown is then required; no second connection is silently attempted.
FAILED_SESSION_OWNERS: list[Any] = []


class SensorError(NucleoCaptureError):
    """Evidence or epoch checks failed; do not continue collection."""


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _label(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise SensorError("session/window name must be a safe nonempty label")
    return value


def _write_json(path: Path, value: dict[str, Any]) -> None:
    # New session/window directories are exclusive; no artifact is overwritten.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def _host_boot_id() -> str:
    linux = Path("/proc/sys/kernel/random/boot_id")
    if linux.is_file():
        return "linux:" + linux.read_text(encoding="ascii").strip()
    # Read-only clock identity for development on macOS, not a fake Linux epoch.
    result = subprocess.run(["sysctl", "-n", "kern.boottime"], capture_output=True,
                            text=True, check=True)
    if not result.stdout.strip():
        raise SensorError("host boot identity is unavailable")
    return "darwin:" + result.stdout.strip()


@dataclass(frozen=True)
class SensorSnapshot:
    staging_path: Path
    byte_count: int
    frames_written: int
    parser_stats: ParserStats
    last_frame_ns: int | None
    started_sync_time_ns: int
    capture_error: str | None = None
    clock_drift: dict[str, Any] | None = None


class ObservedSession(NucleoCaptureSession):
    """Flushable append-only reader; no unconsumed, bounded frame queue."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._rows_lock = threading.Lock()
        self._latest_frame_ns: int | None = None
        self._device_times: dict[int, int] = {}
        self._clock_drift = DeviceClockDrift(
            self.config.expected_channels, require_stable_start=True,
        )

    def _write_frames(self, frames: Any) -> None:
        with self._rows_lock:
            for frame in frames:
                if self._stop_event.is_set():
                    break
                for record in frame.records:
                    current = int(record.nucleo_time_ms_u32)
                    previous = self._device_times.get(record.channel_id)
                    if previous is not None and ((current - previous) & 0xFFFFFFFF) >= 0x80000000:
                        self._record_error("device clock regressed: reset/reorder requires a new explicit epoch")
                    self._device_times[record.channel_id] = current
                # Diagnostics observe the very same input.  They do not alter
                # either clock, any magnetic sample, the parser, or the writer.
                self._clock_drift.update_frame(frame)
                self._writer.writerow(_raw_row(
                    frame, sensor_rx_sync_time_ns=frame.host_monotonic_ns,
                    sensor_rx_ros_time_ns=int(self.ros_time_ns_factory()),
                ))
                self._frames_written += 1
                self._latest_frame_ns = frame.host_monotonic_ns
                self._first_frame_event.set()
            self._stream.flush()

    def snapshot(self) -> SensorSnapshot:
        with self._rows_lock:
            if self._stream is not None and not self._stream.closed:
                self._stream.flush()
            if self.staging_path is None:
                raise SensorError("sensor staging file is unavailable")
            return SensorSnapshot(
                self.staging_path, self.staging_path.stat().st_size,
                self.frames_written, self._parser.stats, self._latest_frame_ns,
                self._started_sync_time_ns, self.capture_error,
                self._clock_drift.report(),
            )


class PersistentSensor:
    """Capture one board boot; close before user resets, then create a NEW ID."""

    @classmethod
    def start_session(
        cls, run_root: str | Path, session_id: str, port: str, *,
        workspace: str | Path | None = None,
        hardware_boot_id: str | None = None,
        allow_initial_sync: bool = True,
        session_factory: Callable[..., Any] | None = None,
        boot_id_factory: Callable[[], str] | None = None,
        clock_ns: Callable[[], int] | None = None,
        sleep: Callable[[float], None] | None = None,
    ) -> "PersistentSensor":
        if not isinstance(allow_initial_sync, bool):
            raise ValueError("allow_initial_sync must be a boolean")
        self = cls()
        self.allow_initial_sync = allow_initial_sync
        self.run_root = Path(run_root).expanduser().resolve()
        self.workspace = Path(workspace).expanduser().resolve() if workspace else next(
            (parent.parent for parent in (self.run_root, *self.run_root.parents) if parent.name == "runs"),
            self.run_root.parent,
        )
        if not self.run_root.is_relative_to(self.workspace / "runs"):
            raise SensorError("run_root must be under workspace/runs")
        self.session_id = _label(session_id)
        self.hardware_boot_id = _label(hardware_boot_id) if hardware_boot_id is not None else None
        self.epoch_id = f"{self.session_id}__{uuid.uuid4().hex}"
        self._boot_id_factory = boot_id_factory or _host_boot_id
        self.host_boot_id = self._boot_id_factory()
        if not self.host_boot_id:
            raise SensorError("host boot identity is empty")
        self._clock = clock_ns or time.monotonic_ns
        self._sleep = sleep or time.sleep
        self.root = self.run_root / "sensor_sessions" / self.session_id
        self.root.mkdir(parents=True, exist_ok=False)
        (self.root / "windows").mkdir()
        self._lock_stream = None
        self._session = None
        self._ownership_retained = False
        self._closed = False
        self._last_window_end_ns = 0
        self._results: list[dict[str, Any]] = []
        self._started_utc = _utc()
        self._start_ns = self._clock()
        self.sync_mode = "not_established"
        self._startup_drift_report: dict[str, Any] | None = None
        self._startup_wait_started_ns: int | None = None
        # Injection is for deterministic tests.  Production must never silently
        # fall back to the controller's process, including on spawn/IPC failure.
        factory = session_factory or self._new_process_session
        try:
            lock_id = hashlib.sha256(str(Path(port).resolve()).encode()).hexdigest()[:20]
            self._lock_stream = (self.workspace / "runs" / f".collection_nucleo_{lock_id}.lock").open("a")
            fcntl.flock(self._lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            passive = factory(self.workspace, NucleoCaptureConfig(
                port=port, perform_sync=False, preflight_timeout_s=2.0,
                sync_retries=1,
            ), time.time_ns)
            self._session = passive
            try:
                passive.start()
                self._session, self.sync_mode = passive, "reused_existing_stream"
            except NucleoCaptureError as exc:
                try:
                    summary = passive.stop()
                except BaseException as stop_error:
                    self._retain_failed_owner(stop_error)
                    raise SensorError("passive reader could not stop; serial ownership retained") from stop_error
                self._session = None
                probe = asdict(summary.parser_stats)
                if summary.staging_path.is_file():
                    summary.staging_path.rename(self.root / "passive_probe_raw.csv")
                _write_json(self.root / "passive_probe.json", {"error": str(exc), "parser": probe})
                if probe["bytes_received"] or "preflight timed out" not in str(exc):
                    raise SensorError("passive probe failed; refusing another SYNC") from exc
                if not self.allow_initial_sync:
                    raise SensorError("passive stream unavailable; initial SYNC is forbidden for this boot") from exc
                active = factory(self.workspace, NucleoCaptureConfig(
                    port=port, perform_sync=True, sync_retries=1,
                ), time.time_ns)
                self._session = active
                active.start()
                self.sync_mode = "performed_once"
            self._await_startup_ready()
            _write_json(self.root / "epoch.json", self._identity())
            return self
        except BaseException as startup_error:
            try:
                self._persist_startup_readiness("failed", error=startup_error)
            finally:
                if not self._ownership_retained:
                    try:
                        if self._session is not None:
                            try:
                                summary = self._session.stop()
                            except BaseException as stop_error:
                                self._retain_failed_owner(stop_error)
                                raise SensorError("startup reader could not stop; serial ownership retained") from stop_error
                            self._preserve_failed_startup(summary, startup_error)
                    finally:
                        if self._lock_stream is not None and not self._ownership_retained:
                            self._lock_stream.close()
            raise

    def _new_process_session(
        self, workspace: Path, config: NucleoCaptureConfig,
        ros_time_ns_factory: Callable[[], int],
    ) -> Any:
        # Import lazily so file-loaded unit tests can still inject their fake
        # session without spawning or touching the serial port.
        if __package__:
            from .capture_process import ProcessObservedSession
        else:
            from workstation_magtrack.collection.capture_process import ProcessObservedSession
        return ProcessObservedSession(
            workspace, config, ros_time_ns_factory,
            ownership_fd=self._lock_stream.fileno(),
        )

    def _persist_startup_readiness(
        self, status: str, *, error: BaseException | None = None,
    ) -> None:
        """Freeze the initial gate outcome; later checks never overwrite it."""
        path = self.root / "startup_readiness.json"
        if path.exists():
            return
        _write_json(path, {
            "status": status, "session_id": self.session_id,
            "epoch_id": self.epoch_id, "hardware_boot_id": self.hardware_boot_id,
            "host_boot_id": self.host_boot_id, "created_at_utc": _utc(),
            "wait_started_monotonic_ns": self._startup_wait_started_ns,
            "finished_monotonic_ns": self._clock(), "timeout_s": 30.0,
            "clock_drift": self._startup_drift_report,
            "error": f"{type(error).__name__}: {error}" if error else None,
            "motion_allowed": status == "ready",
            "timestamps_modified": False,
        })

    def _preserve_failed_startup(
        self, summary: NucleoCaptureSummary, error: BaseException,
    ) -> None:
        target = self.root / "startup_nucleo_raw_telemetry.csv"
        if summary.staging_path.is_file():
            if target.exists():
                raise SensorError("refusing to overwrite failed startup raw evidence")
            summary.staging_path.rename(target)
        _write_json(self.root / "startup_session_manifest.json", {
            "status": "startup_failed", "session_id": self.session_id,
            "epoch_id": self.epoch_id, "hardware_boot_id": self.hardware_boot_id,
            "host_boot_id": self.host_boot_id,
            "error": f"{type(error).__name__}: {error}",
            "capture_error": summary.capture_error,
            "frames": summary.frames_written, "parser": asdict(summary.parser_stats),
            "raw_csv": str(target) if target.is_file() else None,
            "raw_sha256": _hash(target) if target.is_file() else None,
            "stopped_monotonic_ns": summary.stopped_sync_time_ns,
            "serial_reader_stopped": True, "motion_allowed": False,
        })

    @staticmethod
    def _startup_ready(report: dict[str, Any]) -> bool:
        baseline = report.get("startup_baseline")
        if not isinstance(baseline, dict) or baseline.get("required") is not True:
            raise SensorError("live sensor stable-start diagnostics are missing or not required")
        if baseline.get("ready") is True and baseline.get("status") == "ready":
            return True
        if baseline.get("ready") is False and baseline.get("status") == "warming_up":
            return False
        raise SensorError("live sensor stable-start baseline failed or has invalid readiness status")

    def _await_startup_ready(self) -> None:
        """Drain normally into the same raw stream before allowing first motion."""
        self._startup_wait_started_ns = self._clock()
        deadline = self._startup_wait_started_ns + 30_000_000_000
        while True:
            report = self._drift_report(self._check())
            if self._startup_ready(report):
                self._persist_startup_readiness("ready")
                return
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise SensorError("sensor stable-start baseline did not become ready within 30 seconds")
            self._sleep(min(0.05, remaining / 1e9))

    def _retain_failed_owner(self, error: BaseException, *, phase: str = "startup") -> None:
        self._ownership_retained = True
        FAILED_SESSION_OWNERS.append(self)
        _write_json(self.root / f"{phase}_failure.json", {
            "status": "requires_operator_intervention", "session_id": self.session_id,
            "epoch_id": self.epoch_id, "hardware_boot_id": self.hardware_boot_id,
            "host_boot_id": self.host_boot_id, "serial_ownership_retained": True,
            "reader_stop_error": f"{type(error).__name__}: {error}", "created_at_utc": _utc(),
        })

    def _identity(self) -> dict[str, Any]:
        return {"session_id": self.session_id, "epoch_id": self.epoch_id,
                "hardware_boot_id": self.hardware_boot_id,
                "host_boot_id": self.host_boot_id, "sync_mode": self.sync_mode,
                "started_at_utc": self._started_utc, "started_monotonic_ns": self._start_ns,
                "time_mapping": "host_monotonic_receive_time_not_device_time",
                "initial_sync_allowed": self.allow_initial_sync,
                "port_owned_by": "one_persistent_collection_reader",
                "capture_process": dict(getattr(self._session, "process_identity", {})),
                "capture_config": asdict(self._session.config),
                "session_staging_csv": str(getattr(self._session, "staging_path", ""))}

    def _check(self) -> SensorSnapshot:
        if self._closed:
            raise SensorError("sensor session is closed; an explicit new session is required")
        if self._boot_id_factory() != self.host_boot_id:
            raise SensorError("host boot epoch changed; refusing cross-epoch mapping")
        snap = self._session.snapshot()
        drift = self._drift_report(snap)
        self._startup_drift_report = drift
        if drift.get("error"):
            self._persist_drift_failure(drift)
            raise SensorError(f"sensor clock drift guard: {drift['error']}")
        if snap.capture_error:
            raise SensorError(snap.capture_error)
        if snap.last_frame_ns is None:
            raise SensorError("sensor stream has no complete frame")
        if snap.last_frame_ns is not None and self._clock() - snap.last_frame_ns > 500_000_000:
            raise SensorError("sensor stream is stale for more than 0.5 seconds")
        return snap

    @staticmethod
    def _drift_report(snap: SensorSnapshot) -> dict[str, Any]:
        report = getattr(snap, "clock_drift", None)
        if report is not None:
            return report
        report = DeviceClockDrift().report()
        report["unavailable_reason"] = "capture session does not expose timestamp diagnostics"
        return report

    def _persist_drift_failure(self, report: dict[str, Any]) -> None:
        path = self.root / "drift_failure.json"
        if not path.exists():
            _write_json(path, {**self._identity(), **report, "created_at_utc": _utc(),
                               "response": "refuse_next_dispatch_preserve_raw_no_blind_robot_motion"})

    def check_ready(self) -> dict[str, Any]:
        """Fresh stream/epoch/drift check immediately before the next robot goal.

        This never cancels an active goal, returns the robot home, performs SYNC,
        or modifies the timing map.  The collection runner owns motion policy.
        """
        report = self._drift_report(self._check())
        if not self._startup_ready(report):
            raise SensorError("live sensor stable-start baseline is not ready; refusing robot dispatch")
        return report

    def _save_drift(self, directory: Path, report: dict[str, Any], *, scope: str) -> dict[str, Any]:
        path = directory / "drift.json"
        _write_json(path, {**self._identity(), **report, "scope": scope,
                           "created_at_utc": _utc()})
        return {"path": str(path), "sha256": _hash(path), "status": report["status"],
                "error": report.get("error")}

    def _directory(self, name: str) -> Path:
        self._check()
        directory = self.root / "windows" / _label(name)
        directory.mkdir(exist_ok=False)
        return directory

    def _copy_window(self, snap: SensorSnapshot, path: Path, start: int, end: int,
                     *, include_baseline: bool) -> dict[str, Any]:
        count, baseline, last, first_ns, last_ns = 0, None, None, None, None
        # An already-dispatched window is an independent diagnostic, never a
        # new live readiness gate or an opportunity to rebase the live guard.
        drift = DeviceClockDrift(
            getattr(self._session.config, "expected_channels", (1, 2, 3, 4)),
            require_stable_start=False,
        )
        previous_line, previous_row, previous_time = None, None, None
        with snap.staging_path.open("rb") as source, path.open("xb") as target:
            remaining = snap.byte_count
            header = source.readline(remaining)
            remaining -= len(header)
            columns = next(csv.reader([header.decode("utf-8")]))
            if tuple(columns) != RAW_CAPTURE_COLUMNS:
                raise SensorError("raw sensor header differs from the parser authority")
            target.write(header)
            while remaining > 0:
                line = source.readline(remaining)
                if not line or not line.endswith(b"\n"):
                    raise SensorError("sensor snapshot contains a truncated CSV row")
                remaining -= len(line)
                values = next(csv.reader([line.decode("utf-8")]))
                if len(values) != len(columns):
                    raise SensorError("sensor snapshot contains an invalid CSV row")
                row = dict(zip(columns, values, strict=True))
                timestamp = int(row["sensor_rx_sync_time_ns"])
                if previous_time is not None and timestamp < previous_time:
                    raise SensorError("sensor monotonic clock regressed")
                previous_time = timestamp
                if timestamp < start:
                    previous_line, previous_row = line, row
                    continue
                if timestamp >= end:
                    break
                if count == 0:
                    baseline = previous_row
                    if include_baseline and previous_line is not None:
                        target.write(previous_line)
                        assert previous_row is not None
                        drift.update_row(previous_row)
                    first_ns = timestamp
                target.write(line)
                drift.update_row(row)
                count += 1
                last, last_ns = row, timestamp
        counters = [key for key in RAW_CAPTURE_COLUMNS if key.startswith("parser_")
                    and key != "parser_host_monotonic_ns"]
        deltas = {key: int((last or {}).get(key) or 0) - int((baseline or {}).get(key) or 0)
                  for key in counters}
        if any(value < 0 for value in deltas.values()):
            raise SensorError("parser counter epoch changed inside a window")
        return {"frames": count, "first_sample_monotonic_ns": first_ns,
                "last_sample_monotonic_ns": last_ns, "parser_counter_deltas": deltas,
                "parser_baseline": {key: int((baseline or {}).get(key) or 0) for key in counters},
                "raw_rows_including_baseline": count + int(include_baseline and baseline is not None),
                "clock_drift": drift.report()}

    def window(self, name: str, duration_s: float = 30.0, *,
               metadata: dict[str, Any] | None = None,
               check_stationary: Callable[[], None] | None = None,
               on_acquisition_closed: Callable[[], None] | None = None) -> dict[str, Any]:
        """Record an exact time window; the caller supplies motion evidence.

        ``check_stationary`` runs at least every 50 ms (including both ends).
        ``on_acquisition_closed`` closes optional live observation before final
        snapshots, raw copying and statistics, including on acquisition failure.
        The sensor module never claims that the overall robot session was idle.
        """
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise SensorError("background duration must be finite and positive")
        metadata = dict(metadata or {})
        stationary = metadata.get("stationary_evidence", {})
        if not isinstance(stationary, dict) or stationary.get("passed") is not True:
            raise SensorError("background requires explicit passed stationary evidence")
        directory = self._directory(name)
        start = self._clock()
        end = start + round(duration_s * 1e9)
        failure = None
        acquisition_close_error = None
        try:
            if check_stationary:
                check_stationary()
            while self._clock() < end:
                self._check()
                if check_stationary:
                    check_stationary()
                self._sleep(max(0.0, min(0.05, (end - self._clock()) / 1e9)))
            if check_stationary:
                check_stationary()
        except BaseException as exc:
            failure = exc
            end = min(end, self._clock())
        finally:
            if on_acquisition_closed is not None:
                try:
                    on_acquisition_closed()
                except BaseException as exc:
                    acquisition_close_error = exc
                    failure = failure or exc
        try:
            snap = self._check() if failure is None else self._session.snapshot()
        except BaseException as exc:
            failure = failure or exc
            snap = self._session.snapshot()
        raw = directory / "nucleo_raw_telemetry.csv"
        stats = self._copy_window(snap, raw, start, end, include_baseline=False)
        drift_report = stats.pop("clock_drift")
        drift_evidence = self._save_drift(directory, drift_report, scope="background_raw_window")
        minimum = math.ceil(duration_s * 99.5)
        reasons = []
        if drift_report.get("error"):
            reasons.append(f"clock_drift: {drift_report['error']}")
        if stats["frames"] < minimum:
            reasons.append(f"complete_groups={stats['frames']} < {minimum}")
        for key, value in stats["parser_counter_deltas"].items():
            if value and key not in {"parser_hub_packets", "parser_completed_groups"}:
                reasons.append(f"{key}={value}")
        first, last = stats["first_sample_monotonic_ns"], stats["last_sample_monotonic_ns"]
        coverage = (last - first) / 1e9 if first is not None else 0.0
        if coverage < max(0, duration_s - 0.1):
            reasons.append(f"actual_sample_coverage_s={coverage} < {duration_s - .1}")
        if failure:
            reasons.append(f"window_interrupted: {type(failure).__name__}: {failure}")
        result = {**self._identity(), "kind": "background", "name": name,
                  "status": "incomplete" if reasons else "complete",
                  "quality": {"accepted": not reasons, "reasons": reasons,
                              "scope": "sensor_integrity_only_not_stationarity"},
                  "start_monotonic_ns": start, "end_monotonic_ns": end,
                  "requested_duration_s": duration_s, "actual_sample_coverage_s": coverage,
                  "source_snapshot_bytes": snap.byte_count,
                  "robot_session_contains_motion": True,
                  "stationarity_validation": "caller_evidence_and_optional_live_callback",
                  "stationary_check_callback_supplied": check_stationary is not None,
                  "acquisition_closed_callback_supplied": on_acquisition_closed is not None,
                  "acquisition_close_error": (
                      f"{type(acquisition_close_error).__name__}: {acquisition_close_error}"
                      if acquisition_close_error is not None else None),
                  "full_stationarity_status": "pending_external_monitor_finalization",
                  "requires_action_stationarity_terminal_manifest": True,
                  "metadata": metadata, **stats, "raw_csv": str(raw), "raw_sha256": _hash(raw),
                  "clock_drift": drift_evidence,
                  "manifest": str(directory / "window.json")}
        _write_json(directory / "window.json", result)
        self._results.append(result)
        self._last_window_end_ns = end
        if failure:
            raise SensorError(f"background window failed; evidence: {directory}") from failure
        return result

    def finish_robot_window(self, execution_telemetry: str | Path,
                            metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        """Copy this actual-robot interval and map a DISPOSABLE slice copy."""
        guard_error = None
        try:
            snap = self._check()
        except SensorError as exc:
            # A failed same-epoch sensor guard must prevent NEW dispatch, not
            # erase evidence of an already completed robot movement.  Preserve
            # its raw interval and mark the mapping incomplete.  Cross-host or
            # closed epochs remain forbidden even for evidence finalization.
            if self._closed or self._boot_id_factory() != self.host_boot_id:
                raise
            snap = self._session.snapshot()
            guard_error = str(exc)
        metadata = dict(metadata or {})
        if metadata.get("session_id", self.session_id) != self.session_id or metadata.get(
                "host_boot_id", self.host_boot_id) != self.host_boot_id:
            raise SensorError("robot telemetry metadata belongs to a different epoch")
        if metadata.get("hardware_boot_id", self.hardware_boot_id) != self.hardware_boot_id:
            raise SensorError("robot telemetry metadata belongs to a different hardware boot")
        robot = Path(execution_telemetry).expanduser().resolve()
        if robot.name != "execution_telemetry.csv" or not robot.is_relative_to(self.workspace / "runs"):
            raise SensorError("robot telemetry must be workspace/runs/**/execution_telemetry.csv")
        with robot.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise SensorError("robot telemetry is empty")
        times = [int(row["sync_time_ns"]) for row in rows]
        if any(right < left for left, right in zip(times, times[1:])):
            raise SensorError("robot telemetry monotonic clock regressed")
        if times[0] < max(self._start_ns, self._last_window_end_ns) or times[-1] > self._clock():
            raise SensorError("robot telemetry is outside this epoch or overlaps a previous window")
        directory = self.root / "windows" / _label(metadata.get("name", robot.parent.parent.name))
        directory.mkdir(exist_ok=False)
        raw = directory / "nucleo_raw_telemetry.csv"
        stats = self._copy_window(snap, raw, times[0], times[-1] + 1, include_baseline=True)
        drift_report = stats.pop("clock_drift")
        drift_evidence = self._save_drift(directory, drift_report, scope="trajectory_raw_window_including_mapping_baseline")
        # Never let finalize_nucleo_capture consume the append-only original or
        # the durable per-window raw CSV.  Its successful cleanup is intentional.
        temporary = directory / f"nucleo_capture__{uuid.uuid4().hex}.staging.csv"
        shutil.copyfile(raw, temporary)
        if (robot.parent / "nucleo_telemetry.csv").exists() or (robot.parent / "nucleo_capture.yaml").exists():
            raise SensorError("refusing to overwrite existing robot/sensor mapping")
        summary = NucleoCaptureSummary(
            staging_path=temporary, config=self._session.config, workspace=self.workspace,
            started_sync_time_ns=times[0], stopped_sync_time_ns=times[-1],
            frames_written=stats["raw_rows_including_baseline"], parser_stats=snap.parser_stats,
            sync_attempts=getattr(self._session, "_sync_attempts", 0),
            sync_time_ms_u32=getattr(self._session, "_sync_time_ms_u32", None),
            sync_rtt_ns=getattr(self._session, "_sync_rtt_ns", None),
            capture_error=guard_error or snap.capture_error,
        )
        mapped = finalize_nucleo_capture(summary, robot)
        reasons = [] if mapped.complete else [mapped.message]
        if drift_report.get("error"):
            reasons.append(f"clock_drift: {drift_report['error']}")
        accepted = mapped.complete and not reasons
        result = {**self._identity(), "kind": "trajectory", "name": directory.name,
                  "status": "complete" if accepted else "incomplete",
                  "quality": {"accepted": accepted, "reasons": reasons,
                              "scope": "sensor_robot_time_mapping_only"},
                  "metadata": metadata, **stats, "raw_csv": str(raw), "raw_sha256": _hash(raw),
                  "clock_drift": drift_evidence,
                  "session_sensor_guard_error": guard_error,
                  "source_snapshot_bytes": snap.byte_count,
                  "robot_telemetry": str(robot), "robot_telemetry_sha256": _hash(robot),
                  "mapping": {"status": mapped.status, "path": str(mapped.sensor_telemetry_path),
                              "manifest": str(mapped.manifest_path), "message": mapped.message},
                  "manifest": str(directory / "window.json")}
        _write_json(directory / "window.json", result)
        self._results.append(result)
        self._last_window_end_ns = times[-1] + 1
        return result

    def close(self) -> dict[str, Any]:
        if self._closed:
            return self._close_result
        # If stop cannot join its reader AND process, retain serial ownership.
        try:
            summary = self._session.stop()
        except BaseException as stop_error:
            if not self._ownership_retained:
                self._retain_failed_owner(stop_error, phase="shutdown")
            raise SensorError("reader could not stop; serial ownership retained") from stop_error
        target = self.root / "nucleo_raw_telemetry.csv"
        if target.exists():
            raise SensorError("refusing to overwrite existing session raw evidence")
        drift_report = self._drift_report(self._session.snapshot())
        drift_evidence = self._save_drift(self.root, drift_report, scope="complete_hardware_epoch_raw_stream")
        if drift_report.get("error"):
            self._persist_drift_failure(drift_report)
        summary.staging_path.rename(target)
        result = {**self._identity(), "status": "incomplete" if summary.capture_error or drift_report.get("error") else "closed",
                  "capture_error": summary.capture_error, "frames": summary.frames_written,
                  "parser": asdict(summary.parser_stats), "finished_at_utc": _utc(),
                  "stopped_monotonic_ns": summary.stopped_sync_time_ns,
                  "raw_csv": str(target), "raw_sha256": _hash(target),
                  "clock_drift": drift_evidence,
                  "windows": self._results,
                  "next_session_requires_explicit_user_restart_acknowledgment": True}
        _write_json(self.root / "session_manifest.json", result)
        self._closed = True
        self._close_result = result
        self._lock_stream.close()
        return result
