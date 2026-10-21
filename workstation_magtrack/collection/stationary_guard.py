"""RAW-only stationary evidence finalization with an acquisition cutoff.

The frozen motion consumer remains unchanged. Its actual-state, gap, velocity,
joint-position and FK-drift checks are inherited; only shutdown bookkeeping is
different. Slow environment/bundle verification is not part of the live window.
"""
from __future__ import annotations

from pathlib import Path
import time

from .motion import JointStateSnapshot, MotionController, StationaryMonitor, sha256, write_json_new


class RawStationaryMonitor(StationaryMonitor):
    """Seal the observed window only after its reader has stopped safely."""

    def __init__(self, motion: MotionController, path: Path, duration_s: float):
        super().__init__(motion, path, duration_s)
        self._completed_record: tuple[int, int, float, float, float] = (0, 0, 0., 0., 0.)
        self._cutoff_ns: int | None = None
        self._cutoff_previous_ns = 0
        self._acquisition_errors: list[BaseException] = []
        self._acquisition_snapshot = self._completed_record
        self._reader_stopped = False
        self._cutoff_verified = False
        self._shutdown_diagnostics: list[dict[str, object]] = []

    def _record(self, state: JointStateSnapshot) -> None:
        super()._record(state)
        # Publish only a completed row. No finalizer lock can block behind FK or
        # filesystem I/O; one immutable reference supplies consistent metrics.
        self._completed_record = (self.previous_ns, self.rows, self.max_joint_error,
                                  self.max_velocity, self.max_tip_drift)

    def _loop(self) -> None:
        try:
            while not self.stop.is_set():
                try:
                    state = self.motion._state(target=self.motion.park)
                except TimeoutError as error:
                    observed_ns = time.monotonic_ns()
                    if (self.stop.is_set() and self._cutoff_verified and
                            self._cutoff_ns is not None and observed_ns >= self._cutoff_ns):
                        self._shutdown_diagnostics.append({
                            "kind": "post_acquisition_state_wait_timeout",
                            "observed_sync_time_ns": observed_ns,
                            "error": f"{type(error).__name__}: {error}"})
                        break
                    raise
                if self._cutoff_ns is not None and state.sync_time_ns > self._cutoff_ns:
                    self._shutdown_diagnostics.append({
                        "kind": "post_acquisition_actual_state_not_used",
                        "sample_sync_time_ns": state.sync_time_ns,
                        "acquisition_cutoff_sync_time_ns": self._cutoff_ns})
                    break
                # Returned states at/before the cutoff still pass every
                # original gap/FK check. ValueError and all unknown failures
                # remain fatal: no returned timestamp proves them post-window.
                self._record(state)
        except BaseException as error:
            self.error = error
        finally:
            # The writer owns this close. In particular, a join timeout must not
            # close a stream underneath an in-flight _record call.
            try:
                self.stream.close()
            except BaseException as error:
                self.error = self.error or error

    def check(self) -> None:
        if self._cutoff_ns is None:
            super().check()
            return
        self._check_at_cutoff()

    def _check_at_cutoff(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"stationary background invalid: {self.error}") from self.error
        cutoff_ns = self._cutoff_ns
        assert cutoff_ns is not None
        if (self._cutoff_previous_ns and
                (cutoff_ns-self._cutoff_previous_ns)*1e-9 >
                self.motion.options["max_state_age_s"]):
            raise RuntimeError("stationary background invalid: actual JointState became stale")

    def finish_acquisition(self) -> None:
        """Stop live observation before sensor export work, without sealing it.

        Idempotent: later context shutdown reuses this boundary and any failure;
        it never extends a completed window to cover copying or verification.
        """
        if self._cutoff_ns is not None:
            if self._acquisition_errors:
                raise RuntimeError(
                    f"stationary background invalid: {self._acquisition_errors[0]}") from self._acquisition_errors[0]
            return
        errors = self._acquisition_errors
        # Freeze the acquisition end while the reader is still active. Optional
        # post-stop hashing time must never age this final actual-state sample.
        previous_ns, rows, joint_error, velocity, tip_drift = self._completed_record
        self._acquisition_snapshot = (previous_ns, rows, joint_error, velocity, tip_drift)
        self._cutoff_ns = time.monotonic_ns()
        self._cutoff_previous_ns = previous_ns
        try:
            self._check_at_cutoff()
        except BaseException as error:
            errors.append(error)
        span = (previous_ns-self.started_ns)*1e-9
        wall_span = (self._cutoff_ns-self.started_ns)*1e-9
        if (rows < 2 or wall_span < self.duration_s or
                span < self.duration_s-self.motion.options["max_state_age_s"]):
            errors.append(ValueError(
                "actual-state evidence does not cover requested background window"))
        self._cutoff_verified = not errors
        self.stop.set()

        try:
            self.thread.join(self.motion.options["max_state_age_s"]+2.)
        except BaseException as error:
            errors.append(error)
        self._reader_stopped = not self.thread.is_alive()
        if not self._reader_stopped:
            errors.append(RuntimeError("stationary monitor did not stop"))
        elif not self.stream.closed:
            try:
                self.stream.close()
            except BaseException as error:
                errors.append(error)
        if self.error is not None:
            errors.append(self.error)
        self.error = errors[0] if errors else None
        if self.error is not None:
            raise RuntimeError(f"stationary background invalid: {self.error}") from self.error

    def __exit__(self, exception_type, exception, traceback) -> bool:
        try:
            self.finish_acquisition()
        except BaseException as error:
            if not self._acquisition_errors:
                self._acquisition_errors.append(error)
        errors = list(self._acquisition_errors)
        previous_ns, rows, joint_error, velocity, tip_drift = self._acquisition_snapshot
        span = (previous_ns-self.started_ns)*1e-9
        wall_span = (self._cutoff_ns-self.started_ns)*1e-9
        try:
            self.motion._unchanged()
        except BaseException as error:
            errors.append(error)

        checksum: str | None = None
        if self._reader_stopped:
            try:
                checksum = sha256(self.path)
            except BaseException as error:
                errors.append(error)
        self.error = errors[0] if errors else None
        primary_error = exception if exception is not None else self.error
        self.evidence = {
            "passed": primary_error is None,
            "execution_mode": self.motion.mode, "path": str(self.path), "sha256": checksum,
            "requested_duration_s": self.duration_s, "observed_span_s": span,
            "sample_count": self.rows, "acquisition_sample_count": rows,
            "host_window_s": wall_span,
            "started_sync_time_ns": self.started_ns,
            "stopped_sync_time_ns": self.previous_ns,
            "acquisition_cutoff_sync_time_ns": self._cutoff_ns,
            "freshness_sample_sync_time_ns": self._cutoff_previous_ns,
            "reader_stopped": self._reader_stopped,
            "acquisition_cutoff_verified": self._cutoff_verified,
            "post_acquisition_reader_diagnostics": list(self._shutdown_diagnostics),
            "finalization_policy": "raw_acquisition_close_v2",
            "sync_clock": "host CLOCK_MONOTONIC nanoseconds; same as NUCLEO sensor_rx_sync_time_ns",
            "max_joint_position_error_rad": joint_error,
            "max_joint_velocity_rad_s": velocity,
            "max_tip_fk_drift_m": tip_drift, "tip_frame": "plane1",
            "recorded_span_s": (self.previous_ns-self.started_ns)*1e-9,
            "error": str(primary_error) if primary_error is not None else None,
            "finalization_errors": [f"{type(error).__name__}: {error}" for error in errors],
        }
        try:
            write_json_new(self.path.with_suffix(self.path.suffix+".json"), self.evidence)
        except BaseException:
            self.evidence["passed"] = False
            if exception is None:
                raise
            # A failure to save evidence must not replace the original capture
            # exception; the surrounding action also preserves failure status.
            return False
        if exception is None and self.error is not None:
            raise RuntimeError(f"stationary background invalid: {self.error}") from self.error
        return False
