"""Deterministic CPU lifecycle tests; no ROS, GPU, serial or robot."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from .motion import JointStateSnapshot, MotionController, StationaryMonitor
from .stationary_guard import RawStationaryMonitor


REAL_THREAD = threading.Thread


class FakeClock:
    def __init__(self) -> None:
        self.now = 1_000_000_000

    def monotonic_ns(self) -> int:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += round(seconds*1e9)


class ControlledThread:
    """The test supplies feedback deterministically instead of scheduling it."""

    def __init__(self, **kwargs) -> None:
        self.alive = False
        self.fail_to_join = False
        self.join_error: BaseException | None = None
        self.join_timeout = None
        self.join_calls = 0

    def start(self) -> None:
        self.alive = True

    def join(self, timeout: float) -> None:
        self.join_calls += 1
        self.join_timeout = timeout
        if self.join_error is not None:
            raise self.join_error
        if not self.fail_to_join:
            self.alive = False

    def is_alive(self) -> bool:
        return self.alive


class StationaryGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "actual.csv"
        self.clock = FakeClock()
        self.clock_patch = patch(f"{__package__}.stationary_guard.time.monotonic_ns",
                                 side_effect=self.clock.monotonic_ns).start()
        self.thread_patch = patch(f"{__package__}.motion.threading.Thread",
                                  side_effect=ControlledThread).start()
        self.addCleanup(patch.stopall)
        self.sequence = 0
        self.motion = SimpleNamespace(
            mode="real", options={"max_state_age_s": .5}, park=(0.,)*6,
            environment=SimpleNamespace(joint_names=tuple(f"joint_{i}" for i in range(6))),
            verify_runtime_identity=lambda: None,
            _state=lambda **kwargs: self.state(),
            tip_fk=lambda q: {"plane1_position_m": list(q[:3]),
                              "plane1_quaternion_xyzw": [0., 0., 0., 1.]},
            _unchanged=lambda: None)
        self.guard = RawStationaryMonitor(self.motion, self.path, 1.)
        self.guard.__enter__()
        self.addCleanup(self.guard.stream.close)

    def state(self, *, positions=None, velocities=None) -> JointStateSnapshot:
        self.sequence += 1
        return JointStateSnapshot(self.motion.environment.joint_names,
            tuple(positions or (0.,)*6), tuple(velocities or (0.,)*6),
            self.clock.now, self.clock.now, self.sequence)

    def cover_window(self) -> None:
        for _ in range(5):
            self.clock.advance(.2)
            self.guard._record(self.state())

    def terminal(self) -> dict:
        return json.loads(self.path.with_suffix(".csv.json").read_text())

    def test_slow_poststop_verification_and_hashing_do_not_age_valid_capture(self) -> None:
        self.cover_window()
        cutoff = self.clock.now
        verified = []
        def slow_unchanged() -> None:
            self.assertTrue(self.guard.stop.is_set())
            self.assertFalse(self.guard.thread.is_alive())
            self.assertTrue(self.guard.stream.closed)
            verified.append(True)
            self.clock.advance(20.)
        self.motion._unchanged = slow_unchanged
        from .stationary_guard import sha256
        def slow_hash(path: Path) -> str:
            self.clock.advance(30.)
            return sha256(path)
        with patch(f"{__package__}.stationary_guard.sha256", side_effect=slow_hash):
            self.assertFalse(self.guard.__exit__(None, None, None))
        self.guard.check()  # Finalized checks use the sealed cutoff, not now.
        evidence = self.terminal()
        self.assertTrue(evidence["passed"])
        self.assertEqual(evidence["host_window_s"], 1.)
        self.assertEqual(evidence["acquisition_cutoff_sync_time_ns"], cutoff)
        self.assertEqual(evidence["freshness_sample_sync_time_ns"], cutoff)
        self.assertEqual(evidence["sample_count"], 6)
        self.assertEqual(len(evidence["sha256"]), 64)
        self.assertEqual(verified, [True])

    def test_explicit_acquisition_close_precedes_expensive_sensor_processing(self) -> None:
        self.cover_window()
        cutoff = self.clock.now
        with patch.object(self.motion, "_unchanged") as unchanged, patch(
                f"{__package__}.stationary_guard.sha256") as checksum:
            self.guard.finish_acquisition()
            self.assertFalse(self.guard.thread.is_alive())
            self.assertTrue(self.guard.stream.closed)
            unchanged.assert_not_called()
            checksum.assert_not_called()
        self.clock.advance(25.)  # Sensor raw copying/drift statistics/hash.
        self.guard.finish_acquisition()  # Idempotent, does not recut the window.
        self.guard.check()
        self.guard.__exit__(None, None, None)
        self.assertTrue(self.terminal()["passed"])
        self.assertEqual(self.terminal()["acquisition_cutoff_sync_time_ns"], cutoff)
        self.assertEqual(self.terminal()["host_window_s"], 1.)
        self.assertEqual(self.terminal()["acquisition_sample_count"], 6)
        self.assertEqual(self.terminal()["finalization_policy"], "raw_acquisition_close_v2")
        self.assertEqual(self.guard.thread.join_calls, 1)

    def test_explicit_close_failure_is_idempotent_and_final_evidence_stays_false(self) -> None:
        self.cover_window()
        self.clock.advance(.500000001)
        cutoff = self.clock.now
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.finish_acquisition()
        self.assertTrue(self.guard.stream.closed)
        self.clock.advance(25.)
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.finish_acquisition()
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertEqual(self.terminal()["acquisition_cutoff_sync_time_ns"], cutoff)

    def test_postprocessing_body_exception_does_not_reopen_or_extend_acquisition(self) -> None:
        self.cover_window()
        self.guard.finish_acquisition()
        cutoff = self.clock.now
        self.clock.advance(25.)
        original = OSError("raw export failed")
        self.assertFalse(self.guard.__exit__(type(original), original, original.__traceback__))
        self.assertFalse(self.terminal()["passed"])
        self.assertEqual(self.terminal()["error"], "raw export failed")
        self.assertEqual(self.terminal()["acquisition_cutoff_sync_time_ns"], cutoff)

    def test_genuinely_stale_at_cutoff_is_rejected_before_slow_verification(self) -> None:
        self.cover_window()
        self.clock.advance(.500000001)
        self.motion._unchanged = lambda: self.clock.advance(20.)
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertIn("became stale", self.terminal()["error"])

    def test_live_freshness_limit_is_still_half_a_second(self) -> None:
        self.clock.advance(.5)
        self.guard.check()
        self.clock.advance(.000000001)
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.check()

    def test_midwindow_gap_is_rejected_and_preserved_in_terminal_evidence(self) -> None:
        self.clock.advance(.500000001)
        with self.assertRaisesRegex(ValueError, "gap or regressing clock") as caught:
            self.guard._record(self.state())
        self.guard.error = caught.exception  # Same propagation as inherited _loop.
        self.clock.advance(.5)
        with self.assertRaisesRegex(RuntimeError, "gap or regressing clock"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])

    def test_monitor_error_during_join_invalidates_covered_window(self) -> None:
        self.cover_window()
        join = self.guard.thread.join
        def error_at_join(timeout: float) -> None:
            self.guard.error = TimeoutError("no new actual JointState before deadline")
            join(timeout)
        self.guard.thread.join = error_at_join
        with self.assertRaisesRegex(RuntimeError, "no new actual JointState"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])

    def test_short_window_is_not_extended_by_slow_final_verification(self) -> None:
        self.clock.advance(.4)
        self.guard._record(self.state())
        self.motion._unchanged = lambda: self.clock.advance(20.)
        with self.assertRaisesRegex(RuntimeError, "does not cover"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertEqual(self.terminal()["host_window_s"], .4)

    def test_short_actual_coverage_is_rejected_even_when_host_window_is_long(self) -> None:
        self.clock.advance(.4)
        self.guard._record(self.state())
        self.clock.advance(.6)
        with self.assertRaisesRegex(RuntimeError, "became stale"):
            self.guard.__exit__(None, None, None)
        self.assertTrue(any("does not cover" in error
                            for error in self.terminal()["finalization_errors"]))

    def test_join_timeout_leaves_writer_open_and_never_seals_live_csv(self) -> None:
        self.cover_window()
        self.guard.thread.fail_to_join = True
        with patch(f"{__package__}.stationary_guard.sha256") as checksum:
            with self.assertRaisesRegex(RuntimeError, "did not stop"):
                self.guard.__exit__(None, None, None)
            checksum.assert_not_called()
        evidence = self.terminal()
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["reader_stopped"])
        self.assertIsNone(evidence["sha256"])
        self.assertFalse(self.guard.stream.closed)
        self.assertEqual(self.guard.thread.join_timeout, 2.5)
        self.guard._loop()  # A delayed reader finally owns its safe close.
        self.assertTrue(self.guard.stream.closed)
        self.assertFalse(self.terminal()["passed"])

    def test_join_exception_is_failed_evidence_and_does_not_close_live_writer(self) -> None:
        self.cover_window()
        self.guard.thread.join_error = RuntimeError("join failed")
        with self.assertRaisesRegex(RuntimeError, "join failed"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertIsNone(self.terminal()["sha256"])
        self.assertFalse(self.guard.stream.closed)

    def test_blocked_writer_does_not_block_finalizer_before_bounded_join(self) -> None:
        self.cover_window()
        entered, release = threading.Event(), threading.Event()
        writer = self.guard.writer
        class BlockingWriter:
            def writerow(inner_self, row: dict) -> None:
                entered.set()
                if not release.wait(timeout=5.):
                    raise TimeoutError("test writer was not released")
                writer.writerow(row)
        self.guard.writer = BlockingWriter()
        self.motion._state = lambda **kwargs: self.state()
        self.clock.advance(.2)
        self.guard.thread = REAL_THREAD(target=self.guard._loop, daemon=True)
        self.guard.thread.start()
        self.addCleanup(release.set)
        self.assertTrue(entered.wait(timeout=1.))
        # A join that times out immediately isolates whether finalization can
        # reach stop/join despite the writer being inside its inherited _record.
        with patch.object(self.guard.thread, "join") as joined:
            with self.assertRaisesRegex(RuntimeError, "did not stop"):
                self.guard.__exit__(None, None, None)
            joined.assert_called_once_with(2.5)
        self.assertTrue(self.guard.stop.is_set())
        self.assertFalse(self.guard.stream.closed)
        self.assertFalse(self.terminal()["passed"])
        self.assertIsNone(self.terminal()["sha256"])
        self.assertEqual(self.terminal()["acquisition_sample_count"], 6)
        release.set()
        self.guard.thread.join(timeout=1.)
        self.assertFalse(self.guard.thread.is_alive())
        self.assertTrue(self.guard.stream.closed)

    def test_unchanged_failure_is_saved_false_before_exit_raises(self) -> None:
        self.cover_window()
        def changed() -> None:
            raise ValueError("prepared bundle changed")
        self.motion._unchanged = changed
        with self.assertRaisesRegex(RuntimeError, "prepared bundle changed"):
            self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertIn("prepared bundle changed", self.terminal()["error"])

    def test_original_capture_exception_is_preserved_with_false_evidence(self) -> None:
        self.cover_window()
        original = RuntimeError("sensor capture failed")
        self.guard.error = ValueError("actual reader failed too")
        self.assertFalse(self.guard.__exit__(type(original), original, original.__traceback__))
        self.assertFalse(self.terminal()["passed"])
        self.assertEqual(self.terminal()["error"], "sensor capture failed")
        self.assertTrue(any("actual reader failed too" in error
                            for error in self.terminal()["finalization_errors"]))

    def test_hash_failure_is_not_saved_as_a_pass(self) -> None:
        self.cover_window()
        with patch(f"{__package__}.stationary_guard.sha256",
                   side_effect=OSError("cannot seal CSV")):
            with self.assertRaisesRegex(RuntimeError, "cannot seal CSV"):
                self.guard.__exit__(None, None, None)
        self.assertFalse(self.terminal()["passed"])
        self.assertIsNone(self.terminal()["sha256"])

    def test_fk_drift_and_original_state_validation_are_not_reimplemented(self) -> None:
        self.assertIs(RawStationaryMonitor.__mro__[1], StationaryMonitor)
        self.clock.advance(.2)
        with self.assertRaisesRegex(ValueError, "drift exceeds 1mm"):
            self.guard._record(self.state(positions=(.001000001, 0., 0., 0., 0., 0.)))
        # Actual feedback/velocity validation remains MotionController._state;
        # RAW controller only overrides construction of this monitor.
        from .released_motion import ReleasedMotionController
        self.assertIs(ReleasedMotionController._state, MotionController._state)


if __name__ == "__main__":
    unittest.main()
