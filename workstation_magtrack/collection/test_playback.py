from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import MagicMock

from .playback import CancellationUnconfirmed, RosPlaybackBackend, play_recording


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        self.telemetry = self.root / "runs/playback/real/execution_telemetry.csv"
        self.telemetry.parent.mkdir(parents=True)
        self.telemetry.write_text("actual telemetry fixture\n")
        self.backend = MagicMock()
        self.backend.begin.return_value = {"execution_mode": "real"}
        self.handle = MagicMock()
        self.handle.accepted = True
        self.result = SimpleNamespace(status=4, result=SimpleNamespace(
            success=True, message="ok", telemetry_path=str(self.telemetry)))
        self.row = {"recording_id": "recording", "duration_s": 30}

    def play(self):
        return play_recording(self.root, self.row, self.root / "play.log",
                              backend_factory=lambda _: self.backend)

    def test_success_does_not_open_serial_or_cancel(self):
        self.backend.wait.side_effect = [self.handle, self.result]
        self.assertEqual(self.play(), self.telemetry)
        self.handle.cancel_goal_async.assert_not_called()
        self.backend.close.assert_called_once()

    def test_timeout_cancels_owned_goal_and_waits_terminal(self):
        canceled = SimpleNamespace(status=5)
        self.backend.wait.side_effect = [self.handle, TimeoutError("timeout"),
                                         SimpleNamespace(goals_canceling=[1]), canceled]
        with self.assertRaises(TimeoutError):
            self.play()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.handle.get_result_async.call_count, 2)
        self.assertIn("owned_goal_terminal_confirmed", (self.root / "play.log").read_text())
        self.backend.close.assert_called_once()

    def test_keyboard_interrupt_uses_same_cancel_path(self):
        self.backend.wait.side_effect = [self.handle, KeyboardInterrupt(),
                                         SimpleNamespace(goals_canceling=[1]), SimpleNamespace(status=5)]
        with self.assertRaises(KeyboardInterrupt):
            self.play()
        self.handle.cancel_goal_async.assert_called_once()

    def test_unconfirmed_cancel_is_not_claimed_stopped(self):
        self.backend.wait.side_effect = [self.handle, TimeoutError("timeout"), TimeoutError("no cancel ack")]
        with self.assertRaisesRegex(CancellationUnconfirmed, "do not assume stopped"):
            self.play()
        self.assertIn("cancellation_unconfirmed", (self.root / "play.log").read_text())

    def test_submit_timeout_resolves_handle_before_cancel(self):
        self.backend.wait.side_effect = [TimeoutError("submit slow"), self.handle,
                                         SimpleNamespace(goals_canceling=[1]), SimpleNamespace(status=5)]
        with self.assertRaises(TimeoutError):
            self.play()
        self.handle.cancel_goal_async.assert_called_once()

    def test_unknown_goal_handle_requires_attention(self):
        self.backend.wait.side_effect = [TimeoutError("submit slow"), TimeoutError("still unknown")]
        with self.assertRaises(CancellationUnconfirmed):
            self.play()

    def test_rejected_goal_has_no_robot_cancel(self):
        self.handle.accepted = False
        self.backend.wait.return_value = self.handle
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            self.play()
        self.handle.cancel_goal_async.assert_not_called()

    def test_mismatched_runtime_does_not_submit(self):
        self.backend.begin.side_effect = RuntimeError("wrong runtime")
        with self.assertRaisesRegex(RuntimeError, "wrong runtime"):
            self.play()
        self.backend.submit.assert_not_called()
        self.backend.close.assert_called_once()

    def test_nominal_telemetry_not_labeled_actual(self):
        self.result.result.telemetry_path = str(self.telemetry.parent.parent / "simulation/execution_telemetry.csv")
        self.backend.wait.side_effect = [self.handle, self.result]
        with self.assertRaisesRegex(RuntimeError, "actual real telemetry"):
            self.play()

    def test_wait_spins_the_context_owned_executor(self):
        backend = RosPlaybackBackend.__new__(RosPlaybackBackend)
        backend.executor = MagicMock()
        future = MagicMock()
        future.done.side_effect = [False, True, True]
        future.result.return_value = "response"
        self.assertEqual(backend.wait(future, 1), "response")
        backend.executor.spin_once.assert_called_once()


if __name__ == "__main__":
    unittest.main()
