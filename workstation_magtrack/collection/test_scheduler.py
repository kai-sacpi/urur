"""Deterministic orchestration tests; no serial, ROS, or robot is opened."""

from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from workstation_magtrack.collection.scheduler import Engine, SchedulerError


NS = 1_000_000_000


class SchedulerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "checkpoint.json"
        self.now = 0
        self.engine = Engine.create(self.path, [f"recording-{n}" for n in range(20)],
                                    "manual-boot-1", self.now, clock_epoch_id="host-boot-A",
                                    metadata={"manifest_sha256": "abc", "config_sha256": "def"})

    def finish(self, kind: str, *, elapsed_s: float = 0) -> dict[str, object]:
        action = self.engine.take_next(self.now)
        self.assertIsNotNone(action)
        self.assertEqual(action.kind, kind)
        self.now += round(elapsed_s * NS)
        self.engine.complete(action.action_id, {"test_evidence": True}, self.now)
        return action.payload

    def open(self) -> None:
        self.finish("open_sensor_session")

    def background_cycle(self) -> None:
        self.finish("move_home")
        self.finish("move_park")
        self.finish("capture_background", elapsed_s=30)
        self.finish("move_home")

    def pause_after_three(self) -> None:
        self.open()
        for _ in range(3):
            self.now += 600 * NS
            self.background_cycle()
        self.finish("close_sensor_session")

    def test_complete_trajectory_is_not_interrupted_at_deadline(self) -> None:
        self.open()
        self.now = 599 * NS
        action = self.engine.take_next(self.now)
        self.assertEqual(action.kind, "play_trajectory")
        self.assertEqual(action.payload["trajectory_id"], "recording-0")
        self.now = 640 * NS
        with self.assertRaisesRegex(SchedulerError, "already in progress"):
            self.engine.take_next(self.now)
        self.assertEqual(self.engine.next_cursor, 0)
        self.engine.complete(action.action_id, {"success": True}, self.now)
        payload = self.finish("move_home")
        self.assertEqual(payload["purpose"], "background_home")
        self.assertEqual(self.engine.next_cursor, 1)
        due = [x for x in self.engine.snapshot["events"]
               if x["event"] == "background_due_at_trajectory_boundary"]
        self.assertEqual(due[0]["boundary_lateness_ns"], 40 * NS)

    def test_exact_deadline_is_due_and_one_ns_before_is_not(self) -> None:
        self.open()
        self.now = 600 * NS - 1
        self.finish("play_trajectory")
        self.now += 1
        self.finish("move_home")

    def test_background_order_and_deadline_from_capture_end_not_return(self) -> None:
        self.open()
        self.now = 600 * NS
        self.finish("move_home", elapsed_s=5)
        self.finish("move_park", elapsed_s=7)
        payload = self.finish("capture_background", elapsed_s=30)
        self.assertEqual(payload["duration_s"], 30)
        self.assertEqual(payload["background_ordinal"], 1)
        capture_end = self.now
        self.finish("move_home", elapsed_s=12)
        self.assertEqual(self.engine.snapshot["next_due_ns"], capture_end + 600 * NS)
        self.finish("play_trajectory")

    def initial_background_engine(self) -> None:
        self.path = Path(self.temporary.name) / "initial-background.json"
        self.engine = Engine.create(self.path, [f"recording-{n}" for n in range(20)],
                                    "manual-boot-1", self.now, clock_epoch_id="host-boot-A",
                                    initial_background=True)

    def test_initial_background_precedes_first_drawing_and_resets_deadline(self) -> None:
        self.initial_background_engine()
        self.assertTrue(self.engine.snapshot["initial_background_pending"])
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.open()
        self.assertFalse(self.engine.snapshot["initial_background_pending"])
        self.assertEqual(self.engine.snapshot["stage"], "background_home")
        self.assertEqual(self.finish("move_home", elapsed_s=5)["purpose"], "background_home")
        self.finish("move_park", elapsed_s=7)
        payload = self.finish("capture_background", elapsed_s=30)
        self.assertEqual(payload["background_ordinal"], 1)
        self.assertEqual(payload["session_background_ordinal"], 1)
        capture_end = self.now
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.assertEqual(self.engine.next_cursor, 0)
        self.assertEqual(self.finish("move_home", elapsed_s=12)["purpose"],
                         "background_return_home")
        self.assertEqual(self.engine.snapshot["next_due_ns"], capture_end + 600 * NS)
        payload = self.finish("play_trajectory")
        self.assertEqual(payload["trajectory_id"], "recording-0")
        events = self.engine.snapshot["events"]
        kinds = [item["kind"] for item in events if item["event"] == "action_completed"]
        self.assertEqual(kinds[:6], ["open_sensor_session", "move_home", "move_park",
                                    "capture_background", "move_home", "play_trajectory"])

    def test_initial_background_counts_toward_three_and_is_not_repeated_after_restart(self) -> None:
        self.initial_background_engine()
        self.open()
        self.background_cycle()
        for _ in range(2):
            self.now = self.engine.snapshot["next_due_ns"]
            self.background_cycle()
        self.assertEqual(self.engine.snapshot["total_background_count"], 3)
        self.assertEqual(self.engine.snapshot["session_background_count"], 3)
        self.assertEqual(self.engine.next_cursor, 0)
        self.finish("close_sensor_session")
        self.assertEqual(self.engine.status, "waiting_for_nucleo_restart")
        self.engine.confirm_restart("manual-boot-2", self.now)
        self.open()
        self.assertEqual(self.finish("play_trajectory")["trajectory_id"], "recording-0")

    def carryover_engine(self, count: int = 1) -> dict[str, object]:
        self.path = Path(self.temporary.name) / "carryover.json"
        proof = {"hardware_boot_id": "manual-boot-1", "accepted_background_count": count,
                 "evidence_path": "/accepted/backgrounds.json", "evidence_sha256": "a" * 64}
        self.engine = Engine.create(
            self.path, [f"recording-{n}" for n in range(20)], "manual-boot-1", self.now,
            clock_epoch_id="host-boot-A", initial_background=True,
            initial_prior_background_count=count, prior_background_provenance=proof)
        return proof

    def test_same_boot_carryover_pauses_after_two_more_not_three_more_backgrounds(self) -> None:
        proof = self.carryover_engine()
        self.assertEqual(self.engine.snapshot["session_background_count"], 1)
        self.assertEqual(self.engine.snapshot["total_background_count"], 1)
        self.assertEqual(self.engine.snapshot["initial_background_carryover"], proof)
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.open()
        self.finish("move_home")
        self.finish("move_park")
        payload = self.finish("capture_background", elapsed_s=30)
        self.assertEqual(payload["background_ordinal"], 2)
        self.assertEqual(payload["session_background_ordinal"], 2)
        self.finish("move_home")
        self.assertEqual(self.engine.snapshot["stage"], "boundary")
        self.finish("play_trajectory")
        self.now = self.engine.snapshot["next_due_ns"]
        self.background_cycle()
        self.assertEqual(self.engine.snapshot["stage"], "pause_close")
        self.assertEqual(self.engine.snapshot["session_background_count"], 3)
        self.assertEqual(self.engine.snapshot["total_background_count"], 3)
        self.finish("close_sensor_session")
        self.assertEqual(self.engine.status, "waiting_for_nucleo_restart")
        self.assertIsNone(self.engine.take_next(self.now))
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.engine.confirm_restart("manual-boot-2", self.now)
        self.assertEqual(self.engine.snapshot["session_background_count"], 0)
        self.assertEqual(self.engine.snapshot["total_background_count"], 3)
        self.assertEqual(self.engine.snapshot["initial_background_carryover"], proof)
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.open()
        self.assertEqual(self.finish("play_trajectory")["trajectory_id"], "recording-1")

    def test_failed_background_preserves_carryover_without_counting_failure(self) -> None:
        self.carryover_engine()
        self.open()
        self.finish("move_home")
        self.finish("move_park")
        action = self.engine.take_next(self.now)
        self.engine.fail(action.action_id, "failed live capture", self.now + 30 * NS)
        self.assertEqual(self.engine.snapshot["session_background_count"], 1)
        self.assertEqual(self.engine.snapshot["total_background_count"], 1)
        self.assertEqual(self.engine.status, "needs_attention")
        self.assertIsNone(self.engine.take_next(self.now + 30 * NS))

    def test_carryover_proof_is_deep_copied_and_logged(self) -> None:
        proof = self.carryover_engine()
        proof["accepted_background_count"] = 2
        proof["evidence_path"] = "altered"
        snapshot = self.engine.snapshot
        self.assertEqual(snapshot["initial_background_carryover"]["accepted_background_count"], 1)
        self.assertEqual(snapshot["initial_background_carryover"]["evidence_path"],
                         "/accepted/backgrounds.json")
        event = next(row for row in snapshot["events"]
                     if row["event"] == "prior_backgrounds_carried_over")
        self.assertEqual(event["accepted_background_count"], 1)
        self.assertEqual(event["provenance"], snapshot["initial_background_carryover"])

    def test_carryover_requires_valid_count_and_boot_bound_immutable_proof(self) -> None:
        valid = {"hardware_boot_id": "manual-boot-1", "accepted_background_count": 1,
                 "evidence_path": "/accepted/backgrounds.json", "evidence_sha256": "a" * 64}
        cases = [(count, valid) for count in (-1, 3, 4, True, False, 1.0, "1")]
        cases.extend([(1, None), (0, valid), (1, {}),
                      (1, {**valid, "hardware_boot_id": "other-boot"}),
                      (1, {**valid, "accepted_background_count": True}),
                      (1, {**valid, "accepted_background_count": 2}),
                      (1, {**valid, "evidence_path": ""}),
                      (1, {**valid, "evidence_sha256": "x" * 64}),
                      (1, {**valid, "evidence_sha256": "a" * 63})])
        for count, proof in cases:
            with self.subTest(count=count, proof=proof):
                path = Path(self.temporary.name) / "invalid-carryover.json"
                with self.assertRaises(ValueError):
                    Engine.create(path, ["one"], "manual-boot-1", 0,
                                  clock_epoch_id="host-boot-A",
                                  initial_prior_background_count=count,
                                  prior_background_provenance=proof)
                self.assertFalse(path.exists())

    def test_checkpoint_cannot_lose_carryover_or_change_its_boot(self) -> None:
        self.carryover_engine()
        valid = self.engine.snapshot
        cases = [("session_background_count", 0), ("total_background_count", 0),
                 ("initial_background_carryover", {**valid["initial_background_carryover"],
                                                   "hardware_boot_id": "other-boot"})]
        for key, value in cases:
            with self.subTest(key=key):
                self.path.write_text(json.dumps({**valid, key: value}))
                with self.assertRaises(SchedulerError):
                    Engine.load(self.path, clock_epoch_id="host-boot-A")

    def test_legacy_checkpoint_without_initial_background_still_draws_first(self) -> None:
        state = self.engine.snapshot
        state.pop("initial_background_pending")
        state.pop("initial_background_carryover")
        self.path.write_text(json.dumps(state))
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.open()
        self.assertEqual(self.finish("play_trajectory")["trajectory_id"], "recording-0")

    def test_initial_background_failure_does_not_dispatch_drawing(self) -> None:
        self.initial_background_engine()
        self.open()
        self.finish("move_home")
        action = self.engine.take_next(self.now)
        self.assertEqual(action.kind, "move_park")
        self.engine.fail(action.action_id, "parking rejected", self.now)
        self.assertEqual(self.engine.next_cursor, 0)
        self.assertEqual(self.engine.snapshot["total_background_count"], 0)
        self.assertIsNone(self.engine.take_next(self.now))

    def test_capture_cannot_complete_early_and_does_not_consume_cursor(self) -> None:
        self.open()
        self.now = 600 * NS
        self.finish("move_home")
        self.finish("move_park")
        action = self.engine.take_next(self.now)
        self.assertEqual(action.kind, "capture_background")
        with self.assertRaisesRegex(SchedulerError, "entire configured duration"):
            self.engine.complete(action.action_id, {}, self.now + 29 * NS)
        self.assertEqual(self.engine.snapshot["total_background_count"], 0)
        self.now += 30 * NS
        self.engine.complete(action.action_id, {"raw_csv": "capture.csv"}, self.now)
        self.assertEqual(self.engine.next_cursor, 0)
        self.assertEqual(self.engine.snapshot["total_background_count"], 1)

    def test_three_backgrounds_return_home_then_close_then_pause(self) -> None:
        self.open()
        for index in range(3):
            self.now += 600 * NS
            self.background_cycle()
            self.assertEqual(self.engine.snapshot["session_background_count"], index + 1)
        self.assertEqual(self.engine.status, "running")
        self.assertEqual(self.engine.snapshot["stage"], "pause_close")
        payload = self.finish("close_sensor_session")
        self.assertEqual(payload["reason"], "nucleo_restart_required")
        self.assertEqual(self.engine.status, "waiting_for_nucleo_restart")
        self.assertFalse(self.engine.snapshot["session_open"])
        self.assertIsNone(self.engine.take_next(self.now + 3600 * NS))

    def test_explicit_restart_creates_fresh_session_without_skipping_cursor(self) -> None:
        self.open()
        self.finish("play_trajectory")
        for _ in range(3):
            self.now += 600 * NS
            self.background_cycle()
        self.finish("close_sensor_session")
        before = self.engine.snapshot
        self.now += 120 * NS
        with self.assertRaisesRegex(SchedulerError, "never have been used"):
            self.engine.confirm_restart("manual-boot-1", self.now)
        self.engine.confirm_restart("manual-boot-2", self.now)
        after = self.engine.snapshot
        self.assertEqual(after["next_cursor"], before["next_cursor"])
        self.assertEqual(after["total_background_count"], 3)
        self.assertEqual(after["session_background_count"], 0)
        self.assertIsNone(after["next_due_ns"])
        self.assertIsNone(after["last_capture_end_ns"])
        self.assertIsNone(after["session_started_ns"])
        self.assertNotEqual(after["session_id"], before["session_id"])
        self.open()
        self.assertEqual(self.engine.snapshot["next_due_ns"], self.now + 600 * NS)
        payload = self.finish("play_trajectory")
        self.assertEqual(payload["trajectory_id"], "recording-1")
        self.assertEqual(payload["hardware_boot_id"], "manual-boot-2")

    def test_pause_survives_load_and_cannot_resume_without_confirmation(self) -> None:
        self.pause_after_three()
        loaded = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.assertEqual(loaded.status, "waiting_for_nucleo_restart")
        self.assertIsNone(loaded.take_next(self.now + 86400 * NS))
        loaded.confirm_restart("manual-boot-2", self.now + NS)
        self.assertEqual(loaded.take_next(self.now + NS).kind, "open_sensor_session")

    def test_final_trajectory_always_returns_home_then_closes(self) -> None:
        other = Path(self.temporary.name) / "single.json"
        self.engine = Engine.create(other, ["only-recording"], "manual-boot-1", 0,
                                    clock_epoch_id="host-boot-A")
        self.open()
        self.finish("play_trajectory", elapsed_s=700)
        payload = self.finish("move_home")
        self.assertEqual(payload["purpose"], "final_home")
        self.finish("close_sensor_session")
        self.assertEqual(self.engine.status, "complete")
        self.assertEqual(self.engine.next_cursor, 1)
        self.assertIsNone(self.engine.take_next(self.now))
        self.assertEqual(Engine.load(other, clock_epoch_id="host-boot-A").status, "complete")

    def test_trajectory_failure_neither_advances_nor_blindly_returns_home(self) -> None:
        self.open()
        action = self.engine.take_next(self.now)
        self.engine.fail(action.action_id, "controller failed", self.now)
        self.assertEqual(self.engine.status, "needs_attention")
        self.assertEqual(self.engine.next_cursor, 0)
        self.assertIsNone(self.engine.take_next(self.now))
        with self.assertRaises(SchedulerError):
            self.engine.confirm_restart("manual-boot-2", self.now)

    def test_capture_failure_is_not_counted_or_followed_by_blind_motion(self) -> None:
        self.open()
        self.now = 600 * NS
        self.finish("move_home")
        self.finish("move_park")
        action = self.engine.take_next(self.now)
        self.engine.fail(action.action_id, "sensor disconnected", self.now + 10 * NS)
        self.assertEqual(self.engine.snapshot["total_background_count"], 0)
        self.assertIsNone(self.engine.take_next(self.now + 20 * NS))

    def test_capture_clock_backwards_requires_attention_before_duration_check(self) -> None:
        self.open()
        self.now = 600 * NS
        self.finish("move_home")
        self.finish("move_park")
        action = self.engine.take_next(self.now)
        with self.assertRaisesRegex(SchedulerError, "clock moved backwards"):
            self.engine.complete(action.action_id, {}, 0)
        self.assertEqual(self.engine.status, "needs_attention")
        with self.assertRaises(SchedulerError):
            self.engine.complete(action.action_id, {}, self.now + 30 * NS)
        self.assertEqual(self.engine.snapshot["total_background_count"], 0)

    def test_in_progress_checkpoint_is_durable_before_dispatch(self) -> None:
        action = self.engine.take_next(self.now)
        snapshot = json.loads(self.path.read_text())
        self.assertEqual(snapshot["active_action"], action.to_dict())
        loaded = Engine.load(self.path, clock_epoch_id="host-boot-A")
        self.assertEqual(loaded.status, "needs_attention")
        self.assertIn("interrupted_in_progress_action", loaded.snapshot["attention_reason"])
        self.assertIsNone(loaded.take_next(self.now))
        with self.assertRaises(SchedulerError):
            loaded.complete(action.action_id, {}, self.now)

    def test_completed_trajectory_is_not_replayed_after_idle_load(self) -> None:
        self.open()
        self.finish("play_trajectory")
        self.engine = Engine.load(self.path, clock_epoch_id="host-boot-A",
                                  expected_metadata={"manifest_sha256": "abc", "config_sha256": "def"})
        payload = self.finish("play_trajectory")
        self.assertEqual(payload["trajectory_id"], "recording-1")

    def test_duplicate_or_stale_completion_is_refused(self) -> None:
        action = self.engine.take_next(self.now)
        with self.assertRaises(SchedulerError):
            self.engine.complete("not-the-action", {}, self.now)
        self.engine.complete(action.action_id, {}, self.now)
        before = self.path.read_bytes()
        with self.assertRaises(SchedulerError):
            self.engine.complete(action.action_id, {}, self.now)
        self.assertEqual(before, self.path.read_bytes())

    def test_epoch_change_blocks_old_clock_mapping(self) -> None:
        self.open()
        self.finish("play_trajectory")
        loaded = Engine.load(self.path, clock_epoch_id="host-boot-B")
        self.assertEqual(loaded.status, "needs_attention")
        self.assertIn("clock_epoch_changed", loaded.snapshot["attention_reason"])
        self.assertIsNone(loaded.take_next(0))

    def test_clock_backwards_blocks_action(self) -> None:
        self.open()
        self.now = NS
        self.finish("play_trajectory")
        with self.assertRaisesRegex(SchedulerError, "clock moved backwards"):
            self.engine.take_next(0)
        self.assertEqual(self.engine.status, "needs_attention")
        self.assertIsNone(self.engine.take_next(2 * NS))

    def test_metadata_mismatch_refuses_different_queue_configuration(self) -> None:
        before = self.path.read_bytes()
        with self.assertRaisesRegex(SchedulerError, "manifest/config"):
            Engine.load(self.path, clock_epoch_id="host-boot-A", expected_metadata={"manifest_sha256": "changed"})
        self.assertEqual(self.path.read_bytes(), before)

    def test_existing_checkpoint_is_never_overwritten(self) -> None:
        before = self.path.read_bytes()
        with self.assertRaises(FileExistsError):
            Engine.create(self.path, ["new-recording"], "new-boot", 0, clock_epoch_id="host-boot-A")
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])

    def test_checkpoint_write_failure_also_blocks_in_memory_dispatch(self) -> None:
        self.open()
        action = self.engine.take_next(self.now)
        with patch.object(self.engine, "_write_checkpoint", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.engine.complete(action.action_id, {}, self.now)
        self.assertEqual(self.engine.status, "needs_attention")
        self.assertIsNone(self.engine.take_next(self.now))
        self.assertIn("checkpoint_write_failed", self.engine.snapshot["attention_reason"])
        self.assertEqual(Engine.load(self.path, clock_epoch_id="host-boot-A").status, "needs_attention")

    def test_bad_inputs_and_bad_json_evidence_leave_pending_action_intact(self) -> None:
        with self.assertRaises(ValueError):
            self.engine.take_next(-1)
        with self.assertRaises(ValueError):
            self.engine.take_next(True)
        with self.assertRaises(SchedulerError):
            self.engine.confirm_restart("new-boot", 0)
        action = self.engine.take_next(0)
        before = self.path.read_bytes()
        with self.assertRaises(ValueError):
            self.engine.complete(action.action_id, {"invalid": float("nan")}, 0)
        self.assertEqual(before, self.path.read_bytes())
        self.engine.complete(action.action_id, {}, 0)

    def test_invalid_policy_and_source_ids_are_refused(self) -> None:
        cases = [{"trajectory_ids": []}, {"trajectory_ids": ["same", "same"]},
                 {"trajectory_ids": "not-a-sequence"}, {"hardware_boot_id": ""},
                 {"interval_s": 0}, {"capture_s": float("inf")}, {"pause_after": True},
                 {"clock_epoch_id": ""}, {"initial_background": "yes"}]
        for updates in cases:
            with self.subTest(updates=updates):
                args = {"checkpoint_path": self.path.parent / "invalid.json", "trajectory_ids": ["one"],
                        "hardware_boot_id": "manual-boot", "now_ns": 0, "clock_epoch_id": "host-boot-A"}
                args.update(updates)
                with self.assertRaises(ValueError):
                    Engine.create(**args)

    def test_illegal_saved_state_is_refused(self) -> None:
        bad = self.engine.snapshot
        bad["stage"] = "paused"
        self.path.write_text(json.dumps(bad))
        with self.assertRaisesRegex(SchedulerError, "cannot be paused"):
            Engine.load(self.path, clock_epoch_id="host-boot-A")

    def test_snapshot_and_action_copies_cannot_mutate_checkpoint(self) -> None:
        snapshot = self.engine.snapshot
        snapshot["trajectory_ids"].clear()
        self.assertEqual(len(self.engine.snapshot["trajectory_ids"]), 20)
        action = self.engine.take_next(0)
        action.payload["hardware_boot_id"] = "altered"
        self.assertEqual(self.engine.snapshot["active_action"]["payload"]["hardware_boot_id"], "manual-boot-1")


if __name__ == "__main__":
    unittest.main()
