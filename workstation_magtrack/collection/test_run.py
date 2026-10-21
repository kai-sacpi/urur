from __future__ import annotations

import copy
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import yaml

from .manifest import create, immutable_json, require_real_release, sha, verify
from .run import Collector, collect, controlled_termination, main, open_engine, prepared_parking_path, prior_backgrounds
from .scheduler import Action


class CollectionIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / "workstation_magtrack"
        self.root.mkdir()
        self.config = Path(__file__).with_name("config.yaml")
        (self.root / "curobo.lock").write_text("pinned test lock\n")
        env = self.root / "environments/mla_short"
        env.mkdir(parents=True)
        (env / "environment.yaml").write_text("frozen test environment\n")
        source_dir = env / "trajectories"
        source_dir.mkdir()
        rows = []
        # Four indivisible ~10-minute trajectories exercise 3-capture pause.
        for n in range(4):
            name, rid = f"route_{n}", f"route_{n}__sealed"
            source = source_dir / f"{name}.csv"
            source.write_text(f"test authority {n}\n")
            rec = env / "recordings" / rid
            rec.mkdir(parents=True)
            files = {"source_tip_trajectory.csv": source.read_bytes(),
                     "joint_trajectory.csv": b"test joint recording\n", "manifest.yaml": b"schema_version: 2\n"}
            checksums = []
            for filename, content in files.items():
                (rec / filename).write_bytes(content)
                checksums.append(f"{hashlib.sha256(content).hexdigest()}  {filename}\n")
            (rec / "sha256sums.txt").write_text("".join(checksums))
            rows.append({"trajectory_id": name, "recording_id": rid,
                         "source_sha256": sha(source), "duration_s": 601,
                         **{p: "passed" for p in ("compile", "build", "inspect", "play")},
                         "coherent_playback_witness": "passed",
                         "ready_for_simulation_replay_on_this_host": True,
                         "real_execution_authorized": False})
        self.index = self.root / "inventory.json"
        self.index.write_text(json.dumps({"environment_id": "mla_short", "recordings": rows}))
        self.manifest = self.root / "manifest.json"
        self.value = create(self.root, [self.index], self.config)
        immutable_json(self.manifest, self.value)

    def command(self, *args: str) -> int:
        return main(["--workspace", str(self.root), "--config", str(self.config), *args])

    def test_manifest_keeps_order_and_is_not_real_release(self) -> None:
        self.assertEqual([r["trajectory_id"] for r in self.value["recordings"]],
                         [f"route_{n}" for n in range(4)])
        verify(self.root, self.value)
        with self.assertRaisesRegex(ValueError, "simulation_only"):
            require_real_release(self.value)

    def test_source_change_refused(self) -> None:
        (self.root / self.value["recordings"][0]["source_file"]).write_text("changed")
        with self.assertRaisesRegex(ValueError, "source changed"):
            verify(self.root, self.value)

    def test_environment_change_refused(self) -> None:
        (self.root / "environments/mla_short/environment.yaml").write_text("changed")
        with self.assertRaisesRegex(ValueError, "environment changed"):
            verify(self.root, self.value)

    def test_sealed_change_refused(self) -> None:
        rec = self.root / self.value["recordings"][0]["recording_directory"]
        (rec / "joint_trajectory.csv").write_text("changed")
        with self.assertRaisesRegex(ValueError, "sealed file changed"):
            verify(self.root, self.value)

    def test_not_ready_refused_without_silent_skip(self) -> None:
        data = json.loads(self.index.read_text())
        data["recordings"][2]["play"] = "running"
        self.index.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "not four-stage ready"):
            create(self.root, [self.index], self.config)
        selected = create(self.root, [self.index], self.config, ["route_3", "route_0"])
        self.assertEqual([r["trajectory_id"] for r in selected["recordings"]], ["route_3", "route_0"])

    def test_duplicate_inventory_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate trajectory"):
            create(self.root, [self.index, self.index], self.config)

    def test_mixed_original_and_enhanced_witness_fields(self) -> None:
        data = json.loads(self.index.read_text())
        rows = data["recordings"]
        for row in rows[2:]:
            row["coherent_controller_witness"] = row.pop("coherent_playback_witness")
        old_index, new_index = self.root / "original.json", self.root / "enhanced.json"
        old_index.write_text(json.dumps({"environment_id": "mla_short", "recordings": rows[:2]}))
        new_index.write_text(json.dumps({"environment_id": "mla_short", "recordings": rows[2:]}))
        combined = create(self.root, [old_index, new_index], self.config)
        self.assertEqual(len(combined["recordings"]), 4)
        verify(self.root, combined)
        self.assertEqual(combined["recordings"][2]["coherent_witness_provenance"],
                         {"coherent_controller_witness": "passed"})

    def test_witness_alias_conflict_cannot_mask_failure(self) -> None:
        data = json.loads(self.index.read_text())
        data["recordings"][0]["coherent_controller_witness"] = "pending"
        self.index.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "not four-stage ready"):
            create(self.root, [self.index], self.config)

    def test_witness_missing_is_not_ready(self) -> None:
        data = json.loads(self.index.read_text())
        data["recordings"][0].pop("coherent_playback_witness")
        self.index.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "not four-stage ready"):
            create(self.root, [self.index], self.config)

    def test_escaping_checksum_refused(self) -> None:
        row = self.value["recordings"][0]
        rec = self.root / row["recording_directory"]
        (rec / "sha256sums.txt").write_text(f"{'0' * 64}  ../../../../escape\n")
        with self.assertRaisesRegex(ValueError, "escapes"):
            create(self.root, [self.index], self.config)

    def test_manifest_not_overwritten(self) -> None:
        with self.assertRaises(FileExistsError):
            immutable_json(self.manifest, self.value)

    def test_nested_basename_cannot_replace_root_seal(self) -> None:
        rec = self.root / self.value["recordings"][0]["recording_directory"]
        nested = rec / "nested"
        nested.mkdir()
        (nested / "joint_trajectory.csv").write_text("unrelated")
        checksum = rec / "sha256sums.txt"
        lines = [line for line in checksum.read_text().splitlines() if not line.endswith("  joint_trajectory.csv")]
        lines.append(f"{sha(nested / 'joint_trajectory.csv')}  nested/joint_trajectory.csv")
        checksum.write_text("\n".join(lines) + "\n")
        with self.assertRaisesRegex(ValueError, "mandatory"):
            create(self.root, [self.index], self.config)

    def test_duplicate_seal_refused(self) -> None:
        rec = self.root / self.value["recordings"][0]["recording_directory"]
        checksum = rec / "sha256sums.txt"
        checksum.write_text(checksum.read_text() * 2)
        with self.assertRaisesRegex(ValueError, "duplicate"):
            create(self.root, [self.index], self.config)

    def test_nominal_collect_never_constructs_hardware(self) -> None:
        output = self.root / "must_not_start"
        with patch(f"{__package__}.run.Collector") as collector:
            status = self.command("collect", "--manifest", str(self.manifest),
                                  "--output", str(output), "--boot-id", "boot1", "--port", "unused")
        self.assertEqual(status, 2)
        self.assertFalse(output.exists())
        collector.assert_not_called()

    def test_commission_never_accepts_plain_simulation_manifest(self) -> None:
        output = self.root / "runs/must_not_start_trial"
        with patch(f"{__package__}.run.Collector") as collector:
            status = self.command("commission", "--manifest", str(self.manifest),
                                  "--output", str(output), "--boot-id", "boot1", "--port", "unused")
        self.assertEqual(status, 2)
        self.assertFalse(output.exists())
        collector.assert_not_called()

    def test_trial_guard_failure_precedes_motion_and_output(self) -> None:
        output = self.root / "runs/rejected_trial"
        with patch(f"{__package__}.commissioning.require_trial",
                   side_effect=ValueError("trial parent changed")), patch(
                       f"{__package__}.run.Collector") as collector:
            status = self.command("commission", "--manifest", str(self.manifest),
                                  "--output", str(output), "--boot-id", "boot1", "--port", "unused")
        self.assertEqual(status, 2)
        self.assertFalse(output.exists())
        collector.assert_not_called()

    def test_preview_three_captures_home_close_then_manual_restart(self) -> None:
        output = self.root / "preview"
        args = ["preview", "--manifest", str(self.manifest), "--output", str(output)]
        self.assertEqual(self.command(*args, "--boot-id", "boot1"), 0)
        cp = json.loads((output / "checkpoint.json").read_text())
        self.assertEqual(cp["status"], "waiting_for_nucleo_restart")
        self.assertEqual(cp["next_cursor"], 3)
        self.assertEqual(cp["total_background_count"], 3)
        trace = json.loads((output / "schedule_preview_session_001.json").read_text())
        self.assertFalse(trace["robot_used"])
        self.assertFalse(trace["sensor_used"])
        self.assertEqual([a["kind"] for a in trace["actions"][-2:]],
                         ["move_home", "close_sensor_session"])
        self.assertEqual(self.command(*args, "--resume"), 2)
        self.assertEqual(self.command(*args, "--resume", "--confirm-nucleo-restart", "boot2"), 0)
        cp = json.loads((output / "checkpoint.json").read_text())
        self.assertEqual(cp["status"], "complete")
        self.assertEqual(cp["next_cursor"], 4)
        trace2 = json.loads((output / "schedule_preview_session_002.json").read_text())
        plays = [a for a in trace2["actions"] if a["kind"] == "play_trajectory"]
        self.assertEqual([a["payload"]["trajectory_id"] for a in plays], ["route_3"])

    def test_preview_initial_background_is_before_drawing_and_counts_toward_pause(self) -> None:
        output = self.root / "preview_initial_background"
        args = ["preview", "--manifest", str(self.manifest), "--output", str(output)]
        self.assertEqual(self.command(*args, "--boot-id", "boot1", "--initial-background"), 0)
        cp = json.loads((output / "checkpoint.json").read_text())
        self.assertEqual(cp["status"], "waiting_for_nucleo_restart")
        self.assertEqual(cp["total_background_count"], 3)
        self.assertEqual(cp["next_cursor"], 2)
        self.assertFalse(cp["initial_background_pending"])
        trace = json.loads((output / "schedule_preview_session_001.json").read_text())
        kinds = [action["kind"] for action in trace["actions"]]
        self.assertEqual(kinds[:6], ["open_sensor_session", "move_home", "move_park",
                                    "capture_background", "move_home", "play_trajectory"])
        self.assertEqual(kinds[-2:], ["move_home", "close_sensor_session"])
        self.assertFalse(trace["robot_used"])
        self.assertFalse(trace["sensor_used"])
        before = (output / "checkpoint.json").read_bytes()
        self.assertEqual(self.command(*args, "--resume", "--confirm-nucleo-restart", "boot2",
                                      "--initial-background"), 2)
        self.assertEqual((output / "checkpoint.json").read_bytes(), before)
        self.assertFalse((output / "schedule_preview_session_002.json").exists())

    def test_initial_background_resume_is_rejected_before_manifest_or_output(self) -> None:
        output = self.root / "runs/must_not_start_initial_resume"
        with patch(f"{__package__}.run.Collector") as collector:
            status = self.command("collect", "--manifest", str(self.root / "missing.json"),
                                  "--output", str(output), "--resume", "--initial-background",
                                  "--confirm-nucleo-restart", "boot2", "--port", "unused")
        self.assertEqual(status, 2)
        self.assertFalse(output.exists())
        collector.assert_not_called()

    def test_initial_background_not_available_for_commission(self) -> None:
        output = self.root / "runs/must_not_start_commission_initial"
        with self.assertRaises(SystemExit) as exit_status:
            self.command("commission", "--manifest", str(self.manifest), "--output", str(output),
                         "--boot-id", "boot1", "--initial-background", "--port", "unused")
        self.assertEqual(exit_status.exception.code, 2)
        self.assertFalse(output.exists())

    def background_receipt_args(self) -> SimpleNamespace:
        files = {"operational_acceptance": {"accepted": True},
                 "stationary": {"passed": True},
                 "session_epoch": {"hardware_boot_id": "same-boot"}}
        row = {}
        for key, value in files.items():
            path = self.root / f"{key}.json"
            immutable_json(path, value)
            row[key] = {"path": path.relative_to(self.root).as_posix(), "sha256": sha(path)}
        receipt = self.root / "carryover.json"
        immutable_json(receipt, {"hardware_boot_id": "same-boot", "accepted_background_count": 1,
                                 "accepted_backgrounds": [row]})
        return SimpleNamespace(command="collect", workspace=self.root, output=self.root / "runs/new",
                               manifest=self.manifest, boot_id="same-boot", resume=False,
                               confirm_nucleo_restart=None, initial_background=True,
                               reuse_nucleo_stream=True, prior_background_provenance=receipt)

    def test_same_boot_receipt_seeds_counts_and_binds_metadata(self) -> None:
        args = self.background_receipt_args()
        engine = open_engine(args, self.value, 0, "host-boot", "real")
        state = engine.snapshot
        self.assertEqual(state["session_background_count"], 1)
        self.assertEqual(state["total_background_count"], 1)
        self.assertEqual(state["pause_after"], 3)
        self.assertEqual(state["metadata"]["prior_backgrounds"]["evidence_sha256"],
                         sha(args.prior_background_provenance))

    def test_prior_receipt_refuses_wrong_boot_or_rejected_capture(self) -> None:
        args = self.background_receipt_args()
        args.boot_id = "different-boot"
        with self.assertRaisesRegex(ValueError, "boot"):
            prior_backgrounds(args)
        args.boot_id = "same-boot"
        path = self.root / "operational_acceptance.json"
        path.write_text(json.dumps({"accepted": False}))
        receipt = json.loads(args.prior_background_provenance.read_text())
        receipt["accepted_backgrounds"][0]["operational_acceptance"]["sha256"] = sha(path)
        args.prior_background_provenance.write_text(json.dumps(receipt))
        with self.assertRaisesRegex(ValueError, "not accepted"):
            prior_backgrounds(args)

    def test_prior_receipt_rejects_tamper_before_checkpoint(self) -> None:
        args = self.background_receipt_args()
        (self.root / "stationary.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "changed"):
            open_engine(args, self.value, 0, "host-boot", "real")
        self.assertFalse(args.output.exists())

    def test_prior_receipt_requires_passive_new_raw_campaign(self) -> None:
        args = self.background_receipt_args()
        for field, value in (("reuse_nucleo_stream", False), ("resume", True),
                             ("command", "commission")):
            with self.subTest(field=field):
                altered = copy.copy(args)
                setattr(altered, field, value)
                with self.assertRaisesRegex(ValueError, "new collect"):
                    prior_backgrounds(altered)

    def test_orderly_restart_revalidates_historical_carryover_and_resets_session_count(self) -> None:
        args = self.background_receipt_args()
        engine = open_engine(args, self.value, 0, "host-boot", "real")
        now = 0
        while engine.status == "running":
            action = engine.take_next(now)
            now += 601_000_000_000
            engine.complete(action.action_id, {"test_only": True}, now)
        self.assertEqual(engine.status, "waiting_for_nucleo_restart")
        args.resume, args.initial_background = True, False
        args.prior_background_provenance = None
        args.confirm_nucleo_restart = "new-physical-boot"
        restarted = open_engine(args, self.value, now, "host-boot", "real")
        self.assertEqual(restarted.snapshot["session_background_count"], 0)
        self.assertEqual(restarted.snapshot["total_background_count"], 3)
        self.assertEqual(restarted.snapshot["metadata"], engine.snapshot["metadata"])

    def test_failed_readonly_preflight_precedes_full_inventory_and_hardware(self) -> None:
        args = SimpleNamespace(command="collect", workspace=self.root, config=self.config,
                               output=self.root / "runs/preflight_failure", port="unused", resume=False)
        report = {"passed": False, "errors": ["runtime reply missing"], "elapsed_s": 10.}
        with patch(f"{__package__}.run.require_real_release") as release, patch(
                f"{__package__}.preflight.check_readiness", return_value=report) as health, patch(
                f"{__package__}.run.Collector") as hardware:
            with self.assertRaisesRegex(RuntimeError, "runtime reply missing"):
                collect(args, self.value)
        release.assert_called_once_with(self.value, self.root, verify_recordings=False)
        health.assert_called_once_with(self.root, timeout_s=10., domain_id=0)
        hardware.assert_not_called()
        self.assertFalse((args.output / "checkpoint.json").exists())
        saved = list(args.output.glob("startup_ros_readiness_*.json"))
        self.assertEqual(len(saved), 1)
        self.assertFalse(json.loads(saved[0].read_text())["passed"])

    def test_passed_readonly_preflight_does_not_replace_full_inventory_gate(self) -> None:
        args = SimpleNamespace(command="collect", workspace=self.root, config=self.config,
                               output=self.root / "runs/preflight_then_full", port="unused", resume=False)
        calls = []
        def release(*unused, **options):
            calls.append("fast" if options.get("verify_recordings") is False else "full")
            if calls[-1] == "full":
                raise ValueError("full seal changed")
        def health(*unused, **options):
            calls.append("read_only_health")
            return {"passed": True, "errors": [], "elapsed_s": .1}
        with patch(f"{__package__}.run.require_real_release", side_effect=release), patch(
                f"{__package__}.preflight.check_readiness", side_effect=health), patch(
                f"{__package__}.run.Collector") as hardware:
            with self.assertRaisesRegex(ValueError, "full seal changed"):
                collect(args, self.value)
        self.assertEqual(calls, ["fast", "read_only_health", "full"])
        hardware.assert_not_called()

    def test_fake_release_missing_acceptance_is_refused(self) -> None:
        value = copy.deepcopy(self.value)
        value["real_execution_authorized"] = True
        value["config"]["execution"]["real_execution_authorized"] = True
        value["config"]["execution"]["simulation_only"] = False
        for row in value["recordings"]:
            row["real_execution_authorized"] = True
        with self.assertRaisesRegex(ValueError, "missing accepted"):
            require_real_release(value)
        value["accepted_installation_release"] = {
            "accepted": True, "installation_id": "not_physical_evidence",
            "calibration_acceptance_sha256": "x", "active_tcp_binding_sha256": "x",
            "parking_validation_sha256": "x"}
        with self.assertRaisesRegex(ValueError, "missing hash-bound formal release"):
            require_real_release(value)

    def test_hash_bound_release_requires_workspace(self) -> None:
        value = {**self.value, "formal_release": {"accepted": True}}
        with self.assertRaisesRegex(ValueError, "current workspace"):
            require_real_release(value)

    def test_check_release_rejects_nominal_without_output_or_hardware(self) -> None:
        with patch(f"{__package__}.run.Collector") as collector:
            self.assertEqual(self.command("check-release", "--manifest", str(self.manifest)), 2)
        collector.assert_not_called()

    def test_formal_release_verifier_precedes_hardware_and_output(self) -> None:
        value = {**self.value, "formal_release": {"accepted": True}}
        self.manifest.write_text(json.dumps(value))
        output = self.root / "runs/rejected_release"
        with patch(f"{__package__}.release.verify_release", side_effect=ValueError("calibration missing")), \
                patch(f"{__package__}.run.Collector") as collector:
            status = self.command("collect", "--manifest", str(self.manifest),
                                  "--output", str(output), "--boot-id", "boot1", "--port", "unused")
        self.assertEqual(status, 2)
        self.assertFalse(output.exists())
        collector.assert_not_called()

    def test_config_does_not_change_original_home(self) -> None:
        config = yaml.safe_load(self.config.read_text())
        self.assertEqual(config["parking"]["joint_positions_deg"],
                         [-141.48, -84.74, -22.66, 19.73, 99.31, 42.18])
        self.assertNotIn("home_joint_positions", config)

    def fake_collector(self) -> Collector:
        collector = Collector.__new__(Collector)
        collector.args = SimpleNamespace(output=self.root / "runs/collector",
                                         workspace=self.root, port="unused")
        collector.args.output.mkdir(parents=True)
        collector.rows = {r["trajectory_id"]: r for r in self.value["recordings"]}
        collector.motion = MagicMock()
        collector.sensor = MagicMock()
        collector.last_park = {"settled": True}
        return collector

    def test_close_rejects_drift_without_capture_error(self) -> None:
        collector = self.fake_collector()
        sensor = collector.sensor
        sensor.close.return_value = {
            "status": "incomplete", "capture_error": None,
            "clock_drift": {"error": "sustained drift"}}
        with self.assertRaisesRegex(RuntimeError, "did not close cleanly"):
            collector.perform(Action("closing", "close_sensor_session", {}))
        self.assertIsNone(collector.sensor)
        sensor.close.assert_called_once()

    def test_background_guard_contract_and_evidence(self) -> None:
        collector = self.fake_collector()
        guard = MagicMock()
        guard.evidence = {"passed": True, "sample_count": 3001}
        events = []

        @contextmanager
        def monitor(path: Path, *, duration_s: float):
            self.assertEqual(duration_s, 30)
            events.append("guard_entered")
            yield guard
            events.append("guard_verified")

        def window(name: str, *, duration_s: float, metadata: dict,
                   check_stationary):
            self.assertEqual(events, ["guard_entered"])
            self.assertEqual(duration_s, 30)
            self.assertTrue(metadata["stationary_evidence"]["passed"])
            check_stationary()
            events.append("sensor_complete")
            return {"status": "complete", "quality": {"accepted": True}}

        collector.motion.stationary_monitor.side_effect = monitor
        collector.sensor.window.side_effect = window
        action = Action("testbackground", "capture_background", {
            "duration_s": 30, "background_ordinal": 1, "session_id": "boot1-session"})
        result = collector.perform(action)
        self.assertEqual(events, ["guard_entered", "sensor_complete", "guard_verified"])
        self.assertTrue(result["stationary"]["passed"])

    def test_only_raw_collect_closes_live_guard_at_sensor_acquisition_boundary(self) -> None:
        collector = self.fake_collector()
        collector.operational_quality = MagicMock(return_value={"accepted": True})
        guard = MagicMock()
        guard.evidence = {"passed": True, "sample_count": 3001}

        @contextmanager
        def monitor(path: Path, *, duration_s: float):
            self.assertEqual(duration_s, 30)
            yield guard

        collector.motion.stationary_monitor.side_effect = monitor
        for command in ("collect", "commission", None):
            with self.subTest(command=command):
                collector.args.command = command
                collector.sensor.window.reset_mock()
                collector.sensor.window.return_value = {
                    "status": "complete", "quality": {"accepted": True}}
                result = collector.perform(Action(f"boundary_{command}", "capture_background", {
                    "duration_s": 30, "background_ordinal": 1}))
                options = collector.sensor.window.call_args.kwargs
                self.assertIs(options["check_stationary"], guard.check)
                if command == "collect":
                    self.assertIs(options["on_acquisition_closed"], guard.finish_acquisition)
                else:
                    self.assertNotIn("on_acquisition_closed", options)
                self.assertTrue(result["accepted"])

    def test_drift_or_stale_sensor_prevents_robot_dispatch(self) -> None:
        collector = self.fake_collector()
        collector.sensor.check_ready.side_effect = RuntimeError("device clock drift budget exceeded")
        with patch(f"{__package__}.run._play") as play:
            with self.assertRaisesRegex(RuntimeError, "clock drift"):
                collector.perform(Action("drift", "play_trajectory", {"trajectory_id": "route_0"}))
        play.assert_not_called()
        collector.sensor.finish_robot_window.assert_not_called()

    def test_sensor_ready_is_checked_immediately_before_dispatch(self) -> None:
        collector = self.fake_collector()
        events = []
        collector.sensor.check_ready.side_effect = lambda: events.append("sensor_ready")
        collector.sensor.finish_robot_window.side_effect = lambda *a, **kw: (
            events.append("map_raw") or {"status": "complete", "quality": {"accepted": True}})

        def play(*args):
            events.append("submit_robot")
            return self.root / "runs/example/real/execution_telemetry.csv"

        with patch(f"{__package__}.run._play", side_effect=play):
            collector.perform(Action("trial", "play_trajectory", {"trajectory_id": "route_0"}))
        self.assertEqual(events, ["sensor_ready", "submit_robot", "map_raw"])

    def test_home_purpose_only_allows_planning_from_drawing_endpoint(self) -> None:
        collector = self.fake_collector()
        for purpose, dynamic in (("background_home", True), ("final_home", True),
                                 ("background_return_home", False)):
            with self.subTest(purpose=purpose):
                collector.perform(Action(purpose, "move_home", {"purpose": purpose}))
                collector.motion.move_home.assert_called_with(from_trajectory_end=dynamic)
        collector.motion.reset_mock()
        with self.assertRaisesRegex(ValueError, "explicit scheduler purpose"):
            collector.perform(Action("unknown", "move_home", {}))
        collector.motion.move_home.assert_not_called()

    def test_sensor_failure_prevents_new_home_and_park_dispatch(self) -> None:
        collector = self.fake_collector()
        collector.sensor.check_ready.side_effect = RuntimeError("sensor baseline not ready")
        actions = (Action("home", "move_home", {"purpose": "final_home"}),
                   Action("park", "move_park", {}))
        for action in actions:
            with self.subTest(kind=action.kind):
                with self.assertRaisesRegex(RuntimeError, "baseline not ready"):
                    collector.perform(action)
        self.assertEqual(collector.sensor.check_ready.call_count, 2)
        collector.motion.move_home.assert_not_called()
        collector.motion.move_park.assert_not_called()

    def test_missing_sensor_prevents_new_home_and_park_dispatch(self) -> None:
        collector = self.fake_collector()
        collector.sensor = None
        actions = (Action("home", "move_home", {"purpose": "final_home"}),
                   Action("park", "move_park", {}))
        for action in actions:
            with self.subTest(kind=action.kind):
                with self.assertRaisesRegex(RuntimeError, "armed before robot motion"):
                    collector.perform(action)
        collector.motion.move_home.assert_not_called()
        collector.motion.move_park.assert_not_called()

    def test_ready_sensor_precedes_new_home_and_park_dispatch(self) -> None:
        collector = self.fake_collector()
        events = []
        collector.sensor.check_ready.side_effect = lambda: events.append("sensor_ready")
        collector.motion.move_home.side_effect = lambda **kw: events.append("home")
        collector.motion.move_park.side_effect = lambda: events.append("park")
        collector.perform(Action("home", "move_home", {"purpose": "final_home"}))
        collector.perform(Action("park", "move_park", {}))
        self.assertEqual(events, ["sensor_ready", "home", "sensor_ready", "park"])

    def test_trial_preserves_actual_fk_audit_when_sensor_quality_fails(self) -> None:
        collector = self.fake_collector()
        collector.args.command = "commission"
        collector.manifest = {"commissioning": {"scope": "test_only"}}
        telemetry = self.root / "runs/example/real/execution_telemetry.csv"
        collector.sensor.finish_robot_window.return_value = {
            "status": "incomplete", "quality": {"accepted": False, "reasons": ["drift"]},
            "manifest": "window.json"}
        with patch(f"{__package__}.run._play", return_value=telemetry), patch(
                f"{__package__}.actual_telemetry.audit_actual_telemetry") as audit:
            with self.assertRaisesRegex(RuntimeError, "evidence=window.json"):
                collector.perform(Action("trial", "play_trajectory", {"trajectory_id": "route_0"}))
        audit.assert_called_once_with(
            telemetry, collector.motion, collector.args.output / "action_trial_actual_reference")

    def parking_binding(self) -> tuple[dict, Path, MagicMock]:
        directory = self.root / "collection/prepared_parking/test"
        directory.mkdir(parents=True)
        paths = {"prepared_bundle_manifest_sha256": directory / "bundle.json",
                 "gpu_validation_sha256": self.root / "runs/parking/validation.json",
                 "independent_mock_evidence_sha256": self.root / "runs/parking_mock/evidence.json"}
        for path in paths.values():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("test proof; strong verifier is explicitly faked\n")
        proof = {"asset_type": "collision_checked_prepare_joint_bundle", "tipdraw_recording": False,
                 "independent_mock_status": "passed", "real_execution_authorized": False,
                 "prepared_bundle": directory.relative_to(self.root).as_posix(),
                 "gpu_validation_file": paths["gpu_validation_sha256"].relative_to(self.root).as_posix(),
                 "independent_mock_evidence_file": paths["independent_mock_evidence_sha256"].relative_to(self.root).as_posix(),
                 "directions": {"fake_only": "fixture"},
                 **{key: sha(path) for key, path in paths.items()}}
        bundle = MagicMock()
        bundle.manifest = {"source_validation": {"sha256": proof["gpu_validation_sha256"]},
                           "directions": proof["directions"]}
        return {**self.value, "parking_prepare_bundle": proof}, directory.resolve(), bundle

    def test_parking_binding_checks_original_proofs_before_motion(self) -> None:
        value, directory, bundle = self.parking_binding()
        with patch(f"{__package__}.prepared_parking.PreparedParkingBundle",
                   return_value=bundle) as factory, patch(
                f"{__package__}.mock_parking.verify_mock_evidence",
                return_value={"status": "passed"}) as mock_proof:
            self.assertEqual(prepared_parking_path(self.root, self.config, value), directory)
            factory.assert_called_once_with(self.root, directory, self.config)
            bundle.verify.assert_called_once()
            mock_proof.assert_called_once_with(self.root,
                (self.root / value["parking_prepare_bundle"]["independent_mock_evidence_file"]).resolve(), directory)

    def test_parking_binding_rejects_changed_hash_and_cross_run_source(self) -> None:
        value, _, bundle = self.parking_binding()
        with patch(f"{__package__}.prepared_parking.PreparedParkingBundle",
                   return_value=bundle), patch(
                f"{__package__}.mock_parking.verify_mock_evidence",
                return_value={"status": "passed"}) as mock_proof:
            value["parking_prepare_bundle"]["gpu_validation_sha256"] = "0" * 64
            with self.assertRaisesRegex(ValueError, "frozen parking evidence changed"):
                prepared_parking_path(self.root, self.config, value)
            mock_proof.assert_not_called()
            value["parking_prepare_bundle"]["gpu_validation_sha256"] = sha(
                self.root / value["parking_prepare_bundle"]["gpu_validation_file"])
            bundle.manifest["source_validation"]["sha256"] = "0" * 64
            with self.assertRaisesRegex(ValueError, "differs from frozen collection provenance"):
                prepared_parking_path(self.root, self.config, value)
            mock_proof.assert_not_called()

    def test_parking_binding_requires_independent_raw_mock_verifier(self) -> None:
        value, _, bundle = self.parking_binding()
        with patch(f"{__package__}.prepared_parking.PreparedParkingBundle",
                   return_value=bundle), patch(
                f"{__package__}.mock_parking.verify_mock_evidence",
                return_value={"status": "failed"}):
            with self.assertRaisesRegex(ValueError, "independent mock evidence did not pass"):
                prepared_parking_path(self.root, self.config, value)

    def test_parking_binding_absent_never_constructs_motion(self) -> None:
        with patch(f"{__package__}.run.require_real_release"), \
                patch(f"{__package__}.released_motion.ReleasedMotionController") as motion:
            with self.assertRaisesRegex(ValueError, "frozen GPU/mock-proved"):
                Collector(SimpleNamespace(workspace=self.root, config=self.config), self.value)
            motion.assert_not_called()

    def test_collector_passes_only_manifest_bound_parking_to_motion(self) -> None:
        directory = self.root / "fake_bound_bundle"
        args = SimpleNamespace(workspace=self.root, config=self.config, output=self.root / "runs/fake")
        with patch(f"{__package__}.run.require_real_release"), \
                patch(f"{__package__}.run.prepared_parking_path", return_value=directory) as binding, patch(
                f"{__package__}.released_motion.ReleasedMotionController") as motion:
            Collector(args, self.value)
            binding.assert_called_once_with(self.root, self.config, self.value)
            motion.assert_called_once_with(self.root, self.config, self.value,
                                           evidence_root=args.output / "motions", prepared_parking=directory)

    def test_parking_binding_cannot_escape_workspace(self) -> None:
        value, _, _ = self.parking_binding()
        value["parking_prepare_bundle"]["prepared_bundle"] = "../outside_bundle"
        with patch(f"{__package__}.prepared_parking.PreparedParkingBundle") as bundle:
            with self.assertRaisesRegex(ValueError, "escapes workspace"):
                prepared_parking_path(self.root, self.config, value)
            bundle.assert_not_called()

    def test_stationary_guard_failure_prevents_background_completion(self) -> None:
        collector = self.fake_collector()

        @contextmanager
        def monitor(path: Path, *, duration_s: float):
            guard = MagicMock()
            guard.evidence = {"passed": False, "error": "robot moved during background"}
            yield guard
            raise RuntimeError("robot moved during background")

        collector.motion.stationary_monitor.side_effect = monitor
        collector.sensor.window.return_value = {"status": "complete", "quality": {"accepted": True}}
        with self.assertRaisesRegex(RuntimeError, "robot moved"):
            collector.perform(Action("failedbackground", "capture_background", {
                "duration_s": 30, "background_ordinal": 1}))
        terminal = json.loads((collector.args.output / "action_failedbackground.background.json").read_text())
        self.assertFalse(terminal["accepted"])
        self.assertEqual(terminal["status"], "failed")

    def test_unclean_sensor_close_refused(self) -> None:
        collector = self.fake_collector()
        collector.sensor.close.return_value = {"capture_error": "reader stalled"}
        with self.assertRaisesRegex(RuntimeError, "did not close cleanly"):
            collector.perform(Action("close", "close_sensor_session", {}))

    def test_sigterm_uses_controlled_exception_and_restores_handler(self) -> None:
        import signal
        previous = signal.getsignal(signal.SIGTERM)
        with controlled_termination():
            handler = signal.getsignal(signal.SIGTERM)
            with self.assertRaisesRegex(KeyboardInterrupt, "SIGTERM"):
                handler(signal.SIGTERM, None)
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)


if __name__ == "__main__":
    unittest.main()
