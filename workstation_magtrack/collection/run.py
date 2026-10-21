"""Boundary-triggered background collection; preview never opens ROS or serial."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any, Iterator

from .manifest import contained, create, immutable_json, recording_hashes, require_real_release, sha, verify
from .scheduler import Action, Engine


@contextmanager
def controlled_termination() -> Iterator[None]:
    """SIGTERM must take the same owned-goal cancellation path as Ctrl+C."""
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(signum: int, frame: Any) -> None:
        raise KeyboardInterrupt("SIGTERM requested controlled collection cancellation")

    signal.signal(signal.SIGTERM, terminate)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)


@contextmanager
def exclusive(output: Path) -> Iterator[None]:
    output.mkdir(parents=True, exist_ok=True)
    with (output / ".runner.lock").open("a+") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another collection runner owns this checkpoint") from exc
        yield


def prior_backgrounds(args: argparse.Namespace) -> dict[str, Any] | None:
    """Verify a same-boot carryover receipt; never edit an old checkpoint."""
    source = getattr(args, "prior_background_provenance", None)
    if source is None:
        return None
    if (args.command != "collect" or args.resume or
            not getattr(args, "reuse_nucleo_stream", False)):
        raise ValueError("prior backgrounds require a new collect with --reuse-nucleo-stream")
    path = contained(args.workspace, str(source))
    digest = sha(path)
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError("prior background receipt must be an object")
    count = value.get("accepted_background_count")
    if (value.get("hardware_boot_id") != args.boot_id or isinstance(count, bool) or
            not isinstance(count, int) or not 0 < count < 3):
        raise ValueError("prior background count or hardware boot does not match")
    rows = value.get("accepted_backgrounds")
    if not isinstance(rows, list) or len(rows) != count:
        raise ValueError("prior background receipt requires one evidence row per accepted capture")
    seen: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("prior background evidence must be an object")
        evidence = {}
        for key in ("operational_acceptance", "stationary", "session_epoch"):
            binding = row.get(key, {})
            if (not isinstance(binding, dict) or set(binding) != {"path", "sha256"} or
                    not isinstance(binding["path"], str) or not isinstance(binding["sha256"], str)):
                raise ValueError("invalid prior background evidence binding")
            item = contained(args.workspace, binding["path"])
            if sha(item) != binding["sha256"]:
                raise ValueError("prior background evidence changed")
            evidence[key] = json.loads(item.read_text())
            if not isinstance(evidence[key], dict):
                raise ValueError("prior background evidence must be an object")
        identity = str(contained(args.workspace, row["stationary"]["path"]))
        if identity in seen:
            raise ValueError("duplicate prior background evidence")
        seen.add(identity)
        if (evidence["operational_acceptance"].get("accepted") is not True or
                evidence["stationary"].get("passed") is not True or
                evidence["session_epoch"].get("hardware_boot_id") != args.boot_id):
            raise ValueError("prior background evidence was not accepted on this hardware boot")
    if sha(path) != digest:
        raise ValueError("prior background receipt changed during verification")
    return {"hardware_boot_id": args.boot_id, "accepted_background_count": count,
            "evidence_path": path.relative_to(args.workspace.resolve()).as_posix(),
            "evidence_sha256": digest}


def open_engine(args: argparse.Namespace, manifest: dict[str, Any], now_ns: int,
                epoch: str, mode: str) -> Engine:
    if getattr(args, "initial_background", False) and args.resume:
        raise ValueError("--initial-background is only allowed for a new campaign, not --resume")
    checkpoint = args.output / "checkpoint.json"
    metadata = {"manifest_sha256": sha(args.manifest), "mode": mode}
    prior = prior_backgrounds(args)
    if prior is not None:
        metadata["prior_backgrounds"] = prior
    if args.resume:
        historical = json.loads(checkpoint.read_text()).get("metadata", {}).get("prior_backgrounds")
        if historical is not None:
            # Revalidate the original boot's receipt without treating a later
            # explicitly confirmed hardware reboot as a new carryover request.
            historical_args = argparse.Namespace(**vars(args))
            historical_args.resume = False
            historical_args.boot_id = historical["hardware_boot_id"]
            historical_args.reuse_nucleo_stream = True
            historical_args.prior_background_provenance = historical["evidence_path"]
            verified = prior_backgrounds(historical_args)
            if verified != historical:
                raise ValueError("historical background carryover binding changed")
            metadata["prior_backgrounds"] = verified
        engine = Engine.load(checkpoint, clock_epoch_id=epoch, expected_metadata=metadata)
        if not args.confirm_nucleo_restart:
            raise ValueError("resume requires --confirm-nucleo-restart NEW_BOOT_ID after hardware restart")
        engine.confirm_restart(args.confirm_nucleo_restart, now_ns)
        return engine
    if args.confirm_nucleo_restart:
        raise ValueError("restart confirmation requires --resume")
    if not args.boot_id:
        raise ValueError("a new campaign requires explicit --boot-id")
    policy = manifest["config"]["schedule"]
    return Engine.create(checkpoint, [r["trajectory_id"] for r in manifest["recordings"]],
                         args.boot_id, now_ns, clock_epoch_id=epoch,
                         interval_s=policy["interval_s"], capture_s=policy["background_duration_s"],
                         pause_after=policy["pause_after_backgrounds"], metadata=metadata,
                         initial_background=getattr(args, "initial_background", False),
                         initial_prior_background_count=(prior["accepted_background_count"] if prior else 0),
                         prior_background_provenance=prior)


def preview(args: argparse.Namespace, manifest: dict[str, Any]) -> dict[str, Any]:
    """Virtual clock only: NOT robot, collision, telemetry or magnetic evidence."""
    checkpoint = args.output / "checkpoint.json"
    now = int(json.loads(checkpoint.read_text())["last_clock_ns"]) if args.resume else 0
    engine = open_engine(args, manifest, now, "virtual-schedule-preview-v1", "schedule_preview")
    rows = {r["trajectory_id"]: r for r in manifest["recordings"]}
    trace = []
    while engine.status == "running":
        action = engine.take_next(now)
        assert action is not None
        elapsed = (rows[action.payload["trajectory_id"]]["duration_s"]
                   if action.kind == "play_trajectory" else
                   action.payload["duration_s"] if action.kind == "capture_background" else
                   10 if action.kind.startswith("move_") else 0)
        now += round(elapsed * 1e9)
        evidence = {"schedule_preview_only": True, "not_physical_evidence": True,
                    "virtual_elapsed_s": elapsed}
        engine.complete(action.action_id, evidence, now)
        trace.append({**action.to_dict(), "end_virtual_ns": now})
    result = {"mode": "schedule_preview_only", "robot_used": False, "sensor_used": False,
              "collision_validation_performed": False, "status": engine.status,
              "next_cursor": engine.next_cursor, "actions": trace}
    immutable_json(args.output / f"schedule_preview_session_{engine.snapshot['session_number']:03d}.json", result)
    return {k: v for k, v in result.items() if k != "actions"}


def _play(workspace: Path, row: dict[str, Any], output: Path) -> Path:
    from .playback import play_recording
    return play_recording(workspace, row, output.with_suffix(".play.log"))


def prepared_parking_path(workspace: Path, config: Path, manifest: dict[str, Any]) -> Path:
    """Use only the exact GPU/mock-proved prepare assets frozen in this list."""
    from .prepared_parking import PreparedParkingBundle
    from .mock_parking import verify_mock_evidence

    proof = manifest.get("parking_prepare_bundle", {})
    if (not isinstance(proof, dict) or
            proof.get("asset_type") != "collision_checked_prepare_joint_bundle" or
            proof.get("tipdraw_recording") is not False or
            proof.get("independent_mock_status") != "passed" or
            proof.get("real_execution_authorized") is not False):
        raise ValueError("collection requires a frozen GPU/mock-proved parking prepare bundle")
    for key in ("prepared_bundle", "gpu_validation_file", "independent_mock_evidence_file"):
        if not isinstance(proof.get(key), str) or not proof[key]:
            raise ValueError(f"missing frozen parking path: {key}")
    directory = contained(workspace, proof["prepared_bundle"])
    checks = ((directory / "bundle.json", "prepared_bundle_manifest_sha256"),
              (contained(workspace, proof["gpu_validation_file"]), "gpu_validation_sha256"),
              (contained(workspace, proof["independent_mock_evidence_file"]),
               "independent_mock_evidence_sha256"))
    for path, key in checks:
        if sha(path) != proof.get(key):
            raise ValueError(f"frozen parking evidence changed: {key}")
    bundle = PreparedParkingBundle(workspace, directory, config)
    bundle.verify()
    if (bundle.manifest["source_validation"]["sha256"] != proof["gpu_validation_sha256"] or
            bundle.manifest["directions"] != proof.get("directions")):
        raise ValueError("parking prepare bundle differs from frozen collection provenance")
    checked = verify_mock_evidence(workspace, checks[2][0], directory)
    if checked.get("status") != "passed":
        raise ValueError("parking independent mock evidence did not pass")
    return directory


class Collector:
    """Hardware adapter. Constructed only AFTER explicit installation release."""

    def __init__(self, args: argparse.Namespace, manifest: dict[str, Any]):
        self.args, self.manifest = args, manifest
        self.rows = {r["trajectory_id"]: r for r in manifest["recordings"]}
        self.sensor = None
        if getattr(args, "command", None) == "commission":
            from .commissioning import require_trial
            from .commissioning_motion import CommissioningMotionController
            require_trial(args.workspace, manifest)
            parking = contained(args.workspace, manifest["parking_prepare_bundle"]["prepared_bundle"])
            self.motion = CommissioningMotionController(
                args.workspace, args.config, manifest, evidence_root=args.output / "motions",
                prepared_parking=parking)
        else:
            require_real_release(manifest, args.workspace, verify_recordings=False)
            parking = prepared_parking_path(args.workspace, args.config, manifest)
            from .released_motion import ReleasedMotionController
            self.motion = ReleasedMotionController(
                args.workspace, args.config, manifest, evidence_root=args.output / "motions",
                prepared_parking=parking)
        self.last_park: dict[str, Any] | None = None

    def operational_quality(self, result: dict[str, Any], readiness: dict[str, Any],
                            output: Path) -> dict[str, Any]:
        """Save prospective RAW acceptance separately from unchanged strict evidence."""
        if getattr(self.args, "command", None) != "collect":
            return {"accepted": result.get("status") == "complete" and
                    result.get("quality", {}).get("accepted") is True, "policy_id": "strict_v1"}
        from .quality import evaluate_window
        policy = self.motion.release["sensor_quality_policy"]
        accepted = evaluate_window(result, policy, readiness=readiness)
        immutable_json(output.with_suffix(".operational_acceptance.json"), {
            **accepted, "strict_window_manifest": result.get("manifest"),
            "dispatch_readiness": readiness,
            "release_id": self.motion.release["release_id"],
        })
        return accepted

    def perform(self, action: Action) -> dict[str, Any]:
        from .sensor import PersistentSensor
        payload = action.payload
        output = self.args.output / f"action_{action.action_id}"
        if action.kind == "open_sensor_session":
            self.motion.verify_runtime_identity()
            self.sensor = PersistentSensor.start_session(
                self.args.output, payload["session_id"], self.args.port, workspace=self.args.workspace,
                hardware_boot_id=payload["hardware_boot_id"],
                allow_initial_sync=not getattr(self.args, "reuse_nucleo_stream", False))
            return {"session_id": payload["session_id"], "hardware_boot_id": payload["hardware_boot_id"]}
        if action.kind == "close_sensor_session":
            if self.sensor is None:
                raise RuntimeError("no owned sensor session to close")
            evidence = self.sensor.close()
            self.sensor = None
            if (evidence.get("capture_error") or evidence.get("status") == "incomplete" or
                    evidence.get("clock_drift", {}).get("error")):
                raise RuntimeError(f"sensor session did not close cleanly: {evidence}")
            return evidence
        if action.kind == "move_home":
            purpose = payload.get("purpose")
            if purpose not in {"background_home", "background_return_home", "final_home"}:
                raise ValueError("move_home requires an explicit scheduler purpose")
            if self.sensor is None:
                raise RuntimeError("sensor capture must be armed before robot motion")
            self.sensor.check_ready()
            return self.motion.move_home(from_trajectory_end=purpose != "background_return_home")
        if action.kind == "move_park":
            if self.sensor is None:
                raise RuntimeError("sensor capture must be armed before robot motion")
            self.sensor.check_ready()
            self.last_park = self.motion.move_park()
            return self.last_park
        if self.sensor is None:
            raise RuntimeError("sensor capture must be armed before robot playback")
        if action.kind == "play_trajectory":
            self.motion.verify_runtime_identity()
            row = self.rows[payload["trajectory_id"]]
            # Re-check sealed bytes immediately before dispatch, not just at campaign start.
            if sha(contained(self.args.workspace, row["source_file"])) != row["source_sha256"]:
                raise RuntimeError("authoritative source changed before dispatch")
            if recording_hashes(self.args.workspace.resolve(),
                                contained(self.args.workspace, row["recording_directory"]),
                                row["source_sha256"]) != row["sealed_hashes"]:
                raise RuntimeError("sealed recording or file set changed before dispatch")
            readiness = self.sensor.check_ready()
            telemetry = _play(self.args.workspace, row, output)
            metadata = {**payload, **row}
            if getattr(self.args, "command", None) == "commission":
                metadata["commissioning"] = self.manifest["commissioning"]
            elif getattr(self.args, "command", None) == "collect":
                metadata["formal_release"] = self.manifest["formal_release"]
            result = self.sensor.finish_robot_window(telemetry, metadata=metadata)
            evidence = {"robot_telemetry": str(telemetry), "sensor": result}
            if getattr(self.args, "command", None) in {"commission", "collect"}:
                from .actual_telemetry import audit_actual_telemetry
                evidence["actual_robot_reference"] = audit_actual_telemetry(
                    telemetry, self.motion, self.args.output / f"action_{action.action_id}_actual_reference")
            acceptance = self.operational_quality(result, readiness, output)
            evidence["operational_acceptance"] = acceptance
            if not acceptance["accepted"]:
                raise RuntimeError("robot completed but sensor mapping/quality failed: "
                                   f"{acceptance.get('reasons', result.get('quality', {}).get('reasons', []))}; "
                                   f"evidence={result.get('manifest')}")
            return evidence
        if action.kind == "capture_background":
            if self.last_park is None:
                raise RuntimeError("background requires completed parking evidence")
            result = guard = None
            readiness = self.sensor.check_ready()
            try:
                with self.motion.stationary_monitor(output.with_suffix(".stationary.csv"),
                                                    duration_s=payload["duration_s"]) as guard:
                    window_options = ({"on_acquisition_closed": guard.finish_acquisition}
                                      if getattr(self.args, "command", None) == "collect" else {})
                    result = self.sensor.window(
                        f"background_{payload['background_ordinal']:04d}",
                        duration_s=payload["duration_s"], check_stationary=guard.check,
                        metadata={**payload, "stationary_evidence": {
                                      "passed": True, "scope": "pre_capture_settle_only",
                                      "parking_evidence": self.last_park,
                                      "full_window_guard": str(output.with_suffix(".stationary.csv"))},
                                  "magnetic_interference_status": "operator_expected_not_measured"},
                        **window_options)
            except BaseException as exc:
                immutable_json(output.with_suffix(".background.json"), {
                    "status": "failed", "accepted": False, "sensor": result,
                    "stationary": guard.evidence if guard is not None else None,
                    "error": f"{type(exc).__name__}: {exc}"})
                raise
            acceptance = self.operational_quality(result, readiness, output)
            accepted = acceptance["accepted"] and guard.evidence.get("passed") is True
            evidence = {"status": "complete" if accepted else "failed", "accepted": accepted,
                        "sensor": result, "stationary": guard.evidence,
                        "operational_acceptance": acceptance}
            immutable_json(output.with_suffix(".background.json"), evidence)
            if not accepted:
                raise RuntimeError(f"background rejected by sensor quality; see {output}")
            return evidence
        raise RuntimeError(f"unsupported collection action: {action.kind}")

    def close(self) -> None:
        try:
            if self.sensor is not None:
                self.sensor.close()
                self.sensor = None
        finally:
            if hasattr(self.motion, "close"):
                self.motion.close()


def collect(args: argparse.Namespace, manifest: dict[str, Any]) -> dict[str, Any]:
    commissioning = getattr(args, "command", None) == "commission"
    if commissioning:
        from .commissioning import require_trial
        require_trial(args.workspace, manifest)
    else:
        # Reject absent/tampered authority before even read-only ROS access.
        require_real_release(manifest, args.workspace, verify_recordings=False)
    if not args.port:
        raise ValueError("--port is required")
    if not args.output.is_relative_to(args.workspace / "runs"):
        raise ValueError("collection output must be under workspace/runs")
    # The release binds the original config without changing its provenance.
    if sha(args.config) != manifest["config_sha256"]:
        raise ValueError("config differs from frozen collection manifest")
    if args.resume:
        prior = json.loads((args.output / "checkpoint.json").read_text())
        if prior["status"] != "waiting_for_nucleo_restart":
            raise ValueError("only an orderly home/closed restart pause can resume automatically")
    from .preflight import check_readiness
    print(json.dumps({"event": "startup_ros_preflight_started", "read_only": True,
                      "serial_opened": False, "wall_time_ns": time.time_ns()}), flush=True)
    readiness = check_readiness(args.workspace, timeout_s=10., domain_id=0)
    readiness_path = args.output / f"startup_ros_readiness_{time.time_ns()}.json"
    immutable_json(readiness_path, readiness)
    if not readiness["passed"]:
        raise RuntimeError("read-only ROS preflight failed: " + "; ".join(readiness["errors"]) +
                           f"; evidence={readiness_path}")
    print(json.dumps({"event": "startup_ros_preflight_passed", "evidence": str(readiness_path),
                      "elapsed_s": readiness["elapsed_s"], "serial_opened": False,
                      "wall_time_ns": time.time_ns()}), flush=True)
    if not commissioning:
        print(json.dumps({"event": "startup_inventory_verification_started",
                          "serial_opened": False, "wall_time_ns": time.time_ns()}), flush=True)
        require_real_release(manifest, args.workspace)  # Full original inventory gates unchanged.
    epoch = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if commissioning and not args.resume:
        immutable_json(args.output / "commissioning_manifest.json", manifest)
        code = Path(__file__).resolve().parent
        immutable_json(args.output / "software_provenance.json", {
            "scope": "single_trial_commissioning",
            "source_sha256": {name: sha(code / name) for name in (
                "run.py", "commissioning.py", "commissioning_motion.py", "sensor.py", "capture_process.py",
                "drift.py", "actual_telemetry.py", "playback.py", "scheduler.py")},
            "formal_real_release": False,
        })
    elif not commissioning and not args.resume:
        immutable_json(args.output / "released_collection_manifest.json", manifest)
        code = Path(__file__).resolve().parent
        immutable_json(args.output / "software_provenance.json", {
            "scope": "operator_authorized_raw_collection",
            "source_sha256": {p.name: sha(p) for p in sorted(code.glob("*.py"))},
            "formal_release": manifest["formal_release"],
            "raw_actual_fk_reference": "nominal_rod_tip_not_independent_magnet_ground_truth",
            "active_tcp": "not_read_not_used_for_actual_joint_fk_or_joint_trajectory_execution",
        })
    engine = open_engine(args, manifest, time.monotonic_ns(), epoch,
                         "real_commissioning_nominal_reference" if commissioning else "real")
    collector = Collector(args, manifest)
    try:
        while engine.status == "running":
            action = engine.take_next(time.monotonic_ns())
            assert action is not None
            print(json.dumps({"event": "action_started", "action_id": action.action_id,
                              "kind": action.kind, "payload": action.payload,
                              "wall_time_ns": time.time_ns()}, ensure_ascii=False), flush=True)
            try:
                evidence = collector.perform(action)
                engine.complete(action.action_id, evidence, time.monotonic_ns())
                print(json.dumps({"event": "action_completed", "action_id": action.action_id,
                                  "kind": action.kind, "next_cursor": engine.next_cursor,
                                  "status": engine.status, "wall_time_ns": time.time_ns()},
                                 ensure_ascii=False), flush=True)
            except BaseException as exc:
                engine.fail(action.action_id, f"{type(exc).__name__}: {exc}", time.monotonic_ns())
                raise
    finally:
        # Releasing serial is safe; do not issue an unplanned robot return on failure.
        collector.close()
    result = {"status": engine.status, "next_cursor": engine.next_cursor,
              "checkpoint": str(engine.path)}
    if commissioning:
        result.update(scope="single_trial_commissioning",
                      labels="nominal_robot_tip_not_magnet_ground_truth", formal_real_release=False)
        immutable_json(args.output / "trial_summary.json", result)
    else:
        result.update(scope="operator_authorized_raw_collection",
                      labels="actual_joint_derived_nominal_rod_tip_bias_deferred",
                      formal_calibrated_release=False)
        immutable_json(args.output / f"raw_collection_session_{engine.snapshot['session_number']:03d}.json", result)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare", help="freeze an ordered list; read-only to source/recording")
    prep.add_argument("--index", type=Path, action="append", required=True)
    prep.add_argument("--ids", nargs="+")
    prep.add_argument("--manifest", type=Path, required=True)
    trial = sub.add_parser("prepare-trial", help="freeze exactly one operator-confirmed nominal real trial")
    trial.add_argument("--parent", type=Path, required=True)
    trial.add_argument("--trajectory", required=True)
    trial.add_argument("--operator-statement", required=True)
    trial.add_argument("--request-id", required=True)
    trial.add_argument("--manifest", type=Path, required=True)
    accept = sub.add_parser("prepare-raw-acceptance", help="bind explicit raw-acquisition permission; no hardware")
    accept.add_argument("--parent", type=Path, required=True)
    accept.add_argument("--installation-id", required=True)
    accept.add_argument("--operator-statement", required=True)
    accept.add_argument("--request-id", required=True)
    accept.add_argument("--quality-policy", default="strict_v1",
                        choices=("strict_v1", "bounded_group_loss_10_per_3000_v1"))
    accept.add_argument("--acceptance", type=Path, required=True)
    release = sub.add_parser("prepare-release", help="bind explicit raw permission and frozen proofs; no hardware")
    release.add_argument("--parent", type=Path, required=True)
    release.add_argument("--acceptance", type=Path, required=True)
    release.add_argument("--manifest", type=Path, required=True)
    check = sub.add_parser("check-release", help="read-only release/evidence verification; no hardware")
    check.add_argument("--manifest", type=Path, required=True)
    for name in ("preview", "collect", "commission"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--manifest", type=Path, required=True)
        cmd.add_argument("--output", type=Path, required=True)
        cmd.add_argument("--boot-id")
        cmd.add_argument("--resume", action="store_true")
        cmd.add_argument("--confirm-nucleo-restart", metavar="NEW_BOOT_ID")
        if name in {"preview", "collect"}:
            cmd.add_argument("--initial-background", action="store_true",
                             help="new campaign: home, validated park, background, home before drawings")
        if name in {"collect", "commission"}:
            cmd.add_argument("--port")
            cmd.add_argument("--reuse-nucleo-stream", action="store_true",
                             help="passive only: never send SYNC on this already-initialized boot")
        if name == "collect":
            cmd.add_argument("--prior-background-provenance", type=Path,
                             help="same-boot accepted background receipt; preserves three-capture limit")
    args = parser.parse_args(argv)
    args.workspace, args.config = args.workspace.resolve(), args.config.resolve()
    try:
        if getattr(args, "initial_background", False) and args.resume:
            raise ValueError("--initial-background is only allowed for a new campaign, not --resume")
        prior_backgrounds(args)
        if args.command == "prepare":
            result = create(args.workspace, args.index, args.config, args.ids)
            immutable_json(args.manifest, result)
            print(json.dumps({"manifest": str(args.manifest), "recordings": len(result["recordings"]),
                              "real_execution_authorized": result["real_execution_authorized"]}))
            return 0
        if args.command == "prepare-trial":
            from .commissioning import create_trial
            result = create_trial(args.workspace, args.parent, args.trajectory,
                                  args.operator_statement, args.request_id)
            immutable_json(args.manifest, result)
            print(json.dumps({"manifest": str(args.manifest), "recordings": 1,
                              "scope": "single_trial_commissioning", "formal_real_release": False}))
            return 0
        if args.command == "prepare-raw-acceptance":
            from .release import create_raw_acceptance
            result = create_raw_acceptance(args.workspace, args.parent, args.installation_id,
                                           args.operator_statement, args.request_id,
                                           quality_policy=args.quality_policy)
            immutable_json(args.acceptance, result)
            print(json.dumps({"acceptance": str(args.acceptance), "hardware_used": False,
                              "formal_calibrated_release": False}))
            return 0
        if args.command == "prepare-release":
            from .release import create_released_manifest
            # Creation binds metadata, never grants hardware execution by itself.
            # collect() still performs one full startup inventory verification.
            result = create_released_manifest(args.workspace, args.parent, args.acceptance,
                                               verify_recordings=False)
            immutable_json(args.manifest, result)
            print(json.dumps({"manifest": str(args.manifest), "recordings": len(result["recordings"]),
                              "scope": "operator_authorized_raw_collection", "hardware_used": False,
                              "formal_calibrated_release": False}))
            return 0
        if args.command == "check-release":
            manifest = json.loads(args.manifest.read_text())
            require_real_release(manifest, args.workspace)
            print(json.dumps({"status": "passed_offline_evidence_verification",
                              "live_installation_verified": False, "hardware_used": False,
                              "recordings": len(manifest["recordings"])}))
            return 0
        args.manifest, args.output = args.manifest.resolve(), args.output.resolve()
        manifest = json.loads(args.manifest.read_text())
        if args.command != "collect":
            verify(args.workspace, manifest)
        # Release rejection is also read-only to output: no misleading run folder.
        if args.command == "collect":
            # Reject absent/tampered authority without creating output. The
            # collect() entrypoint performs the single full startup pass.
            require_real_release(manifest, args.workspace, verify_recordings=False)
        elif args.command == "commission":
            from .commissioning import require_trial
            require_trial(args.workspace, manifest)
        with controlled_termination(), exclusive(args.output):
            result = preview(args, manifest) if args.command == "preview" else collect(args, manifest)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except KeyboardInterrupt:
        print("collection: interrupted; check checkpoint and cancellation evidence before any further motion",
              file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"collection: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
