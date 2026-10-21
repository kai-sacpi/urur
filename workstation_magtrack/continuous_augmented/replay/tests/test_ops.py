"""Fast file/evidence tests: no ROS imports, robot, GPU, network or sensors."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from workstation_magtrack.continuous_augmented.replay import prepare as prep
from workstation_magtrack.continuous_augmented.replay import export_index as inv
from workstation_magtrack.ops.export_replay_index import check_seal


def put(path: Path, value: str | bytes | dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(prep.encoded(value) if isinstance(value, dict) else value.encode() if isinstance(value, str) else value)


def small_bank(root: Path) -> Path:
    put(root / "environments/mla_short/planner.yaml", "validation:\n  tip_position_tolerance_m: 0.001\n  tip_orientation_tolerance_deg: 0.5\n")
    put(root / "curobo.lock", "commit: fixture\n")
    batch = root / "continuous_augmented/batches" / prep.BATCH
    records, audits = [], []
    for index in range(2):
        name = f"fixture_{index}"
        csv_file, metadata_file = f"csv/{name}.csv", f"metadata/{name}.json"
        csv = ",".join(prep.FIELDS) + "\n" + "".join(f"{j*.05:.2f},plane1,0,0,-0.1,0,0,0,1,{'draw' if j == 0 else 'hold'}\n" for j in range(25))
        put(batch / csv_file, csv)
        source_sha = prep.sha(batch / csv_file)
        height = {"mode": "fixed", "lower_m": .1, "upper_m": .1, "direction": "constant"}
        meta = {"trajectory_id": name, "group_id": "fixed_height", "family_id": "open_line", "seed": index,
                "height_profile": height, "draw_duration_s": .2, "environment_id": "mla_short", "frame_id": "plane1",
                "quaternion_order": "xyzw", "source_sample_period_s": .05, "orientation": {"mode": "smooth_multi_axis"},
                "safety_status": {"simulation_only": True, "real_execution_authorized": False,
                                  "source_authority_may_be_modified_by_runtime": False, "hard_collision_clearance_m": .005,
                                  "rod_length_m": .190, "rod_radius_m": .0025, "provisional_position_error_envelope_m": 0.,
                                  "provisional_orientation_error_envelope_deg": 0.},
                "authority": {"csv_file": csv_file, "sha256": source_sha, "duration_s": 1.2, "source_pose_count": 25}}
        put(batch / metadata_file, meta)
        metadata_sha = prep.sha(batch / metadata_file)
        records.append({"trajectory_id": name, "csv_file": csv_file, "metadata_file": metadata_file,
                        "sha256": source_sha, "metadata_sha256": metadata_sha, "duration_s": 1.2, "draw_duration_s": .2,
                        "group_id": "fixed_height", "family_id": "open_line", "seed": index, "height_profile": height,
                        "orientation_mode": "smooth_multi_axis"})
        audits.append({"trajectory_id": name, "source_level_pass": True, "errors": [],
                       "authority": {"csv_sha256": source_sha, "metadata_sha256": metadata_sha},
                       "byte_exact_holds_pass": True, "phase_contract_pass": True,
                       "analytical_endpoint_v_a_jerk_zero_pass": True,
                       "path": {"passed": True, "strictly_monotone_projection": True},
                       "continuous_motion": {"passed": True}, "so3_finite_difference": {"formula_cross_check_pass": True}})
    files = prep.environment_hashes(root)
    fingerprint = {"files": files, "sha256": hashlib.sha256(json.dumps(files, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
                   "lock_sha256": prep.sha(root / "curobo.lock")}
    put(batch / "manifest.json", {"batch_id": prep.BATCH, "environment_id": "mla_short", "simulation_only": True,
                                   "real_execution_authorized": False, "trajectory_count": 2, "records": records,
                                   "source_contract": {"closed_pattern_families_allowed": False, "repeated_laps_allowed": False},
                                   "environment_fingerprint": fingerprint, "provenance": {"environment_fingerprint": fingerprint}})
    put(batch / "reports/source_audit.json", {"manifest_sha256": prep.sha(batch / "manifest.json"), "count": 2, "passed": 2,
                                             "all_source_level_pass": True, "batch_contract_errors": [], "trajectories": audits})
    return batch


def update_manifest_sha(batch: Path) -> None:
    audit = prep.read(batch / "reports/source_audit.json")
    audit["manifest_sha256"] = prep.sha(batch / "manifest.json")
    put(batch / "reports/source_audit.json", audit)


def complete_evidence(root: Path, *, collision: float = .01) -> tuple[dict, dict, str]:
    name, rid = "fake_open", "fake_open__sealed"
    environment = root / "environments/mla_short"
    for filename in inv.REPRODUCIBLE_ENV_FILES:
        put(environment / filename, "fixture reproducible input\n")
    put(environment / "planner.yaml", "validation:\n  tip_position_tolerance_m: 0.001\n  tip_orientation_tolerance_deg: 0.5\n")
    compiled, sealed = root / "runs/fake_open__compiled__fixture", environment / "recordings" / rid
    source = b"time_s,frame_id\n0,plane1\n1,plane1\n"
    put(compiled / "source_tip_trajectory.csv", source)
    put(compiled / "joint_trajectory.csv", "fixture joints\n")
    source_sha = prep.sha(compiled / "source_tip_trajectory.csv")
    authority = {"trajectory_id": name, "source_sha256": source_sha, "waypoint_count": 2, "duration_s": 1.}
    report = {"schema_version": 2, "compiler_contract_revision": "fixed_time_waypoint_v2", "environment_id": "mla_short",
              "trajectory_id": name, "source_trajectory_sha256": source_sha, "uniform_time_scale": 1., "source_waypoint_count": 2,
              "matched_source_waypoint_count": 2, "source_duration_s": 1., "duration_s": 1., "max_source_time_error_s": 0.,
              "environment_hashes": inv.reproducible_environment_hashes(root), "minimum_collision_distance_m": collision,
              "max_tip_position_error_m": .0001, "max_tip_orientation_error_deg": .01,
              "max_joint_velocity_ratio": .1, "max_joint_acceleration_ratio": .1, "max_joint_jerk_ratio": .1,
              "joint_trajectory_sha256": prep.sha(compiled / "joint_trajectory.csv")}
    put(compiled / "compile_report.yaml", yaml.safe_dump(report))
    for filename in inv.COMPILE_FILES:
        put(sealed / filename, (compiled / filename).read_bytes())
    for filename in inv.SEALED_FILES - inv.COMPILE_FILES - {"sha256sums.txt"}:
        put(sealed / filename, "fixture raw stream\n")
    put(sealed / "manifest.yaml", yaml.safe_dump({"recording_id": rid, "trajectory_id": name, "environment_id": "mla_short", "status": "validated"}))
    put(sealed / "validation_report.yaml", yaml.safe_dump({"simulation_completed": True, "controller_result": "succeeded"}))
    put(sealed / "sha256sums.txt", "".join(f"{prep.sha(p)}  {p.name}\n" for p in sorted(sealed.iterdir()) if p.is_file() and p.name != "sha256sums.txt"))
    origin = "/home/fixture/workstation_magtrack"
    remote = lambda p: str(Path(origin) / p.relative_to(root))
    hashes = lambda directory: {remote(p): prep.sha(p) for p in sorted(directory.rglob("*")) if p.is_file()}
    runtime = lambda domain: {"domain": domain, "execution_mode": "simulation", "telemetry_root": origin + "/runs", "environments_root": origin + "/environments"}
    audit = {"status": "passed", "canonical_recording_inspector_passed": True, "authoritative_duration_s": 1., "source_waypoint_count": 2,
             "controller_witness": {"status": "passed", "source_timing_changed": False, "resampling_or_synthetic_rows": False,
                                    "max_coherent_actual_source_position_error_m": .0001, "max_coherent_actual_source_orientation_error_deg": .01}}
    paths = {"telemetry_path": root / "runs/batch/telemetry.csv", "controller_witness_path": root / "runs/batch/controller.csv",
             "playback_audit_path": root / "runs/batch/audit.json"}
    for key, path in paths.items():
        put(path, audit if key == "playback_audit_path" else "raw state data\n")
    state = {"source": inv.source_identity(authority),
             "compile": {"status": "passed", "compiled_path": remote(compiled), "artifact_hashes": hashes(compiled), "report": report},
             "build": {"status": "passed", "recording_id": rid, "recording_path": remote(sealed), "artifact_hashes": hashes(sealed),
                       "runtime": runtime(219), "success": True, "action_status": 4},
             "inspect": {"status": "passed", "recording_id": rid, "recording_path": remote(sealed), "artifact_hashes": hashes(sealed)},
             "play": {"status": "passed", "runtime": runtime(220), "success": True, "action_status": 4,
                      "playback_audit": audit, **{k: remote(p) for k, p in paths.items()},
                      "artifact_hashes": {**hashes(sealed), **{remote(p): prep.sha(p) for p in paths.values()}}}}
    return authority, state, origin


class PreparationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / "workstation_magtrack"
        self.batch = small_bank(self.root)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def validate(self) -> dict:
        return prep.validate_sources(self.root, expected_count=2, check_distribution=False)

    def test_exact_join_and_portable_paths(self) -> None:
        index = self.validate()
        self.assertEqual(index["trajectory_count"], 2)
        self.assertTrue(all(not Path(r["source_file"]).is_absolute() for r in index["trajectories"]))

    def test_manifest_sha_mismatch(self) -> None:
        with (self.batch / "manifest.json").open("a") as stream:
            stream.write(" ")
        with self.assertRaisesRegex(ValueError, "SHA"):
            self.validate()

    def test_source_sha_mismatch(self) -> None:
        with (self.batch / "csv/fixture_0.csv").open("a") as stream:
            stream.write("tampered")
        with self.assertRaisesRegex(ValueError, "SHA"):
            self.validate()

    def test_duplicate_and_missing_ids(self) -> None:
        original = prep.read(self.batch / "reports/source_audit.json")
        for rows in ([original["trajectories"][0]] * 2, original["trajectories"][:1]):
            audit = {**original, "trajectories": rows}
            put(self.batch / "reports/source_audit.json", audit)
            with self.assertRaises(ValueError):
                self.validate()

    def test_seed_mismatch_is_not_hidden_by_rehashed_metadata(self) -> None:
        path = self.batch / "metadata/fixture_0.json"
        meta = prep.read(path)
        meta["seed"] = 999
        put(path, meta)
        manifest = prep.read(self.batch / "manifest.json")
        manifest["records"][0]["metadata_sha256"] = prep.sha(path)
        put(self.batch / "manifest.json", manifest)
        audit = prep.read(self.batch / "reports/source_audit.json")
        audit["trajectories"][0]["authority"]["metadata_sha256"] = prep.sha(path)
        put(self.batch / "reports/source_audit.json", audit)
        update_manifest_sha(self.batch)
        with self.assertRaisesRegex(ValueError, "seed"):
            self.validate()

    def test_source_model_fail_blocks_admission(self) -> None:
        audit = prep.read(self.batch / "reports/source_audit.json")
        audit["trajectories"][0]["continuous_motion"]["passed"] = False
        put(self.batch / "reports/source_audit.json", audit)
        with self.assertRaisesRegex(ValueError, "model audit"):
            self.validate()

    def test_immutable_outputs_preflight_all_destinations(self) -> None:
        first, second = self.root / "out/new.csv", self.root / "out/existing.csv"
        put(second, "user content")
        with self.assertRaisesRegex(ValueError, "refusing overwrite"):
            prep.immutable_outputs({first: b"new", second: b"different"})
        self.assertFalse(first.exists())
        self.assertEqual(second.read_text(), "user content")
        prep.immutable_outputs({second: b"user content"})


class EvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / "workstation_magtrack"
        self.authority, self.state, self.origin = complete_evidence(self.root)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def verify(self) -> dict:
        return inv.verify_complete(self.root, self.authority, self.state, self.origin, check_seal)

    def test_complete_bytes_and_controller_witness(self) -> None:
        result = self.verify()
        self.assertTrue(result["artifact_fingerprints_verified"])
        self.assertEqual(result["sealed_checksum_status"], "passed")

    def test_source_mismatch(self) -> None:
        self.state["source"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "source SHA"):
            self.verify()

    def test_four_status_flags_cannot_replace_controller_witness(self) -> None:
        del self.state["play"]["playback_audit"]["controller_witness"]
        with self.assertRaisesRegex(ValueError, "witness"):
            self.verify()

    def test_sealed_checksum_mismatch(self) -> None:
        put(self.root / "environments/mla_short/recordings/fake_open__sealed/sim_actual_tip.csv", "changed actual FK\n")
        with self.assertRaisesRegex(ValueError, "sealed checksum"):
            self.verify()

    def test_hard_five_mm_not_waived_by_nominal_h70_admission(self) -> None:
        self.authority, self.state, self.origin = complete_evidence(self.root, collision=.004)
        with self.assertRaisesRegex(ValueError, "hard collision"):
            self.verify()

    def test_hard_clearance_threshold_has_no_export_side_relaxation(self) -> None:
        self.authority, self.state, self.origin = complete_evidence(self.root, collision=.005-5e-13)
        with self.assertRaisesRegex(ValueError, "hard collision"):
            self.verify()

    def test_play_fingerprint_is_required(self) -> None:
        del self.state["play"]["artifact_hashes"][self.state["play"]["controller_witness_path"]]
        with self.assertRaisesRegex(ValueError, "required artifact"):
            self.verify()

    def test_duplicate_cross_shard_id_rejected(self) -> None:
        paths = [self.root / "status1.json", self.root / "status2.json"]
        for path in paths:
            put(path, {"workspace": self.origin, "environment_id": "mla_short", "trajectories": {"fake_open": self.state}})
        with self.assertRaisesRegex(ValueError, "duplicate cross-shard"):
            inv.load_states(paths, {"fake_open"})

    def test_wrong_environment_report_rejected(self) -> None:
        put(self.root / "environments/mla_short/controllers.yaml", "changed actual execution configuration\n")
        with self.assertRaisesRegex(ValueError, "environment bytes"):
            self.verify()

    def test_runtime_environment_map_is_exact_fixed_fourteen_not_readme(self) -> None:
        put(self.root / "environments/mla_short/README.md", "source provenance documentation\n")
        self.assertEqual(len(inv.reproducible_environment_hashes(self.root)), 14)
        self.assertNotIn("README.md", inv.reproducible_environment_hashes(self.root))
        self.assertEqual(len(prep.environment_hashes(self.root)), 15)
        self.assertTrue(self.verify()["artifact_fingerprints_verified"])

    def test_runtime_environment_cannot_silently_omit_one_of_fourteen_inputs(self) -> None:
        (self.root / "environments/mla_short/setup.yaml").unlink()
        with self.assertRaises(OSError):
            inv.reproducible_environment_hashes(self.root)


if __name__ == "__main__":
    unittest.main()
