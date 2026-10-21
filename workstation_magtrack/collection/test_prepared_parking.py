"""CPU-only tests with explicitly synthetic GPU proofs, never production evidence.

The helper below fabricates GPU metrics solely for isolated unit tests.  The
joint dynamics and discrete geometry are computed by the real CPU validators;
no GPU planner, ROS transport, serial connection, SSH, or robot is used.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest
from typing import Any, Callable
from unittest.mock import patch

import yaml

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
from hx_waypoint_curobo.trajectory_io import read_joint_trajectory_csv

from .geometry import screen_home_parking, screen_joint_positions
from .motion import sha256, validate_trajectory, write_joint_csv_exact, write_json_new


SOURCE = Path(__file__).resolve().parents[1]
PRODUCER_NAMES = ("motion.py", "geometry.py", "validate_parking.py")
DIRECTIONS = ("home_to_park", "park_to_home")


def _code_directory(path: Path) -> Path:
    return path / "collection" if (path / "collection").is_dir() else path


def make_validation(workspace: Path, source_dir: Path,
                    producer_snapshot_dir: Path | None = None) -> Path:
    """Create an isolated, explicitly synthetic validation fixture for tests.

    ``source_dir`` may be a workspace root or its collection directory.  The
    optional snapshot uses the aliases produced by the remote launch wrapper.
    This helper must never be called to construct a real parking certificate.
    """
    workspace = workspace.resolve()
    source = _code_directory(source_dir.resolve())
    environment_root = workspace / "environments/mla_short"
    if not environment_root.exists():
        shutil.copytree(SOURCE / "environments/mla_short", environment_root,
                        ignore=shutil.ignore_patterns("trajectories", "recordings"))
    collection = workspace / "collection"
    collection.mkdir(parents=True, exist_ok=True)
    for name in (*PRODUCER_NAMES, "config.yaml", "prepared_parking.py"):
        src = source / name
        dst = collection / name
        if src.is_file() and src.resolve() != dst.resolve():
            shutil.copy2(src, dst)
    if not (collection / "config.yaml").exists():
        shutil.copy2(SOURCE / "collection/config.yaml", collection / "config.yaml")
    if not (workspace / "curobo.lock").exists():
        shutil.copy2(SOURCE / "curobo.lock", workspace / "curobo.lock")
    code_hashes = {name: sha256(source / name) for name in PRODUCER_NAMES}
    if producer_snapshot_dir is not None:
        producer_snapshot_dir.mkdir(parents=True, exist_ok=False)
        for name in PRODUCER_NAMES:
            shutil.copy2(source / name,
                         producer_snapshot_dir / f"{Path(name).stem}_source_at_launch.py")

    environment = EnvironmentBuilder().load(environment_root)
    config = yaml.safe_load((collection / "config.yaml").read_text())
    home = environment.home_joint_positions
    park = tuple(math.radians(float(x)) for x in config["parking"]["joint_positions_deg"])
    revision = yaml.safe_load((workspace / "curobo.lock").read_text())["commit"]
    output = workspace / "runs/synthetic_unit_test_parking_validation"
    output.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "schema_version": 1, "environment_id": "mla_short", "simulation_only": True,
        "real_execution_authorized": False, "robot_execution": "not_requested_not_run",
        "ros_runtime": "not_started", "started_utc": "2026-10-17T00:00:00+00:00",
        "finished_utc": "2026-10-17T00:00:01+00:00", "status": "passed",
        "unit_test_fixture": True,
        "gpu_evidence_origin": "synthetic_unit_test_not_production_evidence",
        "parking_joint_positions_deg": config["parking"]["joint_positions_deg"],
        "parking_joint_positions_rad": list(park), "home_joint_positions_rad": list(home),
        "joint_names": list(environment.joint_names),
        "environment_hashes": dict(environment.hashes),
        "curobo_lock_sha256": sha256(workspace / "curobo.lock"),
        "config_sha256": sha256(collection / "config.yaml"), "code_sha256": code_hashes,
        "hard_world_self_clearance_m": .005, "rod_length_m": .190, "rod_diameter_m": .005,
        "uniform_time_scale": 1., "authority_mutated": False,
        "gpu_full_path_validation": "passed_both_directions", "runtime_ready": True,
        "directions": {}, "waited_for_batches": {},
        "gpu_exclusive_lock": str(workspace / "continuous_augmented/replay/.campaign.lock"),
        "gpu_inventory_before": {"stdout": "SYNTHETIC_UNIT_TEST_NO_GPU_USED"},
    }
    report["cpu_precheck"] = screen_home_parking(environment, park)
    report["cpu_precheck"]["admission_role"] = "straight-line diagnostics only; GPU may find a detour"
    report["cpu_endpoints"] = screen_joint_positions(environment, [home, park])
    for direction, start, target in ((DIRECTIONS[0], home, park), (DIRECTIONS[1], park, home)):
        zero = (0.,) * 6
        compiled = CompiledTrajectory(environment.joint_names, (
            JointSample(0., tuple(start), zero, zero, "approach"),
            JointSample(20., tuple(target), zero, zero, "approach")))
        dynamics = validate_trajectory(environment, compiled, start, target)
        directory = output / direction
        directory.mkdir()
        csv = directory / "joint_trajectory.csv"
        write_joint_csv_exact(csv, compiled)
        gpu_metrics = {
            "max_tip_position_error_m": 0., "max_tip_orientation_error_deg": 0.,
            "minimum_collision_distance_m": .031, "max_scene_collision_cost": 0.,
            "max_self_collision_cost": 0., "max_joint_bound_cost": 0.,
            "checked_sample_count": 2,
        }
        item = {
            "status": "passed", "start_positions_rad": list(start),
            "target_positions_rad": list(target),
            "started_utc": report["started_utc"], "finished_utc": report["finished_utc"],
            "planner": "PreparePlanner.plan -> cuRoboV2 plan_cspace -> fixed-clock optimizer -> full-path GPU validator",
            "curobo_revision": revision, "joint_trajectory_path": str(csv),
            "joint_trajectory_sha256": sha256(csv), "dynamics": dynamics,
            "gpu_metrics": deepcopy(gpu_metrics), "serialized_gpu_metrics": deepcopy(gpu_metrics),
            "cpu_sampled_geometry": screen_joint_positions(environment, [start, target]),
            "prepare_metadata": {"prepare": {
                "curobo_revision": revision, "uniform_time_scale": 1.,
                **{key: dynamics[key] for key in ("max_joint_velocity_ratio",
                    "max_joint_acceleration_ratio", "max_joint_jerk_ratio")}}},
        }
        write_json_new(directory / "evidence.json", item)
        report["directions"][direction] = item
    path = output / "validation.json"
    write_json_new(path, report)
    return path


def make_minimum_jerk_validation(workspace: Path, source_dir: Path) -> Path:
    """Real 2001-point CPU checks with synthetic GPU metrics, for temp tests only."""
    from .validate_parking import MINIMUM_JERK_SOURCE_KIND, write_minimum_jerk_candidate

    validation = make_validation(workspace, source_dir)
    report = json.loads(validation.read_text())
    report["source_kind"] = MINIMUM_JERK_SOURCE_KIND
    environment = EnvironmentBuilder().load(workspace / "environments/mla_short")
    for direction in DIRECTIONS:
        item = report["directions"][direction]
        local = validation.parent / direction
        compiled, candidate, data = write_minimum_jerk_candidate(
            local, environment.joint_names, item["start_positions_rad"],
            item["target_positions_rad"], item["curobo_revision"],
            generator_sha256=report["code_sha256"]["validate_parking.py"])
        csv = local / "joint_trajectory.csv"
        csv.write_bytes(data)
        item.update(
            upstream_candidate=candidate,
            joint_trajectory_sha256=sha256(csv),
            dynamics=validate_trajectory(environment, compiled,
                                         item["start_positions_rad"], item["target_positions_rad"]),
            cpu_sampled_geometry=screen_joint_positions(environment, [s.positions for s in compiled.samples]),
            prepare_metadata=dict(compiled.metadata),
            planner="upstream analytic minimum-jerk joint bridge -> immutable authoritative CSV -> unchanged full-path GPU validator")
        for label in ("gpu_metrics", "serialized_gpu_metrics"):
            item[label]["checked_sample_count"] = len(compiled.samples)
        (local / "evidence.json").write_text(json.dumps(item, indent=2, sort_keys=True, allow_nan=False) + "\n")
    validation.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return validation


class PreparedParkingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve() / "workspace"
        self.root.mkdir()
        self.validation = make_validation(self.root, SOURCE)
        self.bundle = self.root / "collection/prepared_parking_unit_test_bundle"

    def tearDown(self) -> None:
        self.temp.cleanup()

    @staticmethod
    def _api():
        from .prepared_parking import PreparedParkingBundle, create_bundle
        return PreparedParkingBundle, create_bundle

    def _create(self, **kwargs: Any) -> Any:
        _, create_bundle = self._api()
        return create_bundle(self.root, self.validation, self.bundle, **kwargs)

    def _load(self, root: Path | None = None, bundle: Path | None = None) -> Any:
        cls, _ = self._api()
        return cls(root or self.root, bundle or self.bundle)

    def _mutate_report(self, mutate: Callable[[dict[str, Any]], None], *,
                       synchronize_directions: bool = True) -> None:
        report = json.loads(self.validation.read_text())
        mutate(report)
        self.validation.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
        if synchronize_directions:
            for direction, item in report["directions"].items():
                path = self.validation.parent / direction / "evidence.json"
                path.write_text(json.dumps(item, indent=2, sort_keys=True, allow_nan=False) + "\n")

    def _replace_authority(self, replace: Callable[[CompiledTrajectory], CompiledTrajectory]) -> None:
        """Update CSV SHA and both report copies, so hash checks alone cannot pass."""
        environment = EnvironmentBuilder().load(self.root / "environments/mla_short")
        csv = self.validation.parent / "home_to_park/joint_trajectory.csv"
        compiled = read_joint_trajectory_csv(csv, environment.joint_names)
        csv.unlink()
        write_joint_csv_exact(csv, replace(compiled))
        self._mutate_report(lambda report: report["directions"]["home_to_park"].update({
            "joint_trajectory_sha256": sha256(csv)}))

    def test_valid_fixture_is_explicitly_not_production_gpu_evidence(self) -> None:
        report = json.loads(self.validation.read_text())
        self.assertTrue(report["unit_test_fixture"])
        self.assertEqual(report["gpu_evidence_origin"], "synthetic_unit_test_not_production_evidence")
        for item in report["directions"].values():
            self.assertTrue(item["dynamics"]["passed"])
            self.assertEqual(item["cpu_sampled_geometry"]["status"], "CPU_sampled_passed")

    def test_valid_bundle_round_trips_both_authoritative_trajectories(self) -> None:
        result = self._create()
        self.assertIsInstance(result, dict)
        prepared = self._load()
        self.assertIsNone(prepared.verify())
        environment = EnvironmentBuilder().load(self.root / "environments/mla_short")
        for direction in DIRECTIONS:
            compiled, evidence = prepared.trajectory(direction)
            source_csv = self.validation.parent / direction / "joint_trajectory.csv"
            expected = read_joint_trajectory_csv(
                source_csv, environment.joint_names)
            self.assertEqual(compiled.samples, expected.samples)
            self.assertEqual(compiled.joint_names, expected.joint_names)
            self.assertEqual((self.bundle / result["directions"][direction]["joint_trajectory_file"]).read_bytes(),
                             source_csv.read_bytes())
            self.assertIsInstance(evidence, dict)

    def test_rejects_incomplete_or_failed_gpu_admission(self) -> None:
        for key, invalid in (("status", "failed"), ("gpu_full_path_validation", "not_run"),
                             ("runtime_ready", False)):
            with self.subTest(key=key):
                original = self.validation.read_text()
                self._mutate_report(lambda report: report.update({key: invalid}))
                with self.assertRaises((ValueError, RuntimeError)):
                    self._create()
                self.validation.write_text(original)

    def test_rejects_real_authorization_or_mutated_clock(self) -> None:
        for key, invalid in (("simulation_only", False), ("real_execution_authorized", True),
                             ("uniform_time_scale", 2.), ("authority_mutated", True)):
            with self.subTest(key=key):
                original = self.validation.read_text()
                self._mutate_report(lambda report: report.update({key: invalid}))
                with self.assertRaises((ValueError, RuntimeError)):
                    self._create()
                self.validation.write_text(original)

    def test_rejects_changed_environment_hash(self) -> None:
        self._mutate_report(lambda report: report["environment_hashes"].update({"setup.yaml": "0" * 64}))
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_rejects_changed_config_or_curobo_lock_hash(self) -> None:
        for key in ("config_sha256", "curobo_lock_sha256"):
            with self.subTest(key=key):
                original = self.validation.read_text()
                self._mutate_report(lambda report: report.update({key: "0" * 64}))
                with self.assertRaises((ValueError, RuntimeError)):
                    self._create()
                self.validation.write_text(original)

    def test_rejects_wrong_rod_or_hard_clearance(self) -> None:
        for key, invalid in (("rod_length_m", .33365), ("rod_diameter_m", .006),
                             ("hard_world_self_clearance_m", .004)):
            with self.subTest(key=key):
                original = self.validation.read_text()
                self._mutate_report(lambda report: report.update({key: invalid}))
                with self.assertRaises((ValueError, RuntimeError)):
                    self._create()
                self.validation.write_text(original)

    def test_rejects_missing_reverse_direction(self) -> None:
        self._mutate_report(lambda report: report["directions"].pop("park_to_home"))
        with self.assertRaises((ValueError, RuntimeError, KeyError)):
            self._create()

    def test_rejects_report_and_direction_evidence_disagreement(self) -> None:
        self._mutate_report(lambda report: report["directions"]["home_to_park"].update({"status": "failed"}),
                            synchronize_directions=False)
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_rejects_changed_authoritative_csv_before_bundle_creation(self) -> None:
        csv = self.validation.parent / "home_to_park/joint_trajectory.csv"
        csv.write_text(csv.read_text() + "\n")
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_revalidates_authoritative_endpoint_even_with_matching_new_csv_hash(self) -> None:
        def wrong_endpoint(compiled: CompiledTrajectory) -> CompiledTrajectory:
            last = compiled.samples[-1]
            moved = (last.positions[0] + .02, *last.positions[1:])
            return CompiledTrajectory(compiled.joint_names, (compiled.samples[0],
                JointSample(last.time_s, moved, last.velocities, last.accelerations, last.phase)))
        self._replace_authority(wrong_endpoint)
        with self.assertRaisesRegex(ValueError, "start/target|endpoint|target differs"):
            self._create()

    def test_revalidates_continuous_dynamics_even_with_matching_new_csv_hash(self) -> None:
        def too_fast(compiled: CompiledTrajectory) -> CompiledTrajectory:
            last = compiled.samples[-1]
            return CompiledTrajectory(compiled.joint_names, (compiled.samples[0],
                JointSample(.1, last.positions, last.velocities, last.accelerations, last.phase)))
        self._replace_authority(too_fast)
        with self.assertRaisesRegex(ValueError, "joint dynamics"):
            self._create()

    def test_revalidates_static_endpoints_even_with_matching_new_csv_hash(self) -> None:
        def moving_endpoint(compiled: CompiledTrajectory) -> CompiledTrajectory:
            last = compiled.samples[-1]
            velocity = (.01, *last.velocities[1:])
            return CompiledTrajectory(compiled.joint_names, (compiled.samples[0],
                JointSample(last.time_s, last.positions, velocity, last.accelerations, last.phase)))
        self._replace_authority(moving_endpoint)
        with self.assertRaisesRegex(ValueError, "zero velocity"):
            self._create()

    def test_rejects_non_nanosecond_authoritative_clock_without_rounding_it(self) -> None:
        csv = self.validation.parent / "home_to_park/joint_trajectory.csv"
        text = csv.read_text()
        self.assertIn("20.000000000,", text)
        csv.write_text(text.replace("20.000000000,", "20.0000000001,"))
        self._mutate_report(lambda report: report["directions"]["home_to_park"].update({
            "joint_trajectory_sha256": sha256(csv)}))
        with self.assertRaisesRegex(ValueError, "nanosecond"):
            self._create()

    def test_rejects_bad_gpu_metrics_even_when_report_and_evidence_agree(self) -> None:
        for key, invalid in (("minimum_collision_distance_m", .004), ("max_scene_collision_cost", .1),
                             ("max_self_collision_cost", .1), ("max_joint_bound_cost", .1),
                             ("checked_sample_count", 1)):
            with self.subTest(key=key):
                original = self.validation.read_text()
                def change(report: dict[str, Any]) -> None:
                    for metric in ("gpu_metrics", "serialized_gpu_metrics"):
                        report["directions"]["home_to_park"][metric][key] = invalid
                self._mutate_report(change)
                with self.assertRaises((ValueError, RuntimeError)):
                    self._create()
                self.validation.write_text(original)

    def test_rejects_dynamics_claim_inconsistent_with_authority(self) -> None:
        self._mutate_report(lambda report: report["directions"]["home_to_park"]["dynamics"].update({"duration_s": 19.}))
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_rejects_wrong_curobo_revision(self) -> None:
        self._mutate_report(lambda report: report["directions"]["home_to_park"].update({"curobo_revision": "0" * 40}))
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_rejects_changed_producer_hash(self) -> None:
        self._mutate_report(lambda report: report["code_sha256"].update({"motion.py": "0" * 64}))
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()

    def test_accepts_original_producer_source_alias_snapshots(self) -> None:
        snapshot = self.root / "producer_at_launch"
        snapshot.mkdir()
        for name in PRODUCER_NAMES:
            shutil.copy2(self.root / "collection" / name,
                         snapshot / f"{Path(name).stem}_source_at_launch.py")
        self._create(producer_code_root=snapshot)
        self.assertIsNone(self._load().verify())

    def test_rejects_tampered_producer_source_snapshot(self) -> None:
        snapshot = self.root / "producer_at_launch"
        snapshot.mkdir()
        for name in PRODUCER_NAMES:
            shutil.copy2(self.root / "collection" / name,
                         snapshot / f"{Path(name).stem}_source_at_launch.py")
        path = snapshot / "motion_source_at_launch.py"
        path.write_text(path.read_text() + "\n# unit test tamper\n")
        with self.assertRaises((ValueError, RuntimeError)):
            self._create(producer_code_root=snapshot)

    def test_old_producer_snapshot_is_not_relabelled_as_current_consumer(self) -> None:
        snapshot = self.root / "older_producer_at_launch"
        snapshot.mkdir()
        for name in PRODUCER_NAMES:
            text = (self.root / "collection" / name).read_text()
            (snapshot / f"{Path(name).stem}_source_at_launch.py").write_text(
                text + "\n# Synthetic older producer version for unit test only.\n")
        self._mutate_report(lambda report: report.update({"code_sha256": {
            name: sha256(snapshot / f"{Path(name).stem}_source_at_launch.py")
            for name in PRODUCER_NAMES}}))
        manifest = self._create(producer_code_root=snapshot)
        self.assertNotEqual(manifest["producer_code_sha256"]["motion.py"],
                            manifest["consumer_code_sha256"]["motion.py"])
        self.assertEqual(manifest["producer_code_sha256"]["motion.py"],
                         sha256(self.bundle / "producer_code/motion.py"))
        self.assertEqual(manifest["consumer_code_sha256"]["motion.py"],
                         sha256(SOURCE / "collection/motion.py"))
        self.assertIsNone(self._load().verify())

    def test_output_is_immutable_no_overwrite(self) -> None:
        self._create()
        with self.assertRaises((FileExistsError, ValueError, RuntimeError)):
            self._create()

    def test_bundle_remains_valid_after_workspace_relocation_and_source_removal(self) -> None:
        self._create()
        relocated = self.root.parent / "relocated_workspace"
        shutil.copytree(self.root, relocated)
        shutil.rmtree(self.root)
        relative_bundle = self.bundle.relative_to(self.root)
        prepared = self._load(relocated, relocated / relative_bundle)
        self.assertIsNone(prepared.verify())
        for direction in DIRECTIONS:
            self.assertEqual(len(prepared.trajectory(direction)[0].samples), 2)

    def test_environment_changed_after_bundle_creation_is_rejected(self) -> None:
        self._create()
        setup = self.root / "environments/mla_short/setup.yaml"
        setup.write_text(setup.read_text() + "\n# unit test environment drift\n")
        with self.assertRaises((ValueError, RuntimeError)):
            self._load().verify()

    def test_config_changed_after_bundle_creation_is_rejected(self) -> None:
        self._create()
        config = self.root / "collection/config.yaml"
        config.write_text(config.read_text() + "\n# unit test config drift\n")
        with self.assertRaises((ValueError, RuntimeError)):
            self._load().verify()

    def test_lock_changed_after_bundle_creation_is_rejected(self) -> None:
        self._create()
        lock = self.root / "curobo.lock"
        lock.write_text(lock.read_text() + "\n# unit test lock drift\n")
        with self.assertRaises((ValueError, RuntimeError)):
            self._load().verify()

    def test_consumer_source_version_is_rechecked_at_read_time(self) -> None:
        self._create()
        prepared = self._load()
        from . import prepared_parking
        original_hash = prepared_parking.sha256
        actual_motion_source = SOURCE / "collection/motion.py"
        def changed_source(path: Path) -> str:
            return "0" * 64 if path.resolve() == actual_motion_source.resolve() else original_hash(path)
        with patch.object(prepared_parking, "sha256", side_effect=changed_source):
            with self.assertRaisesRegex(ValueError, "consumer code version changed"):
                prepared.verify()

    def test_bundle_csv_mutation_is_rejected_before_consumption(self) -> None:
        self._create()
        prepared = self._load()
        csv = self.bundle / "home_to_park/joint_trajectory.csv"
        csv.write_text(csv.read_text() + "\n")
        with self.assertRaises((ValueError, RuntimeError)):
            prepared.trajectory("home_to_park")

    def test_original_proof_and_source_snapshot_are_sealed(self) -> None:
        self._create()
        for relative in ("source/validation.json", "source/home_to_park/evidence.json",
                         "producer_code/motion.py", "consumer_code/motion.py"):
            with self.subTest(file=relative):
                path = self.bundle / relative
                original = path.read_bytes()
                path.write_bytes(original + b"\n")
                with self.assertRaises((ValueError, RuntimeError)):
                    self._load().verify()
                path.write_bytes(original)

    def test_manifest_or_checksum_changes_are_rejected_for_loaded_bundle(self) -> None:
        self._create()
        prepared = self._load()
        for name in ("bundle.json", "sha256sums.txt"):
            with self.subTest(file=name):
                path = self.bundle / name
                original = path.read_bytes()
                path.write_bytes(original + b"\n")
                with self.assertRaises((ValueError, RuntimeError)):
                    prepared.verify()
                path.write_bytes(original)

    def test_symlink_direct_trajectory_is_not_accepted_even_with_identical_bytes(self) -> None:
        self._create()
        csv = self.bundle / "home_to_park/joint_trajectory.csv"
        other = self.root / "identical_bytes_outside_bundle.csv"
        shutil.copy2(csv, other)
        csv.unlink()
        csv.symlink_to(other)
        with self.assertRaises((ValueError, RuntimeError)):
            self._load().verify()

    def test_unknown_direction_does_not_return_an_unrelated_trajectory(self) -> None:
        self._create()
        with self.assertRaises((ValueError, KeyError)):
            self._load().trajectory("home_to_first_point")


class MinimumJerkPreparedParkingTests(unittest.TestCase):
    """Candidate provenance tests; synthetic GPU values are not a GPU certificate."""

    _api = staticmethod(PreparedParkingTests._api)
    _create = PreparedParkingTests._create
    _load = PreparedParkingTests._load
    _mutate_report = PreparedParkingTests._mutate_report

    @classmethod
    def setUpClass(cls) -> None:
        cls.template_temp = tempfile.TemporaryDirectory()
        cls.template = Path(cls.template_temp.name).resolve() / "synthetic_template"
        cls.template.mkdir()
        make_minimum_jerk_validation(cls.template, SOURCE)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.template_temp.cleanup()

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve() / "workspace"
        shutil.copytree(self.template, self.root)
        self.validation = self.root / "runs/synthetic_unit_test_parking_validation/validation.json"
        self.bundle = self.root / "collection/prepared_parking_minimum_jerk_unit_test_bundle"
        # A copied fixture is a distinct synthetic source. Bind its source paths
        # locally before the test, so relocation cannot silently use the template.
        report = json.loads(self.validation.read_text())
        for direction in DIRECTIONS:
            local = self.validation.parent / direction
            manifest_path = local / "candidate.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["candidate_path"] = str(local / "candidate_joint_trajectory.csv")
            manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
            item = report["directions"][direction]
            item["joint_trajectory_path"] = str(local / "joint_trajectory.csv")
            item["upstream_candidate"] = {**manifest, "manifest_path": str(manifest_path),
                                           "manifest_sha256": sha256(manifest_path)}
            item["prepare_metadata"]["prepare"]["candidate_manifest_sha256"] = sha256(manifest_path)
            (local / "evidence.json").write_text(json.dumps(item, indent=2, sort_keys=True, allow_nan=False) + "\n")
        self.validation.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _candidate(self, direction: str = DIRECTIONS[0]) -> Path:
        return self.validation.parent / direction / "candidate.json"

    def _mutate_candidate(self, mutate: Callable[[dict[str, Any]], None], *,
                          inline_matches: bool = True) -> None:
        path = self._candidate()
        manifest = json.loads(path.read_text())
        mutate(manifest)
        path.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
        digest = sha256(path)
        def update(report: dict[str, Any]) -> None:
            item = report["directions"][DIRECTIONS[0]]
            if inline_matches:
                item["upstream_candidate"] = {**manifest, "manifest_path": str(path), "manifest_sha256": digest}
            else:
                item["upstream_candidate"]["manifest_sha256"] = digest
            item["prepare_metadata"]["prepare"]["candidate_manifest_sha256"] = digest
        self._mutate_report(update)

    def _assert_rejected(self) -> None:
        self.assertFalse(self.bundle.exists())
        with self.assertRaises((ValueError, RuntimeError)):
            self._create()
        self.assertFalse(self.bundle.exists(), "invalid provenance must be rejected before creating a bundle")

    def _replace_candidate_and_formal_bytes(self, data: bytes) -> None:
        local = self.validation.parent / DIRECTIONS[0]
        (local / "candidate_joint_trajectory.csv").write_bytes(data)
        (local / "joint_trajectory.csv").write_bytes(data)
        digest = sha256(local / "candidate_joint_trajectory.csv")
        self._mutate_candidate(lambda value: value.update({"candidate_sha256": digest}))
        def update(report: dict[str, Any]) -> None:
            item = report["directions"][DIRECTIONS[0]]
            item["joint_trajectory_sha256"] = digest
            item["prepare_metadata"]["prepare"]["candidate_sha256"] = digest
        self._mutate_report(update)

    def test_valid_minimum_jerk_bundle_preserves_all_original_source_bytes(self) -> None:
        originals = {str(path.relative_to(self.validation.parent)): path.read_bytes()
                     for path in self.validation.parent.rglob("*") if path.is_file()}
        manifest = self._create()
        self.assertEqual(set(manifest["producer_code_sha256"]), set(PRODUCER_NAMES))
        self.assertTrue(json.loads(self.validation.read_text())["unit_test_fixture"])
        for direction in DIRECTIONS:
            for name in ("evidence.json", "joint_trajectory.csv", "candidate_joint_trajectory.csv", "candidate.json"):
                relative = f"{direction}/{name}"
                self.assertIn(f"source/{relative}", manifest["files"])
                self.assertEqual((self.bundle / "source" / relative).read_bytes(), originals[relative])
                self.assertEqual((self.validation.parent / relative).read_bytes(), originals[relative])
            self.assertEqual((self.bundle / direction / "joint_trajectory.csv").read_bytes(),
                             originals[f"{direction}/candidate_joint_trajectory.csv"])
        self.assertEqual((self.bundle / "source/validation.json").read_bytes(), originals["validation.json"])
        self.assertIsNone(self._load().verify())

    def test_minimum_jerk_bundle_uses_local_copies_after_original_source_is_deleted(self) -> None:
        self._create()
        relocated = self.root.parent / "relocated_minimum_jerk_workspace"
        shutil.copytree(self.root, relocated)
        shutil.rmtree(relocated / "runs")
        relative_bundle = self.bundle.relative_to(self.root)
        shutil.rmtree(self.root)
        prepared = self._load(relocated, relocated / relative_bundle)
        self.assertIsNone(prepared.verify())
        for direction in DIRECTIONS:
            compiled, _ = prepared.trajectory(direction)
            self.assertEqual(len(compiled.samples), 2001)
            self.assertEqual(compiled.duration_s, 20.)

    def test_missing_either_candidate_file_is_rejected(self) -> None:
        for name in ("candidate.json", "candidate_joint_trajectory.csv"):
            with self.subTest(file=name):
                path = self.validation.parent / DIRECTIONS[0] / name
                data = path.read_bytes()
                path.unlink()
                self._assert_rejected()
                path.write_bytes(data)

    def test_tampered_candidate_bytes_are_rejected(self) -> None:
        path = self.validation.parent / DIRECTIONS[0] / "candidate_joint_trajectory.csv"
        path.write_bytes(path.read_bytes() + b"\n")
        self._assert_rejected()

    def test_swapped_forward_and_reverse_candidates_are_rejected(self) -> None:
        forward = self.validation.parent / DIRECTIONS[0] / "candidate_joint_trajectory.csv"
        reverse = self.validation.parent / DIRECTIONS[1] / "candidate_joint_trajectory.csv"
        a, b = forward.read_bytes(), reverse.read_bytes()
        forward.write_bytes(b)
        reverse.write_bytes(a)
        self._assert_rejected()

    def test_candidate_manifest_and_inline_proof_must_be_identical(self) -> None:
        self._mutate_candidate(lambda value: value.update({"created_utc": "2026-10-17T01:00:00+00:00"}),
                               inline_matches=False)
        self._assert_rejected()

    def test_candidate_manifest_digest_cannot_be_changed_only_in_inline_proof(self) -> None:
        self._mutate_report(lambda report: report["directions"][DIRECTIONS[0]]["upstream_candidate"].update({
            "manifest_sha256": "0" * 64}))
        self._assert_rejected()

    def test_minimum_jerk_source_requires_inline_candidate_provenance(self) -> None:
        self._mutate_report(lambda report: report["directions"][DIRECTIONS[0]].pop("upstream_candidate"))
        self._assert_rejected()

    def test_candidate_clock_and_count_are_fixed_even_if_claims_are_synchronized(self) -> None:
        baseline = self._candidate().read_bytes()
        report_bytes = self.validation.read_bytes()
        evidence = self.validation.parent / DIRECTIONS[0] / "evidence.json"
        evidence_bytes = evidence.read_bytes()
        for key, invalid in (("duration_s", 21.), ("duration_ns", 21_000_000_000),
                             ("sample_dt_s", .02), ("sample_dt_ns", 20_000_000), ("point_count", 2000)):
            with self.subTest(key=key):
                self._mutate_candidate(lambda value: value.update({key: invalid}))
                self._assert_rejected()
                self._candidate().write_bytes(baseline)
                self.validation.write_bytes(report_bytes)
                evidence.write_bytes(evidence_bytes)

    def test_candidate_endpoint_metadata_cannot_redefine_confirmed_parking(self) -> None:
        self._mutate_candidate(lambda value: value["target_positions_rad"].__setitem__(0,
                               value["target_positions_rad"][0] + .01))
        self._assert_rejected()

    def test_candidate_joint_order_must_match_frozen_environment(self) -> None:
        self._mutate_candidate(lambda value: value["joint_names"].reverse())
        self._assert_rejected()

    def test_candidate_generator_hash_must_match_original_producer_source(self) -> None:
        self._mutate_candidate(lambda value: value.update({"generator_code_sha256": "0" * 64}))
        self._assert_rejected()

    def test_actual_candidate_clock_must_match_exact_ten_millisecond_grid(self) -> None:
        local = self.validation.parent / DIRECTIONS[0]
        original = (local / "candidate_joint_trajectory.csv").read_bytes()
        mutated = original.replace(b"5.000000000,", b"5.000000001,", 1)
        self.assertNotEqual(original, mutated)
        self._replace_candidate_and_formal_bytes(mutated)
        self._assert_rejected()

    def test_joint_authority_is_rechecked_against_formula_even_after_all_hashes_match(self) -> None:
        environment = EnvironmentBuilder().load(self.root / "environments/mla_short")
        local = self.validation.parent / DIRECTIONS[0]
        csv = local / "candidate_joint_trajectory.csv"
        compiled = read_joint_trajectory_csv(csv, environment.joint_names)
        middle = compiled.samples[1000]
        changed = JointSample(middle.time_s,
            (math.nextafter(middle.positions[0], math.inf), *middle.positions[1:]),
            middle.velocities, middle.accelerations, middle.phase)
        samples = (*compiled.samples[:1000], changed, *compiled.samples[1001:])
        alternate = self.root / "one_ulp_candidate_for_unit_test.csv"
        write_joint_csv_exact(alternate, CompiledTrajectory(compiled.joint_names, samples))
        self._replace_candidate_and_formal_bytes(alternate.read_bytes())
        self._assert_rejected()

    def test_candidate_provenance_cannot_be_bypassed_by_removing_source_kind(self) -> None:
        self._mutate_report(lambda report: report.pop("source_kind"))
        self._assert_rejected()

    def test_formal_csv_must_match_candidate_bytes_not_just_joint_values(self) -> None:
        csv = self.validation.parent / DIRECTIONS[0] / "joint_trajectory.csv"
        original = csv.read_bytes()
        # Different decimal spelling parses to identical q/qd/qdd/time, but the
        # formal replay must still be the exact authoritative candidate bytes.
        mutated = original.replace(b"20.000000000,", b"20.0,", 1)
        self.assertNotEqual(mutated, original)
        csv.write_bytes(mutated)
        self._mutate_report(lambda report: report["directions"][DIRECTIONS[0]].update({
            "joint_trajectory_sha256": sha256(csv)}))
        self._assert_rejected()

    def test_candidate_copy_inside_bundle_remains_covered_by_seal(self) -> None:
        self._create()
        for name in ("candidate_joint_trajectory.csv", "candidate.json"):
            with self.subTest(file=name):
                path = self.bundle / "source" / DIRECTIONS[0] / name
                data = path.read_bytes()
                path.write_bytes(data + b"\n")
                with self.assertRaises((ValueError, RuntimeError)):
                    self._load().verify()
                path.write_bytes(data)


if __name__ == "__main__":
    unittest.main()
