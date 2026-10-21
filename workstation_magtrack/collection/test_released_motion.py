"""Pure CPU raw-authority adapter tests; no physical/GPU acceptance claims."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import yaml

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder

from .manifest import sha
from .motion import MotionController
from .prepared_parking import create_bundle
from .released_motion import ReleasedMotionController
from .stationary_guard import RawStationaryMonitor
from .test_motion import FakeBackend
from .test_prepared_parking import make_validation


SOURCE = Path(__file__).resolve().parents[1]


class ReleasedMotionTests(unittest.TestCase):
    """Only adapter consumption is tested with a synthetic release verifier.

    The separate release tests cover real schema/provenance checks. Synthetic
    parking certificates below are never presented as actual GPU evidence.
    """

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        shutil.copytree(SOURCE / "environments/mla_short", self.root / "environments/mla_short",
                        ignore=shutil.ignore_patterns("trajectories", "recordings"))
        (self.root / "collection").mkdir()
        self.config = self.root / "collection/config.yaml"
        shutil.copy2(SOURCE / "collection/config.yaml", self.config)
        shutil.copy2(SOURCE / "curobo.lock", self.root / "curobo.lock")
        self.environment = EnvironmentBuilder().load(self.root / "environments/mla_short")
        self.backend = FakeBackend(self.root, self.environment, self.environment.home_joint_positions)
        self.backend.mode = "real"
        validation = make_validation(self.root, SOURCE)
        self.bundle = self.root / "collection/prepared_parking/synthetic_unit_test_bundle"
        create_bundle(self.root, validation, self.bundle)
        self.manifest = {"config": yaml.safe_load(self.config.read_text()),
                         "config_sha256": sha(self.config),
                         "formal_release": {"synthetic_test_only": True}}
        self.authority = {"scope": "operator_authorized_raw_collection",
                          "formal_calibrated_release": False,
                          "labels": "actual_joint_derived_nominal_rod_tip"}
        self.verifier = patch(f"{__package__}.released_motion.verify_release",
                              return_value=self.authority).start()
        self.addCleanup(patch.stopall)
        self.originals = {name: sha(SOURCE / "collection" / name)
                          for name in ("motion.py", "geometry.py", "prepared_parking.py", "config.yaml")}

    def adapter(self, **overrides):
        options = {"workspace": self.root, "config_path": self.config, "manifest": self.manifest,
                   "evidence_root": self.root / "raw_motions", "prepared_parking": self.bundle,
                   "backend": self.backend, "domain_id": 0}
        options.update(overrides)
        # The real full-proof entry is exercised separately; no synthetic fixture
        # can pretend that its native GPU/mock run actually happened.
        with patch(f"{__package__}.run.prepared_parking_path", return_value=self.bundle) as proof:
            result = ReleasedMotionController(**options)
            proof.assert_called_once_with(self.root, self.config, self.manifest)
        return result

    def test_release_rejected_before_environment_or_transport_construction(self) -> None:
        self.verifier.side_effect = ValueError("release unavailable")
        with patch.object(EnvironmentBuilder, "load") as environment, patch(
                f"{__package__}.motion.RosMotionBackend") as backend:
            with self.assertRaisesRegex(ValueError, "release unavailable"):
                self.adapter()
            environment.assert_not_called()
            backend.assert_not_called()
        self.assertEqual(self.backend.execute_calls, 0)

    def test_wrong_or_calibrated_scope_cannot_enter_raw_adapter(self) -> None:
        for value in ({"scope": "single_trial_commissioning", "formal_calibrated_release": False},
                      {"scope": "operator_authorized_raw_collection", "formal_calibrated_release": True},
                      {"scope": "operator_authorized_raw_collection"}):
            self.verifier.return_value = value
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "raw-only"):
                self.adapter()

    def test_only_domain_zero_allowed(self) -> None:
        for domain in (True, False, -1, 1, 219, 220, 221, 223, 233, "0"):
            with self.subTest(domain=domain), patch(
                    f"{__package__}.run.prepared_parking_path") as parking:
                with self.assertRaisesRegex(ValueError, "ROS_DOMAIN_ID=0"):
                    self.adapter(domain_id=domain)
                parking.assert_not_called()
        with patch.dict("os.environ", {"ROS_DOMAIN_ID": "223"}):
            with self.assertRaisesRegex(ValueError, "ROS_DOMAIN_ID=0"):
                self.adapter(domain_id=None)

    def test_nominal_config_and_original_consumer_are_not_relabelled(self) -> None:
        original_manifest, config_bytes = copy.deepcopy(self.manifest), self.config.read_bytes()
        motion = self.adapter()
        self.assertEqual(motion.mode, "real")
        self.assertTrue(motion.config["execution"]["simulation_only"])
        self.assertFalse(motion.config["execution"]["real_execution_authorized"])
        self.assertEqual(self.manifest, original_manifest)
        self.assertEqual(self.config.read_bytes(), config_bytes)
        with self.assertRaises(PermissionError):
            MotionController(self.root, self.config, mode="real", real_release=True,
                             domain_id=0, backend=self.backend)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_simulation_constructor_is_never_used_as_a_bypass(self) -> None:
        with patch.object(MotionController, "__init__", side_effect=AssertionError("do not call")):
            self.assertEqual(self.adapter().mode, "real")

    def test_raw_only_stationary_lifecycle_adapter_keeps_original_consumer(self) -> None:
        motion = self.adapter()
        path = self.root / "raw_stationary.csv"
        guard = motion.stationary_monitor(path, duration_s=30.)
        self.assertIsInstance(guard, RawStationaryMonitor)
        self.assertIs(guard.motion, motion)
        self.assertEqual(guard.path, path)
        self.assertEqual(guard.duration_s, 30.)
        self.assertEqual(self.originals["motion.py"], sha(SOURCE / "collection/motion.py"))

    def test_motion_revalidates_metadata_without_rehashing_whole_inventory(self) -> None:
        motion = self.adapter()
        self.verifier.assert_called_once_with(self.root, self.manifest, verify_recordings=False)
        self.verifier.reset_mock()
        motion.verify_runtime_identity()
        self.verifier.assert_called_once_with(
            self.root, motion.released_manifest, verify_recordings=False)

    def test_original_config_path_and_sha_required(self) -> None:
        alternative = self.root / "same_bytes.yaml"
        alternative.write_bytes(self.config.read_bytes())
        with self.assertRaisesRegex(ValueError, "unchanged parent-bound"):
            self.adapter(config_path=alternative)
        self.config.write_bytes(self.config.read_bytes()+b"\n# changed\n")
        with self.assertRaisesRegex(ValueError, "unchanged parent-bound"):
            self.adapter()

    def test_exact_parent_bound_parking_required(self) -> None:
        for bad in (None, self.root / "other_bundle"):
            with self.subTest(path=bad), self.assertRaisesRegex(ValueError, "exact parent-bound"):
                self.adapter(prepared_parking=bad)
        with patch(f"{__package__}.run.prepared_parking_path",
                   side_effect=ValueError("native mock failed")):
            with self.assertRaisesRegex(ValueError, "native mock failed"):
                ReleasedMotionController(self.root, self.config, self.manifest,
                    self.root / "out", self.bundle, backend=self.backend, domain_id=0)

    def test_inherits_unscaled_exact_prepared_paths_and_fresh_actual_state(self) -> None:
        motion = self.adapter()
        originals = {direction: (self.bundle / direction / "joint_trajectory.csv").read_bytes()
                     for direction in ("home_to_park", "park_to_home")}
        for direction, method in (("home_to_park", motion.move_park), ("park_to_home", motion.move_home)):
            evidence = method()
            self.assertTrue(evidence["passed"])
            self.assertEqual(evidence["execution_mode"], "real")
            self.assertFalse(evidence["planning_performed"])
            self.assertEqual(Path(evidence["joint_trajectory_path"]).read_bytes(), originals[direction])
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 2)

    def test_unknown_start_not_replanned_except_explicit_drawing_end_home(self) -> None:
        motion = self.adapter()
        self.backend.position = tuple((a+b)/2 for a,b in zip(motion.home, motion.park))
        for method in (motion.move_park, motion.move_home):
            with self.assertRaisesRegex(ValueError, "fresh start mismatch"):
                method()
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)
        evidence = motion.move_home(from_trajectory_end=True)
        self.assertTrue(evidence["passed"])
        self.assertEqual(evidence["motion_source"], "dynamic_collision_checked_prepare")
        self.assertEqual(self.backend.plan_calls, 1)

    def test_frozen_environment_and_full_path_gate_remain(self) -> None:
        motion = self.adapter()
        self.backend.position = tuple((a+b)/2 for a,b in zip(motion.home, motion.park))
        self.backend.bad_hash = True
        with self.assertRaisesRegex(ValueError, "environment/revision"):
            motion.move_home(from_trajectory_end=True)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_original_runtime_identity_gate_remains(self) -> None:
        motion = self.adapter()
        self.backend.mode = "simulation"
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            motion.move_park()
        self.assertEqual(self.backend.execute_calls, 0)

    def test_runtime_identity_has_dedicated_ten_second_timeout(self) -> None:
        motion = self.adapter()
        with patch.object(self.backend, "runtime_identity", wraps=self.backend.runtime_identity) as identity:
            result = motion.verify_runtime_identity()
        identity.assert_called_once_with(10.)
        self.assertTrue(result["verified"])
        self.assertEqual(result["domain_id"], 0)

    def test_runtime_identity_respects_a_shorter_planning_budget(self) -> None:
        motion = self.adapter()
        motion.options["planning_timeout_s"] = 3.
        with patch.object(self.backend, "runtime_identity", wraps=self.backend.runtime_identity) as identity:
            motion.verify_runtime_identity()
        identity.assert_called_once_with(3.)

    def test_each_wrong_runtime_identity_field_is_rejected_without_action(self) -> None:
        motion = self.adapter()
        valid = dict(self.backend.runtime_identity(10.))
        for field, value in (("execution_mode", "simulation"),
                             ("environments_root", str(self.root / "other_environments")),
                             ("telemetry_root", str(self.root / "other_runs")),
                             ("use_sim_time", True), ("use_sim_time", 0)):
            with self.subTest(field=field, value=value), patch.object(
                    self.backend, "runtime_identity", return_value={**valid, field: value}) as identity:
                with self.assertRaisesRegex(ValueError, "runtime identity mismatch"):
                    motion.move_park()
                identity.assert_called_once_with(10.)
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_runtime_timeout_propagates_once_without_motion_or_retry(self) -> None:
        motion = self.adapter()
        error = TimeoutError("runtime identity unavailable")
        with patch.object(self.backend, "runtime_identity", side_effect=error) as identity:
            with self.assertRaisesRegex(TimeoutError, "runtime identity unavailable") as caught:
                motion.move_park()
            self.assertIs(caught.exception, error)
            identity.assert_called_once_with(10.)
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_runtime_domain_guard_precedes_backend_access(self) -> None:
        motion = self.adapter()
        motion.domain_id = 1
        with patch.object(self.backend, "runtime_identity") as identity:
            with self.assertRaisesRegex(ValueError, "ROS_DOMAIN_ID=0"):
                motion.verify_runtime_identity()
            identity.assert_not_called()

    def test_reverification_failure_precedes_transport(self) -> None:
        motion = self.adapter()
        self.verifier.side_effect = ValueError("accepted source changed")
        with patch.object(self.backend, "runtime_identity") as identity:
            with self.assertRaisesRegex(ValueError, "accepted source changed"):
                motion.move_park()
            identity.assert_not_called()

    def test_internal_authority_cannot_change_after_construction(self) -> None:
        for target in ("released_manifest", "release"):
            motion = self.adapter()
            getattr(motion, target)["changed"] = True
            with self.subTest(target=target), patch.object(self.backend, "runtime_identity") as identity:
                with self.assertRaisesRegex(ValueError, "authority changed"):
                    motion.move_park()
                identity.assert_not_called()

    def test_external_mutation_cannot_broaden_internal_authority(self) -> None:
        motion = self.adapter()
        self.manifest["formal_release"]["scope"] = "all_unknown_recordings"
        self.assertNotIn("scope", motion.released_manifest["formal_release"])
        motion.verify_runtime_identity()

    def test_verified_authority_change_is_rejected(self) -> None:
        motion = self.adapter()
        self.verifier.return_value = {**self.authority, "changed": True}
        with self.assertRaisesRegex(ValueError, "verified installation release changed"):
            motion.verify_runtime_identity()

    def test_sealed_consumer_and_config_still_byte_identical(self) -> None:
        motion = self.adapter()
        motion.move_park()
        self.assertEqual(self.originals, {name: sha(SOURCE / "collection" / name)
                                         for name in self.originals})
        manifest = json.loads((self.bundle / "bundle.json").read_text())
        self.assertEqual(manifest["consumer_code_sha256"],
                         {name: self.originals[name] for name in ("motion.py", "geometry.py", "prepared_parking.py")})


if __name__ == "__main__":
    unittest.main()
