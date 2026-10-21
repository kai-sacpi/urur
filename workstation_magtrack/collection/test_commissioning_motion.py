"""Pure CPU commissioning adapter tests: synthetic certificates, no hardware."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder

from .commissioning import create_trial
from .commissioning_motion import CommissioningMotionController
from .manifest import create, immutable_json, require_real_release, sha
from .motion import MotionController
from .prepared_parking import create_bundle
from .test_motion import FakeBackend
from .test_prepared_parking import make_validation


SOURCE = Path(__file__).resolve().parents[1]


class CommissioningMotionTests(unittest.TestCase):
    """Synthetic fixture certificates only exercise consumption, never certify GPU."""

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
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
        source = self.root / "environments/mla_short/trajectories/unit_route.csv"
        source.parent.mkdir()
        source.write_text("synthetic authority for adapter consumption tests\n")
        directory = self.root / "environments/mla_short/recordings/unit_route__sealed"
        directory.mkdir(parents=True)
        (directory / "source_tip_trajectory.csv").write_bytes(source.read_bytes())
        (directory / "joint_trajectory.csv").write_text("synthetic drawing joints\n")
        (directory / "manifest.yaml").write_text("schema_version: 2\n")
        names = ("source_tip_trajectory.csv", "joint_trajectory.csv", "manifest.yaml")
        (directory / "sha256sums.txt").write_text("".join(
            f"{sha(directory / name)}  {name}\n" for name in names))
        inventory = self.root / "inventory.json"
        immutable_json(inventory, {"environment_id": "mla_short", "recordings": [{
            "trajectory_id": "unit_route", "recording_id": directory.name,
            "source_sha256": sha(source), "duration_s": 60.,
            **{phase: "passed" for phase in ("compile", "build", "inspect", "play")},
            "coherent_controller_witness": "passed",
            "ready_for_simulation_replay_on_this_host": True,
            "real_execution_authorized": False,
        }]})
        self.parent = self.root / "reports/unit_test_delivery/collection_manifest.json"
        value = create(self.root, [inventory], self.config)
        value["parking_prepare_bundle"] = {
            "asset_type": "collision_checked_prepare_joint_bundle", "tipdraw_recording": False,
            "independent_mock_status": "passed", "real_execution_authorized": False,
            "prepared_bundle": self.bundle.relative_to(self.root).as_posix(),
        }
        immutable_json(self.parent, value)
        self.trial = create_trial(self.root, self.parent, "unit_route",
                                  "已连接真机，确认现场后开始一次试采。", "unit_trial")
        self.originals = {name: sha(SOURCE / "collection" / name)
                          for name in ("motion.py", "geometry.py", "prepared_parking.py", "config.yaml")}

    def adapter(self, **overrides):
        kwargs = {"workspace": self.root, "config_path": self.config, "manifest": self.trial,
                  "evidence_root": self.root / "trial_motions", "prepared_parking": self.bundle,
                  "backend": self.backend, "domain_id": 0}
        kwargs.update(overrides)
        # This temporary fixture has no actual GPU/mock run; only the existing
        # full provenance verifier is replaced.  Bundle itself is checked normally.
        with patch(f"{__package__}.run.prepared_parking_path", return_value=self.bundle) as verifier:
            result = CommissioningMotionController(**kwargs)
            verifier.assert_called_once_with(self.root, self.config, self.trial)
        return result

    def test_invalid_trial_rejected_before_environment_or_backend_construction(self) -> None:
        with patch(f"{__package__}.commissioning_motion.require_trial", side_effect=ValueError("not a trial")), \
                patch.object(EnvironmentBuilder, "load") as environment, \
                patch(f"{__package__}.motion.RosMotionBackend") as transport:
            with self.assertRaisesRegex(ValueError, "not a trial"):
                CommissioningMotionController(self.root, self.config, {}, self.root / "out", self.bundle)
            environment.assert_not_called()
            transport.assert_not_called()
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_only_real_domain_zero_allowed_before_parking_verifier(self) -> None:
        for domain in (True, False, -1, 1, 219, 220, 221, 222, 223, 233, "0"):
            with self.subTest(domain=domain), \
                    patch(f"{__package__}.run.prepared_parking_path") as parking:
                with self.assertRaisesRegex(ValueError, "ROS_DOMAIN_ID=0"):
                    CommissioningMotionController(self.root, self.config, self.trial,
                        self.root / "out", self.bundle, backend=self.backend, domain_id=domain)
                parking.assert_not_called()

    def test_default_domain_is_checked_against_environment(self) -> None:
        with patch.dict("os.environ", {"ROS_DOMAIN_ID": "223"}):
            with self.assertRaisesRegex(ValueError, "ROS_DOMAIN_ID=0"):
                self.adapter(domain_id=None)

    def test_real_authority_does_not_fake_formal_release_or_mutate_inputs(self) -> None:
        original = copy.deepcopy(self.trial)
        config_bytes = self.config.read_bytes()
        motion = self.adapter()
        self.assertEqual(motion.mode, "real")
        self.assertEqual(motion.domain_id, 0)
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)
        self.assertEqual(self.trial, original)
        self.assertEqual(self.config.read_bytes(), config_bytes)
        self.assertTrue(motion.config["execution"]["simulation_only"])
        self.assertFalse(motion.config["execution"]["real_execution_authorized"])
        with self.assertRaises(ValueError):
            require_real_release(self.trial)
        with self.assertRaises(PermissionError):
            MotionController(self.root, self.config, mode="real", real_release=True,
                             domain_id=0, backend=self.backend)

    def test_super_constructor_is_not_used_as_a_simulation_bypass(self) -> None:
        with patch.object(MotionController, "__init__", side_effect=AssertionError("must not call")):
            self.assertEqual(self.adapter().mode, "real")

    def test_config_must_be_original_parent_bound_file(self) -> None:
        alternate = self.root / "same_bytes.yaml"
        alternate.write_bytes(self.config.read_bytes())
        with self.assertRaisesRegex(ValueError, "unchanged parent-bound"):
            self.adapter(config_path=alternate)
        self.config.write_bytes(self.config.read_bytes() + b"\n# changed\n")
        with self.assertRaisesRegex(ValueError, "frozen collection config changed"):
            self.adapter()

    def test_parking_path_must_match_fully_verified_binding(self) -> None:
        for bad in (None, self.root / "other_bundle"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, "exact parent-bound prepared bundle"):
                self.adapter(prepared_parking=bad)

    def test_full_parking_proof_failure_is_not_converted_to_permission(self) -> None:
        with patch(f"{__package__}.run.prepared_parking_path", side_effect=ValueError("native mock failed")), \
                patch(f"{__package__}.motion.RosMotionBackend") as transport:
            with self.assertRaisesRegex(ValueError, "native mock failed"):
                CommissioningMotionController(self.root, self.config, self.trial,
                    self.root / "out", self.bundle, backend=self.backend, domain_id=0)
            transport.assert_not_called()

    def test_inherited_exact_prepared_parking_and_home_have_no_replanning(self) -> None:
        motion = self.adapter()
        originals = {direction: (self.bundle / direction / "joint_trajectory.csv").read_bytes()
                     for direction in ("home_to_park", "park_to_home")}
        for direction, method in (("home_to_park", motion.move_park), ("park_to_home", motion.move_home)):
            result = method()
            self.assertTrue(result["passed"])
            self.assertEqual(result["execution_mode"], "real")
            self.assertFalse(result["planning_performed"])
            self.assertEqual(Path(result["joint_trajectory_path"]).read_bytes(), originals[direction])
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 2)

    def test_strict_parking_never_replans_but_explicit_endpoint_home_does(self) -> None:
        motion = self.adapter()
        self.backend.position = tuple((a+b)/2 for a,b in zip(motion.home, motion.park))
        for method in (motion.move_park, motion.move_home):
            with self.assertRaisesRegex(ValueError, "fresh start mismatch; no implicit replanning"):
                method()
        self.assertEqual(self.backend.plan_calls, 0)
        self.assertEqual(self.backend.execute_calls, 0)
        result = motion.move_home(from_trajectory_end=True)
        self.assertTrue(result["passed"])
        self.assertTrue(result["planning_performed"])
        self.assertEqual(result["motion_source"], "dynamic_collision_checked_prepare")
        self.assertEqual(self.backend.plan_calls, 1)
        self.assertEqual(self.backend.execute_calls, 1)

    def test_dynamic_home_retains_planner_environment_gate(self) -> None:
        motion = self.adapter()
        self.backend.position = tuple((a+b)/2 for a,b in zip(motion.home, motion.park))
        self.backend.bad_hash = True
        with self.assertRaisesRegex(ValueError, "environment/revision"):
            motion.move_home(from_trajectory_end=True)
        self.assertEqual(self.backend.execute_calls, 0)

    def test_trial_assets_and_parent_rechecked_before_backend_access(self) -> None:
        motion = self.adapter()
        self.parent.write_text(self.parent.read_text() + " ")
        with patch.object(self.backend, "runtime_identity") as identity:
            with self.assertRaisesRegex(ValueError, "parent manifest SHA256 changed"):
                motion.move_park()
            identity.assert_not_called()
        self.assertEqual(self.backend.execute_calls, 0)

    def test_internal_trial_cannot_be_broadened_after_construction(self) -> None:
        motion = self.adapter()
        motion.commissioning_manifest["commissioning"]["scope"] = "all_recordings"
        with patch.object(self.backend, "runtime_identity") as identity:
            with self.assertRaisesRegex(ValueError, "authority changed"):
                motion.move_park()
            identity.assert_not_called()

    def test_external_trial_mutation_does_not_mutate_bound_authority(self) -> None:
        motion = self.adapter()
        self.trial["commissioning"]["scope"] = "all_recordings"
        self.assertEqual(motion.commissioning_manifest["commissioning"]["scope"], "single_trial_commissioning")
        motion.verify_runtime_identity()

    def test_original_consumers_and_config_unchanged(self) -> None:
        motion = self.adapter()
        motion.move_park()
        self.assertEqual(self.originals, {name: sha(SOURCE / "collection" / name) for name in self.originals})
        manifest = json.loads((self.bundle / "bundle.json").read_text())
        self.assertEqual(manifest["consumer_code_sha256"],
                         {name: self.originals[name] for name in ("motion.py", "geometry.py", "prepared_parking.py")})


if __name__ == "__main__":
    unittest.main()
