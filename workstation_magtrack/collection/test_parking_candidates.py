"""CPU-only analytic-candidate regressions; synthetic adapter is NOT GPU evidence.

The real continuous-quintic dynamics and full emitted-row CPU geometry checks
remain enabled. No GPU, ROS, SSH, serial, or robot is used by this test module.
"""
from __future__ import annotations

from contextlib import contextmanager
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
from typing import Any, Callable, Iterator
import unittest
from unittest.mock import patch

import yaml

from hx_waypoint_curobo.curobo_adapter_v2 import PathValidationMetrics
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample

from . import validate_parking as subject
from .motion import MotionController, sha256, validate_trajectory
from .prepared_parking import _joint_bytes


SOURCE = Path(__file__).resolve().parents[1]
DIRECTIONS = ("home_to_park", "park_to_home")


class SyntheticGpuAdapter:
    """Explicit unit-test substitute: never claim these metrics are real CUDA."""

    def __init__(self, environment: Any, revision: str,
                 callback: Callable[[CompiledTrajectory, int], None] | None = None):
        self.environment = environment
        self.revision = revision
        self.joint_names = environment.joint_names
        self.callback = callback
        self.construction_calls = 0
        self.warmup_calls = 0
        self.trajectories: list[CompiledTrajectory] = []

    def warmup(self) -> None:
        self.warmup_calls += 1

    def validate_joint_trajectory(self, compiled: CompiledTrajectory) -> PathValidationMetrics:
        self.trajectories.append(compiled)
        if self.callback:
            self.callback(compiled, len(self.trajectories))
        return PathValidationMetrics(0., 0., .03, 0., 0., 0., len(compiled.samples))


class ParkingCandidateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        shutil.copytree(SOURCE / "environments/mla_short", self.root / "environments/mla_short",
                        ignore=shutil.ignore_patterns("trajectories", "recordings"))
        (self.root / "collection").mkdir()
        shutil.copy2(SOURCE / "collection/config.yaml", self.root / "collection/config.yaml")
        shutil.copy2(SOURCE / "curobo.lock", self.root / "curobo.lock")
        self.motion = MotionController(self.root, mode="simulation", domain_id=221)
        self.environment = self.motion.environment
        self.home, self.park = self.motion.home, self.motion.park
        self.names = self.environment.joint_names
        self.revision = yaml.safe_load((self.root / "curobo.lock").read_text())["commit"]
        self.output = self.root / "runs/synthetic_unit_test_candidate_validation"
        self.config = self.root / "collection/config.yaml"

    def tearDown(self) -> None:
        self.motion.close()
        self.temp.cleanup()

    def candidate(self, start: tuple[float, ...] | None = None,
                  target: tuple[float, ...] | None = None, **kwargs: Any) -> CompiledTrajectory:
        return subject.minimum_jerk_joint_bridge(
            self.names, self.home if start is None else start,
            self.park if target is None else target, self.revision, **kwargs)

    @contextmanager
    def synthetic_gpu(self, callback: Callable[[CompiledTrajectory, int], None] | None = None
                      ) -> Iterator[tuple[SyntheticGpuAdapter, Any]]:
        adapter = SyntheticGpuAdapter(self.environment, self.revision, callback)
        def construct(environment: Any, lock: Path) -> SyntheticGpuAdapter:
            adapter.construction_calls += 1
            return adapter
        with (patch("hx_waypoint_curobo.curobo_adapter_v2.CuroboAdapterV2", side_effect=construct),
              patch("hx_waypoint_curobo.prepare_planner.PreparePlanner") as planner,
              patch.object(subject, "gpu_inventory", return_value={
                  "stdout": "SYNTHETIC_UNIT_TEST_NO_GPU_USED"})):
            yield adapter, planner

    def run_validation(self, **kwargs: Any) -> dict[str, object]:
        return subject.validate(self.root, self.config, self.output,
                                source_kind=subject.MINIMUM_JERK_SOURCE_KIND, **kwargs)

    def test_both_directions_exact_ticks_analytic_derivatives_and_original_dynamics(self) -> None:
        for start, target in ((self.home, self.park), (self.park, self.home)):
            with self.subTest(start=start):
                trajectory = self.candidate(start, target)
                self.assertEqual(len(trajectory.samples), 2001)
                self.assertEqual(trajectory.duration_s, 20.)
                ticks = [int(Decimal(f"{sample.time_s:.9f}") * 1_000_000_000)
                         for sample in trajectory.samples]
                self.assertEqual(ticks, list(range(0, 20_000_000_001, 10_000_000)))
                self.assertEqual(trajectory.samples[0].positions, start)
                self.assertEqual(trajectory.samples[-1].positions, target)
                for sample in (trajectory.samples[0], trajectory.samples[-1]):
                    self.assertEqual(sample.velocities, (0.,) * 6)
                    self.assertEqual(sample.accelerations, (0.,) * 6)
                for index, sample in enumerate(trajectory.samples[1:-1], 1):
                    u = index / 2000
                    for joint, (a, b) in enumerate(zip(start, target, strict=True)):
                        delta = b - a
                        self.assertTrue(all(math.isfinite(value) for value in (
                            sample.positions[joint], sample.velocities[joint], sample.accelerations[joint])))
                        self.assertAlmostEqual(sample.positions[joint], a + delta * (10*u**3-15*u**4+6*u**5), places=13)
                        self.assertAlmostEqual(sample.velocities[joint], delta * (30*u**2-60*u**3+30*u**4)/20, places=13)
                        self.assertAlmostEqual(sample.accelerations[joint], delta * (60*u-180*u**2+120*u**3)/400, places=13)
                result = validate_trajectory(self.environment, trajectory, start, target)
                self.assertTrue(result["passed"])
                self.assertTrue(result["continuous_quintic_extrema_checked"])
                self.assertEqual(result["point_count"], 2001)
                self.assertEqual(result["uniform_time_scale"], 1.)
                self.assertLess(result["max_joint_velocity_ratio"], .122)
                self.assertLess(result["max_joint_acceleration_ratio"], .0094)
                self.assertLess(result["max_joint_jerk_ratio"], .00049)

    def test_stationary_candidate_is_strictly_static_for_every_row(self) -> None:
        trajectory = self.candidate(self.home, self.home)
        for sample in trajectory.samples:
            self.assertEqual(sample.positions, self.home)
            self.assertEqual(sample.velocities, (0.,) * 6)
            self.assertEqual(sample.accelerations, (0.,) * 6)
        result = validate_trajectory(self.environment, trajectory, self.home, self.home)
        for metric in ("max_joint_velocity_ratio", "max_joint_acceleration_ratio", "max_joint_jerk_ratio"):
            self.assertEqual(result[metric], 0.)

    def test_new_candidate_is_written_before_validation_and_exactly_round_trips(self) -> None:
        directory = self.root / "fresh_upstream_candidate"
        directory.mkdir()
        with patch.object(subject, "validate_trajectory", side_effect=AssertionError("must not admit inside generator")):
            trajectory, proof, data = subject.write_minimum_jerk_candidate(
                directory, self.names, self.home, self.park, self.revision,
                generator_sha256="a" * 64)
        csv = directory / "candidate_joint_trajectory.csv"
        manifest_path = directory / "candidate.json"
        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(csv.read_bytes(), data)
        self.assertEqual(_joint_bytes(data, self.names).samples, trajectory.samples)
        self.assertEqual(proof["candidate_sha256"], hashlib.sha256(data).hexdigest())
        self.assertEqual(proof["manifest_sha256"], sha256(manifest_path))
        self.assertEqual(manifest["generator_code_sha256"], "a" * 64)
        self.assertEqual(manifest["source_kind"], subject.MINIMUM_JERK_SOURCE_KIND)
        self.assertEqual(manifest["duration_ns"], 20_000_000_000)
        self.assertEqual(manifest["sample_dt_ns"], 10_000_000)
        self.assertEqual(manifest["point_count"], 2001)
        self.assertEqual(manifest["start_positions_rad"], list(self.home))
        self.assertEqual(manifest["target_positions_rad"], list(self.park))
        self.assertTrue(manifest["candidate_written_before_validation"])
        self.assertFalse(manifest["runtime_modification_permitted"])
        self.assertFalse(manifest["real_execution_authorized"])
        self.assertEqual(trajectory.metadata["prepare"]["candidate_manifest_sha256"], proof["manifest_sha256"])

    def test_too_fast_fresh_candidate_is_rejected_not_retimed_or_rewritten(self) -> None:
        directory = self.root / "rejected_new_design"
        directory.mkdir()
        trajectory, proof, data = subject.write_minimum_jerk_candidate(
            directory, self.names, self.home, self.park, self.revision, duration_s=.02)
        manifest_bytes = (directory / "candidate.json").read_bytes()
        with self.assertRaisesRegex(ValueError, "joint dynamics; no retiming allowed"):
            validate_trajectory(self.environment, trajectory, self.home, self.park)
        self.assertEqual((directory / "candidate_joint_trajectory.csv").read_bytes(), data)
        self.assertEqual((directory / "candidate.json").read_bytes(), manifest_bytes)
        self.assertEqual(sha256(directory / "candidate_joint_trajectory.csv"), proof["candidate_sha256"])
        self.assertEqual(trajectory.duration_s, .02)
        self.assertFalse((directory / "joint_trajectory.csv").exists())

    def test_producer_preserves_fresh_dynamics_rejection_before_gpu_admission(self) -> None:
        # Design a different, deliberately infeasible upstream source for this
        # isolated test. The producer still uses its unmodified validator.
        too_fast = self.candidate(duration_s=.02)
        with (self.synthetic_gpu() as (adapter, planner),
              patch.object(subject, "minimum_jerk_joint_bridge", return_value=too_fast)):
            with self.assertRaisesRegex(ValueError, "joint dynamics; no retiming allowed"):
                self.run_validation(gpu=True)
        planner.assert_not_called()
        self.assertEqual(adapter.warmup_calls, 1)
        self.assertEqual(adapter.trajectories, [])
        directory = self.output / DIRECTIONS[0]
        data = (directory / "candidate_joint_trajectory.csv").read_bytes()
        self.assertEqual(_joint_bytes(data, self.names).samples, too_fast.samples)
        manifest = json.loads((directory / "candidate.json").read_text())
        self.assertEqual(manifest["candidate_sha256"], hashlib.sha256(data).hexdigest())
        self.assertEqual(manifest["duration_s"], .02)
        self.assertFalse((directory / "joint_trajectory.csv").exists())
        self.assertFalse((self.output / DIRECTIONS[1]).exists())
        report = json.loads((self.output / "validation.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["directions"][DIRECTIONS[0]]["upstream_candidate"]["candidate_sha256"],
                         manifest["candidate_sha256"])

    def test_duplicate_candidate_generation_cannot_overwrite_existing_authority(self) -> None:
        directory = self.root / "exclusive_candidate"
        directory.mkdir()
        subject.write_minimum_jerk_candidate(directory, self.names, self.home, self.park, self.revision)
        before = {path.name: path.read_bytes() for path in directory.iterdir()}
        with self.assertRaises(FileExistsError):
            subject.write_minimum_jerk_candidate(directory, self.names, self.park, self.home, self.revision)
        self.assertEqual({path.name: path.read_bytes() for path in directory.iterdir()}, before)

    def test_invalid_candidate_inputs_are_rejected(self) -> None:
        for kwargs in ({"duration_s": 0.}, {"duration_s": float("nan")}, {"duration_s": True},
                       {"sample_dt_s": -.01}, {"sample_dt_s": .0000000001},
                       {"duration_s": 120.01}, {"duration_s": .015}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.candidate(**kwargs)
        for names, start, target in ((self.names[:-1], self.home, self.park),
                                     ((self.names[0],) * 6, self.home, self.park),
                                     (self.names, (True,) * 6, self.park),
                                     (self.names, ("bad",) * 6, self.park),
                                     (self.names, self.home, (float("inf"),) * 6)):
            with self.subTest(start=start), self.assertRaises(ValueError):
                subject.minimum_jerk_joint_bridge(names, start, target, self.revision)

    def test_full_admission_uses_four_gpu_calls_real_dynamics_and_all_cpu_rows(self) -> None:
        calls_with_source: list[int] = []

        def observe_authority(compiled: CompiledTrajectory, call: int) -> None:
            direction = DIRECTIONS[(call - 1) // 2]
            directory = self.output / direction
            self.assertTrue((directory / "candidate.json").is_file())
            self.assertEqual(_joint_bytes((directory / "candidate_joint_trajectory.csv").read_bytes(), self.names).samples,
                             compiled.samples)
            calls_with_source.append(call)

        with self.synthetic_gpu(observe_authority) as (adapter, planner):
            report = self.run_validation(gpu=True)
        planner.assert_not_called()
        self.assertEqual(adapter.construction_calls, 1)
        self.assertEqual(adapter.warmup_calls, 1)
        self.assertEqual(calls_with_source, [1, 2, 3, 4])
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["source_kind"], subject.MINIMUM_JERK_SOURCE_KIND)
        self.assertEqual(report["gpu_full_path_validation"], "passed_both_directions")
        self.assertEqual(report["robot_execution"], "not_requested_not_run")
        self.assertFalse(report["real_execution_authorized"])
        self.assertEqual(report["ros_runtime"], "not_started")
        for direction in DIRECTIONS:
            directory = self.output / direction
            self.assertEqual((directory / "candidate_joint_trajectory.csv").read_bytes(),
                             (directory / "joint_trajectory.csv").read_bytes())
            item = report["directions"][direction]
            self.assertEqual(item["status"], "passed")
            self.assertIn("upstream analytic minimum-jerk", item["planner"])
            self.assertNotIn("PreparePlanner.plan", item["planner"])
            self.assertEqual(item["cpu_sampled_geometry"]["sample_count"], 2001)
            self.assertEqual(item["cpu_sampled_geometry"]["status"], "CPU_sampled_passed")
            self.assertGreater(item["cpu_sampled_geometry"]["minimum_world_clearance_m"], .005)
            self.assertGreater(item["cpu_sampled_geometry"]["minimum_self_clearance_m"], .005)
            self.assertTrue(item["dynamics"]["continuous_quintic_extrema_checked"])
            self.assertEqual(item["dynamics"]["point_count"], 2001)
            for key in ("gpu_metrics", "serialized_gpu_metrics"):
                self.assertEqual(item[key]["checked_sample_count"], 2001)
            metadata = item["prepare_metadata"]["prepare"]
            self.assertEqual(metadata["curobo_revision"], self.revision)
            self.assertEqual(metadata["source_kind"], subject.MINIMUM_JERK_SOURCE_KIND)
            self.assertEqual(metadata["uniform_time_scale"], 1.)
            self.assertEqual(metadata["duration_s"], 20.)
            self.assertEqual(metadata["sample_dt_s"], .01)
            self.assertEqual(metadata["candidate_sha256"], sha256(directory / "joint_trajectory.csv"))
            self.assertEqual(metadata["candidate_manifest_sha256"], sha256(directory / "candidate.json"))
        self.assertEqual(set(report["code_sha256"]), {"motion.py", "geometry.py", "validate_parking.py"})
        for name, digest in report["code_sha256"].items():
            snapshot = self.output / f"{Path(name).stem}_source_at_launch.py"
            self.assertEqual(snapshot.read_bytes(), Path(subject.__file__).with_name(name).read_bytes())
            self.assertEqual(sha256(snapshot), digest)
        self.assertEqual(json.loads((self.output / "validation.json").read_text()), report)

    def test_gpu_rejection_preserves_original_candidate_and_stops_without_fallback(self) -> None:
        seen: dict[str, bytes] = {}

        def reject(compiled: CompiledTrajectory, call: int) -> None:
            directory = self.output / DIRECTIONS[0]
            seen.update({path.name: path.read_bytes() for path in directory.iterdir()})
            raise ValueError("synthetic unit-test GPU rejection; not production evidence")

        with self.synthetic_gpu(reject) as (adapter, planner):
            with self.assertRaisesRegex(ValueError, "unit-test GPU rejection"):
                self.run_validation(gpu=True)
        planner.assert_not_called()
        self.assertEqual(len(adapter.trajectories), 1)
        directory = self.output / DIRECTIONS[0]
        self.assertEqual({path.name: path.read_bytes() for path in directory.iterdir()}, seen)
        self.assertFalse((self.output / DIRECTIONS[1]).exists())
        self.assertFalse((directory / "joint_trajectory.csv").exists())
        report = json.loads((self.output / "validation.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["runtime_ready"])

    def test_mutated_candidate_or_manifest_fails_without_overwriting_original_bytes(self) -> None:
        for filename in ("candidate_joint_trajectory.csv", "candidate.json"):
            with self.subTest(filename=filename):
                self.output = self.root / "runs" / filename.replace(".", "_")
                original: dict[str, bytes] = {}

                def mutate(compiled: CompiledTrajectory, call: int) -> None:
                    if call != 1:
                        return
                    directory = self.output / DIRECTIONS[0]
                    original["csv"] = (directory / "candidate_joint_trajectory.csv").read_bytes()
                    path = directory / filename
                    original["modified"] = path.read_bytes() + b"\n"
                    path.write_bytes(original["modified"])

                with self.synthetic_gpu(mutate) as (adapter, planner):
                    with self.assertRaisesRegex(ValueError, "immutable upstream candidate"):
                        self.run_validation(gpu=True)
                planner.assert_not_called()
                self.assertEqual(len(adapter.trajectories), 2)
                directory = self.output / DIRECTIONS[0]
                self.assertEqual((directory / "joint_trajectory.csv").read_bytes(), original["csv"])
                self.assertEqual((directory / filename).read_bytes(), original["modified"])
                self.assertEqual(json.loads((self.output / "validation.json").read_text())["status"], "failed")
                self.assertFalse((self.output / DIRECTIONS[1]).exists())

    def test_default_source_still_uses_original_prepare_planner_and_no_candidate(self) -> None:
        def old_planner_source(names: tuple[str, ...], start: tuple[float, ...],
                               target: tuple[float, ...]) -> CompiledTrajectory:
            zero = (0.,) * 6
            return CompiledTrajectory(names, (
                JointSample(0., start, zero, zero, "approach"),
                JointSample(20., target, zero, zero, "approach")),
                {"prepare": {"curobo_revision": self.revision, "uniform_time_scale": 1.}})

        with self.synthetic_gpu() as (adapter, planner):
            planner.return_value.plan.side_effect = old_planner_source
            report = subject.validate(self.root, self.config, self.output, gpu=True)
        planner.assert_called_once()
        self.assertEqual(planner.return_value.plan.call_count, 2)
        self.assertEqual(len(adapter.trajectories), 4)
        self.assertEqual(report["source_kind"], subject.DEFAULT_SOURCE_KIND)
        for direction in DIRECTIONS:
            directory = self.output / direction
            self.assertFalse((directory / "candidate_joint_trajectory.csv").exists())
            self.assertFalse((directory / "candidate.json").exists())
            self.assertIn("PreparePlanner.plan", report["directions"][direction]["planner"])

    def test_cpu_only_never_creates_adapter_or_candidate(self) -> None:
        with self.synthetic_gpu() as (adapter, planner):
            report = self.run_validation(gpu=False)
        self.assertEqual(report["status"], "cpu_precheck_only")
        self.assertEqual(adapter.construction_calls, 0)
        self.assertEqual(adapter.warmup_calls, 0)
        self.assertEqual(adapter.trajectories, [])
        planner.assert_not_called()
        for direction in DIRECTIONS:
            self.assertFalse((self.output / direction).exists())

    def test_unknown_source_kind_rejected_before_output_creation(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown parking source kind"):
            subject.validate(self.root, self.config, self.output, source_kind="automatic_retime")
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
