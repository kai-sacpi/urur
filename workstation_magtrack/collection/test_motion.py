"""Motion tests use a fake ROS transport; no CUDA, ROS, serial, or robot."""
from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
from hx_waypoint_curobo.trajectory_io import read_joint_trajectory_csv

from .motion import (CancellationUnconfirmed, JointStateSnapshot, MotionController, PlannerReply, RosMotionBackend,
                     validate_trajectory, write_joint_csv_exact)
from .validate_parking import campaign_gpu_lock, wait_for_batches


SOURCE = Path(__file__).resolve().parents[1]


def trajectory(environment, start, target, duration=20.):
    zero = (0.,)*6
    return CompiledTrajectory(environment.joint_names, (
        JointSample(0.,tuple(start),zero,zero,"approach"),
        JointSample(duration,tuple(target),zero,zero,"approach")))


class FakeBackend:
    def __init__(self, workspace, environment, start):
        self.workspace, self.environment = workspace, environment
        self.position, self.velocity = tuple(start), (0.,)*6
        self.mode, self.sequence, self.revision = "simulation", 0, "a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9"
        self.plan_calls, self.execute_calls, self.closed = 0, 0, False
        self.executed = []
        self.bad_hash = self.controller_failure = self.stale = self.after_planning_change = False
        self.header_override = None

    def runtime_identity(self, timeout_s):
        return {"execution_mode":self.mode, "environments_root":str(self.workspace / "environments"),
                "telemetry_root":str(self.workspace / "runs"), "use_sim_time":False}

    def fresh_state(self, timeout_s):
        time.sleep(.005)
        self.sequence += 1
        return JointStateSnapshot(self.environment.joint_names,self.position,self.velocity,
            time.monotonic_ns()-(1_000_000_000 if self.stale else 0),
            self.header_override if self.header_override is not None else time.time_ns(),self.sequence)

    def plan(self, environment_id, names, start, target, timeout_s):
        self.plan_calls += 1
        result = trajectory(self.environment,start,target)
        hashes = dict(self.environment.hashes)
        if self.bad_hash:
            hashes["setup.yaml"] = "0"*64
        if self.after_planning_change:
            self.position = tuple(x+.01 for x in self.position)
        return PlannerReply(result,hashes,self.revision,True)

    def execute(self, compiled, timeout_s):
        self.execute_calls += 1
        self.executed.append(compiled)
        if not self.controller_failure:
            self.position = compiled.samples[-1].positions
        return {"success":not self.controller_failure,"fake_transport_for_unit_test":True}

    def close(self):
        self.closed = True


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        shutil.copytree(SOURCE / "environments/mla_short",self.root / "environments/mla_short",
                        ignore=shutil.ignore_patterns("trajectories","recordings"))
        (self.root / "collection").mkdir()
        shutil.copy2(SOURCE / "collection/config.yaml",self.root / "collection/config.yaml")
        shutil.copy2(SOURCE / "curobo.lock",self.root / "curobo.lock")
        self.environment = EnvironmentBuilder().load(self.root / "environments/mla_short")
        self.backend = FakeBackend(self.root,self.environment,self.environment.home_joint_positions)
        self.motion = MotionController(self.root,mode="simulation",domain_id=221,backend=self.backend)

    def tearDown(self):
        self.temp.cleanup()

    def test_simulation_must_use_private_domain(self):
        for domain in (0,219,220,-1,233):
            with self.assertRaises(ValueError):
                MotionController(self.root,domain_id=domain,backend=self.backend)

    def test_real_rejected_even_with_boolean_release_when_config_unreleased(self):
        for explicit in (False,True):
            with self.assertRaises(PermissionError):
                MotionController(self.root,mode="real",real_release=explicit,backend=self.backend)
        self.assertEqual(self.backend.plan_calls,0)

    def test_runtime_identity_mode_must_match(self):
        self.backend.mode = "real"
        with self.assertRaisesRegex(ValueError,"identity mismatch"):
            self.motion.verify_runtime_identity()

    def test_parking_then_home_executes_two_independent_paths(self):
        parked = self.motion.move_park()
        returned = self.motion.move_home()
        self.assertTrue(parked["passed"] and returned["passed"])
        self.assertEqual(self.backend.execute_calls,2)
        self.assertEqual(returned["target_positions_rad"],list(self.environment.home_joint_positions))
        self.assertEqual(parked["validation"]["uniform_time_scale"],1.)
        self.assertTrue(parked["validation"]["continuous_quintic_extrema_checked"])
        self.assertIn("plane1_position_m",parked["tip_fk"])

    def test_different_environment_rejected_before_motion(self):
        self.backend.bad_hash = True
        with self.assertRaisesRegex(ValueError,"environment/revision"):
            self.motion.move_park()
        self.assertEqual(self.backend.execute_calls,0)

    def test_changed_start_during_planning_rejected(self):
        self.backend.after_planning_change = True
        with self.assertRaisesRegex(ValueError,"left requested"):
            self.motion.move_park()
        self.assertEqual(self.backend.execute_calls,0)

    def test_stale_actual_state_rejected(self):
        self.backend.stale = True
        with self.assertRaisesRegex(ValueError,"stale"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)

    def test_old_source_header_is_not_refreshed_by_new_callback(self):
        self.backend.header_override = time.time_ns()-1_000_000_000
        with self.assertRaisesRegex(ValueError,"source header is stale"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)

    def test_duplicate_source_header_rejected_even_with_new_callback(self):
        self.backend.header_override = time.time_ns()
        self.motion._state()
        with self.assertRaisesRegex(ValueError,"did not advance"):
            self.motion._state()

    def test_unsettled_actual_state_rejected(self):
        self.backend.velocity = (.02,0.,0.,0.,0.,0.)
        with self.assertRaisesRegex(ValueError,"not stationary"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)

    def test_failure_never_automatically_returns_home(self):
        self.backend.controller_failure = True
        with self.assertRaisesRegex(RuntimeError,"controller"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,1)
        self.assertEqual(self.backend.execute_calls,1)
        files = list((self.root / "collection/motion_runs").rglob("evidence.json"))
        self.assertEqual(len(files),1)
        evidence = json.loads(files[0].read_text())
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["automatic_return_on_failure"])

    def test_configuration_change_fails_before_any_ros_command(self):
        with (self.root / "collection/config.yaml").open("a") as stream:
            stream.write("\n# changed\n")
        with self.assertRaisesRegex(ValueError,"configuration changed"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)

    def test_continuous_quintic_dynamics_not_only_endpoint_derivatives(self):
        too_fast = trajectory(self.environment,self.motion.home,self.motion.park,.1)
        self.assertEqual(max(abs(v) for s in too_fast.samples for v in s.velocities),0.)
        with self.assertRaisesRegex(ValueError,"joint dynamics"):
            validate_trajectory(self.environment,too_fast,self.motion.home,self.motion.park)

    def test_joint_csv_round_trips_exact_authority(self):
        compiled = trajectory(self.environment,self.motion.home,self.motion.park)
        path = self.root / "exact.csv"
        write_joint_csv_exact(path,compiled)
        restored = read_joint_trajectory_csv(path,self.environment.joint_names)
        self.assertEqual(compiled.samples,restored.samples)
        with self.assertRaises(FileExistsError):
            write_joint_csv_exact(path,compiled)

    def _prepared(self):
        from .prepared_parking import create_bundle
        from .test_prepared_parking import make_validation
        validation = make_validation(self.root,SOURCE)
        bundle = self.root / "collection/prepared_parking_unit_test_bundle"
        create_bundle(self.root,validation,bundle)
        self.motion = MotionController(self.root,mode="simulation",domain_id=221,
            backend=self.backend,prepared_parking=bundle)
        return bundle

    def test_prepared_parking_replays_exact_both_directions_without_planning(self):
        bundle = self._prepared()
        originals = {direction:(bundle/direction/"joint_trajectory.csv").read_bytes()
                     for direction in ("home_to_park","park_to_home")}
        parked = self.motion.move_park()
        returned = self.motion.move_home()
        self.assertEqual(self.backend.plan_calls,0)
        self.assertEqual(self.backend.execute_calls,2)
        for index,(direction,evidence) in enumerate((("home_to_park",parked),("park_to_home",returned))):
            self.assertTrue(evidence["passed"])
            self.assertEqual(evidence["motion_source"],"prepared_parking_bundle")
            self.assertFalse(evidence["planning_performed"])
            self.assertEqual(Path(evidence["joint_trajectory_path"]).read_bytes(),originals[direction])
            expected = read_joint_trajectory_csv(bundle/direction/"joint_trajectory.csv",self.environment.joint_names)
            self.assertEqual(self.backend.executed[index].samples,expected.samples)
            self.assertEqual(evidence["prepared_asset"]["bundle_manifest_sha256"],
                             self.motion.prepared_parking.manifest_sha256)

    def test_prepared_wrong_start_never_implicitly_replans(self):
        self._prepared()
        for method in (self.motion.move_park,self.motion.move_home):
            with self.subTest(method=method.__name__):
                self.backend.position = tuple((a+b)/2 for a,b in zip(self.motion.home,self.motion.park))
                with self.assertRaisesRegex(ValueError,"fresh start mismatch; no implicit replanning"):
                    method()
        self.assertEqual(self.backend.plan_calls,0)
        self.assertEqual(self.backend.execute_calls,0)

    def test_explicit_trajectory_end_home_uses_full_dynamic_collision_path(self):
        self._prepared()
        self.backend.position = tuple((a+b)/2 for a,b in zip(self.motion.home,self.motion.park))
        result = self.motion.move_home(from_trajectory_end=True)
        self.assertTrue(result["passed"])
        self.assertTrue(result["planning_performed"])
        self.assertTrue(result["from_trajectory_end"])
        self.assertEqual(result["motion_source"],"dynamic_collision_checked_prepare")
        self.assertEqual(result["gpu_full_path_validation"]["source"],"fixed-environment PlanJointPath service")
        self.assertEqual(self.backend.plan_calls,1)
        self.assertEqual(self.backend.execute_calls,1)
        self.assertNotIn("prepared_asset",result)

    def test_prepared_start_is_rechecked_immediately_before_submission(self):
        self._prepared()
        fresh = self.backend.fresh_state
        def changed_after_initial_state(timeout_s):
            if self.backend.sequence:
                self.backend.position = tuple(x+.01 for x in self.motion.home)
            return fresh(timeout_s)
        self.backend.fresh_state = changed_after_initial_state
        with self.assertRaisesRegex(ValueError,"left requested stationary pose"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)
        self.assertEqual(self.backend.execute_calls,0)

    def test_prepared_tamper_after_loading_rejected_before_motion(self):
        bundle = self._prepared()
        with (bundle/"home_to_park/joint_trajectory.csv").open("ab") as stream:
            stream.write(b"\n")
        with self.assertRaisesRegex(ValueError,"file SHA mismatch"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)
        self.assertEqual(self.backend.execute_calls,0)

    def test_prepared_configuration_change_fails_before_transport(self):
        self._prepared()
        with (self.root/"collection/config.yaml").open("a") as stream:
            stream.write("\n# changed after loading bundle\n")
        with self.assertRaisesRegex(ValueError,"configuration changed"):
            self.motion.move_park()
        self.assertEqual(self.backend.plan_calls,0)
        self.assertEqual(self.backend.execute_calls,0)

    def test_stationary_monitor_records_real_feedback_and_fk(self):
        self.backend.position = self.motion.park
        path = self.root / "stationary.csv"
        with self.motion.stationary_monitor(path,duration_s=.04) as guard:
            time.sleep(.07)
            guard.check()
        self.assertTrue(guard.evidence["passed"])
        self.assertGreater(guard.evidence["sample_count"],2)
        self.assertEqual(guard.evidence["max_tip_fk_drift_m"],0.)
        self.assertIn("tip_qw",path.read_text().splitlines()[0])

    def test_stationary_failure_preserves_csv_and_invalidates_window(self):
        self.backend.position = self.motion.park
        path = self.root / "stationary_bad.csv"
        with self.assertRaisesRegex(RuntimeError,"stationary background invalid"):
            with self.motion.stationary_monitor(path,duration_s=.04) as guard:
                self.backend.position = tuple(x+.02 for x in self.motion.park)
                time.sleep(.04)
                guard.check()
        self.assertFalse(guard.evidence["passed"])
        self.assertTrue(path.exists())

    def test_campaign_gpu_lock_refuses_active_owner(self):
        with campaign_gpu_lock(self.root):
            with self.assertRaisesRegex(RuntimeError,"batch campaign is active"):
                with campaign_gpu_lock(self.root):
                    self.fail("must not obtain shared GPU campaign lock twice")

    def test_after_batch_requires_complete_acceptance(self):
        directory = self.root / "runs/queued"
        directory.mkdir(parents=True)
        summary = directory / "campaign_summary.json"
        summary.write_text(json.dumps({"state":"complete","workflow_passed":True}))
        result = wait_for_batches(self.root,["queued"])
        self.assertTrue(result["queued"]["workflow_passed"])
        summary.write_text(json.dumps({"state":"complete","workflow_passed":False}))
        with self.assertRaisesRegex(RuntimeError,"without full acceptance"):
            wait_for_batches(self.root,["queued"])
        with self.assertRaisesRegex(ValueError,"unsafe"):
            wait_for_batches(self.root,["../elsewhere"])

    def _transport(self, wait_results):
        backend = object.__new__(RosMotionBackend)
        backend.controller_name = "/joint_trajectory_controller/follow_joint_trajectory"
        backend.controller = SimpleNamespace(wait_for_server=Mock(return_value=True),
                                            send_goal_async=Mock(return_value=object()))
        backend.FollowJointTrajectory = SimpleNamespace(Goal=SimpleNamespace)
        backend._wait = Mock(side_effect=wait_results)
        return backend

    def test_late_goal_acceptance_is_canceled_and_terminal_confirmed(self):
        handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(return_value="cancel"),
                                 get_result_async=Mock(return_value="terminal"))
        backend = self._transport([TimeoutError("submit timeout"),handle,SimpleNamespace(),SimpleNamespace(status=5)])
        with patch("hx_waypoint_curobo.curobo_server._to_ros_trajectory",return_value=object()):
            with self.assertRaisesRegex(TimeoutError,"submit timeout"):
                backend.execute(object(),30.)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_called_once()
        self.assertEqual(backend._wait.call_count,4)

    def test_unconfirmed_submission_never_claims_robot_stopped(self):
        backend = self._transport([TimeoutError("submit timeout"),TimeoutError("no late acceptance")])
        with patch("hx_waypoint_curobo.curobo_server._to_ros_trajectory",return_value=object()):
            with self.assertRaisesRegex(CancellationUnconfirmed,"do not assume robot stopped"):
                backend.execute(object(),30.)

    def test_submission_interrupt_still_cancels_late_accepted_goal(self):
        handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(return_value="cancel"),
                                 get_result_async=Mock(return_value="terminal"))
        backend = self._transport([KeyboardInterrupt(),handle,SimpleNamespace(),SimpleNamespace(status=5)])
        with patch("hx_waypoint_curobo.curobo_server._to_ros_trajectory",return_value=object()):
            with self.assertRaises(KeyboardInterrupt):
                backend.execute(object(),30.)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_called_once()

    def test_execution_timeout_waits_for_terminal_cancel_result(self):
        handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(return_value="cancel"),
                                 get_result_async=Mock(return_value="terminal"))
        backend = self._transport([handle,TimeoutError("execution timeout"),SimpleNamespace(),SimpleNamespace(status=5)])
        with patch("hx_waypoint_curobo.curobo_server._to_ros_trajectory",return_value=object()):
            with self.assertRaisesRegex(TimeoutError,"execution timeout"):
                backend.execute(object(),30.)
        self.assertEqual(handle.get_result_async.call_count,2)


if __name__ == "__main__":
    unittest.main()
