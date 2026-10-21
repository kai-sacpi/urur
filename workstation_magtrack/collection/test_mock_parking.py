"""No ROS, GPU, hardware, or remote jobs: exercise the frozen-path mock gate."""
from __future__ import annotations

import copy
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import signal
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
from hx_waypoint_curobo.robot_kinematics import QuinticHermiteSegment
from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition

from .motion import (CancellationUnconfirmed, JointStateSnapshot, MotionController, RosMotionBackend,
                     sha256, write_joint_csv_exact, write_json_new)
from .mock_parking import (DOMAIN, MockSession, _live_producer_identity, _producer_output, audit_witness, goal_payload, load_authority, main, run,
                           verify_mock_evidence, verify_runtime_identity, verify_scene_snapshot, wait_for_validation)


ROOT = Path(__file__).resolve().parents[1]


def trajectory(motion, start=None, target=None, duration=.08):
    start = motion.home if start is None else tuple(start)
    target = tuple(q+.00001 for q in start) if target is None else tuple(target)
    return CompiledTrajectory(motion.environment.joint_names,(
        JointSample(0.,start,(0.,)*6,(0.,)*6,"approach"),
        JointSample(duration,target,(0.,)*6,(0.,)*6,"approach")))


def rows_for(compiled, dt=.008):
    times = [min(i*dt,compiled.duration_s) for i in range(round(compiled.duration_s/dt)+1)]
    segments = [QuinticHermiteSegment.from_boundary_conditions(
        duration_s=compiled.duration_s,start_position=a,start_velocity=0.,start_acceleration=0.,
        end_position=b,end_velocity=0.,end_acceleration=0.)
        for a,b in zip(compiled.samples[0].positions,compiled.samples[-1].positions)]
    ros, sync = 1_800_000_000_000_000_003, 123456789123456789
    rows = []
    for t in times:
        q,qd,qdd = ([s.evaluate(t,d) for s in segments] for d in range(3))
        rows.append({"rx_sync_time_ns":sync+round(t*1e9),"controller_ros_time_ns":ros+round(t*1e9),
            "reference_time_s":t,"feedback_time_s":t-2e-7,"raw_speed_scaling_factor":0.,
            "joint_names":json.dumps(list(compiled.joint_names)),"reference_q":json.dumps(q),
            "reference_qd":json.dumps(qd),"reference_qdd":json.dumps(qdd),
            "actual_q":json.dumps(q),"actual_qd":json.dumps(qd)})
    return rows


def write_rows(path, rows):
    with path.open("w",newline="") as stream:
        writer = csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def snapshot_for(motion):
    definition = load_planning_scene_definition(motion.workspace,"mla_short")
    def obj(name,frame,kind,dims,p,q):
        return {"id":name,"frame_id":frame,"object_pose":{"position_m":[0.,0.,0.],"quaternion_xyzw":[0.,0.,0.,1.]},
                "primitives":[{"type":kind,"dimensions":list(dims)}],
                "primitive_poses":[{"position_m":list(p),"quaternion_xyzw":list(q)}],"mesh_count":0,"plane_count":0}
    rod = definition.attached_rod
    return {"world":[obj(c.object_id,c.frame_id,1,c.dimensions_m,c.position_m,c.quaternion_xyzw) for c in definition.cuboids],
            "attached":[{"link_name":rod.link_name,"touch_links":list(rod.touch_links),
                         "object":obj(rod.object_id,rod.link_name,3,[rod.length_m,rod.radius_m],rod.center_in_link_m,rod.quaternion_xyzw)}]}


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temp.name).resolve()
        shutil.copytree(ROOT / "environments/mla_short",self.workspace / "environments/mla_short",
                        ignore=shutil.ignore_patterns("trajectories","recordings"))
        (self.workspace / "collection").mkdir()
        shutil.copy2(ROOT / "collection/config.yaml",self.workspace / "collection/config.yaml")
        shutil.copy2(ROOT / "curobo.lock",self.workspace / "curobo.lock")
        self.motion = MotionController(self.workspace,domain_id=DOMAIN)
        self.short = trajectory(self.motion)
        self.witness = self.workspace / "witness.csv"
        self.rows = rows_for(self.short)
        write_rows(self.witness,self.rows)

    def tearDown(self):
        self.motion.close()
        self.temp.cleanup()

    def authority(self):
        producer = self.workspace / "runs/producer"
        producer.mkdir(parents=True)
        value = {"status":"passed","runtime_ready":True,"gpu_full_path_validation":"passed_both_directions",
                 "simulation_only":True,"real_execution_authorized":False,"environment_id":"mla_short",
                 "environment_hashes":self.motion.environment_hashes,"config_sha256":self.motion.config_sha256,
                 "curobo_lock_sha256":self.motion.lock_sha256,"rod_length_m":.190,"rod_diameter_m":.005,
                 "hard_world_self_clearance_m":.005,"uniform_time_scale":1.,"authority_mutated":False,
                 "joint_names":list(self.motion.environment.joint_names),"directions":{}}
        for direction,start,target in (("home_to_park",self.motion.home,self.motion.park),
                                       ("park_to_home",self.motion.park,self.motion.home)):
            directory = producer / direction
            directory.mkdir()
            source = directory / "joint_trajectory.csv"
            compiled = trajectory(self.motion,start,target,20.)
            write_joint_csv_exact(source,compiled)
            metrics = {"minimum_collision_distance_m":.05,"checked_sample_count":2,
                       "max_scene_collision_cost":0.,"max_self_collision_cost":0.,"max_joint_bound_cost":0.,
                       "max_tip_position_error_m":0.,"max_tip_orientation_error_deg":0.}
            item = {"status":"passed","joint_trajectory_path":str(source),"joint_trajectory_sha256":sha256(source),
                    "gpu_metrics":metrics,"serialized_gpu_metrics":metrics}
            value["directions"][direction] = item
            write_json_new(directory / "evidence.json",item)
        write_json_new(producer / "validation.json",value)
        return producer / "validation.json"

    def test_all_native_rows_recomputed_with_frozen_clock(self):
        output = self.workspace / "actual_fk.csv"
        audit = audit_witness(self.witness,self.short,self.motion,output)
        self.assertEqual(audit["status"],"passed")
        self.assertEqual(audit["selected_rows"],len(self.rows))
        self.assertEqual(audit["maximum_allowed_controller_gap_s"],.012)
        self.assertEqual(audit["max_errors"]["reference_qdd_rad_s2"],0.)
        self.assertFalse(audit["resampling_or_synthetic_rows"])
        self.assertIn("actual_tip_plane1_m",output.read_text())

    def test_gap_cannot_redefine_its_own_coverage_tolerance(self):
        write_rows(self.witness,self.rows[::2])
        with self.assertRaisesRegex(ValueError,"no complete native-clock"):
            audit_witness(self.witness,self.short,self.motion)

    def test_bad_interior_joint_is_not_hidden_by_correct_endpoints(self):
        q = json.loads(self.rows[4]["actual_q"])
        q[0] += .03
        self.rows[4]["actual_q"] = json.dumps(q)
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"mock actual_reference_q_rad"):
            audit_witness(self.witness,self.short,self.motion)

    def test_reference_acceleration_is_authority_too(self):
        qdd = json.loads(self.rows[4]["reference_qdd"])
        qdd[0] += .001
        self.rows[4]["reference_qdd"] = json.dumps(qdd)
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"mock reference_qdd"):
            audit_witness(self.witness,self.short,self.motion)

    def test_same_time_fk_gate_catches_small_joint_error(self):
        q = json.loads(self.rows[4]["actual_q"])
        q[0] += .019
        self.rows[4]["actual_q"] = json.dumps(q)
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"actual_fk"):
            audit_witness(self.witness,self.short,self.motion)

    def test_reference_clock_cannot_be_stretched(self):
        for row in self.rows:
            row["controller_ros_time_ns"] += round(row["reference_time_s"]*1e9)
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"mock clock_error_s"):
            audit_witness(self.witness,self.short,self.motion)

    def test_nonfinite_clock_is_rejected(self):
        self.rows[4]["feedback_time_s"] = float("nan")
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"coherent controller clock"):
            audit_witness(self.witness,self.short,self.motion)

    def test_duplicate_header_cannot_be_accepted_as_more_samples(self):
        self.rows[4]["controller_ros_time_ns"] = self.rows[3]["controller_ros_time_ns"]
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"strictly advance"):
            audit_witness(self.witness,self.short,self.motion)

    def test_speed_field_change_rejected_even_in_unscaled_controller(self):
        self.rows[4]["raw_speed_scaling_factor"] = 1.
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"speed-scaling field changed"):
            audit_witness(self.witness,self.short,self.motion)

    def test_bad_joint_name_rejected(self):
        self.rows[4]["joint_names"] = json.dumps(["wrong"]*6)
        write_rows(self.witness,self.rows)
        with self.assertRaisesRegex(ValueError,"joint names mismatch"):
            audit_witness(self.witness,self.short,self.motion)

    def test_no_endpoint_only_acceptance(self):
        write_rows(self.witness,[self.rows[0],self.rows[-1]])
        with self.assertRaisesRegex(ValueError,"no complete native-clock"):
            audit_witness(self.witness,self.short,self.motion)

    def test_submitted_goal_preserves_every_value_and_time(self):
        payload = goal_payload(self.short)
        self.assertEqual(len(payload["points"]),len(self.short.samples))
        self.assertEqual(payload["points"][1]["time_from_start_ns"],80_000_000)
        self.assertEqual(payload["points"][1]["positions"],list(self.short.samples[-1].positions))

    def test_full_world_and_rod_geometry_must_match(self):
        snapshot = snapshot_for(self.motion)
        self.assertEqual(verify_scene_snapshot(snapshot,self.motion)["rod_length_m"],.190)
        for mutate in (lambda s:s["world"].pop(),lambda s:s["attached"][0]["object"]["primitives"][0]["dimensions"].__setitem__(0,.33365),
                       lambda s:s["world"][0]["primitive_poses"][0]["position_m"].__setitem__(0,100.),
                       lambda s:s["world"][0]["object_pose"]["position_m"].__setitem__(0,.1)):
            bad = copy.deepcopy(snapshot)
            mutate(bad)
            with self.assertRaises(ValueError):
                verify_scene_snapshot(bad,self.motion)

    def test_identity_rejects_real_external_and_other_domains(self):
        identity = {"domain_id":223,"execution_mode":"simulation","environments_root":str(self.workspace / "environments"),
                    "telemetry_root":str(self.workspace / "runs"),"use_sim_time":False,"externally_managed":False}
        verify_runtime_identity(identity,self.workspace)
        for key,value in (("domain_id",0),("domain_id",219),("domain_id",220),("execution_mode","real"),("externally_managed",True)):
            with self.assertRaises(ValueError):
                verify_runtime_identity({**identity,key:value},self.workspace)

    def test_jazzy_normalized_single_shape_object_and_primitive_poses_are_equivalent(self):
        snapshot = snapshot_for(self.motion)
        objects = [*snapshot["world"],snapshot["attached"][0]["object"]]
        for obj in objects:
            obj["object_pose"] = obj["primitive_poses"][0]
            obj["primitive_poses"] = [{"position_m":[0.,0.,0.],"quaternion_xyzw":[0.,0.,0.,1.]}]
        for obj in snapshot["world"]:
            obj["frame_id"] = "world"
        self.assertEqual(verify_scene_snapshot(snapshot,self.motion)["status"],"passed")

    def test_world_cannot_be_reinterpreted_through_a_moving_joint(self):
        snapshot = snapshot_for(self.motion)
        snapshot["world"][0]["frame_id"] = "wrist_3_link"
        with self.assertRaisesRegex(ValueError,"geometry/frame differs"):
            verify_scene_snapshot(snapshot,self.motion)

    def test_gpu_authority_requires_actual_csv_sha_and_two_directions(self):
        validation = self.authority()
        paths,binding = load_authority(self.motion,validation)
        self.assertEqual(set(paths),{"home_to_park","park_to_home"})
        self.assertEqual(binding["validation_sha256"],sha256(validation))
        paths["park_to_home"]["source"].write_bytes(b"tampered\n")
        with self.assertRaisesRegex(ValueError,"source missing, failed, or changed"):
            load_authority(self.motion,validation)

    def test_authority_hash_and_parser_share_one_snapshot_during_replace_restore(self):
        """20 s -> 40 s during parse -> 20 s on disk cannot launder timing."""
        import importlib
        module = importlib.import_module(load_authority.__module__)
        validation = self.authority()
        source = validation.parent / "home_to_park/joint_trajectory.csv"
        original_bytes = source.read_bytes()
        changed_bytes = original_bytes.replace(b"20.000000000",b"40.000000000")
        self.assertNotEqual(original_bytes,changed_bytes)
        original_read, original_validate = module.read_json,module.validate_trajectory
        replaced = []
        def replace_between_hash_and_parse(path):
            item = original_read(path)
            if path == source.with_name("evidence.json"):
                source.write_bytes(changed_bytes)
                replaced.append(True)
            return item
        def restore_before_last_hash(*args,**kwargs):
            source.write_bytes(original_bytes)
            return original_validate(*args,**kwargs)
        with patch.object(module,"read_json",side_effect=replace_between_hash_and_parse):
            with patch.object(module,"validate_trajectory",side_effect=restore_before_last_hash):
                paths,_ = load_authority(self.motion,validation)
        self.assertEqual(replaced,[True])
        self.assertEqual(paths["home_to_park"]["trajectory"].duration_s,20.)
        self.assertEqual(paths["home_to_park"]["sha256"],sha256(source))
        self.assertEqual(source.read_bytes(),original_bytes)

    def test_no_wait_on_stale_file_or_missing_process(self):
        absent = self.workspace / "missing/validation.json"
        with self.assertRaisesRegex(RuntimeError,"not available"):
            wait_for_validation(absent,wait=False,producer_pid=None)
        with self.assertRaisesRegex(ValueError,"requires --producer-pid"):
            wait_for_validation(absent,wait=True,producer_pid=None)
        with self.assertRaisesRegex(RuntimeError,"exited without"):
            wait_for_validation(absent,wait=True,producer_pid=999999999)

    def test_completed_certificate_needs_no_live_pid(self):
        validation = self.authority()
        wait_for_validation(validation,wait=True,producer_pid=None)

    def test_producer_relative_output_is_resolved_from_live_proc_cwd(self):
        validation = self.workspace / "runs/parking_gpu_validation_20261017_v1/validation.json"
        argv = ["/home/huaxin/Codes/ur-waypoint/.venv/bin/python","-m","collection.validate_parking",
                "--workspace",str(self.workspace),"--gpu","--wait-lock","--output","runs/parking_gpu_validation_20261017_v1"]
        raw = b"\0".join(x.encode() for x in argv)+b"\0"
        stat = "705832 (python) " + " ".join(["S"]+["0"]*18+["fixed-start-ticks"]+["0"]*8)
        with patch.object(Path,"read_bytes",return_value=raw):
            with patch.object(Path,"readlink",return_value=self.workspace):
                with patch.object(Path,"read_text",return_value=stat):
                    with patch(f"{run.__module__}.os.kill") as alive:
                        self.assertEqual(_live_producer_identity(705832,validation),"fixed-start-ticks")
        alive.assert_called_once_with(705832,0)

    def test_producer_output_supports_equals_but_rejects_wrong_duplicate_nested_and_non_gpu(self):
        prefix = ["/usr/bin/python3.12","-m","collection.validate_parking"]
        output = "runs/parking_gpu_validation_20261017_v1"
        self.assertEqual(_producer_output(prefix+["--gpu",f"--output={output}"],self.workspace),
                         (self.workspace/output).resolve())
        wrong = prefix+["--gpu","--output","runs/wrong"]
        self.assertNotEqual(_producer_output(wrong,self.workspace),(self.workspace/output).resolve())
        for argv in (["tmux","new-session","python -m collection.validate_parking --gpu --output "+output],
                     prefix+["--output",output],prefix+["--gpu","--output",output,f"--output={output}"],
                     prefix+["--gpu","--output"],prefix+["--gpu","--output="]):
            with self.assertRaises(ValueError):
                _producer_output(argv,self.workspace)
        raw = b"\0".join(x.encode() for x in wrong)+b"\0"
        stat = "705832 (python) " + " ".join(["S"]+["0"]*18+["fixed-start-ticks"]+["0"]*8)
        with patch.object(Path,"read_bytes",return_value=raw):
            with patch.object(Path,"readlink",return_value=self.workspace):
                with patch.object(Path,"read_text",return_value=stat):
                    with patch(f"{run.__module__}.os.kill") as alive:
                        with self.assertRaisesRegex(ValueError,"not the live matching"):
                            _live_producer_identity(705832,self.workspace/output/"validation.json")
        alive.assert_not_called()

    def test_failed_producer_not_restarted(self):
        validation = self.workspace / "failed.json"
        write_json_new(validation,{"status":"failed"})
        with self.assertRaisesRegex(RuntimeError,"ended with failed"):
            wait_for_validation(validation,wait=True,producer_pid=None)

    def test_wait_tolerates_empty_then_partial_json_only_with_live_matching_producer(self):
        validation = self.workspace / "non_atomic_validation.json"
        validation.write_text("")
        updates = iter(['{"status":', '{"status":"passed"}'])
        def advance(*args): validation.write_text(next(updates))
        module = run.__module__
        with patch(f"{module}._live_producer_identity",return_value="fixed-start-ticks") as live:
            with patch(f"{module}.time.sleep",side_effect=advance):
                wait_for_validation(validation,wait=True,producer_pid=123,poll_s=0.)
        self.assertEqual(live.call_count,2)

    def test_partial_certificate_after_producer_exit_is_strictly_rejected(self):
        validation = self.workspace / "partial.json"
        validation.write_text('{"status":')
        with patch(f"{run.__module__}._live_producer_identity",side_effect=ProcessLookupError):
            with self.assertRaisesRegex(RuntimeError,"exited with a malformed certificate"):
                wait_for_validation(validation,wait=True,producer_pid=123,poll_s=0.)
        with self.assertRaises(json.JSONDecodeError):
            wait_for_validation(validation,wait=False,producer_pid=None)

    def test_partial_json_does_not_allow_wrong_or_reused_producer(self):
        validation = self.workspace / "partial_wrong_pid.json"
        validation.write_text("")
        with patch(f"{run.__module__}._live_producer_identity",side_effect=ValueError("not matching producer")):
            with self.assertRaisesRegex(ValueError,"not matching producer"):
                wait_for_validation(validation,wait=True,producer_pid=123,poll_s=0.)
        with patch(f"{run.__module__}._live_producer_identity",side_effect=["start1","start2"]):
            with patch(f"{run.__module__}.time.sleep"):
                with self.assertRaisesRegex(ValueError,"PID was reused"):
                    wait_for_validation(validation,wait=True,producer_pid=123,poll_s=0.)

    def test_mock_failure_preserves_status_without_auto_return(self):
        validation = self.authority()
        source = validation.parent / "home_to_park/joint_trajectory.csv"
        before = source.read_bytes()
        class FailedSession:
            def __init__(self,motion,output):
                self.identity = {"unit_test":True}
                self.scene = {"unit_test":True}
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def play(self,*args): raise CancellationUnconfirmed("unit test cancellation unconfirmed")
        output = self.workspace / "runs/mock_failed"
        with self.assertRaises(CancellationUnconfirmed):
            run(self.workspace,validation,output,session_factory=FailedSession)
        self.assertEqual(source.read_bytes(),before)
        evidence = json.loads((output / "evidence.json").read_text())
        self.assertEqual(evidence["status"],"failed")
        self.assertFalse(evidence["automatic_return_on_failure"])
        self.assertEqual(evidence["directions"],{})

    def test_fresh_output_cannot_overwrite_existing_experiment(self):
        output = self.workspace / "runs/existing"
        output.mkdir(parents=True)
        with self.assertRaises(FileExistsError):
            run(self.workspace,self.workspace / "missing.json",output)

    def test_source_byte_mutation_during_playback_rejects_completion(self):
        validation = self.authority()
        source = validation.parent / "home_to_park/joint_trajectory.csv"
        class MutatingSession:
            def __init__(self,motion,output): self.identity,self.scene = {},{}
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def play(self,*args):
                source.write_bytes(source.read_bytes()+b"\n")
                return {"status":"passed"}
            def settle(self,*args): return {}
        output = self.workspace / "runs/mutated"
        with patch(f"{run.__module__}.audit_witness",return_value={"telemetry_rows":[]}):
            with self.assertRaisesRegex(ValueError,"CSV changed during mock playback"):
                run(self.workspace,validation,output,session_factory=MutatingSession)
        self.assertEqual(json.loads((output / "evidence.json").read_text())["status"],"failed")

    def test_sigterm_unwinds_owned_session_and_preserves_failure_evidence(self):
        validation = self.authority()
        output = self.workspace / "runs/sigterm"
        closed = []
        class SignalSession:
            def __init__(self,motion,output): self.identity,self.scene = {},{}
            def __enter__(self): return self
            def __exit__(self,*args): closed.append(True)
            def play(self,*args): os.kill(os.getpid(),signal.SIGTERM)
        prior = signal.getsignal(signal.SIGTERM)
        real_run = run
        def fake_run(*args,**kwargs): return real_run(*args,session_factory=SignalSession,**kwargs)
        with patch(f"{run.__module__}.run",side_effect=fake_run):
            code = main(["--workspace",str(self.workspace),"--validation",str(validation),"--output",str(output)])
        self.assertEqual(code,130)
        self.assertEqual(closed,[True])
        self.assertIs(signal.getsignal(signal.SIGTERM),prior)
        evidence = json.loads((output / "evidence.json").read_text())
        self.assertEqual(evidence["status"],"failed")
        self.assertIn("KeyboardInterrupt",evidence["error"])

    def test_transport_failure_still_closes_raw_witness_and_saves_result(self):
        directory = self.workspace / "failed_transport"
        directory.mkdir()
        state = JointStateSnapshot(self.motion.environment.joint_names,self.motion.home,(0.,)*6,
                                   time.monotonic_ns(),time.time_ns(),1)
        motion = SimpleNamespace(verify_runtime_identity=Mock(),_state=Mock(return_value=state))
        session = MockSession(motion,self.workspace)
        session.backend = SimpleNamespace(node=object(),execute=Mock(side_effect=CancellationUnconfirmed("goal may be live")))
        witness = SimpleNamespace(close=Mock())
        with patch("runtime.controller_witness.ControllerWitness",return_value=witness):
            with self.assertRaises(CancellationUnconfirmed):
                session.play("home_to_park",self.short,directory)
        witness.close.assert_called_once()
        self.assertEqual(json.loads((directory / "action_result.json").read_text())["status"],"failed")
        self.assertTrue((directory / "submitted_goal.json").is_file())

    def test_native_witness_must_belong_to_this_submission(self):
        action = {"submitted_sync_time_ns":1,"completed_sync_time_ns":2,
                  "submitted_ros_time_ns":1,"completed_ros_time_ns":2}
        with self.assertRaisesRegex(ValueError,"outside this FJT execution window"):
            audit_witness(self.witness,self.short,self.motion,action=action)

    def test_full_fake_transport_then_independent_verifier_and_tamper_rejection(self):
        validation = self.authority()
        hashes = {p:sha256(p) for p in validation.parent.glob("*/joint_trajectory.csv")}
        class FakeSession:
            def __init__(self,motion,output):
                self.motion = motion
                self.sequence = 0
                self.identity = {"domain_id":223,"execution_mode":"simulation",
                    "environments_root":str(motion.workspace / "environments"),
                    "telemetry_root":str(motion.workspace / "runs"),"use_sim_time":False,
                    "externally_managed":False,"unit_test_fake_transport":True,
                    "controller":{"name":"joint_trajectory_controller","type":"joint_trajectory_controller/JointTrajectoryController","state":"active"}}
                self.scene = verify_scene_snapshot(snapshot_for(motion),motion)
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def state(self,q,ros=None,sync=None):
                self.sequence += 1
                return asdict(JointStateSnapshot(self.motion.environment.joint_names,tuple(q),(0.,)*6,
                    sync or time.monotonic_ns(),ros or time.time_ns(),self.sequence))
            def play(self,direction,compiled,directory):
                rows = rows_for(compiled,.01)
                sync,ros = time.monotonic_ns(),time.time_ns()
                for row in rows:
                    offset = round(row["reference_time_s"]*1e9)
                    row.update(rx_sync_time_ns=sync+offset,controller_ros_time_ns=ros+offset)
                write_rows(directory / "controller_state.csv",rows)
                write_json_new(directory / "submitted_goal.json",goal_payload(compiled))
                action = {"status":"passed","fresh_start":self.state(compiled.samples[0].positions,ros,sync),
                    "submitted_sync_time_ns":sync,"submitted_ros_time_ns":ros,
                    "completed_sync_time_ns":sync+round(compiled.duration_s*1e9),
                    "completed_ros_time_ns":ros+round(compiled.duration_s*1e9),
                    "controller_result":{"success":True,"action_status":4,"controller_error_code":0,
                        "action_name":"/joint_trajectory_controller/follow_joint_trajectory",
                        "unit_test_fake_transport":True}}
                write_json_new(directory / "action_result.json",action)
                return action
            def settle(self,target): return self.state(target)
        output = self.workspace / "runs/mock_success"
        result = run(self.workspace,validation,output,session_factory=FakeSession)
        self.assertEqual(result["status"],"passed")
        independent = verify_mock_evidence(self.workspace,output / "evidence.json")
        self.assertEqual(independent["status"],"passed")
        self.assertEqual(hashes,{p:sha256(p) for p in hashes})
        goal = output / "home_to_park/submitted_goal.json"
        bad = json.loads(goal.read_text())
        bad["points"][-1]["time_from_start_ns"] += 1
        goal.write_text(json.dumps(bad))
        with self.assertRaisesRegex(ValueError,"artifact changed"):
            verify_mock_evidence(self.workspace,output / "evidence.json")

    def test_backend_cancels_late_goal_after_submission_timeout(self):
        backend = object.__new__(RosMotionBackend)
        backend.controller_name = "/joint_trajectory_controller/follow_joint_trajectory"
        backend.controller = SimpleNamespace(wait_for_server=Mock(return_value=True),send_goal_async=Mock(return_value=object()))
        backend.FollowJointTrajectory = SimpleNamespace(Goal=SimpleNamespace)
        handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(return_value="cancel"),get_result_async=Mock(return_value="terminal"))
        backend._wait = Mock(side_effect=[TimeoutError("submit"),handle,SimpleNamespace(),SimpleNamespace(status=5)])
        with patch("hx_waypoint_curobo.curobo_server._to_ros_trajectory",return_value=object()):
            with self.assertRaises(TimeoutError):
                backend.execute(self.short,30.)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_called_once()


if __name__ == "__main__":
    unittest.main()
