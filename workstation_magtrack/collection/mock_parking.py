#!/usr/bin/env python3
"""Independent fixed-clock mock playback of the two sealed parking prepare paths.

This is not a Cartesian drawing recording. It never plans, retimes, edits a
source CSV, opens a sensor, or connects to a real driver. ROS imports are lazy.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import asdict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Any, Mapping

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from collection.motion import MotionController, RosMotionBackend, sha256, utc_now, validate_trajectory, write_json_new
    from collection.validate_parking import campaign_gpu_lock
else:
    from .motion import MotionController, RosMotionBackend, sha256, utc_now, validate_trajectory, write_json_new
    from .validate_parking import campaign_gpu_lock


DOMAIN = 223
DIRECTIONS = ("home_to_park", "park_to_home")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    require(isinstance(value, dict), f"JSON object required: {path}")
    return value


def goal_payload(trajectory: Any) -> dict[str, Any]:
    """Exactly the values submitted to FJT, including every original ROS time."""
    return {"joint_names": list(trajectory.joint_names), "points": [
        {"time_from_start_ns": round(s.time_s*1e9), "positions": list(s.positions),
         "velocities": list(s.velocities), "accelerations": list(s.accelerations)}
        for s in trajectory.samples]}


def load_authority(motion: MotionController, validation_path: Path,
                   bundle_dir: Path | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    if __package__ in {None, ""}:
        from collection.prepared_parking import PreparedParkingBundle, _joint_bytes
    else:
        from .prepared_parking import PreparedParkingBundle, _joint_bytes
    motion._unchanged()
    validation_path = validation_path.resolve()
    require(validation_path.is_relative_to(motion.workspace / "runs"), "GPU validation must be inside this workspace runs/")
    value = read_json(validation_path)
    require(value.get("status") == "passed" and value.get("runtime_ready") is True
            and value.get("gpu_full_path_validation") == "passed_both_directions", "GPU validation not complete and accepted")
    require(value.get("simulation_only") is True and value.get("real_execution_authorized") is False
            and value.get("environment_id") == "mla_short", "GPU certificate is not mla_short simulation-only")
    require(value.get("environment_hashes") == motion.environment_hashes
            and value.get("config_sha256") == motion.config_sha256
            and value.get("curobo_lock_sha256") == motion.lock_sha256, "GPU certificate environment/config/lock mismatch")
    require(value.get("rod_length_m") == .190 and value.get("rod_diameter_m") == .005
            and value.get("hard_world_self_clearance_m") == .005
            and value.get("uniform_time_scale") == 1. and value.get("authority_mutated") is False,
            "GPU certificate changed rod, safety, or fixed-clock authority")
    require(value.get("joint_names") == list(motion.environment.joint_names), "GPU certificate joint names mismatch")
    require(set(value.get("directions",{})) == set(DIRECTIONS),"GPU certificate must contain exactly both prepare directions")
    expected_ends = {"home_to_park": (motion.home, motion.park), "park_to_home": (motion.park, motion.home)}
    paths: dict[str, Any] = {}
    for direction in DIRECTIONS:
        item = value["directions"][direction]
        original = Path(item["joint_trajectory_path"])
        source = original.resolve()
        require(not original.is_symlink() and source == validation_path.parent / direction / "joint_trajectory.csv",
                "prepare source escaped its producer directory")
        # Hash and parse one byte snapshot. Separate file reads would permit a
        # path replacement to bind one CSV's digest to another CSV's timing.
        source_data = source.read_bytes()
        source_sha = hashlib.sha256(source_data).hexdigest()
        require(item.get("status") == "passed" and source_sha == item.get("joint_trajectory_sha256"), "GPU source missing, failed, or changed")
        require(read_json(source.with_name("evidence.json")) == item, "per-direction GPU evidence differs from final certificate")
        trajectory = _joint_bytes(source_data, motion.environment.joint_names)
        validate_trajectory(motion.environment, trajectory, *expected_ends[direction])
        for key in ("gpu_metrics", "serialized_gpu_metrics"):
            metric = item[key]
            require(all(math.isfinite(float(x)) for x in metric.values()), "nonfinite GPU metric")
            require(metric["minimum_collision_distance_m"] >= .005
                    and metric["checked_sample_count"] == len(trajectory.samples)
                    and all(0 <= metric[k] <= 1e-7 for k in ("max_scene_collision_cost", "max_self_collision_cost", "max_joint_bound_cost")),
                    "GPU full-path safety metric rejected")
        paths[direction] = {"trajectory": trajectory, "source": source, "sha256": source_sha}
    binding = {"validation_path": str(validation_path), "validation_sha256": sha256(validation_path)}
    if bundle_dir is not None:
        bundle = PreparedParkingBundle(motion.workspace, bundle_dir, config_path=motion.config_path)
        bundle.verify()
        for direction in DIRECTIONS:
            sealed, _ = bundle.trajectory(direction)
            require(sealed.joint_names == paths[direction]["trajectory"].joint_names
                    and sealed.samples == paths[direction]["trajectory"].samples,
                    "prepared bundle changed authoritative q/qd/qdd/t")
        binding.update(bundle_path=str(bundle_dir.resolve()), bundle_manifest_sha256=bundle.manifest_sha256)
    return paths, binding


def _vector(value: str, width: int, name: str) -> list[float]:
    result = json.loads(value)
    require(isinstance(result, list) and len(result) == width
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in result),
            f"invalid controller {name}")
    return result


def audit_witness(path: Path, trajectory: Any, motion: MotionController,
                  telemetry_path: Path | None = None, action: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Recheck every native controller row against continuous frozen quintics."""
    from runtime.controller_witness import select_episode
    from hx_waypoint_curobo.robot_kinematics import Pose, QuinticHermiteSegment, pose_error
    with path.open(newline="") as stream:
        raw = list(csv.DictReader(stream))
    config = motion.environment.recording["recording"]
    limits = config["validation"]
    dt, jitter = float(config["sample_dt_s"]), float(limits["max_time_jitter_s"])
    selected, coverage = select_episode(raw, trajectory.duration_s, dt, jitter)
    qtimes = [s.time_s for s in trajectory.samples]
    cache: dict[int, list[Any]] = {}
    maxima = {k: 0. for k in ("reference_q_rad", "reference_qd_rad_s", "reference_qdd_rad_s2",
              "actual_reference_q_rad", "actual_reference_qd_rad_s", "actual_expected_q_rad",
              "actual_expected_qd_rad_s", "actual_fk_position_m", "actual_fk_orientation_deg",
              "clock_error_s", "feedback_clock_offset_s")}
    telemetry: list[dict[str, Any]] = []
    source_ros = source_rx = None
    first_t = first_ros = None
    scales = set()
    for row in selected:
        names = json.loads(row["joint_names"])
        require(len(names) == len(set(names)) == 6 and set(names) == set(trajectory.joint_names), "controller joint names mismatch")
        order = [names.index(n) for n in trajectory.joint_names]
        vectors = [[_vector(row[k], 6, k)[i] for i in order]
                   for k in ("reference_q", "reference_qd", "reference_qdd", "actual_q", "actual_qd")]
        ref, ref_v, ref_a, actual, actual_v = vectors
        t, ft = float(row["reference_time_s"]), float(row["feedback_time_s"])
        require(math.isfinite(t) and math.isfinite(ft) and abs(t-ft) <= jitter,
                "reference and actual lack a coherent controller clock")
        maxima["feedback_clock_offset_s"] = max(maxima["feedback_clock_offset_s"],abs(t-ft))
        ros_ns, rx_ns = int(row["controller_ros_time_ns"]), int(row["rx_sync_time_ns"])
        require(ros_ns > 0 and rx_ns > 0 and (source_ros is None or ros_ns > source_ros)
                and (source_rx is None or rx_ns > source_rx), "controller/monotonic timestamps do not strictly advance")
        source_ros, source_rx = ros_ns, rx_ns
        if action is not None:
            # Bind this native-clock episode to this very controller request,
            # not a copied old witness or a preceding prepare/hold goal.
            tolerance_ns = round((dt+jitter)*1e9)
            require(action["submitted_sync_time_ns"]-tolerance_ns <= rx_ns <= action["completed_sync_time_ns"]+tolerance_ns
                    and action["submitted_ros_time_ns"]-tolerance_ns <= ros_ns <= action["completed_ros_time_ns"]+tolerance_ns,
                    "controller witness lies outside this FJT execution window")
        if first_t is None:
            first_t, first_ros = t, ros_ns
        clock_error = abs((ros_ns-first_ros)*1e-9-(t-first_t))
        maxima["clock_error_s"] = max(maxima["clock_error_s"], clock_error)
        scale = float(row["raw_speed_scaling_factor"])
        require(math.isfinite(scale) and scale in (0., 1.), "unexpected unscaled controller speed factor")
        scales.add(scale)
        index = min(max(bisect_right(qtimes, t)-1, 0), len(qtimes)-2)
        before, after = trajectory.samples[index:index+2]
        if index not in cache:
            cache[index] = [QuinticHermiteSegment.from_boundary_conditions(
                duration_s=after.time_s-before.time_s, start_position=before.positions[j],
                start_velocity=before.velocities[j], start_acceleration=before.accelerations[j],
                end_position=after.positions[j], end_velocity=after.velocities[j],
                end_acceleration=after.accelerations[j]) for j in range(6)]
        expected = [[segment.evaluate(t-before.time_s, derivative) for segment in cache[index]] for derivative in range(3)]
        comparisons = (("reference_q_rad", ref, expected[0]), ("reference_qd_rad_s", ref_v, expected[1]),
                       ("reference_qdd_rad_s2", ref_a, expected[2]), ("actual_reference_q_rad", actual, ref),
                       ("actual_reference_qd_rad_s", actual_v, ref_v), ("actual_expected_q_rad", actual, expected[0]),
                       ("actual_expected_qd_rad_s", actual_v, expected[1]))
        for key, observed, target in comparisons:
            maxima[key] = max(maxima[key], max(abs(a-b) for a,b in zip(observed,target,strict=True)))
        actual_fk, expected_fk = motion.tip_fk(actual), motion.tip_fk(expected[0])
        error = pose_error(Pose(actual_fk["base_link_position_m"], actual_fk["base_link_quaternion_xyzw"]),
                           Pose(expected_fk["base_link_position_m"], expected_fk["base_link_quaternion_xyzw"]))
        maxima["actual_fk_position_m"] = max(maxima["actual_fk_position_m"], error.position_m)
        maxima["actual_fk_orientation_deg"] = max(maxima["actual_fk_orientation_deg"], error.orientation_deg)
        telemetry.append({"controller_time_s": t, "controller_ros_time_ns": ros_ns, "rx_sync_time_ns": rx_ns,
                          "actual_q": actual, "actual_qd": actual_v, "canonical_q": expected[0],
                          "canonical_qd": expected[1], "canonical_qdd": expected[2],
                          "actual_tip_plane1_m": actual_fk["plane1_position_m"],
                          "actual_tip_plane1_quaternion_xyzw": actual_fk["plane1_quaternion_xyzw"],
                          "canonical_tip_plane1_m": expected_fk["plane1_position_m"],
                          "tip_position_error_m": error.position_m, "tip_orientation_error_deg": error.orientation_deg})
    planner = motion.environment.planner["validation"]
    checks = {"reference_q_rad": limits["max_controller_reference_position_error_rad"],
              "reference_qd_rad_s": limits["max_controller_reference_velocity_error_rad_s"],
              # Original action acceleration fields are fixed authority; this
              # precision check introduces no looser new dynamics tolerance.
              "reference_qdd_rad_s2": 1e-6,
              "actual_reference_q_rad": limits["max_actual_controller_position_error_rad"],
              "actual_reference_qd_rad_s": limits["max_actual_controller_velocity_error_rad_s"],
              "actual_expected_q_rad": limits["max_joint_position_tracking_error_rad"],
              "actual_expected_qd_rad_s": limits["max_joint_velocity_tracking_error_rad_s"],
              "actual_fk_position_m": planner["tip_position_tolerance_m"],
              "actual_fk_orientation_deg": planner["tip_orientation_tolerance_deg"],
              "clock_error_s": dt+jitter, "feedback_clock_offset_s": jitter}
    for key, bound in checks.items():
        require(maxima[key] <= float(bound)+1e-9, f"mock {key} {maxima[key]} exceeds original limit {bound}")
    require(len(scales) == 1, "raw controller speed-scaling field changed during fixed-clock playback")
    # A derived view of real native controller rows, never synthetic 100 Hz data.
    if telemetry_path is not None:
        with telemetry_path.open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(telemetry[0]))
            writer.writeheader()
            for row in telemetry:
                writer.writerow({k: json.dumps(v) if isinstance(v, list) else v for k,v in row.items()})
    return {"status": "passed", **coverage, "max_errors": maxima, "limits": checks,
            "authoritative_duration_s": trajectory.duration_s, "authoritative_point_count": len(trajectory.samples),
            "raw_speed_scaling_factor_values": sorted(scales), "source_timing_changed": False,
            "all_native_rows_audited": True, "resampling_or_synthetic_rows": False,
            "expected_curve": "original q/qd/qdd/time_s continuous quintic; same controller clock as actual",
            "speed_scaling_semantics": "standard unscaled JointTrajectoryController; raw default 0 is not effective scaling",
            "telemetry_rows": telemetry}


def scene_snapshot(scene: Any) -> dict[str, Any]:
    def object_data(obj: Any) -> dict[str, Any]:
        poses = []
        for pose in obj.primitive_poses:
            poses.append({"position_m": [getattr(pose.position,a) for a in "xyz"],
                          "quaternion_xyzw": [getattr(pose.orientation,a) for a in "xyzw"]})
        object_pose = getattr(obj,"pose",None)
        origin = {"position_m":[0.,0.,0.],"quaternion_xyzw":[0.,0.,0.,1.]}
        if object_pose is not None:
            origin = {"position_m":[getattr(object_pose.position,a) for a in "xyz"],
                      "quaternion_xyzw":[getattr(object_pose.orientation,a) for a in "xyzw"]}
        return {"id": obj.id, "frame_id": obj.header.frame_id, "object_pose":origin,
                "primitives": [{"type": int(p.type), "dimensions": list(p.dimensions)} for p in obj.primitives],
                "primitive_poses": poses, "mesh_count": len(obj.meshes), "plane_count": len(obj.planes)}
    return {"world": [object_data(o) for o in scene.world.collision_objects],
            "attached": [{"link_name": o.link_name, "touch_links": list(o.touch_links),
                          "object": object_data(o.object)} for o in scene.robot_state.attached_collision_objects]}


def verify_scene_snapshot(snapshot: Mapping[str, Any], motion: MotionController) -> dict[str, Any]:
    import numpy as np
    from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition
    from hx_waypoint_curobo.math3d import pose_matrix, matrix_pose
    definition = load_planning_scene_definition(motion.workspace, "mla_short")
    # The parsed model has already validated the frozen generated URDF. Only
    # fixed-joint paths may reinterpret a world object's returned header frame;
    # a moving link that happens to coincide at home must never be admitted.
    fixed_frames = {motion.environment.base_frame:np.eye(4)}
    fixed_joints = [j for j in motion.model._parent_joint_by_child.values() if j.joint_type == "fixed"]
    changed = True
    while changed:
        changed = False
        for joint in fixed_joints:
            if joint.parent_link in fixed_frames and joint.child_link not in fixed_frames:
                fixed_frames[joint.child_link] = fixed_frames[joint.parent_link] @ np.asarray(joint.origin)
                changed = True
            elif joint.child_link in fixed_frames and joint.parent_link not in fixed_frames:
                fixed_frames[joint.parent_link] = fixed_frames[joint.child_link] @ np.linalg.inv(np.asarray(joint.origin))
                changed = True
    def close(a: Any, b: Any) -> bool:
        return len(a) == len(b) and all(math.isfinite(float(x)) and abs(x-y) <= 1e-8 for x,y in zip(a,b,strict=True))
    def geometry(obj: Mapping[str, Any], frame: str, kind: int, dimensions: Any, position: Any, quaternion: Any,
                 *, world: bool = False) -> None:
        frame_allowed = obj["frame_id"] in fixed_frames if world else obj["frame_id"] == frame
        require(frame_allowed and obj["mesh_count"] == 0 and obj["plane_count"] == 0
                and len(obj["primitives"]) == len(obj["primitive_poses"]) == 1, "mock scene geometry/frame differs")
        primitive, pose = obj["primitives"][0], obj["primitive_poses"][0]
        origin = obj["object_pose"]
        for transform in (origin,pose):
            require(len(transform["position_m"]) == 3 and len(transform["quaternion_xyzw"]) == 4
                    and all(math.isfinite(float(v)) for v in (*transform["position_m"],*transform["quaternion_xyzw"]))
                    and abs(sum(v*v for v in transform["quaternion_xyzw"])-1.) <= 1e-6,
                    "mock scene contains an invalid rigid transform")
        # MoveIt/Jazzy may lift a single primitive's pose to CollisionObject.pose
        # and return an identity primitive pose. Compare physical geometry, not
        # which equivalent serialization field carries the transform.
        composed = pose_matrix(origin["position_m"],origin["quaternion_xyzw"]) @ pose_matrix(pose["position_m"],pose["quaternion_xyzw"])
        if world:
            require(frame == motion.environment.base_frame,"expected cuboid must use environment base frame")
            composed = fixed_frames[obj["frame_id"]] @ composed
        physical_position, physical_quaternion = matrix_pose(composed)
        require(primitive["type"] == kind and close(primitive["dimensions"], dimensions)
                and close(physical_position,position)
                and (close(physical_quaternion,quaternion) or close(physical_quaternion,[-x for x in quaternion])),
                "mock scene primitive dimensions/pose differs from mla_short")
    world = {o["id"]:o for o in snapshot["world"]}
    require(len(world) == len(snapshot["world"]) and set(world) == {c.object_id for c in definition.cuboids}, "mock world is incomplete or has extra obstacles")
    for cuboid in definition.cuboids:
        geometry(world[cuboid.object_id],cuboid.frame_id,1,cuboid.dimensions_m,cuboid.position_m,cuboid.quaternion_xyzw,world=True)
    require(len(snapshot["attached"]) == 1, "mock must have exactly one attached 190mm rod")
    attached, rod = snapshot["attached"][0], definition.attached_rod
    require(attached["link_name"] == rod.link_name and attached["object"]["id"] == rod.object_id
            and set(attached["touch_links"]) == set(rod.touch_links), "attached rod linkage/touch policy differs")
    geometry(attached["object"],rod.link_name,3,[rod.length_m,rod.radius_m],rod.center_in_link_m,rod.quaternion_xyzw)
    return {"status":"passed", "world_object_ids":sorted(world), "rod_length_m":rod.length_m,
            "rod_radius_m":rod.radius_m,"world_frame_resolution":"frozen generated URDF fixed joints only",
            "world_object_frames":{name:obj["frame_id"] for name,obj in sorted(world.items())},"snapshot":dict(snapshot)}


def verify_runtime_identity(identity: Mapping[str, Any], workspace: Path) -> None:
    require(identity.get("domain_id") == DOMAIN and identity.get("execution_mode") == "simulation"
            and identity.get("environments_root") == str(workspace / "environments")
            and identity.get("telemetry_root") == str(workspace / "runs")
            and identity.get("use_sim_time") is False and identity.get("externally_managed") is False,
            "parking mock runtime identity is not the owned domain 223 simulation")


class MockSession:
    """One owned ROS launch group. Existing domains are never reused or killed."""
    def __init__(self, motion: MotionController, output: Path, startup_timeout_s: float = 120.):
        self.motion, self.output, self.timeout = motion, output, startup_timeout_s
        self.process = self.log = self.backend = None

    def __enter__(self) -> MockSession:
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        from controller_manager_msgs.srv import ListControllers
        os.environ.pop("ROS_DISCOVERY_SERVER", None)
        os.environ.pop("ROS_STATIC_PEERS", None)
        os.environ.update(ROS_DOMAIN_ID=str(DOMAIN), ROS_LOCALHOST_ONLY="1", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                          ROS_LOG_DIR=str(self.output / "ros_logs"))
        try:
            self.backend = RosMotionBackend(self.motion.environment,"simulation",DOMAIN)
            self.motion.backend = self.backend
            for _ in range(15):
                self.backend.executor.spin_once(timeout_sec=.1)
            names = self.backend.node.get_node_names()
            require(not any(n != self.backend.node.get_name() for n in names),
                    "domain 223 already contains ROS nodes; refusing to attach")
            command = ["ros2","launch","hx_waypoint_bringup","simulation.launch.py","ur_type:=ur5",
                       "environment_id:=mla_short",f"workspace:={self.motion.workspace}","launch_rviz:=false"]
            self.log = (self.output / "mock_runtime.log").open("x")
            self.log.write("COMMAND " + json.dumps(command) + "\n")
            self.log.flush()
            self.process = subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline = time.monotonic()+self.timeout
            while time.monotonic() < deadline:
                self.check_alive()
                if self.backend.parameters.service_is_ready() and self.backend.controller.server_is_ready():
                    break
                self.backend.executor.spin_once(timeout_sec=.1)
            else:
                raise TimeoutError("owned parking mock runtime startup timed out")
            identity = self.motion.verify_runtime_identity()
            identity.update(domain_id=DOMAIN,externally_managed=False,launch_pid=self.process.pid,
                            launch_command=command,runtime_log=str(self.output / "mock_runtime.log"))
            verify_runtime_identity(identity,self.motion.workspace)
            controllers = self.backend.node.create_client(ListControllers,"/controller_manager/list_controllers")
            require(controllers.wait_for_service(timeout_sec=20.),"controller identity service unavailable")
            response = self.backend._wait(controllers.call_async(ListControllers.Request()),20.)
            matches = [c for c in response.controller if c.name == "joint_trajectory_controller"]
            require(len(matches) == 1 and matches[0].state == "active"
                    and matches[0].type == "joint_trajectory_controller/JointTrajectoryController", "mock is not the standard unscaled active JTC")
            identity["controller"] = {"name":matches[0].name,"type":matches[0].type,"state":matches[0].state}
            client = self.backend.node.create_client(GetPlanningScene,"/get_planning_scene")
            require(client.wait_for_service(timeout_sec=30.),"mock planning scene unavailable")
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            while time.monotonic() < deadline:
                response = self.backend._wait(client.call_async(request),10.)
                try:
                    self.scene = verify_scene_snapshot(scene_snapshot(response.scene),self.motion)
                    break
                except ValueError:
                    self.check_alive()
                    self.backend.executor.spin_once(timeout_sec=.1)
            else:
                raise TimeoutError("complete mla_short world and 190mm rod were not installed")
            self.identity = identity
            self.motion._state(target=self.motion.home)
            return self
        except BaseException:
            self.close()
            raise

    def check_alive(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError(f"owned parking mock launch exited {self.process.returncode}")

    def play(self, direction: str, trajectory: Any, directory: Path) -> dict[str, Any]:
        from runtime.controller_witness import ControllerWitness
        self.check_alive()
        self.motion.verify_runtime_identity()
        start = self.motion._state(target=trajectory.samples[0].positions)
        write_json_new(directory / "submitted_goal.json",goal_payload(trajectory))
        witness = ControllerWitness(self.backend.node,directory / "controller_state.csv")
        result: dict[str, Any] = {"status":"running","started_utc":utc_now(),"direction":direction,
                                  "fresh_start":asdict(start),"source_point_count":len(trajectory.samples)}
        try:
            result.update(submitted_sync_time_ns=time.monotonic_ns(),submitted_ros_time_ns=time.time_ns())
            result["controller_result"] = dict(self.backend.execute(trajectory,trajectory.duration_s+30.))
            require(result["controller_result"].get("success") is True
                    and result["controller_result"].get("action_status") == 4
                    and result["controller_result"].get("controller_error_code") == 0,"mock controller did not succeed")
            self.check_alive()
            result["status"] = "passed"
            return result
        except BaseException as error:
            result.update(status="failed",error=f"{type(error).__name__}: {error}")
            raise
        finally:
            witness.close()
            result.update(completed_sync_time_ns=time.monotonic_ns(),completed_ros_time_ns=time.time_ns())
            result["finished_utc"] = utc_now()
            write_json_new(directory / "action_result.json",result)

    def settle(self, target: Any) -> dict[str, Any]:
        deadline = time.monotonic()+float(self.motion.options["settle_timeout_s"])
        while time.monotonic() < deadline:
            self.check_alive()
            try:
                return asdict(self.motion._state(target=target))
            except ValueError as error:
                if "not stationary" not in str(error) and "left requested" not in str(error):
                    raise
        raise TimeoutError("mock controller failed to settle at the authoritative endpoint")

    def close(self) -> None:
        if self.process is not None:
            from runtime.batch_recordings import stop_process
            stop_process(self.process)
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if self.backend is not None:
            self.backend.close()
            self.backend = None
            self.motion.backend = None

    def __exit__(self, *args: Any) -> None:
        self.close()


def _audit_summary(audit: dict[str, Any]) -> dict[str, Any]:
    return {k:v for k,v in audit.items() if k != "telemetry_rows"}


def verify_mock_evidence(workspace: Path, evidence_path: Path,
                         bundle_dir: Path | None = None) -> dict[str, Any]:
    """Read-only verification: recompute the complete witness, not passed flags."""
    workspace, evidence_path = Path(workspace).resolve(), Path(evidence_path).resolve()
    value = read_json(evidence_path)
    require(value.get("schema_version") == 1 and value.get("kind") == "parking_prepare_mock_validation"
            and value.get("status") == "passed" and value.get("simulation_only") is True
            and value.get("real_execution_authorized") is False and value.get("environment_id") == "mla_short"
            and value.get("domain_id") == DOMAIN and value.get("runtime_replanning") is False
            and value.get("source_timing_changed") is False,"parking mock evidence is not accepted simulation evidence")
    motion = MotionController(workspace,mode="simulation",domain_id=DOMAIN)
    try:
        require(value.get("code_sha256",{}).get(Path(__file__).name) == sha256(Path(__file__)),"mock validator code changed since playback")
        paths, binding = load_authority(motion,Path(value["authority"]["validation_path"]),bundle_dir)
        require(value["environment_hashes"] == motion.environment_hashes
                and value["config_sha256"] == motion.config_sha256
                and value["curobo_lock_sha256"] == motion.lock_sha256,"mock environment/config/lock changed")
        require(all(value["authority"].get(k) == v for k,v in binding.items()),"mock authority binding changed")
        verify_runtime_identity(value["runtime"],workspace)
        require(value["runtime"].get("controller") == {"name":"joint_trajectory_controller",
                    "type":"joint_trajectory_controller/JointTrajectoryController","state":"active"},
                "mock controller identity is not the active standard unscaled JTC")
        require(verify_scene_snapshot(value["scene"]["snapshot"],motion) == value["scene"],"mock world/rod evidence changed")
        result = {}
        for direction in DIRECTIONS:
            item = value["directions"][direction]
            require(item["status"] == "passed" and item["source_sha256_before"] == item["source_sha256_after"] == paths[direction]["sha256"],"mock prepare source authority changed")
            directory = evidence_path.parent / direction
            for name in ("submitted_goal.json","action_result.json","controller_state.csv","actual_fk.csv"):
                require(sha256(directory / name) == item["artifact_sha256"][name],f"mock artifact changed: {direction}/{name}")
            trajectory = paths[direction]["trajectory"]
            require(read_json(directory / "submitted_goal.json") == goal_payload(trajectory),"submitted FJT skipped/changed/retimed original points")
            action = read_json(directory / "action_result.json")
            controller = action["controller_result"]
            require(action["status"] == "passed" and controller.get("success") is True
                    and controller.get("action_status") == 4 and controller.get("controller_error_code") == 0
                    and controller.get("action_name") == motion.environment.controllers["simulation"]["action_name"],"real mock FJT result did not succeed")
            require(0 <= action["submitted_sync_time_ns"]-action["fresh_start"]["sync_time_ns"] <= 500_000_000
                    and 0 <= action["submitted_ros_time_ns"]-action["fresh_start"]["ros_stamp_ns"] <= 500_000_000,
                    "mock start was not fresh at FJT submission")
            for label, target in (("fresh_start",trajectory.samples[0].positions),("settled_end",trajectory.samples[-1].positions)):
                state = action[label] if label == "fresh_start" else item[label]
                require(state["joint_names"] == list(motion.environment.joint_names)
                        and max(abs(a-b) for a,b in zip(state["positions"],target,strict=True)) <= motion.options["endpoint_tolerance_rad"]
                        and max(abs(v) for v in state["velocities"]) <= motion.options["settle_velocity_rad_s"],"mock actual start/end state differs")
            audit = audit_witness(directory / "controller_state.csv",trajectory,motion,action=action)
            require(_audit_summary(audit) == item["audit"],"mock witness audit evidence differs from recomputation")
            with (directory / "actual_fk.csv").open(newline="") as stream:
                derived = list(csv.DictReader(stream))
            require(len(derived) == len(audit["telemetry_rows"]),"mock FK view omitted native rows")
            for row, expected in zip(derived,audit["telemetry_rows"],strict=True):
                for key, target in expected.items():
                    observed = (json.loads(row[key]) if isinstance(target,list)
                                else int(row[key]) if isinstance(target,int) else float(row[key]))
                    require(observed == target,f"mock actual FK view differs at {key}")
            result[direction] = _audit_summary(audit)
        return {"status":"passed","evidence_path":str(evidence_path),"evidence_sha256":sha256(evidence_path),
                "directions":result,"authority":binding,"real_execution_authorized":False}
    finally:
        motion.close()


def _producer_output(argv: list[str], cwd: Path) -> Path:
    require(len(argv) >= 4 and re.fullmatch(r"python(?:[0-9]+(?:\.[0-9]+)*)?",Path(argv[0]).name) is not None
            and argv[1:3] == ["-m","collection.validate_parking"] and argv[3:].count("--gpu") == 1,
            "producer PID is not the actual Python GPU parking validator")
    outputs = []
    for index, argument in enumerate(argv[3:],3):
        if argument == "--output":
            require(index+1 < len(argv) and argv[index+1] and not argv[index+1].startswith("--"),
                    "GPU producer --output has no value")
            outputs.append(argv[index+1])
        elif argument.startswith("--output="):
            outputs.append(argument.split("=",1)[1])
    require(len(outputs) == 1 and bool(outputs[0]),"GPU producer must have exactly one explicit --output")
    require(cwd.is_absolute(),"GPU producer cwd is not absolute")
    output = Path(outputs[0])
    return (output if output.is_absolute() else cwd/output).resolve()


def _live_producer_identity(producer_pid: int | None, path: Path) -> str:
    require(producer_pid is not None and producer_pid > 1,"--wait requires --producer-pid for a live GPU validation producer")
    proc = Path("/proc") / str(producer_pid)
    raw = (proc / "cmdline").read_bytes().split(b"\0")
    require(raw[-1] == b"" and all(raw[:-1]),"GPU producer cmdline is not complete NUL-delimited argv")
    argv = [part.decode() for part in raw[:-1]]
    output = _producer_output(argv,(proc / "cwd").readlink())
    stat = (proc / "stat").read_text().rsplit(")",1)[1].split()
    require(stat[0] != "Z" and output == path.parent.resolve(),"producer PID is not the live matching parking GPU validator")
    os.kill(producer_pid,0)
    return stat[19]  # Kernel process start ticks: protect against PID reuse.


def wait_for_validation(path: Path, *, wait: bool, producer_pid: int | None,
                        poll_s: float = 1.) -> None:
    """A wait requires a live exact producer; a stale file/lock is not a job.

    The old producer writes JSON non-atomically. A partial read is transient
    only while that exact producer is still proven live, never after it exits.
    """
    identity = None
    while True:
        if path.is_file():
            try:
                status = read_json(path).get("status")
            except json.JSONDecodeError:
                if not wait:
                    raise
                status = None
            if status == "passed":
                return
            if status in {"failed","cpu_precheck_only"}:
                raise RuntimeError(f"parking GPU producer ended with {status}")
        if not wait:
            raise RuntimeError("accepted parking GPU certificate not available; no mock launched")
        try:
            current = _live_producer_identity(producer_pid,path)
            if identity is None:
                identity = current
            require(current == identity,"GPU producer PID was reused")
        except (FileNotFoundError,ProcessLookupError) as error:
            # Final strict read: a complete accepted report may have appeared
            # between the first read and process exit; partial JSON cannot pass.
            try:
                if path.is_file() and read_json(path).get("status") == "passed":
                    return
            except json.JSONDecodeError as malformed:
                raise RuntimeError("GPU producer exited with a malformed certificate; no mock launched") from malformed
            raise RuntimeError("GPU producer exited without an accepted certificate; no mock launched") from error
        time.sleep(poll_s)


def run(workspace: Path, validation_path: Path, output: Path, *, bundle_dir: Path | None = None,
        wait: bool = False, producer_pid: int | None = None, session_factory: Any = MockSession) -> dict[str, Any]:
    workspace, validation_path, output = workspace.resolve(), validation_path.resolve(), output.resolve()
    require(output.is_relative_to(workspace / "runs"),"mock artifacts must stay inside remote workspace runs/")
    output.mkdir(parents=True,exist_ok=False)
    evidence: dict[str, Any] = {"schema_version":1,"kind":"parking_prepare_mock_validation","status":"running",
        "simulation_only":True,"real_execution_authorized":False,"environment_id":"mla_short","domain_id":DOMAIN,
        "started_utc":utc_now(),"robot_execution":"not_requested_not_run","sensor_access":"not_requested_not_run",
        "runtime_replanning":False,"source_timing_changed":False,"directions":{},
        "code_sha256":{Path(__file__).name:sha256(Path(__file__))}}
    motion = None
    try:
        wait_for_validation(validation_path,wait=wait,producer_pid=producer_pid)
        # Producer already queued after all campaigns. Do not wait on a stale
        # lock file or let this mock compete with unfinished 100 Hz campaigns.
        with campaign_gpu_lock(workspace,False):
            motion = MotionController(workspace,mode="simulation",domain_id=DOMAIN)
            paths, binding = load_authority(motion,validation_path,bundle_dir)
            evidence.update(authority=binding,environment_hashes=motion.environment_hashes,
                            config_sha256=motion.config_sha256,curobo_lock_sha256=motion.lock_sha256)
            with session_factory(motion,output) as session:
                evidence.update(runtime=session.identity,scene=session.scene)
                for direction in DIRECTIONS:
                    directory = output / direction
                    directory.mkdir()
                    source = paths[direction]["source"]
                    before = sha256(source)
                    require(before == paths[direction]["sha256"],"prepare source changed before mock submission")
                    motion._unchanged()
                    trajectory = paths[direction]["trajectory"]
                    action = session.play(direction,trajectory,directory)
                    require(before == sha256(source),"original prepare CSV changed during mock playback")
                    # Witness closes immediately at controller result, before
                    # endpoint settling so extra hold rows cannot hide gaps.
                    audit = audit_witness(directory / "controller_state.csv",trajectory,motion,directory / "actual_fk.csv",action)
                    end = session.settle(trajectory.samples[-1].positions)
                    require(before == sha256(source),"original prepare CSV changed during mock playback")
                    evidence["directions"][direction] = {"status":"passed","source_path":str(source),
                        "source_sha256_before":before,"source_sha256_after":sha256(source),
                        "settled_end":end,"audit":_audit_summary(audit),
                        "artifact_sha256":{name:sha256(directory / name) for name in
                            ("submitted_goal.json","action_result.json","controller_state.csv","actual_fk.csv")}}
                motion._unchanged()
                for direction in DIRECTIONS:
                    require(sha256(paths[direction]["source"]) == paths[direction]["sha256"],"source authority changed")
            evidence["status"] = "passed"
            return evidence
    except BaseException as error:
        evidence.update(status="failed",error=f"{type(error).__name__}: {error}",automatic_return_on_failure=False)
        raise
    finally:
        if motion is not None:
            motion.close()
        evidence["finished_utc"] = utc_now()
        write_json_new(output / "evidence.json",evidence)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument("--validation",required=True,type=Path)
    parser.add_argument("--bundle",type=Path)
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--wait",action="store_true")
    parser.add_argument("--producer-pid",type=int)
    parser.add_argument("--verify-only",action="store_true")
    args = parser.parse_args(argv)
    # The launched ROS process owns a separate process group. SIGTERM must
    # unwind FJT cancellation, witness close, and that owned launch's cleanup,
    # exactly like Ctrl+C; Python's default SIGTERM would bypass every finally.
    if __package__ in {None, ""}:
        from collection.run import controlled_termination
    else:
        from .run import controlled_termination
    try:
        with controlled_termination():
            if args.verify_only:
                result = verify_mock_evidence(args.workspace,args.output / "evidence.json",args.bundle)
            else:
                result = run(args.workspace,args.validation,args.output,bundle_dir=args.bundle,
                             wait=args.wait,producer_pid=args.producer_pid)
                verify_mock_evidence(args.workspace,args.output / "evidence.json",args.bundle)
    except KeyboardInterrupt:
        print("parking mock interrupted: owned goal and ROS launch cleanup attempted; inspect failure evidence",file=sys.stderr)
        return 130
    print(json.dumps({"status":result["status"],"evidence":str(args.output / "evidence.json")}),flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
