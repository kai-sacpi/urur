"""Passive, native-rate controller-state witness; never commands anything."""
from bisect import bisect_right
import csv
import math
from pathlib import Path
import statistics
import time


class ControllerWitness:
    def __init__(self, node, path):
        from control_msgs.msg import JointTrajectoryControllerState
        from rclpy.qos import QoSProfile
        self.path = Path(path)
        self.stream = self.path.open("w", newline="")
        self.writer = csv.DictWriter(self.stream, ["rx_sync_time_ns", "controller_ros_time_ns",
                                                  "reference_time_s", "feedback_time_s",
                                                  "raw_speed_scaling_factor", "joint_names",
                                                  "reference_q", "reference_qd", "reference_qdd",
                                                  "actual_q", "actual_qd"])
        self.writer.writeheader()
        self.node = node
        self.subscription = node.create_subscription(JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state", self.receive, QoSProfile(depth=1000))

    def receive(self, message):
        import json
        seconds = lambda duration: duration.sec + duration.nanosec*1e-9
        self.writer.writerow({"rx_sync_time_ns": time.monotonic_ns(),
                              "controller_ros_time_ns": message.header.stamp.sec*1000000000+message.header.stamp.nanosec,
                              "reference_time_s": seconds(message.reference.time_from_start),
                              "feedback_time_s": seconds(message.feedback.time_from_start),
                              "raw_speed_scaling_factor": message.speed_scaling_factor,
                              "joint_names": json.dumps(list(message.joint_names)),
                              "reference_q": json.dumps(list(message.reference.positions)),
                              "reference_qd": json.dumps(list(message.reference.velocities)),
                              "reference_qdd": json.dumps(list(message.reference.accelerations)),
                              "actual_q": json.dumps(list(message.feedback.positions)),
                              "actual_qd": json.dumps(list(message.feedback.velocities))})

    def close(self):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        self.stream.close()


def select_episode(rows, duration, nominal_dt, jitter):
    """Select the last complete native-time goal; retain all raw rows on disk."""
    episodes = []
    episode = []
    last_time = None
    for row in rows:
        t = float(row["reference_time_s"])
        if last_time is not None and t < last_time - 1e-6:
            if episode:
                episodes.append(episode)
            episode = []
        episode.append(row)
        last_time = t
    if episode:
        episodes.append(episode)
    for episode in reversed(episodes):
        selected = [row for row in episode if 0 <= float(row["reference_time_s"]) <= duration]
        if len(selected) < 2:
            continue
        times = [float(row["reference_time_s"]) for row in selected]
        intervals = [b-a for a, b in zip(times, times[1:]) if b>a]
        if not intervals:
            continue
        native = statistics.median(intervals)
        # Use the environment's fixed sampling contract as the coverage bound.
        # A sparse/outlier stream must never increase its own acceptance limit.
        # Native125Hz is retained unchanged; the10ms+2ms contract accommodates
        # its real8ms samples without pretending they are canonical100Hz rows.
        endpoint = nominal_dt+jitter
        min_rows = math.ceil(max(0.0, duration-2*endpoint)/endpoint)+1
        if (times[0] <= endpoint and times[-1] >= duration-endpoint and
                max(intervals) <= endpoint+1e-9 and len(set(times)) >= min_rows):
            return selected, {"raw_rows": len(rows), "selected_rows": len(selected),
                              "raw_goal_episodes": len(episodes),
                              "first_controller_time_s": times[0], "last_controller_time_s": times[-1],
                              "native_median_interval_s": native, "native_max_interval_s": max(intervals),
                              "native_max_jitter_s": max(abs(x-native) for x in intervals),
                              "endpoint_coverage_tolerance_s": endpoint,
                              "maximum_allowed_controller_gap_s": endpoint,
                              "minimum_distinct_controller_samples": min_rows,
                              "coverage_bound_source": "original recording sample_dt_s + max_time_jitter_s; never inferred from observed gaps",
                              "selection": "last complete reference.time_from_start episode; all raw prepare/hold rows retained"}
    raise ValueError("Independent controller-state witness has no complete native-clock source episode")


def audit_controller_witness(path, recording_path, environment_path):
    import json
    from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
    from hx_waypoint_curobo.recording_store import RecordingInspector
    from hx_waypoint_curobo.robot_kinematics import Pose, QuinticHermiteSegment, RobotKinematics, pose_error
    from hx_waypoint_curobo.trajectory_io import read_joint_trajectory_csv, read_timed_pose_csv
    environment = EnvironmentBuilder().load(environment_path)
    recording_path = Path(recording_path)
    source = read_timed_pose_csv(recording_path / "source_tip_trajectory.csv")
    trajectory = read_joint_trajectory_csv(recording_path / "joint_trajectory.csv", environment.joint_names)
    config = environment.recording["recording"]
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    selected, metrics = select_episode(rows, source[-1].time_s, float(config["sample_dt_s"]),
                                       float(config["validation"]["max_time_jitter_s"]))
    qtimes = [sample.time_s for sample in trajectory.samples]
    source_times = [pose.time_s for pose in source]
    source_base = [environment.transform_pose_to_base(pose) for pose in source]
    targets = [Pose(position_m=pose.position_m, quaternion_xyzw=pose.quaternion_xyzw) for pose in source_base]
    generated = environment.asset_provenance["generated_urdf"]["path"]
    model = RobotKinematics.from_urdf_bytes((environment.root / generated).read_bytes(),
                                          base_link=environment.base_frame, joint_names=environment.joint_names)
    tool = environment.setup_config.tool
    transform = tool.parent_to_tip
    tool_pose = Pose(position_m=transform.translation_m, quaternion_xyzw=transform.quaternion_xyzw)
    cache = {}
    max_ref = max_ref_velocity = max_q = max_qd = max_pos = max_angle = 0.0
    for row in selected:
        names = json.loads(row["joint_names"])
        if len(set(names)) != len(names) or set(names) != set(environment.joint_names):
            raise ValueError("Controller witness joint names mismatch")
        order = [names.index(name) for name in environment.joint_names]
        vectors = [[json.loads(row[field])[index] for index in order]
                   for field in ("reference_q", "reference_qd", "actual_q", "actual_qd")]
        reference, reference_velocity, actual, velocity = vectors
        if any(not math.isfinite(value) for vector in vectors for value in vector):
            raise ValueError("Controller witness has nonfinite joint state")
        t = float(row["reference_time_s"])
        index = min(max(bisect_right(qtimes, t)-1, 0), len(qtimes)-2)
        before, after = trajectory.samples[index:index+2]
        if index not in cache:
            cache[index] = [QuinticHermiteSegment.from_boundary_conditions(
                duration_s=after.time_s-before.time_s,
                start_position=before.positions[j], start_velocity=before.velocities[j],
                start_acceleration=before.accelerations[j], end_position=after.positions[j],
                end_velocity=after.velocities[j], end_acceleration=after.accelerations[j])
                for j in range(len(order))]
        elapsed = t-before.time_s
        expected_q = [segment.evaluate(elapsed) for segment in cache[index]]
        expected_qd = [segment.evaluate(elapsed, 1) for segment in cache[index]]
        max_ref = max(max_ref, max(abs(a-b) for a,b in zip(reference, expected_q)))
        max_ref_velocity = max(max_ref_velocity, max(abs(a-b) for a,b in zip(reference_velocity, expected_qd)))
        max_q = max(max_q, max(abs(a-b) for a,b in zip(actual, reference)))
        max_qd = max(max_qd, max(abs(a-b) for a,b in zip(velocity, reference_velocity)))
        tip = model.forward_kinematics(actual, (tool.tip_frame,), tool_to_tip=tool_pose,
                                       tool_frame=tool.parent_link, tip_frame=tool.tip_frame)[tool.tip_frame]
        si = min(max(bisect_right(source_times, t)-1, 0), len(source_times)-1)
        target = targets[si]
        if si+1 < len(source_times) and t > source_times[si]:
            target = RecordingInspector._interpolate_pose(target, targets[si+1],
                (t-source_times[si])/(source_times[si+1]-source_times[si]))
        error = pose_error(tip, target)
        max_pos = max(max_pos, error.position_m)
        max_angle = max(max_angle, error.orientation_deg)
    limits = config["validation"]
    planner = environment.planner["validation"]
    checks = (("controller reference position", max_ref, limits["max_controller_reference_position_error_rad"]),
              ("controller reference velocity", max_ref_velocity, limits["max_controller_reference_velocity_error_rad_s"]),
              ("actual/reference joint position", max_q, limits["max_actual_controller_position_error_rad"]),
              ("actual/reference joint velocity", max_qd, limits["max_actual_controller_velocity_error_rad_s"]),
              ("coherent actual/source position", max_pos, planner["tip_position_tolerance_m"]),
              ("coherent actual/source orientation", max_angle, planner["tip_orientation_tolerance_deg"]))
    for label, value, limit in checks:
        if value > float(limit)+1e-9:
            raise ValueError(f"Independent {label} error {value} exceeds original environment limit {limit}")
    return {"schema_version": 1, "status": "passed", **metrics,
            "max_controller_reference_position_error_rad": max_ref,
            "max_controller_reference_velocity_error_rad_s": max_ref_velocity,
            "max_actual_reference_position_error_rad": max_q,
            "max_actual_reference_velocity_error_rad_s": max_qd,
            "max_coherent_actual_source_position_error_m": max_pos,
            "max_coherent_actual_source_orientation_error_deg": max_angle,
            "raw_speed_scaling_factor_values": sorted({float(row["raw_speed_scaling_factor"]) for row in selected}),
            "speed_scaling_semantics": "locked standard unscaled JointTrajectoryController; raw default0 is not an effective speed scale",
            "source_timing_changed": False, "resampling_or_synthetic_rows": False}
