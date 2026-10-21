"""Independent playback telemetry audit against immutable source and joints.

The canonical 100 Hz recording streams are audited by RecordingInspector. The
playback CSV is asynchronous JointState telemetry: repeated trajectory times are
valid and are reported, never deleted from the artifact.
"""
from bisect import bisect_right
import csv
import math
from pathlib import Path
import statistics


def audit_timing(rows, duration_s, dt, jitter, completion_overrun):
    times = [float(row["trajectory_time_s"]) for row in rows]
    sync = [int(row["sync_time_ns"]) for row in rows]
    if len(rows) < 2 or any(not math.isfinite(t) for t in times):
        raise ValueError("Independent playback has insufficient finite telemetry rows")
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("Independent playback cached trajectory times regress")
    if any(b <= a for a, b in zip(sync, sync[1:])):
        raise ValueError("Independent playback monotonic timestamps are not strictly increasing")
    tolerance = dt + jitter
    if times[0] > tolerance or times[0] < 0 or times[-1] > duration_s + 1e-9:
        raise ValueError("Independent cached feedback times extend outside authoritative execution")
    # Runtime's trajectory_time field is the LAST ACTION FEEDBACK, typically
    # 20 Hz. It is intentionally neither advanced nor overwritten here. Its
    # last value may precede completion by a feedback interval even when the
    # controller and the 125 Hz actual JointState CSV cover the complete path.
    elapsed = (sync[-1]-sync[0])*1e-9
    if elapsed < duration_s - tolerance or elapsed > duration_s + completion_overrun + tolerance:
        raise ValueError(f"Independent host duration {elapsed}s does not cover authoritative {duration_s}s")
    advancing = [b-a for a, b in zip(times, times[1:]) if b > a]
    if not advancing:
        raise ValueError("Independent playback action feedback never advanced")
    return times, sync, advancing, elapsed


def audit_terminal_hold(rows, source, final_joints, joint_names, joint_tolerance, velocity_tolerance):
    last = source[-1]
    hold_start = last.time_s
    for pose in reversed(source[:-1]):
        if (math.dist(pose.position_m, last.position_m) > 1e-12 or
                min(math.dist(pose.quaternion_xyzw, last.quaternion_xyzw),
                    math.dist(pose.quaternion_xyzw, tuple(-x for x in last.quaternion_xyzw))) > 1e-12):
            break
        hold_start = pose.time_s
    if hold_start >= last.time_s or float(rows[-1]["trajectory_time_s"]) < hold_start:
        raise ValueError("Independent playback lacks authoritative terminal hold evidence")
    span = min(0.2, last.time_s-hold_start)
    cutoff = int(rows[-1]["sync_time_ns"]) - round(span*1e9)
    tail = [row for row in rows if int(row["sync_time_ns"]) >= cutoff]
    if len(tail) < 2:
        raise ValueError("Independent terminal hold has insufficient actual-state samples")
    max_joint = max(abs(float(row[f"{name}_pos"])-target)
                    for row in tail for name, target in zip(joint_names, final_joints))
    max_velocity = max(abs(float(row[f"{name}_vel"])) for row in tail for name in joint_names)
    if max_joint > joint_tolerance or max_velocity > velocity_tolerance:
        raise ValueError(f"Independent terminal hold not settled at sealed goal: q={max_joint}, qdot={max_velocity}")
    return {"source_terminal_hold_start_s": hold_start, "observed_terminal_hold_s": span,
            "terminal_hold_samples": len(tail), "terminal_max_joint_error_rad": max_joint,
            "terminal_max_joint_velocity_rad_s": max_velocity}


def audit_playback(telemetry_path, recording_path, environment_path):
    from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
    from hx_waypoint_curobo.recording_store import RecordingInspector
    from hx_waypoint_curobo.robot_kinematics import Pose, RobotKinematics, pose_error
    from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv, read_joint_trajectory_csv

    environment = EnvironmentBuilder().load(environment_path)
    recording = Path(recording_path)
    # Reuse the existing immutable-artifact/canonical-time/FK/strict100Hz audit.
    RecordingInspector().inspect(recording)
    source = read_timed_pose_csv(recording / "source_tip_trajectory.csv")
    source_times = [pose.time_s for pose in source]
    source_base = [environment.transform_pose_to_base(pose) for pose in source]
    source_poses = [Pose(position_m=pose.position_m, quaternion_xyzw=pose.quaternion_xyzw)
                    for pose in source_base]
    generated = environment.asset_provenance["generated_urdf"]["path"]
    model = RobotKinematics.from_urdf_bytes((environment.root / generated).read_bytes(),
                                          base_link=environment.base_frame,
                                          joint_names=environment.joint_names)
    configured_tool = environment.setup_config.tool
    transform = configured_tool.parent_to_tip
    tool_pose = Pose(position_m=transform.translation_m, quaternion_xyzw=transform.quaternion_xyzw)
    def fk(positions):
        return model.forward_kinematics(positions, (configured_tool.tip_frame,),
                                       tool_to_tip=tool_pose, tool_frame=configured_tool.parent_link,
                                       tip_frame=configured_tool.tip_frame)[configured_tool.tip_frame]
    with Path(telemetry_path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError("Independent playback has fewer than two telemetry rows")
    config = environment.recording["recording"] if hasattr(environment, "recording") else None
    if config is None:
        import yaml
        config = yaml.safe_load((environment.root / "recording.yaml").read_text())["recording"]
    dt = float(config["sample_dt_s"])
    limits = config["validation"]
    times, sync, advancing, elapsed = audit_timing(
        rows, source_times[-1], dt, float(limits["max_time_jitter_s"]),
        float(limits["max_completion_overrun_s"]))
    canonical = read_joint_trajectory_csv(recording / "joint_trajectory.csv", environment.joint_names)
    terminal = audit_terminal_hold(rows, source, canonical.samples[-1].positions,
                                   environment.joint_names,
                                   float(limits["max_joint_position_tracking_error_rad"]),
                                   float(environment.recording["start_state"]["settle_velocity_rad_s"]))
    max_fk = max_tracking = max_source_position = max_source_orientation = 0.0
    max_desired_source_position = max_desired_source_orientation = 0.0
    max_reported_position_delta = 0.0
    for row, t in zip(rows, times):
        actual = [float(row[f"{name}_pos"]) for name in environment.joint_names]
        desired = [float(row[f"desired_{name}_pos"]) for name in environment.joint_names]
        if any(not math.isfinite(x) for x in actual + desired):
            raise ValueError("Independent playback has nonfinite joint positions")
        scale = float(row["speed_scaling"])
        if not math.isfinite(scale) or abs(scale - 1.0) > 1e-9:
            raise ValueError(f"Independent playback speed scaling changed: {scale}")
        actual_tip = fk(actual)
        desired_tip = fk(desired)
        xyz = [float(row[f"tip_{axis}"]) for axis in "xyz"]
        recorded_fk_error = math.dist(actual_tip.position_m, xyz)
        max_fk = max(max_fk, recorded_fk_error)
        max_tracking = max(max_tracking, max(abs(a-b) for a, b in zip(actual, desired)))
        recomputed_error = math.dist(actual_tip.position_m, desired_tip.position_m)
        max_reported_position_delta = max(max_reported_position_delta,
                                          abs(recomputed_error - float(row["position_error_m"])))
        index = min(max(bisect_right(source_times, t)-1, 0), len(source)-1)
        expected = source_poses[index]
        if index + 1 < len(source) and t > source_times[index]:
            ratio = (t-source_times[index]) / (source_times[index+1]-source_times[index])
            expected = RecordingInspector._interpolate_pose(expected, source_poses[index+1], ratio)
        error = pose_error(actual_tip, expected)
        max_source_position = max(max_source_position, error.position_m)
        max_source_orientation = max(max_source_orientation, error.orientation_deg)
        desired_error = pose_error(desired_tip, expected)
        max_desired_source_position = max(max_desired_source_position, desired_error.position_m)
        max_desired_source_orientation = max(max_desired_source_orientation, desired_error.orientation_deg)
    planner = environment.planner["validation"]
    checks = (("FK consistency", max_fk, float(limits["max_frame_fk_position_error_m"])),
              ("reported error consistency", max_reported_position_delta, 1e-6),
              ("desired source position", max_desired_source_position, float(planner["tip_position_tolerance_m"])),
              ("desired source orientation", max_desired_source_orientation, float(planner["tip_orientation_tolerance_deg"])))
    for label, value, limit in checks:
        if not math.isfinite(value) or value > limit + 1e-9:
            raise ValueError(f"Independent playback {label} error {value} exceeds {limit}")
    host_intervals = [(b-a)*1e-9 for a, b in zip(sync, sync[1:])]
    median_host = statistics.median(host_intervals)
    return {"schema_version": 2, "status": "passed", "telemetry_rows": len(rows), **terminal,
            "first_trajectory_time_s": times[0], "last_trajectory_time_s": times[-1],
            "authoritative_duration_s": source_times[-1], "source_waypoint_count": len(source),
            "observed_host_duration_s": elapsed,
            "action_success_required_by_caller": True,
            "canonical_sample_dt_s": dt, "canonical_recording_inspector_passed": True,
            "trajectory_feedback_max_gap_s": max(advancing),
            "trajectory_feedback_repeated_timestamps": sum(b == a for a, b in zip(times, times[1:])),
            "delivery_median_interval_s": median_host,
            "delivery_max_interval_s": max(host_intervals),
            "delivery_max_jitter_from_median_s": max(abs(x-median_host) for x in host_intervals),
            "delivery_note": "JointState arrival intervals; trajectory_time_s is cached action feedback, not per-row controller time; canonical strict100Hz streams are separately sealed and inspected",
            "max_fk_consistency_error_m": max_fk,
            "diagnostic_actual_vs_cached_desired_joint_delta_rad": max_tracking,
            "max_reported_position_error_delta_m": max_reported_position_delta,
            "max_desired_source_position_error_m": max_desired_source_position,
            "max_desired_source_orientation_error_deg": max_desired_source_orientation,
            "diagnostic_actual_vs_cached_source_position_m": max_source_position,
            "diagnostic_actual_vs_cached_source_orientation_deg": max_source_orientation,
            "actual_vs_cached_source_note": "Diagnostic only: actual JointState and cached action progress/desired joints have different sampling instants; same-time actual/reference tracking and actual/source FK are gated by the mandatory coherent controller witness",
            "speed_scaling": 1.0}
