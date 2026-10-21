from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.errors import WaypointError
from hx_waypoint_curobo.recording_store import RecordingInspector
from hx_waypoint_curobo.timed_pose_compiler import CompilationResult, CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_io import (
    read_joint_trajectory_csv,
    read_timed_pose_csv,
    sha256_file,
    write_joint_trajectory_csv,
)

from hx_waypoint_tools.ur_setup_tools import (
    DEFAULT_DASHBOARD_PORT,
    DEFAULT_DASHBOARD_TIMEOUT_S,
    DEFAULT_ROBOT_IP,
    extract_teach_pendant_pose_from_dashboard_response,
    parse_teach_pendant_pose,
    plane_transform_from_pose,
    query_dashboard_variable,
    static_transform_publisher_command,
)

_COMPILE_REPORT_SCHEMA_VERSION = 2
_COMPILER_CONTRACT_REVISION = "fixed_time_waypoint_v2"
_SAFE_TRAJECTORY_PREFIX = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _parse_nucleo_channels(value: str) -> tuple[int, ...]:
    try:
        channels = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "NUCLEO channels must be comma-separated integers"
        ) from error
    if not channels or len(set(channels)) != len(channels):
        raise argparse.ArgumentTypeError("NUCLEO channels must be non-empty and unique")
    if any(channel < 1 or channel > 4 for channel in channels):
        raise argparse.ArgumentTypeError("NUCLEO channels must be selected from 1,2,3,4")
    return tuple(sorted(channels))


def _parse_nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("value must be an integer") from error
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed


def _arm_nucleo_capture(capture_session: Any) -> None:
    """Start the optional sidecar through the normal tipdraw error path."""

    try:
        capture_session.start()
    except (OSError, RuntimeError, ValueError) as error:
        raise WaypointError(f"NUCLEO preflight failed: {error}") from error


def _arm_tracker_capture(capture_session: Any) -> None:
    """Require one valid plane1 PoseStamped before any robot goal is sent."""

    try:
        capture_session.start()
    except (OSError, RuntimeError, ValueError) as error:
        raise WaypointError(f"tracker preflight failed: {error}") from error


def _nucleo_robot_telemetry_path(
    capture_summary: Any | None,
    playback_result: Any,
) -> Path | None:
    """Return a finalization target without masking an authoritative robot failure."""

    if capture_summary is None:
        return None
    telemetry_path = str(getattr(playback_result, "telemetry_path", "") or "")
    if telemetry_path:
        return Path(telemetry_path)
    if bool(getattr(playback_result, "success", False)):
        raise WaypointError(
            "NUCLEO capture completed but playback returned no robot telemetry path"
        )
    return None


def _nucleo_quality_flags_within_limits(
    sensor_result: Any,
    *,
    max_incomplete_groups: int,
    max_channel_sequence_drops: int,
) -> bool:
    """Accept only explicitly bounded parser losses, never other sync failures."""

    if str(getattr(sensor_result, "status", "")) != "incomplete":
        return False
    try:
        manifest = yaml.safe_load(
            Path(sensor_result.manifest_path).read_text(encoding="utf-8")
        )
        if not isinstance(manifest, dict) or manifest.get("status") != "incomplete":
            return False
        capture = manifest.get("capture")
        mapping = manifest.get("mapping")
        if not isinstance(capture, dict) or capture.get("capture_error") is not None:
            return False
        if not isinstance(mapping, dict):
            return False
        deltas = mapping.get("parser_integrity_deltas")
        if not isinstance(deltas, dict):
            return False
        parsed_deltas = {str(name): int(value) for name, value in deltas.items()}
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return False

    incomplete_groups = parsed_deltas.get("parser_incomplete_groups", 0)
    channel_sequence_drops = parsed_deltas.get(
        "parser_channel_sequence_drops", 0
    )
    allowed_names = {
        "parser_incomplete_groups",
        "parser_channel_sequence_drops",
    }
    if any(
        value != 0 for name, value in parsed_deltas.items() if name not in allowed_names
    ):
        return False
    if incomplete_groups > max_incomplete_groups:
        return False
    if channel_sequence_drops > max_channel_sequence_drops:
        return False
    if incomplete_groups == 0 and channel_sequence_drops == 0:
        return False

    expected_messages: list[str] = []
    if incomplete_groups:
        expected_messages.append(
            f"parser observed {incomplete_groups} incomplete groups during playback"
        )
    if channel_sequence_drops:
        expected_messages.append(
            "parser observed "
            f"{channel_sequence_drops} channel sequence drops during playback"
        )
    return str(getattr(sensor_result, "message", "")) == "; ".join(
        expected_messages
    )


def _tracker_robot_telemetry_path(
    capture_summary: Any | None,
    playback_result: Any,
) -> Path | None:
    """Return the actual-FK table used to synchronize tracker samples."""

    if capture_summary is None:
        return None
    if not bool(getattr(playback_result, "success", False)):
        return None
    telemetry_path = str(getattr(playback_result, "telemetry_path", "") or "")
    if telemetry_path:
        return Path(telemetry_path)
    raise WaypointError(
        "tracker capture completed but playback returned no robot telemetry path"
    )


def _workspace(path: str | None) -> Path:
    root = Path(path or Path.cwd()).expanduser().resolve()
    if not (root / "environments").is_dir() or not (root / "curobo.lock").is_file():
        raise WaypointError(f"not an ur-waypoint workspace: {root}")
    return root


def _environment(root: Path, environment_id: str) -> Path:
    path = (root / "environments" / environment_id).resolve()
    if root not in path.parents:
        raise WaypointError("environment path escapes the workspace")
    return path


def _compile_report_payload(
    environment: EnvironmentBundle,
    trajectory_id: str,
    result: CompilationResult,
    source_path: Path,
    joint_trajectory_path: Path,
    *,
    canonical_audit: Any | None = None,
) -> dict[str, object]:
    """Return the immutable evidence consumed by the C++ BuildRecording gate."""
    if not math.isclose(
        result.metrics.uniform_time_scale,
        1.0,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise WaypointError(
            "compile result changed authoritative trajectory timing; "
            "uniform_time_scale must be exactly 1.0"
        )
    source_times = tuple(float(pose.time_s) for pose in result.source_poses)
    compiled_times = tuple(float(sample.time_s) for sample in result.trajectory.samples)
    if len(source_times) < 2 or not compiled_times:
        raise WaypointError("compile result has incomplete authoritative timing evidence")
    matched = 0
    compiled_index = 0
    for source_time in source_times:
        while (
            compiled_index < len(compiled_times)
            and compiled_times[compiled_index] < source_time
        ):
            compiled_index += 1
        if (
            compiled_index >= len(compiled_times)
            or compiled_times[compiled_index] != source_time
        ):
            raise WaypointError(
                "compiled trajectory does not contain every authoritative source time exactly"
            )
        matched += 1
        compiled_index += 1
    if compiled_times[0] != 0.0 or compiled_times[-1] != source_times[-1]:
        raise WaypointError(
            "compiled trajectory duration differs from authoritative source duration"
        )
    metadata = getattr(result.trajectory, "metadata", None)
    winding = metadata.get("joint_winding") if isinstance(metadata, dict) else None
    if not isinstance(winding, dict) or winding.get("policy") != (
        "constant_whole_path_nearest_home_without_margin_loss"
    ):
        raise WaypointError("compile result is missing audited joint-winding evidence")
    winding_payload: dict[str, object] = {"policy": winding["policy"]}
    for field in (
        "reference_positions_rad",
        "selected_seed_offsets_rad",
        "post_optimization_offsets_rad",
    ):
        values = winding.get(field)
        if (
            not isinstance(values, list)
            or len(values) != len(environment.joint_names)
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in values
            )
        ):
            raise WaypointError("compile result contains invalid joint-winding evidence")
        winding_payload[field] = [float(value) for value in values]
    return {
        "schema_version": _COMPILE_REPORT_SCHEMA_VERSION,
        "compiler_contract_revision": _COMPILER_CONTRACT_REVISION,
        "environment_id": environment.environment_id,
        "trajectory_id": trajectory_id,
        "curobo_revision": result.curobo_revision,
        "source_trajectory_sha256": sha256_file(source_path),
        "joint_trajectory_sha256": sha256_file(joint_trajectory_path),
        "environment_hashes": dict(environment.hashes),
        "duration_s": result.trajectory.duration_s,
        "point_count": len(result.trajectory.samples),
        "source_waypoint_count": len(source_times),
        "matched_source_waypoint_count": matched,
        "source_duration_s": source_times[-1],
        "max_source_time_error_s": 0.0,
        "uniform_time_scale": result.metrics.uniform_time_scale,
        "max_joint_velocity_ratio": (
            canonical_audit.max_joint_velocity_ratio
            if canonical_audit is not None
            else result.metrics.max_joint_velocity_ratio
        ),
        "max_joint_acceleration_ratio": (
            canonical_audit.max_joint_acceleration_ratio
            if canonical_audit is not None
            else result.metrics.max_joint_acceleration_ratio
        ),
        "max_joint_jerk_ratio": (
            canonical_audit.max_joint_jerk_ratio
            if canonical_audit is not None
            else result.metrics.max_joint_jerk_ratio
        ),
        "max_tip_position_error_m": (
            canonical_audit.max_tip_position_error_m
            if canonical_audit is not None
            else result.validation_metrics.max_tip_position_error_m
        ),
        "max_tip_orientation_error_deg": (
            canonical_audit.max_tip_orientation_error_deg
            if canonical_audit is not None
            else result.validation_metrics.max_tip_orientation_error_deg
        ),
        "minimum_collision_distance_m": (
            result.validation_metrics.minimum_collision_distance_m
        ),
        "max_scene_collision_cost": result.validation_metrics.max_scene_collision_cost,
        "max_self_collision_cost": result.validation_metrics.max_self_collision_cost,
        "max_joint_bound_cost": result.validation_metrics.max_joint_bound_cost,
        "checked_sample_count": result.validation_metrics.checked_sample_count,
        "joint_winding": winding_payload,
    }


def _build_recording(args: argparse.Namespace) -> int:
    if not math.isfinite(args.launch_timeout) or args.launch_timeout <= 0.0:
        raise WaypointError("--launch-timeout must be finite and greater than zero")
    root = _workspace(args.workspace)
    environment_path = _environment(root, args.env)
    environment = EnvironmentBuilder().load(environment_path)
    source_path = environment_path / "trajectories" / f"{args.trajectory}.csv"
    source = read_timed_pose_csv(source_path)
    adapter = CuroboAdapterV2(environment, root / "curobo.lock")
    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_path = root / "runs" / f"{args.trajectory}__compiled__{stamp}"
    run_path.mkdir(parents=True)
    compiled_path = run_path / "joint_trajectory.csv"
    source_copy = run_path / "source_tip_trajectory.csv"
    write_joint_trajectory_csv(compiled_path, result.trajectory)
    shutil.copy2(source_path, source_copy)
    canonical_audit = RecordingInspector.audit_compiled_trajectory(
        environment,
        read_joint_trajectory_csv(compiled_path, environment.joint_names),
        read_timed_pose_csv(source_copy),
    )
    (run_path / "compile_report.yaml").write_text(
        yaml.safe_dump(
            _compile_report_payload(
                environment,
                args.trajectory,
                result,
                source_copy,
                compiled_path,
                canonical_audit=canonical_audit,
            ),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    if args.compile_only:
        print(run_path)
        return 0
    return _send_build_action(
        environment,
        args.trajectory,
        source_copy,
        compiled_path,
        args.rviz,
        workspace=root,
        auto_launch=not args.external_runtime,
        launch_timeout_s=args.launch_timeout,
    )


def _simulation_launch_command(
    environment: EnvironmentBundle,
    workspace: Path,
    launch_rviz: bool,
) -> list[str]:
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise WaypointError("ros2 executable is unavailable; source the ROS 2 Jazzy environment")
    return [
        ros2,
        "launch",
        "hx_waypoint_bringup",
        "simulation.launch.py",
        f"ur_type:={environment.robot_type}",
        f"environment_id:={environment.environment_id}",
        f"workspace:={workspace}",
        f"launch_rviz:={'true' if launch_rviz else 'false'}",
    ]


def _stop_managed_launch(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def _wait_for_build_server(client, process: subprocess.Popen[bytes], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise WaypointError(
                f"managed simulation launch exited before BuildRecording became ready "
                f"(exit={return_code})"
            )
        if client.wait_for_server(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
            return
    raise WaypointError(
        f"managed simulation did not expose /hx_waypoint/build_recording within {timeout_s:.1f} s"
    )


def _send_build_action(
    environment,
    trajectory_id,
    source_path,
    compiled_path,
    rviz,
    *,
    workspace: Path,
    auto_launch: bool,
    launch_timeout_s: float,
) -> int:
    try:
        import rclpy
        from rclpy.action import ActionClient

        from hx_waypoint_interfaces.action import BuildRecording
    except ImportError as error:
        raise WaypointError("ROS 2 Jazzy environment is required to run the simulation") from error

    rclpy.init()
    node = rclpy.create_node("tipdraw_build_client")
    client = ActionClient(node, BuildRecording, "/hx_waypoint/build_recording")
    managed_launch: subprocess.Popen[bytes] | None = None
    try:
        # With ``--external-runtime`` the simulation launch is already up, but
        # DDS discovery can briefly lag while the previous build client exits
        # (especially on a long batch of GPU compiles).  A two-second probe
        # made otherwise valid compiled trajectories fail spuriously.  Keep
        # the short probe for managed launches, while giving an external
        # runtime the same bounded launch timeout to expose its action server.
        initial_wait_s = launch_timeout_s if not auto_launch else 2.0
        if not client.wait_for_server(timeout_sec=initial_wait_s):
            if not auto_launch:
                raise WaypointError(
                    "BuildRecording action server is unavailable and "
                    "--external-runtime was selected"
                )
            command = _simulation_launch_command(environment, workspace, bool(rviz))
            print("starting managed simulation: " + " ".join(command), file=sys.stderr)
            managed_launch = subprocess.Popen(command, start_new_session=True)
            _wait_for_build_server(client, managed_launch, launch_timeout_s)
        goal = BuildRecording.Goal()
        goal.environment_id = environment.environment_id
        goal.environment_path = str(environment.root)
        goal.trajectory_id = trajectory_id
        goal.source_trajectory_path = str(source_path)
        goal.compiled_trajectory_path = str(compiled_path)
        goal.launch_rviz = bool(rviz)
        handle_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, handle_future)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            raise WaypointError("BuildRecording goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        if not result.success:
            raise WaypointError(result.message)
        print(result.recording_path)
        return 0
    finally:
        _stop_managed_launch(managed_launch)
        node.destroy_node()
        rclpy.shutdown()


def _inspect(args: argparse.Namespace) -> int:
    root = _workspace(args.workspace)
    recording_path = _environment(root, args.env) / "recordings" / args.recording
    inspection = RecordingInspector().inspect(recording_path)
    print(yaml.safe_dump(dict(inspection.manifest), sort_keys=False).rstrip())
    return 0


def _play(args: argparse.Namespace) -> int:
    root = _workspace(args.workspace)
    environment_path = _environment(root, args.env)
    recording_path = environment_path / "recordings" / args.recording
    RecordingInspector().inspect(recording_path)
    try:
        import rclpy
        from rclpy.action import ActionClient

        from hx_waypoint_interfaces.action import PlayRecording
    except ImportError as error:
        raise WaypointError("ROS 2 Jazzy environment is required to play a recording") from error
    rclpy.init()
    node = rclpy.create_node("tipdraw_play_client")
    client = ActionClient(node, PlayRecording, "/hx_waypoint/play_recording")
    capture_session = None
    capture_summary = None
    capture_stop_error: Exception | None = None
    tracker_session = None
    tracker_summary = None
    tracker_stop_error: Exception | None = None
    result = None
    try:
        if not client.wait_for_server(timeout_sec=20.0):
            raise WaypointError("PlayRecording action server is unavailable")
        if args.record_nucleo:
            if not args.nucleo_port:
                raise WaypointError(
                    "--record-nucleo requires --nucleo-port or "
                    "HX_WAYPOINT_NUCLEO_PORT"
                )
            try:
                from hx_waypoint_nucleo import NucleoCaptureConfig, NucleoCaptureSession
            except ImportError as error:
                raise WaypointError(
                    "NUCLEO capture package is unavailable; rebuild the ROS workspace"
                ) from error
            capture_session = NucleoCaptureSession(
                root,
                NucleoCaptureConfig(
                    port=args.nucleo_port,
                    baudrate=args.nucleo_baudrate,
                    channels=args.nucleo_channels,
                    perform_sync=not args.nucleo_reuse_sync,
                    preflight_timeout_s=args.nucleo_ready_timeout,
                ),
                lambda: int(node.get_clock().now().nanoseconds),
            )
            _arm_nucleo_capture(capture_session)
            print(
                f"NUCLEO preflight ready on {args.nucleo_port}; "
                "capture is armed before controller submission; "
                + (
                    "SYNC performed"
                    if not args.nucleo_reuse_sync
                    else "existing NUCLEO SYNC reused without another handshake"
                ),
                file=sys.stderr,
            )
        if args.record_tracker:
            if not args.tracker_topic:
                raise WaypointError(
                    "--record-tracker requires --tracker-topic or "
                    "HX_WAYPOINT_TRACKER_TOPIC"
                )
            try:
                from hx_waypoint_tools.tracker_capture import (
                    TrackerCaptureConfig,
                    TrackerCaptureSession,
                )
            except ImportError as error:
                raise WaypointError(
                    "tracker capture support is unavailable; rebuild the ROS workspace"
                ) from error
            tracker_session = TrackerCaptureSession(
                root,
                node,
                TrackerCaptureConfig(
                    topic=args.tracker_topic,
                    frame_id=args.tracker_frame,
                    preflight_timeout_s=args.tracker_ready_timeout,
                    max_mapping_offset_s=args.tracker_max_mapping_offset_ms / 1000.0,
                    minimum_mapped_samples=args.tracker_minimum_samples,
                ),
            )
            _arm_tracker_capture(tracker_session)
            print(
                f"tracker capture armed on {args.tracker_topic}; "
                f"every received PoseStamped must use {args.tracker_frame}; "
                "the warmup stage may initialize tracking before valid poses arrive",
                file=sys.stderr,
            )
        goal = PlayRecording.Goal()
        goal.environment_id = args.env
        goal.environment_path = str(environment_path)
        goal.recording_id = args.recording
        goal.auto_prepare = bool(args.auto_prepare)
        goal.externally_prepared_start = bool(args.externally_prepared_start)
        handle_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, handle_future)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            raise WaypointError("PlayRecording goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
    finally:
        if capture_session is not None:
            try:
                capture_summary = capture_session.stop()
            except Exception as error:  # Preserve robot execution/result handling.
                capture_stop_error = error
        if tracker_session is not None:
            try:
                tracker_summary = tracker_session.stop()
            except Exception as error:  # Preserve robot execution/result handling.
                tracker_stop_error = error
        node.destroy_node()
        rclpy.shutdown()

    if result is None:
        raise WaypointError("PlayRecording action returned no result")

    sensor_result = None
    sensor_finalize_error: Exception | None = None
    robot_telemetry_path = _nucleo_robot_telemetry_path(capture_summary, result)
    if robot_telemetry_path is not None:
        try:
            from hx_waypoint_nucleo import finalize_nucleo_capture

            sensor_result = finalize_nucleo_capture(
                capture_summary,
                robot_telemetry_path,
            )
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            sensor_finalize_error = error
        else:
            print(
                f"NUCLEO telemetry: {sensor_result.sensor_telemetry_path}",
                file=sys.stderr,
            )
            print(f"NUCLEO manifest: {sensor_result.manifest_path}", file=sys.stderr)
    elif capture_summary is not None:
        print(
            f"NUCLEO raw staging retained: {capture_summary.staging_path}",
            file=sys.stderr,
        )

    tracker_result = None
    tracker_finalize_error: Exception | None = None
    tracker_robot_telemetry_path = _tracker_robot_telemetry_path(
        tracker_summary, result
    )
    if tracker_robot_telemetry_path is not None:
        try:
            from hx_waypoint_tools.yaw_calibration import (
                finalize_yaw_tracker_capture,
            )

            tracker_result = finalize_yaw_tracker_capture(
                tracker_summary,
                tracker_robot_telemetry_path,
                recording_path,
            )
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            tracker_finalize_error = error
        else:
            print(
                f"tracker telemetry: {tracker_result.tracker_telemetry_path}",
                file=sys.stderr,
            )
            print(
                f"yaw calibration report: {tracker_result.report_path}",
                file=sys.stderr,
            )
    elif tracker_summary is not None:
        print(
            f"tracker raw staging retained: {tracker_summary.staging_path}",
            file=sys.stderr,
        )

    if not result.success:
        if sensor_finalize_error is not None:
            print(
                "NUCLEO finalization also failed after robot playback failure: "
                + str(sensor_finalize_error),
                file=sys.stderr,
            )
        if tracker_finalize_error is not None:
            print(
                "tracker finalization also failed after robot playback failure: "
                + str(tracker_finalize_error),
                file=sys.stderr,
            )
        raise WaypointError(result.message)
    if sensor_finalize_error is not None:
        raise WaypointError(
            "robot playback finished, but NUCLEO synchronization failed: "
            + str(sensor_finalize_error)
        ) from sensor_finalize_error
    if capture_stop_error is not None:
        raise WaypointError(
            "robot playback succeeded, but NUCLEO capture could not stop cleanly: "
            + str(capture_stop_error)
        )
    if tracker_finalize_error is not None:
        raise WaypointError(
            "robot playback finished, but tracker synchronization/fitting failed: "
            + str(tracker_finalize_error)
        ) from tracker_finalize_error
    if tracker_stop_error is not None:
        raise WaypointError(
            "robot playback succeeded, but tracker capture could not stop cleanly: "
            + str(tracker_stop_error)
        )
    if sensor_result is not None and sensor_result.status != "complete":
        if _nucleo_quality_flags_within_limits(
            sensor_result,
            max_incomplete_groups=args.nucleo_max_incomplete_groups,
            max_channel_sequence_drops=args.nucleo_max_channel_sequence_drops,
        ):
            print(
                "NUCLEO quality flags accepted by explicit batch limits; "
                f"manifest remains incomplete: {sensor_result.message}",
                file=sys.stderr,
            )
        else:
            raise WaypointError(
                "robot playback succeeded, but NUCLEO capture is incomplete: "
                + sensor_result.message
            )
    print(result.telemetry_path)
    return 0


def _generate_yaw_calibration(args: argparse.Namespace) -> int:
    root = _workspace(args.workspace)
    environment_path = _environment(root, args.env)
    if not _SAFE_TRAJECTORY_PREFIX.fullmatch(args.trajectory_prefix):
        raise WaypointError(
            "--trajectory-prefix must be one path-safe identifier"
        )
    reference_path = (
        environment_path / "trajectories" / f"{args.reference_trajectory}.csv"
    )
    reference = read_timed_pose_csv(reference_path)
    from hx_waypoint_tools.yaw_calibration import (
        build_yaw_calibration_trajectory,
        resolve_yaw_calibration_target,
        write_authoritative_trajectory,
    )

    target = resolve_yaw_calibration_target(
        environment_path,
        sensor_distance_m=args.sensor_distance_mm / 1000.0,
        offset_xy_m=(args.offset_x_mm / 1000.0, args.offset_y_mm / 1000.0),
        sensor_plane_offset_z_m=args.sensor_plane_offset_z_mm / 1000.0,
    )
    output_directory = environment_path / "trajectories"
    sweep_label = int(args.sweep_deg)
    generated = write_authoritative_trajectory(
        output_directory / f"{args.trajectory_prefix}_yaw_{sweep_label}deg.csv",
        build_yaw_calibration_trajectory(
            position_m=target.position_m,
            base_quaternion_xyzw=reference[0].quaternion_xyzw,
            frame_id=target.frame_id,
            sweep_deg=args.sweep_deg,
            leg_duration_s=args.leg_duration_s,
            sample_period_s=args.sample_period_s,
            hold_duration_s=args.hold_duration_s,
            warmup_duration_s=args.warmup_duration_s,
            warmup_amplitude_m=args.warmup_amplitude_mm / 1000.0,
            warmup_settle_duration_s=args.warmup_settle_s,
        ),
    )
    print(
        yaml.safe_dump(
            {
                "frame_id": target.frame_id,
                "tip_position_m": list(target.position_m),
                "sensor_plane_local_z_m": target.sensor_plane_local_z_m,
                "sensor_distance_m": target.sensor_distance_m,
                "stages": ["warmup_wave", "center_settle", "yaw_out_and_back", "final_hold"],
                "generated": str(generated),
            },
            sort_keys=False,
        ).rstrip()
    )
    return 0


def _yaw_calibrate(args: argparse.Namespace) -> int:
    """Capture and fit one combined warmup-plus-yaw authority."""

    common = vars(args).copy()
    common.update(
        {
            "auto_prepare": True,
            "externally_prepared_start": False,
            "record_nucleo": False,
            "nucleo_port": None,
            "nucleo_baudrate": 115200,
            "nucleo_channels": (1, 2, 3, 4),
            "nucleo_ready_timeout": 5.0,
            "nucleo_reuse_sync": False,
            "nucleo_max_incomplete_groups": 0,
            "nucleo_max_channel_sequence_drops": 0,
            "record_tracker": True,
        }
    )
    return _play(argparse.Namespace(**common))


def _format_float_list(values: tuple[float, ...]) -> str:
    return "[" + ", ".join(repr(float(value)) for value in values) + "]"


def _print_plane_tf_result(
    pose: tuple[float, float, float, float, float, float],
    *,
    parent_frame: str,
    child_frame: str,
) -> int:
    transform = plane_transform_from_pose(pose)
    print(f"translation_m: {_format_float_list(transform.translation_m)}")
    print(f"quaternion_xyzw: {_format_float_list(transform.quaternion_xyzw)}")
    print("setup.yaml work_frame field (merge into the existing block):")
    print(
        "  teach_pendant_pose_xyz_rotvec: "
        + _format_float_list(transform.setup_yaml_pose_xyz_rotvec)
    )
    print("static_tf_command:")
    print(
        static_transform_publisher_command(
            pose,
            parent_frame=parent_frame,
            child_frame=child_frame,
        )
    )
    return 0


def _pendant_to_tf(args: argparse.Namespace) -> int:
    return _print_plane_tf_result(
        parse_teach_pendant_pose(args.pose),
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
    )


def _query_plane_variable(args: argparse.Namespace):
    return query_dashboard_variable(
        robot_ip=args.robot_ip,
        variable=args.variable,
        port=args.port,
        timeout_s=args.timeout,
    )


def _get_plane(args: argparse.Namespace) -> int:
    result = _query_plane_variable(args)
    print(f"UR Dashboard welcome: {result.banner}", file=sys.stderr)
    print(result.response)
    return 0


def _get_plane_tf(args: argparse.Namespace) -> int:
    result = query_dashboard_variable(
        robot_ip=args.robot_ip,
        variable=args.plane_name,
        port=args.port,
        timeout_s=args.timeout,
    )
    print(f"UR Dashboard welcome: {result.banner}", file=sys.stderr)
    print(
        f"UR Dashboard response ({result.variable}): {result.response}",
        file=sys.stderr,
    )
    pose = extract_teach_pendant_pose_from_dashboard_response(result.response)
    return _print_plane_tf_result(
        pose,
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
    )


def _add_dashboard_connection_arguments(
    parser: argparse.ArgumentParser,
    *,
    require_robot_ip: bool,
) -> None:
    if require_robot_ip:
        parser.add_argument("--robot-ip", required=True)
    else:
        parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_DASHBOARD_PORT)
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_DASHBOARD_TIMEOUT_S,
        help="Dashboard connect/read timeout in seconds",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tipdraw")
    parser.add_argument(
        "--workspace",
        help="Repository/workspace root (default: current directory)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser("build-recording", help="Compile, simulate, validate, and seal")
    build.add_argument("--env", required=True)
    build.add_argument("--trajectory", required=True)
    build.add_argument("--rviz", action="store_true")
    build.add_argument("--compile-only", action="store_true")
    build.add_argument(
        "--external-runtime",
        action="store_true",
        help="require an already-running simulation instead of managing ros2 launch",
    )
    build.add_argument(
        "--launch-timeout",
        type=float,
        default=120.0,
        help="seconds to wait for a managed simulation to expose BuildRecording",
    )
    build.set_defaults(handler=_build_recording)

    inspect = subparsers.add_parser("inspect", help="Verify and display a recording")
    inspect.add_argument("--env", required=True)
    inspect.add_argument("--recording", required=True)
    inspect.set_defaults(handler=_inspect)

    play = subparsers.add_parser("play", help="Play one validated recording")
    play.add_argument("--env", required=True)
    play.add_argument("--recording", required=True)
    prepare_mode = play.add_mutually_exclusive_group()
    prepare_mode.add_argument("--auto-prepare", action="store_true")
    prepare_mode.add_argument(
        "--externally-prepared-start",
        action="store_true",
        help=(
            "require the robot to already be settled at the sealed recording start; "
            "runtime verifies the start gate and sends the recording without planning"
        ),
    )
    play.add_argument(
        "--record-nucleo",
        action="store_true",
        help="capture and synchronize NUCLEO sensor frames during authoritative playback",
    )
    play.add_argument(
        "--nucleo-port",
        default=os.environ.get("HX_WAYPOINT_NUCLEO_PORT"),
        help="serial device used with --record-nucleo (or HX_WAYPOINT_NUCLEO_PORT)",
    )
    play.add_argument("--nucleo-baudrate", type=int, default=115200)
    play.add_argument(
        "--nucleo-channels",
        type=_parse_nucleo_channels,
        default=(1, 2, 3, 4),
        metavar="LIST",
        help="comma-separated hub channels, default: 1,2,3,4",
    )
    play.add_argument("--nucleo-ready-timeout", type=float, default=5.0)
    play.add_argument(
        "--nucleo-reuse-sync",
        action="store_true",
        help=(
            "open an already-streaming NUCLEO session without sending another SYNC; "
            "use only after one successful handshake and before the board is reset"
        ),
    )
    play.add_argument(
        "--nucleo-max-incomplete-groups",
        type=_parse_nonnegative_int,
        default=0,
        help=(
            "continue after robot success when the playback-window parser reports at "
            "most this many incomplete groups; default 0 keeps strict behavior"
        ),
    )
    play.add_argument(
        "--nucleo-max-channel-sequence-drops",
        type=_parse_nonnegative_int,
        default=0,
        help=(
            "continue after robot success when the playback-window parser reports at "
            "most this many channel sequence drops; default 0 keeps strict behavior"
        ),
    )
    play.add_argument(
        "--record-tracker",
        action="store_true",
        help=(
            "capture a geometry_msgs/PoseStamped tracker stream, map it to actual FK, "
            "and fit yaw calibration offsets"
        ),
    )
    play.add_argument(
        "--tracker-topic",
        default=os.environ.get("HX_WAYPOINT_TRACKER_TOPIC"),
        help=(
            "absolute PoseStamped topic used with --record-tracker "
            "(or HX_WAYPOINT_TRACKER_TOPIC)"
        ),
    )
    play.add_argument(
        "--tracker-frame",
        default="plane1",
        help="required PoseStamped frame_id; no implicit tracker-frame transform is guessed",
    )
    play.add_argument("--tracker-ready-timeout", type=float, default=5.0)
    play.add_argument(
        "--tracker-max-mapping-offset-ms",
        type=float,
        default=50.0,
        help="maximum host-monotonic tracker/robot nearest-neighbor offset",
    )
    play.add_argument(
        "--tracker-minimum-samples",
        type=int,
        default=50,
        help="minimum mapped PoseStamped samples required for the yaw fit",
    )
    play.set_defaults(handler=_play)

    generate_yaw = subparsers.add_parser(
        "generate-yaw-calibration",
        help="generate immutable warmup and fixed-XYZ yaw calibration authorities",
    )
    generate_yaw.add_argument("--env", required=True)
    generate_yaw.add_argument("--trajectory-prefix", required=True)
    generate_yaw.add_argument(
        "--reference-trajectory",
        default="bafe_continuous_wave_3d_zplus26mm",
        help="existing authority whose first quaternion fixes the rod-axis direction",
    )
    generate_yaw.add_argument("--sensor-distance-mm", type=float, default=67.5)
    generate_yaw.add_argument("--offset-x-mm", type=float, default=0.0)
    generate_yaw.add_argument("--offset-y-mm", type=float, default=0.0)
    generate_yaw.add_argument(
        "--sensor-plane-offset-z-mm",
        type=float,
        default=0.0,
        help=(
            "physical sensor-plane translation along plane1 Z; use -50 for "
            "the up50 board installation"
        ),
    )
    generate_yaw.add_argument(
        "--sweep-deg", type=float, choices=(180.0, 360.0), default=360.0
    )
    generate_yaw.add_argument("--leg-duration-s", type=float, default=18.0)
    generate_yaw.add_argument("--sample-period-s", type=float, default=0.05)
    generate_yaw.add_argument("--hold-duration-s", type=float, default=2.0)
    generate_yaw.add_argument("--warmup-duration-s", type=float, default=6.0)
    generate_yaw.add_argument("--warmup-amplitude-mm", type=float, default=5.0)
    generate_yaw.add_argument("--warmup-settle-s", type=float, default=2.0)
    generate_yaw.set_defaults(handler=_generate_yaw_calibration)

    yaw_calibrate = subparsers.add_parser(
        "yaw-calibrate",
        help="play and fit one combined warmup-plus-yaw calibration recording",
    )
    yaw_calibrate.add_argument("--env", required=True)
    yaw_calibrate.add_argument("--recording", required=True)
    yaw_calibrate.add_argument(
        "--tracker-topic",
        default=os.environ.get("HX_WAYPOINT_TRACKER_TOPIC"),
        required=os.environ.get("HX_WAYPOINT_TRACKER_TOPIC") is None,
    )
    yaw_calibrate.add_argument("--tracker-frame", default="plane1")
    yaw_calibrate.add_argument("--tracker-ready-timeout", type=float, default=5.0)
    yaw_calibrate.add_argument(
        "--tracker-max-mapping-offset-ms", type=float, default=50.0
    )
    yaw_calibrate.add_argument("--tracker-minimum-samples", type=int, default=50)
    yaw_calibrate.set_defaults(handler=_yaw_calibrate)

    pendant = subparsers.add_parser(
        "pendant-to-tf",
        help="convert one UR teach-pendant p[...] pose to base -> plane1 TF",
    )
    pendant.add_argument(
        "--pose",
        required=True,
        help="six-value UR pose, for example p[x,y,z,rx,ry,rz]",
    )
    pendant.add_argument("--parent-frame", default="base")
    pendant.add_argument("--child-frame", default="plane1")
    pendant.set_defaults(handler=_pendant_to_tf)

    get_plane = subparsers.add_parser(
        "get-plane",
        help="query one installation variable from the UR Dashboard Server",
    )
    _add_dashboard_connection_arguments(get_plane, require_robot_ip=False)
    get_plane.add_argument("--variable", default="opt1")
    get_plane.set_defaults(handler=_get_plane)

    get_plane_tf = subparsers.add_parser(
        "get-plane-tf",
        help="query a UR Plane Feature pose and convert it to a ROS TF command",
    )
    _add_dashboard_connection_arguments(get_plane_tf, require_robot_ip=True)
    get_plane_tf.add_argument(
        "--plane-name",
        required=True,
        help="name of the Plane Feature defined in the UR Installation",
    )
    get_plane_tf.add_argument("--parent-frame", default="base")
    get_plane_tf.add_argument("--child-frame", default="plane1")
    get_plane_tf.set_defaults(handler=_get_plane_tf)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (WaypointError, OSError, ValueError) as error:
        print(f"tipdraw: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
