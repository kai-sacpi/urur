"""Own the ROS playback goal, including acknowledged cancellation on failure."""
from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any, Callable


class CancellationUnconfirmed(RuntimeError):
    """Do not assume the robot has stopped, and never dispatch a return motion."""


class RosPlaybackBackend:
    def __init__(self, workspace: Path):
        import rclpy
        from hx_waypoint_interfaces.action import PlayRecording
        from rcl_interfaces.srv import GetParameters
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.rclpy, self.action_type = rclpy, PlayRecording
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("wm_collection_playback", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, PlayRecording, "/hx_waypoint/play_recording")
        self.parameters = self.node.create_client(GetParameters, "/hx_waypoint_runtime/get_parameters")
        self.GetParameters, self.workspace = GetParameters, workspace

    def wait(self, future: Any, timeout: float) -> Any:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(.1, max(0, deadline - time.monotonic())))
        if not future.done():
            raise TimeoutError(f"ROS playback operation timed out after {timeout} s")
        value = future.result()
        if value is None:
            raise RuntimeError("ROS playback operation returned no response")
        return value

    def begin(self) -> dict[str, Any]:
        if not self.client.wait_for_server(timeout_sec=20):
            raise RuntimeError("PlayRecording action unavailable")
        if not self.parameters.wait_for_service(timeout_sec=10):
            raise RuntimeError("runtime identity unavailable")
        request = self.GetParameters.Request()
        request.names = ["execution_mode", "environments_root", "telemetry_root", "use_sim_time"]
        response = self.wait(self.parameters.call_async(request), 10)
        values = response.values
        expected = ["real", str(self.workspace / "environments"), str(self.workspace / "runs")]
        if (len(values) != 4 or [v.string_value for v in values[:3]] != expected or
                values[3].bool_value):
            raise RuntimeError("real runtime identity/clock does not match collection workspace")
        return {"execution_mode": "real", "environments_root": expected[1],
                "telemetry_root": expected[2], "use_sim_time": False}

    def submit(self, row: dict[str, Any], feedback: Callable[..., None]) -> Any:
        goal = self.action_type.Goal()
        goal.environment_id = "mla_short"
        goal.environment_path = str(self.workspace / "environments/mla_short")
        goal.recording_id = row["recording_id"]
        goal.auto_prepare = True
        goal.externally_prepared_start = False
        return self.client.send_goal_async(goal, feedback_callback=feedback)

    def close(self) -> None:
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=2.)
        self.node.destroy_node()
        self.context.shutdown()


def play_recording(workspace: Path, row: dict[str, Any], log_path: Path, *,
                   backend_factory: Callable[[Path], Any] = RosPlaybackBackend) -> Path:
    """No serial reader or subprocess; retain the goal handle through cleanup."""
    workspace = workspace.resolve()
    handle = pending = backend = None
    terminal = False
    with log_path.open("x") as log:
        def record(value: dict[str, Any]) -> None:
            log.write(json.dumps(value, ensure_ascii=False) + "\n")
            log.flush()

        def feedback(message: Any) -> None:
            item = message.feedback
            record({"state": item.state, "trajectory_time_s": item.trajectory_time_s,
                    "progress": item.progress, "detail": item.detail})

        try:
            backend = backend_factory(workspace)
            record({"runtime": backend.begin(), "recording_id": row["recording_id"]})
            pending = backend.submit(row, feedback)
            handle = backend.wait(pending, 30)
            if not handle.accepted:
                terminal = True
                raise RuntimeError("robot playback goal rejected")
            wrapped = backend.wait(handle.get_result_async(), row["duration_s"] + 600)
            terminal = True
            result = wrapped.result
            record({"status": int(wrapped.status), "success": bool(result.success),
                    "message": result.message, "telemetry_path": result.telemetry_path})
            if wrapped.status != 4 or not result.success:
                raise RuntimeError(f"robot playback failed: {result.message}")
            telemetry = Path(result.telemetry_path).resolve()
        except BaseException as original:
            # A process exit/client death does NOT cancel a ROS action. Explicitly
            # cancel our own goal and await its terminal result before teardown.
            if backend is not None and pending is not None and not terminal:
                try:
                    if handle is None:
                        handle = backend.wait(pending, 5)
                    if handle.accepted:
                        response = backend.wait(handle.cancel_goal_async(), 10)
                        wrapped = backend.wait(handle.get_result_async(), 10)
                        if wrapped.status not in (4, 5, 6):
                            raise RuntimeError("goal still has a nonterminal action status")
                        record({"cleanup": "owned_goal_terminal_confirmed", "status": int(wrapped.status),
                                "goals_canceling": len(response.goals_canceling)})
                except BaseException as cancel_error:
                    record({"cleanup": "cancellation_unconfirmed_do_not_assume_stopped",
                            "error": str(cancel_error), "original_error": str(original)})
                    raise CancellationUnconfirmed(
                        f"robot goal cancellation unconfirmed; do not assume stopped; see {log_path}") from original
            raise
        finally:
            if backend is not None:
                backend.close()
    if (not telemetry.is_relative_to(workspace / "runs") or not telemetry.is_file() or
            telemetry.name != "execution_telemetry.csv" or telemetry.parent.name != "real"):
        raise RuntimeError(f"playback did not return actual real telemetry: {log_path}")
    return telemetry
