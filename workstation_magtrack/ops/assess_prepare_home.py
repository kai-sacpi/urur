"""Plan and validate a fresh actual-state-to-home request; never execute it."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from collection.motion import RosMotionBackend, validate_trajectory, write_joint_csv_exact, write_json_new


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root, output = args.workspace.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
    environment = EnvironmentBuilder().load(root / "environments/mla_short")
    backend = RosMotionBackend(environment, "real", 0)
    report = {"purpose": "planner_only_no_robot_goal_no_serial", "passed": False,
              "started_unix_ns": time.time_ns()}
    try:
        identity = dict(backend.runtime_identity(20.0))
        report["runtime"] = identity
        expected = {"execution_mode": "real", "environments_root": str(root / "environments"),
                    "telemetry_root": str(root / "runs"), "use_sim_time": False}
        if identity != expected:
            raise ValueError("runtime identity mismatch")
        state = backend.fresh_state(5.0)
        if set(state.joint_names) != set(environment.joint_names):
            raise ValueError("actual joint names mismatch")
        start = tuple(state.positions[state.joint_names.index(name)] for name in environment.joint_names)
        report["start_positions_rad"] = start
        report["home_positions_rad"] = environment.home_joint_positions
        reply = backend.plan(environment.environment_id, environment.joint_names, start,
                             environment.home_joint_positions, 120.0)
        if not reply.collision_checked or dict(reply.environment_hashes) != dict(environment.hashes):
            raise ValueError("planner environment or collision evidence mismatch")
        trajectory = reply.trajectory
        write_joint_csv_exact(output / "joint_trajectory.csv", trajectory)
        report["validation"] = validate_trajectory(environment, trajectory, start,
                                                    environment.home_joint_positions)
        intervals = [b.time_s - a.time_s for a, b in zip(trajectory.samples, trajectory.samples[1:])]
        report.update(passed=True, min_dt_s=min(intervals), max_dt_s=max(intervals),
                      point_count=len(trajectory.samples), curobo_revision=reply.curobo_revision,
                      environment_hashes=dict(reply.environment_hashes))
    except BaseException as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        backend.close()
        report["finished_unix_ns"] = time.time_ns()
        write_json_new(output / "report.json", report)
        print(json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
