#!/usr/bin/env python3
"""Recheck failed static-grid targets from configured home without robot motion."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import sys

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition
from hx_waypoint_tools.tcp_console import (
    AbsolutePoseCommand,
    RosTcpSession,
    _validate_args as validate_console_args,
    absolute_tip_target_in_base,
    build_argument_parser as build_console_parser,
    rod_tip_target_to_tool0,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--environment", default="mla")
    parser.add_argument(
        "--free-roll-step-deg",
        type=float,
        default=0.0,
        help="enumerate equivalent roll about the vertical rod axis; 0 tests only roll 0",
    )
    return parser.parse_args()


def _roll_candidates(step_deg: float) -> tuple[float, ...]:
    step = float(step_deg)
    if not math.isfinite(step) or step < 0.0 or step > 180.0:
        raise ValueError("--free-roll-step-deg must be finite and in [0, 180]")
    if step == 0.0:
        return (0.0,)
    result = [0.0]
    turns = int(math.floor(180.0 / step + 1.0e-12))
    for index in range(1, turns + 1):
        angle = index * step
        if angle < 180.0 - 1.0e-9:
            result.extend((angle, -angle))
        elif angle <= 180.0 + 1.0e-9:
            result.append(180.0)
    return tuple(result)


def _console_args(workspace: Path, environment: str) -> argparse.Namespace:
    args = build_console_parser().parse_args(
        [
            "--workspace",
            str(workspace),
            "--env",
            environment,
            "--max-translation-mm",
            "1000",
            "--max-absolute-translation-mm",
            "1000",
            "--max-rotation-deg",
            "180",
            "--max-joint-excursion-deg",
            "180",
            "--planning-attempts",
            "10",
            "--allowed-planning-time",
            "5",
            "--velocity-scale",
            "0.15",
            "--acceleration-scale",
            "0.15",
        ]
    )
    validate_console_args(args)
    return args


def main() -> int:
    args = _arguments()
    workspace = args.workspace.expanduser().resolve()
    source = args.results.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite {output}")
    rolls = _roll_candidates(args.free_roll_step_deg)

    with source.open("r", encoding="utf-8", newline="") as stream:
        failed = tuple(
            row
            for row in csv.DictReader(stream)
            if row.get("status") == "UNREACHABLE"
        )
    if not failed:
        raise RuntimeError("input contains no UNREACHABLE targets")

    environment = EnvironmentBuilder().load(workspace / "environments" / args.environment)
    scene = load_planning_scene_definition(workspace, args.environment)
    console_args = _console_args(workspace, args.environment)

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    session: RosTcpSession | None = None
    rows: list[dict[str, str | int]] = []
    attempt_rows: list[dict[str, str | int | float]] = []
    try:
        session = RosTcpSession(environment, scene, console_args)
        session.live_authoritative_transforms()
        session.verify_scene()
        base_to_plane1, _tool0_to_tip = session.live_authoritative_transforms()
        home = tuple(float(value) for value in environment.home_joint_positions)
        success_code = int(session.types["MoveItErrorCodes"].SUCCESS)
        succeeded_status = int(session.types["GoalStatus"].STATUS_SUCCEEDED)

        for index, source_row in enumerate(failed, start=1):
            point_id = source_row["point_id"]
            status = "HOME_SEED_IK_FAILED"
            message = ""
            planned_points = 0
            planned_duration_s = ""
            selected_roll: str | float = ""
            ik_solutions_found = 0
            for roll_deg in rolls:
                attempt_status = "IK_FAILED"
                attempt_message = ""
                command = AbsolutePoseCommand(
                    frame="plane1",
                    position_m=(
                        float(source_row["target_x_m"]),
                        float(source_row["target_y_m"]),
                        float(source_row["target_plane1_z_m"]),
                    ),
                    # Roll is about plane1/tip Z.  It leaves the rod axis and
                    # therefore the vertical magnetic-moment direction unchanged.
                    rotation_vector_rad=(0.0, 0.0, math.radians(roll_deg)),
                )
                base_to_tip = absolute_tip_target_in_base(command, base_to_plane1)
                target_tool = rod_tip_target_to_tool0(
                    session.tool_builder,
                    base_to_tip,
                    frame_id=session.base_frame,
                )
                try:
                    solution = session._solve_collision_checked_ik(target_tool, home)
                    ik_solutions_found += 1
                    attempt_status = "IK_OK_PLAN_FAILED"
                    goal = session._build_goal(solution, home)
                    wrapped = session._run_tracked_action(
                        session.move_client,
                        goal,
                        action_label=f"home free-roll {roll_deg:+g} plan-only {point_id}",
                        cancel_event=None,
                    )
                    result = wrapped.result
                    code = int(result.error_code.val)
                    if code != success_code or int(wrapped.status) != succeeded_status:
                        attempt_message = session._moveit_error_description(result.error_code)
                    else:
                        trajectory = result.planned_trajectory
                        session._validate_planned_trajectory(trajectory, home, solution)
                        points = tuple(trajectory.joint_trajectory.points)
                        planned_points = len(points)
                        end = points[-1].time_from_start
                        planned_duration_s = (
                            f"{int(end.sec) + int(end.nanosec) * 1e-9:.9f}"
                        )
                        attempt_status = "HOME_TO_TARGET_PLAN_OK"
                        attempt_message = (
                            "exact position and vertical rod axis have a "
                            "collision-checked plan from home"
                        )
                except Exception as error:
                    attempt_message = f"{type(error).__name__}: {error}"

                attempt_rows.append(
                    {
                        "original_sequence": int(source_row["sequence"]),
                        "point_id": point_id,
                        "target_x_m": source_row["target_x_m"],
                        "target_y_m": source_row["target_y_m"],
                        "target_height_m": source_row["target_height_m"],
                        "target_plane1_z_m": source_row["target_plane1_z_m"],
                        "free_roll_deg": roll_deg,
                        "status": attempt_status,
                        "message": attempt_message.replace("\n", " "),
                    }
                )
                message = attempt_message
                if attempt_status == "HOME_TO_TARGET_PLAN_OK":
                    status = attempt_status
                    selected_roll = roll_deg
                    break
                if attempt_status == "IK_OK_PLAN_FAILED":
                    status = "HOME_TO_TARGET_PLAN_FAILED"

            row: dict[str, str | int] = {
                "original_sequence": int(source_row["sequence"]),
                "point_id": point_id,
                "target_x_m": source_row["target_x_m"],
                "target_y_m": source_row["target_y_m"],
                "target_height_m": source_row["target_height_m"],
                "target_plane1_z_m": source_row["target_plane1_z_m"],
                "status": status,
                "selected_free_roll_deg": selected_roll,
                "roll_candidates_tested": sum(
                    attempt["point_id"] == point_id for attempt in attempt_rows
                ),
                "ik_solutions_found": ik_solutions_found,
                "planned_point_count": planned_points,
                "planned_duration_s": planned_duration_s,
                "message": message.replace("\n", " "),
            }
            rows.append(row)
            print(f"[{index}/{len(failed)}] {point_id}: {status}", flush=True)
    finally:
        if session is not None:
            session.close()
        rclpy.shutdown()

    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = tuple(rows[0])
    with output.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    attempts_output = output.with_name(output.stem + "__attempts.csv")
    with attempts_output.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=tuple(attempt_rows[0]),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(attempt_rows)
    counts = {
        status: sum(row["status"] == status for row in rows)
        for status in sorted({str(row["status"]) for row in rows})
    }
    print(f"generated_utc: {datetime.now(timezone.utc).isoformat()}")
    print(f"output: {output}")
    print(f"attempts_output: {attempts_output}")
    print(f"counts: {counts}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"diagnose_static_grid_from_home: {error}", file=sys.stderr)
        raise SystemExit(2)
