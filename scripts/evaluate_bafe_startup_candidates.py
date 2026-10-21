#!/usr/bin/env python3
"""Replay continuous robot trajectories through the production BAFE runtime."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Sequence


for _thread_variable in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
):
    os.environ.setdefault(_thread_variable, "1")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _write_rows(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames = tuple(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bafe-root",
        type=Path,
        required=True,
        help="BAFE-code repository containing runtime/.",
    )
    parser.add_argument(
        "--experiment-root",
        type=Path,
        required=True,
        help="Repository containing experiments/simulation_mechanism helpers.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=32001)
    parser.add_argument("--max-duration-s", type=float, default=8.0)
    parser.add_argument("--post-entry-tracking-s", type=float, default=0.0)
    parser.add_argument(
        "--orientation-amplitude-deg",
        type=float,
        action="append",
        dest="orientation_amplitudes_deg",
        help="Repeat to evaluate multiple amplitudes; defaults to 0 and 15 degrees.",
    )
    parser.add_argument(
        "--trajectory",
        action="append",
        dest="trajectories",
        help="Optional trajectory ID subset; repeat for multiple IDs.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    repository_root = Path(__file__).resolve().parents[1]
    bafe_root = args.bafe_root.expanduser().resolve()
    experiment_root = args.experiment_root.expanduser().resolve()
    runtime_root = bafe_root / "runtime"
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    for import_root in (
        repository_root / "src/hx_waypoint_curobo",
        repository_root / "src/hx_waypoint_tools",
        experiment_root,
        runtime_root,
    ):
        sys.path.insert(0, str(import_root))

    import numpy as np

    from experiments.simulation_mechanism.packets import (
        PacketScenario,
        build_packet_bundle,
    )
    from experiments.simulation_mechanism.runtime_adapter import (
        run_runtime_startup_sync,
    )
    from hx_waypoint_tools.trajectory_generator import (
        NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS,
        NewBoardOrientationMotion,
        new_board_continuous_geometry_position,
        new_board_continuous_quaternion,
    )
    from magloc.algorithms.estimators.bmb_cv_ekf_9d.estimator import (
        BmbCvEkf9DEstimator,
    )
    from magloc.configuration import config_use
    from magloc.domain import load_sensor_array_csv

    trajectory_ids = tuple(args.trajectories or NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS)
    unknown = sorted(set(trajectory_ids) - set(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS))
    if unknown:
        raise ValueError(f"unknown trajectory IDs: {unknown}")
    amplitudes_deg = tuple(args.orientation_amplitudes_deg or (0.0, 15.0))
    if args.max_duration_s <= 2.72:
        raise ValueError("max-duration-s must extend beyond the final startup checkpoint")

    cfg = config_use(
        "mag_bmb_cv",
        "sim",
        {
            "source": {"trajectory_path": str(runtime_root / "unused.csv")},
            "sink": {
                "headless": True,
                "persist_results": False,
                "generate_plots": False,
                "browser_vis": False,
            },
        },
    )
    magnet = cfg.require("loaded.magnet_config").get("magnet", {})
    sensor_array = load_sensor_array_csv(cfg.path("sensor_array"))
    moment_strength_Am2 = float(magnet["moment_strength"])
    sample_dt_s = 0.01
    timestamps_s = np.arange(
        0.0,
        float(args.max_duration_s) + 0.5 * sample_dt_s,
        sample_dt_s,
    )
    scenario = PacketScenario(
        seed=int(args.seed),
        noise_sigma_uT=0.3,
        background_kind="arbitrary_axis",
        background_rms_uT=20.0,
    )

    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    total = len(trajectory_ids) * len(amplitudes_deg)
    run_index = 0
    suite_started = time.perf_counter()

    for amplitude_deg in amplitudes_deg:
        orientation = NewBoardOrientationMotion(
            enabled=amplitude_deg > 0.0,
            amplitude_deg=float(amplitude_deg),
            cycle_count=3,
        )
        for trajectory_id in trajectory_ids:
            run_index += 1
            positions_m = np.asarray(
                [
                    new_board_continuous_geometry_position(
                        trajectory_id,
                        float(timestamp_s),
                    )
                    for timestamp_s in timestamps_s
                ],
                dtype=float,
            )
            # The robot board frame stores distance below the plane as negative z;
            # the simulation array frame uses positive standoff from the array.
            positions_m[:, 2] *= -1.0
            quaternions_xyzw = np.asarray(
                [
                    new_board_continuous_quaternion(float(timestamp_s), orientation)
                    for timestamp_s in timestamps_s
                ],
                dtype=float,
            )
            velocities_mps = np.gradient(
                positions_m,
                sample_dt_s,
                axis=0,
                edge_order=2,
            )
            bundle = build_packet_bundle(
                {
                    "timestamps_s": timestamps_s,
                    "positions_m": positions_m,
                    "quaternions_xyzw": quaternions_xyzw,
                    "velocities_mps": velocities_mps,
                },
                sensor_array,
                moment_strength_Am2=moment_strength_Am2,
                scenario=scenario,
            )

            estimator = BmbCvEkf9DEstimator(cfg)
            try:
                result = run_runtime_startup_sync(
                    estimator,
                    bundle.packets,
                    realtime_pacing=True,
                    post_entry_tracking_s=float(args.post_entry_tracking_s),
                )
            finally:
                estimator.close()

            accepted = bool(result.get("startup_accepted", False))
            gross = bool(result.get("gross_false_release", False))
            background_rmse_uT = result.get("entry_background_rmse_uT")
            position_error_m = result.get("entry_position_error_m")
            strict_accurate = bool(
                accepted
                and not gross
                and background_rmse_uT is not None
                and float(background_rmse_uT) < 1.0
                and position_error_m is not None
                and float(position_error_m) < 0.003
            )
            row = {
                "trajectory_id": trajectory_id,
                "orientation_amplitude_deg": float(amplitude_deg),
                "seed": int(args.seed),
                "accepted": accepted,
                "strict_accurate": strict_accurate,
                "gross_false_release": gross,
                "startup_stage": result.get("startup_stage"),
                "entry_packet_time_s": result.get("startup_entry_time_s"),
                "entry_wall_time_s": result.get("startup_entry_wall_time_s"),
                "background_rmse_uT": background_rmse_uT,
                "position_error_mm": (
                    None
                    if position_error_m is None
                    else 1000.0 * float(position_error_m)
                ),
                "direction_error_deg": result.get("entry_direction_error_deg"),
                "reason": result.get("startup_reason"),
                "wall_time_s": result.get("wall_time_s"),
            }
            rows.append(row)
            details.append(
                {
                    "summary": row,
                    "runtime": _json_ready(result),
                    "packet_bundle": _json_ready(bundle.metadata),
                }
            )
            _write_rows(output_dir / "startup_results.csv", rows)
            (output_dir / "startup_results.json").write_text(
                json.dumps(details, indent=2, sort_keys=True),
                encoding="utf-8",
            )
            outcome = "strict" if strict_accurate else "accepted" if accepted else "refused"
            print(
                f"[{run_index:02d}/{total:02d}] {trajectory_id} "
                f"tilt={amplitude_deg:g} deg: {outcome}, "
                f"entry={result.get('startup_entry_time_s')}, "
                f"reason={result.get('startup_reason')}",
                flush=True,
            )

    metadata = {
        "schema": "ur-waypoint-bafe-startup-candidate-evaluation-v1",
        "bafe_root": str(bafe_root),
        "experiment_root": str(experiment_root),
        "trajectory_repository": str(repository_root),
        "runtime_config_sha256": _sha256(
            runtime_root / "configs/mag_bmb_cv/runtime.yaml"
        ),
        "trajectory_generator_sha256": _sha256(
            repository_root
            / "src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py"
        ),
        "seed": int(args.seed),
        "noise_sigma_uT": 0.3,
        "background_kind": "arbitrary_axis",
        "background_rms_uT": 20.0,
        "sample_rate_hz": 100.0,
        "maximum_packet_time_s": float(args.max_duration_s),
        "orientation_amplitudes_deg": list(amplitudes_deg),
        "trajectory_count": len(trajectory_ids),
        "condition_count": len(rows),
        "strict_accurate_count": sum(bool(row["strict_accurate"]) for row in rows),
        "accepted_count": sum(bool(row["accepted"]) for row in rows),
        "gross_false_release_count": sum(
            bool(row["gross_false_release"]) for row in rows
        ),
        "suite_wall_time_s": time.perf_counter() - suite_started,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(metadata, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
