#!/usr/bin/env python3
"""Aggregate candidate startup replays and render compact audit figures."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Sequence


SHORT_NAMES = {
    "bafe_continuous_wave_3d": "Wave",
    "bafe_continuous_helix_3d": "Helix",
    "bafe_continuous_rounded_square_3d": "Rounded square",
    "bafe_continuous_figure8_3d": "Figure eight",
    "bafe_continuous_lissajous_3d": "Lissajous",
    "bafe_continuous_rosette_3d": "Rosette",
    "bafe_continuous_trefoil_knot_3d": "Trefoil knot",
    "bafe_continuous_clover_3d": "Clover",
    "bafe_continuous_rounded_star_3d": "Rounded star",
    "bafe_continuous_heart_3d": "Heart",
    "bafe_continuous_spirograph_3d": "Spirograph",
    "bafe_continuous_guilloche_3d": "Guilloche",
    "bafe_continuous_seven_petal_rose_3d": "Seven-petal rose",
    "bafe_continuous_torus_knot_2_5_3d": "Torus knot (2,5)",
    "bafe_continuous_viviani_3d": "Viviani",
    "bafe_continuous_spherical_spiral_3d": "Spherical spiral",
    "bafe_continuous_harmonograph_3d": "Harmonograph",
    "bafe_continuous_butterfly_3d": "Butterfly",
    "bafe_continuous_superformula_flower_3d": "Superformula flower",
    "bafe_continuous_lorenz_3d": "Lorenz tour",
    "bafe_continuous_celtic_knot_3d": "Celtic knot",
    "bafe_continuous_braided_helix_3d": "Braided helix",
}


def _parse_bool(value: str) -> bool:
    return str(value).strip().lower() == "true"


def _parse_float(value: str) -> float | None:
    text = str(value).strip()
    return None if not text else float(text)


def _load_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row = dict(raw)
            row["seed"] = int(raw["seed"])
            row["orientation_amplitude_deg"] = float(
                raw["orientation_amplitude_deg"]
            )
            row["accepted"] = _parse_bool(raw["accepted"])
            row["strict_accurate"] = _parse_bool(raw["strict_accurate"])
            row["gross_false_release"] = _parse_bool(raw["gross_false_release"])
            for key in (
                "entry_packet_time_s",
                "entry_wall_time_s",
                "background_rmse_uT",
                "position_error_mm",
                "direction_error_deg",
                "wall_time_s",
            ):
                row[key] = _parse_float(raw[key])
            rows.append(row)
    return rows


def _write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    repository_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repository_root / "src/hx_waypoint_curobo"))
    sys.path.insert(0, str(repository_root / "src/hx_waypoint_tools"))

    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    from hx_waypoint_tools.trajectory_generator import (
        NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS,
        NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS,
        new_board_continuous_geometry_position,
    )

    input_root = args.input_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for seed in (32001, 32002, 32003):
        rows.extend(
            _load_rows(
                input_root
                / f"seed{seed}_fixed_and_15deg"
                / "startup_results.csv"
            )
        )

    braided_id = "bafe_continuous_braided_helix_3d"
    lorenz_id = "bafe_continuous_lorenz_3d"
    rows = [
        row
        for row in rows
        if row["trajectory_id"] not in {braided_id, lorenz_id}
    ]
    for seed in (32001, 32002, 32003):
        rows.extend(
            _load_rows(
                input_root / f"braided_v2_seed{seed}" / "startup_results.csv"
            )
        )
        rows.extend(
            _load_rows(
                input_root / f"lorenz_true_seed{seed}" / "startup_results.csv"
            )
        )
    order = {trajectory_id: index for index, trajectory_id in enumerate(
        NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS
    )}
    rows.sort(
        key=lambda row: (
            order[row["trajectory_id"]],
            row["orientation_amplitude_deg"],
            row["seed"],
        )
    )
    _write_csv(output_dir / "combined_results.csv", rows)

    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[row["trajectory_id"]].append(row)
    summary_rows: list[dict[str, Any]] = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        group = grouped[trajectory_id]
        entries = [
            float(row["entry_packet_time_s"])
            for row in group
            if row["entry_packet_time_s"] is not None
        ]
        summary_rows.append(
            {
                "trajectory_id": trajectory_id,
                "display_name": SHORT_NAMES[trajectory_id],
                "persistent_motion_contract": (
                    trajectory_id in NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS
                ),
                "accepted_count": sum(bool(row["accepted"]) for row in group),
                "strict_accurate_count": sum(
                    bool(row["strict_accurate"]) for row in group
                ),
                "gross_false_release_count": sum(
                    bool(row["gross_false_release"]) for row in group
                ),
                "entry_by_3s_count": sum(entry <= 3.0 for entry in entries),
                "entry_by_4s_count": sum(entry <= 4.0 for entry in entries),
                "median_entry_s": statistics.median(entries) if entries else None,
                "max_entry_s": max(entries) if entries else None,
                "fixed_accepted_count": sum(
                    bool(row["accepted"])
                    for row in group
                    if row["orientation_amplitude_deg"] == 0.0
                ),
                "tilt15_accepted_count": sum(
                    bool(row["accepted"])
                    for row in group
                    if row["orientation_amplitude_deg"] == 15.0
                ),
                "fixed_strict_count": sum(
                    bool(row["strict_accurate"])
                    for row in group
                    if row["orientation_amplitude_deg"] == 0.0
                ),
                "tilt15_strict_count": sum(
                    bool(row["strict_accurate"])
                    for row in group
                    if row["orientation_amplitude_deg"] == 15.0
                ),
            }
        )
    _write_csv(output_dir / "trajectory_summary.csv", summary_rows)

    accepted_count = sum(bool(row["accepted"]) for row in rows)
    strict_count = sum(bool(row["strict_accurate"]) for row in rows)
    gross_count = sum(bool(row["gross_false_release"]) for row in rows)
    non_gross_authorized_count = sum(
        bool(row["accepted"]) and not bool(row["gross_false_release"])
        for row in rows
    )
    entry_values = [
        float(row["entry_packet_time_s"])
        for row in rows
        if row["entry_packet_time_s"] is not None
    ]
    executable_rows = [
        row for row in rows if row["trajectory_id"] != lorenz_id
    ]
    executable_entries = [
        float(row["entry_packet_time_s"])
        for row in executable_rows
        if row["entry_packet_time_s"] is not None
    ]
    manifest = {
        "schema": "ur-waypoint-bafe-startup-candidate-summary-v1",
        "condition_count": len(rows),
        "trajectory_count": len(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS),
        "scene_seeds": [32001, 32002, 32003],
        "orientation_amplitudes_deg": [0.0, 15.0],
        "accepted_count": accepted_count,
        "non_gross_authorized_count": non_gross_authorized_count,
        "strict_accurate_count": strict_count,
        "gross_false_release_count": gross_count,
        "entry_by_3s_count": sum(entry <= 3.0 for entry in entry_values),
        "entry_by_4s_count": sum(entry <= 4.0 for entry in entry_values),
        "median_entry_s": statistics.median(entry_values),
        "p95_entry_s": float(np.percentile(entry_values, 95.0)),
        "max_entry_s": max(entry_values),
        "executable_candidate_condition_count": len(executable_rows),
        "executable_candidate_authorized_count": sum(
            bool(row["accepted"]) for row in executable_rows
        ),
        "executable_candidate_strict_count": sum(
            bool(row["strict_accurate"]) for row in executable_rows
        ),
        "executable_candidate_gross_count": sum(
            bool(row["gross_false_release"]) for row in executable_rows
        ),
        "executable_candidate_entry_by_3s_count": sum(
            entry <= 3.0 for entry in executable_entries
        ),
        "executable_candidate_median_entry_s": statistics.median(
            executable_entries
        ),
        "executable_candidate_p95_entry_s": float(
            np.percentile(executable_entries, 95.0)
        ),
        "executable_candidate_max_entry_s": max(executable_entries),
        "strict_definition": "accepted, non-gross, background RMSE < 1 uT, position error < 3 mm",
        "braided_helix_version": "low-frequency asymmetry v2",
        "lorenz_version": "single non-closed sparse-control Lorenz tour",
    }
    (output_dir / "summary.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 9,
            "axes.titlesize": 9,
        }
    )

    columns = [
        (0.0, 32001),
        (0.0, 32002),
        (0.0, 32003),
        (15.0, 32001),
        (15.0, 32002),
        (15.0, 32003),
    ]
    matrix = np.zeros((len(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS), len(columns)))
    labels = [["" for _ in columns] for _ in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS]
    lookup = {
        (
            row["trajectory_id"],
            row["orientation_amplitude_deg"],
            row["seed"],
        ): row
        for row in rows
    }
    for row_index, trajectory_id in enumerate(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS):
        for column_index, (amplitude_deg, seed) in enumerate(columns):
            row = lookup[(trajectory_id, amplitude_deg, seed)]
            if row["gross_false_release"]:
                matrix[row_index, column_index] = 0
            elif row["strict_accurate"]:
                matrix[row_index, column_index] = 2
            elif row["accepted"]:
                matrix[row_index, column_index] = 1
            labels[row_index][column_index] = (
                "--"
                if row["entry_packet_time_s"] is None
                else f"{float(row['entry_packet_time_s']):.2f}"
            )

    figure, axis = plt.subplots(figsize=(7.0, 8.0), constrained_layout=True)
    axis.imshow(
        matrix,
        aspect="auto",
        interpolation="nearest",
        cmap=ListedColormap(["#b84545", "#e3a23b", "#2f8f69"]),
        vmin=0,
        vmax=2,
    )
    for row_index in range(matrix.shape[0]):
        for column_index in range(matrix.shape[1]):
            color = "white" if matrix[row_index, column_index] != 1 else "#2b2114"
            axis.text(
                column_index,
                row_index,
                labels[row_index][column_index],
                ha="center",
                va="center",
                color=color,
                fontsize=7.5,
            )
    axis.set_xticks(
        range(len(columns)),
        ["Fixed\n32001", "Fixed\n32002", "Fixed\n32003", "15 deg\n32001", "15 deg\n32002", "15 deg\n32003"],
    )
    axis.set_yticks(
        range(len(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS)),
        [SHORT_NAMES[trajectory_id] for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS],
    )
    axis.set_title("Production-runtime startup audit; cell value is packet-time entry (s)")
    axis.tick_params(length=0)
    axis.set_xticks(np.arange(-0.5, len(columns), 1), minor=True)
    axis.set_yticks(
        np.arange(-0.5, len(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS), 1),
        minor=True,
    )
    axis.grid(which="minor", color="white", linewidth=1.2)
    axis.legend(
        handles=[
            Line2D([0], [0], marker="s", color="none", markerfacecolor="#2f8f69", markersize=8, label="Strict accurate"),
            Line2D([0], [0], marker="s", color="none", markerfacecolor="#e3a23b", markersize=8, label="Authorized outside strict endpoint"),
            Line2D([0], [0], marker="s", color="none", markerfacecolor="#b84545", markersize=8, label="Refusal / gross"),
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, -0.055),
        ncol=3,
        frameon=False,
        fontsize=8,
    )
    for suffix in ("png", "pdf"):
        figure.savefig(
            output_dir / f"startup_validation_matrix.{suffix}",
            dpi=240,
            bbox_inches="tight",
        )
    plt.close(figure)

    figure = plt.figure(figsize=(10.5, 13.0), constrained_layout=True)
    times_s = np.linspace(0.0, 30.0, 1201)
    for index, trajectory_id in enumerate(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS):
        axis = figure.add_subplot(6, 4, index + 1, projection="3d")
        positions = np.asarray(
            [
                new_board_continuous_geometry_position(trajectory_id, float(time_s))
                for time_s in times_s
            ],
            dtype=float,
        )
        segments = np.stack([positions[:-1], positions[1:]], axis=1)
        collection = Line3DCollection(
            segments,
            cmap="viridis",
            linewidth=1.15,
        )
        collection.set_array(times_s[:-1])
        axis.add_collection3d(collection)
        axis.scatter(*positions[0], color="#111111", s=6, depthshade=False)
        axis.set_xlim(-0.047, 0.047)
        axis.set_ylim(-0.036, 0.036)
        axis.set_zlim(-0.060, -0.040)
        axis.set_box_aspect((94, 72, 38))
        axis.view_init(elev=24, azim=-58)
        axis.set_proj_type("ortho")
        axis.set_xticks([])
        axis.set_yticks([])
        axis.set_zticks([])
        suffix = " (stress)" if trajectory_id == "bafe_continuous_lorenz_3d" else ""
        axis.set_title(f"{SHORT_NAMES[trajectory_id]}{suffix}", pad=1.0)
        axis.grid(False)
    for index in range(len(NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS), 24):
        axis = figure.add_subplot(6, 4, index + 1)
        axis.axis("off")
    for suffix in ("png", "pdf"):
        figure.savefig(
            output_dir / f"formal_trajectory_catalog_3d.{suffix}",
            dpi=220,
            bbox_inches="tight",
        )
    plt.close(figure)

    lines = [
        "# BAFE startup candidate audit",
        "",
        f"- Conditions: {len(rows)} (22 trajectories x 3 seeds x 2 orientation modes)",
        f"- Authorized: {accepted_count}/{len(rows)}",
        f"- Non-gross authorized: {non_gross_authorized_count}/{len(rows)}",
        f"- Strict accurate: {strict_count}/{len(rows)}",
        f"- Gross false releases: {gross_count}/{len(rows)}",
        f"- Entry by 3 s: {manifest['entry_by_3s_count']}/{len(rows)}",
        f"- Median / p95 / max entry: {manifest['median_entry_s']:.2f} / {manifest['p95_entry_s']:.2f} / {manifest['max_entry_s']:.2f} s",
        f"- Excluding Lorenz stress: {manifest['executable_candidate_authorized_count']}/{manifest['executable_candidate_condition_count']} authorized, {manifest['executable_candidate_gross_count']} gross, {manifest['executable_candidate_strict_count']} strict",
        "",
        "| Trajectory | Authorized | Strict | Gross | <=3 s | Median entry | Max entry |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        median_entry = (
            "--"
            if row["median_entry_s"] is None
            else f"{row['median_entry_s']:.2f} s"
        )
        max_entry = (
            "--"
            if row["max_entry_s"] is None
            else f"{row['max_entry_s']:.2f} s"
        )
        lines.append(
            f"| {row['display_name']} | {row['accepted_count']}/6 | "
            f"{row['strict_accurate_count']}/6 | "
            f"{row['gross_false_release_count']}/6 | "
            f"{row['entry_by_3s_count']}/6 | "
            f"{median_entry} | {max_entry} |"
        )
    (output_dir / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
