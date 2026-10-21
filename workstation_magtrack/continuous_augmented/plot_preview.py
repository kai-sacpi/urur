#!/usr/bin/env python3
"""Render immutable, CSV-driven previews of the open-curve candidate bank.

This script does not generate, edit, resample for execution, or validate robot
motion. All plotted trajectory vertices come from the authoritative source CSV.
The display frame is (plane1.x, plane1.y, height=-plane1.z), in millimetres.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.cm import ScalarMappable
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
from scipy.spatial.transform import Rotation


REQUIRED_COLUMNS = (
    "time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m", "tip_qx",
    "tip_qy", "tip_qz", "tip_qw", "phase",
)
PALETTE = {"start": "#18734c", "end": "#d55e00", "axis": "#505c70"}


@dataclass
class Source:
    trajectory_id: str
    record: dict
    metadata: dict
    csv_path: Path
    metadata_path: Path
    csv_sha256: str
    metadata_sha256: str
    time_s: np.ndarray
    position_m: np.ndarray
    quaternion_xyzw: np.ndarray
    phase: np.ndarray

    @property
    def display_mm(self) -> np.ndarray:
        return self.position_m * np.array([1000.0, 1000.0, -1000.0])

    @property
    def fixed_height(self) -> bool:
        return bool(np.ptp(self.position_m[:, 2]) <= 1e-9)

    @property
    def rod_axis_xyz(self) -> np.ndarray:
        return Rotation.from_quat(self.quaternion_xyzw).apply([0.0, 0.0, 1.0])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def authority_path(batch: Path, relative: str) -> Path:
    path = (batch / relative).resolve()
    if not path.is_relative_to(batch):
        raise ValueError(f"Source path escapes batch: {relative}")
    if not path.is_file():
        raise ValueError(f"Missing authority file: {path}")
    return path


def load_sources(batch: Path, expected_count: int) -> tuple[dict, list[Source]]:
    manifest = json.loads((batch / "manifest.json").read_text())
    records = manifest["records"]
    if len(records) != expected_count:
        raise ValueError(f"Expected {expected_count} records, got {len(records)}")
    if len({r["trajectory_id"] for r in records}) != len(records):
        raise ValueError("Duplicate trajectory IDs")
    sources = []
    for record in records:
        csv_path = authority_path(batch, record["csv_file"])
        metadata_path = authority_path(batch, record["metadata_file"])
        csv_hash, metadata_hash = sha256(csv_path), sha256(metadata_path)
        if csv_hash != record["sha256"]:
            raise ValueError(f"CSV SHA256 mismatch: {record['trajectory_id']}")
        if metadata_hash != record["metadata_sha256"]:
            raise ValueError(f"Metadata SHA256 mismatch: {record['trajectory_id']}")
        with csv_path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != REQUIRED_COLUMNS:
                raise ValueError(f"Unexpected CSV columns: {csv_path}")
            rows = list(reader)
        if len(rows) < 3 or any(row["frame_id"] != "plane1" for row in rows):
            raise ValueError(f"Expected at least 3 plane1 samples: {csv_path}")
        t = np.asarray([float(row["time_s"]) for row in rows])
        p = np.asarray([[float(row[c]) for c in REQUIRED_COLUMNS[2:5]] for row in rows])
        q = np.asarray([[float(row[c]) for c in REQUIRED_COLUMNS[5:9]] for row in rows])
        if not all(np.isfinite(a).all() for a in (t, p, q)):
            raise ValueError(f"Non-finite CSV sample: {csv_path}")
        if t[0] != 0.0 or np.any(np.diff(t) <= 0.0):
            raise ValueError(f"Invalid source time: {csv_path}")
        if np.max(np.abs(np.linalg.norm(q, axis=1) - 1.0)) > 1e-7:
            raise ValueError(f"Invalid quaternion norm: {csv_path}")
        meta = json.loads(metadata_path.read_text())
        source = Source(record["trajectory_id"], record, meta, csv_path, metadata_path,
                        csv_hash, metadata_hash, t, p, q,
                        np.asarray([row["phase"] for row in rows]))
        expected_mode = meta.get("height_profile", {}).get("mode")
        if expected_mode is not None:
            if (expected_mode == "fixed") != source.fixed_height:
                raise ValueError(f"Height mode disagrees with CSV: {source.trajectory_id}")
        sources.append(source)
    return manifest, sources


def bernstein_curve(control_points: list, sample_count: int = 401) -> np.ndarray:
    """For a labelled reference overlay only, never an execution source."""
    cps = np.asarray(control_points, dtype=float)
    if cps.ndim != 2 or cps.shape[1] != 3 or not np.isfinite(cps).all():
        raise ValueError("Invalid prototype control points")
    u = np.linspace(0.0, 1.0, sample_count)
    degree = len(cps) - 1
    weights = np.asarray([math.comb(degree, k) * u**k * (1 - u)**(degree - k)
                          for k in range(degree + 1)]).T
    return weights @ cps


def draw_indices(source: Source) -> np.ndarray:
    # All source samples remain in the plotted polyline. These indices are only
    # used to place a few orientation arrows away from the strictly static holds.
    moving = np.flatnonzero(np.linalg.norm(np.gradient(source.position_m, source.time_s,
                                                     axis=0), axis=1) > 1e-7)
    if len(moving) == 0:
        return np.asarray([], dtype=int)
    return moving[np.linspace(0, len(moving) - 1, 5).round().astype(int)]


def short_title(source: Source, number: int) -> str:
    family = str(source.metadata.get("family_id", source.metadata.get("family", "Open route")))
    family = family.replace("_", " ").capitalize()
    h = source.display_mm[:, 2]
    height = f"h = {h[0]:.0f} mm" if source.fixed_height else f"h: {h[0]:.0f} -> {h[-1]:.0f} mm"
    return f"{number:02d}  {family}\n{height} | {source.time_s[-1]:.1f} s"


def save_figure(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=170, facecolor="white")
    plt.close(fig)


def plot_overview(sources: list[Source], output: Path, title: str) -> list[str]:
    rows = math.ceil(len(sources) / 3)
    fig = plt.figure(figsize=(16.0, 5.1 * rows + 0.7), constrained_layout=True)
    axes = [fig.add_subplot(rows, 3, i + 1, projection="3d") for i in range(len(sources))]
    norm = colors.Normalize(vmin=0.0, vmax=100.0)
    all_points = np.concatenate([s.display_mm for s in sources])
    xy_center = (all_points[:, :2].min(axis=0) + all_points[:, :2].max(axis=0)) / 2
    h_center = float((all_points[:, 2].min() + all_points[:, 2].max()) / 2)
    # Identical physical scale within each overview; one mm is equally long on
    # X, Y and h. A flat route therefore remains flat rather than exaggerated.
    span = max(float(np.ptp(all_points[:, 0])), float(np.ptp(all_points[:, 1])),
               float(np.ptp(all_points[:, 2])), 100.0) * 1.15
    has_prototype = False
    for i, (ax, source) in enumerate(zip(axes, sources), 1):
        xyz = source.display_mm
        points = xyz.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = Line3DCollection(segments, cmap="viridis", norm=norm, linewidth=2.5)
        lc.set_array((source.time_s[:-1] + source.time_s[1:]) * 50.0 / source.time_s[-1])
        ax.add_collection3d(lc)
        prototype = source.metadata.get("curve", {}).get("prototype_control_points_m")
        if prototype is not None:
            proto = bernstein_curve(prototype) * np.array([1000.0, 1000.0, -1000.0])
            ax.plot(*proto.T, color="#aab0b7", linestyle="--", linewidth=1.15, alpha=0.8)
            has_prototype = True
        ax.scatter(*xyz[0], s=52, c=PALETTE["start"], marker="o", edgecolors="white",
                   linewidths=0.7, depthshade=False, zorder=7)
        ax.scatter(*xyz[-1], s=58, c=PALETTE["end"], marker="X", edgecolors="white",
                   linewidths=0.7, depthshade=False, zorder=7)
        indices = draw_indices(source)
        if len(indices):
            axes_display = source.rod_axis_xyz[indices] * np.array([1.0, 1.0, -1.0])
            ax.quiver(*xyz[indices].T, *axes_display.T, length=16.0, normalize=False,
                      color=PALETTE["axis"], linewidth=0.9, arrow_length_ratio=0.22)
        ax.set_xlim(xy_center[0] - span / 2, xy_center[0] + span / 2)
        ax.set_ylim(xy_center[1] - span / 2, xy_center[1] + span / 2)
        ax.set_zlim(h_center - span / 2, h_center + span / 2)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=24, azim=-55)
        ax.set_xlabel("plane1 X (mm)", labelpad=5)
        ax.set_ylabel("plane1 Y (mm)", labelpad=5)
        ax.set_zlabel("Height h = -Z (mm)", labelpad=6)
        ax.set_title(short_title(source, i), fontsize=11.5, pad=10)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.2)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor((0.97, 0.98, 0.99, 0.45))
    fig.suptitle(title + "\nOpen paths; no trajectory is repeated", fontsize=17)
    handles = [Line2D([0], [0], linestyle="", marker="o", markersize=7, color=PALETTE["start"], label="Start"),
               Line2D([0], [0], linestyle="", marker="X", markersize=7, color=PALETTE["end"], label="End"),
               Line2D([0], [0], color=PALETTE["axis"], label="Tool +Z axis (16 mm arrows)")]
    if has_prototype:
        handles.append(Line2D([0], [0], color="#aab0b7", linestyle="--", label="Unwarped prototype (reference only)"))
    fig.legend(handles=handles, loc="outside lower center", ncol=len(handles), frameon=False, fontsize=10)
    fig.colorbar(ScalarMappable(norm=norm, cmap="viridis"), ax=axes,
                 location="right", shrink=0.63, aspect=28, pad=0.01,
                 label="Authoritative elapsed time (%)")
    save_figure(fig, output)
    return [s.trajectory_id for s in sources]


def motion_profiles(source: Source) -> dict[str, np.ndarray]:
    velocity = np.gradient(source.position_m, source.time_s, axis=0, edge_order=2)
    rotations = Rotation.from_quat(source.quaternion_xyzw)
    increments = rotations[:-1].inv() * rotations[1:]
    omega_interval = np.linalg.norm(increments.as_rotvec(), axis=1) / np.diff(source.time_s)
    omega = np.interp(source.time_s, (source.time_s[:-1] + source.time_s[1:]) / 2, omega_interval)
    rod_axis = source.rod_axis_xyz
    tilt = np.rad2deg(np.arccos(np.clip(rod_axis[:, 2], -1.0, 1.0)))
    # Rz(yaw) Ry(pitch) Rx(roll): yaw = atan2(R[1,0], R[0,0]).
    matrix = rotations.as_matrix()
    yaw = np.rad2deg(np.unwrap(np.arctan2(matrix[:, 1, 0], matrix[:, 0, 0])))
    azimuth = np.rad2deg(np.unwrap(np.arctan2(rod_axis[:, 1], rod_axis[:, 0])))
    azimuth[tilt < 0.1] = np.nan  # A nearly vertical axis has no useful azimuth.
    return {"speed_mm_s": np.linalg.norm(velocity, axis=1) * 1000,
            "angular_speed_deg_s": np.rad2deg(omega), "rod_tilt_deg": tilt,
            "tool_yaw_deg": yaw, "rod_azimuth_deg": azimuth}


def plot_profiles(sources: list[Source], output: Path) -> list[str]:
    selected = [sources[0]]
    full = [s for s in sources if not s.fixed_height]
    if full:
        selected.append(full[0])
        if len(full) > 1:
            selected.append(full[-1])
    if len(selected) < 3:
        selected += [s for s in sources if s not in selected][:3 - len(selected)]
    fig, axes = plt.subplots(4, len(selected), figsize=(15.5, 10.8),
                             constrained_layout=True, sharex="col", squeeze=False)
    keys = ["speed_mm_s", "angular_speed_deg_s", "rod_tilt_deg", "rod_azimuth_deg"]
    labels = ["Linear speed (mm/s)", "Angular speed (deg/s)", "Rod-axis tilt (deg)", "Rod azimuth in XY (deg)"]
    for col, source in enumerate(selected):
        metrics = motion_profiles(source)
        for row, (key, label) in enumerate(zip(keys, labels)):
            ax = axes[row, col]
            ax.plot(source.time_s, metrics[key], color=["#18734c", "#147d92", "#7157a4", "#a75250"][row], linewidth=1.7)
            static = np.char.find(source.phase.astype(str), "hold") >= 0
            if row == 0:
                draw_intervals = (~static[:-1]) | (~static[1:])
                draw_time = float(np.diff(source.time_s)[draw_intervals].sum())
                length_mm = float(np.linalg.norm(np.diff(source.position_m, axis=0), axis=1).sum() * 1000)
                if draw_time > 0:
                    ax.axhline(length_mm / draw_time, color="#738575", linestyle="--", linewidth=1,
                               label="Draw-phase mean (statistic)")
                    ax.legend(loc="upper right", frameon=False, fontsize=8)
            if row == 3 and not np.isfinite(metrics[key]).any():
                ax.text(0.5, 0.5, "Near-vertical rod; azimuth undefined", transform=ax.transAxes,
                        ha="center", va="center", fontsize=9, color="#637080")
            edges = np.flatnonzero(np.diff(np.r_[False, static, False]))
            for begin, end in zip(edges[::2], edges[1::2]):
                ax.axvspan(source.time_s[begin], source.time_s[min(end, len(source.time_s) - 1)],
                           color="#d9dde2", alpha=0.7, zorder=-2)
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.2)
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_xlim(0, source.time_s[-1])
            if row != 3:
                ax.set_ylim(bottom=0)
            if (row == 1 and source.metadata.get("orientation", {}).get("mode") == "fixed"
                    and float(np.max(metrics[key])) < 1e-6):
                # Do not magnify floating-point q^-1*q residue into motion.
                ax.set_ylim(0, 1)
                ax.text(0.5, 0.55, "Fixed orientation (zero angular motion)",
                        transform=ax.transAxes, ha="center", color="#637080", fontsize=9)
            if row == 0:
                ax.set_title(short_title(source, sources.index(source) + 1), fontsize=11.5)
            if row == 3:
                ax.set_xlabel("Authoritative time (s)")
    fig.suptitle("Smooth, independently varied speed and tool orientation\n"
                 "Source CSV only; grey = static holds; dashed = statistical mean, not a timing baseline", fontsize=15)
    save_figure(fig, output)
    return [s.trajectory_id for s in selected]


def render(batch: Path, expected_count: int) -> dict:
    if "continuous_augmented" not in batch.parts:
        raise ValueError("Only the new continuous_augmented bank may be previewed here")
    manifest_hash = sha256(batch / "manifest.json")
    manifest, sources = load_sources(batch, expected_count)
    plots = batch / "plots"
    plots.mkdir(exist_ok=True)
    fixed = [s for s in sources if s.fixed_height]
    full = [s for s in sources if not s.fixed_height]
    if not fixed or not full:
        raise ValueError("Preview expects both fixed-height and one-way height routes")
    outputs = {}
    output_specs = ((fixed, "fixed_height_overview.png", "Fixed-height routes | smooth spatial deformation"),
                    (full, "one_way_full_height_overview.png", "One-way 70-200 mm routes | smooth spatial deformation"))
    for subset, filename, title in output_specs:
        outputs[filename] = {"trajectory_ids": plot_overview(subset, plots / filename, title),
                             "sha256": sha256(plots / filename)}
    filename = "speed_orientation_profiles.png"
    outputs[filename] = {"trajectory_ids": plot_profiles(sources, plots / filename),
                         "sha256": sha256(plots / filename)}
    index = ("<!doctype html><meta charset='utf-8'><title>Open trajectory previews</title>"
             "<style>body{font:16px system-ui;max-width:1700px;margin:2rem auto;padding:0 1rem}"
             "img{width:100%;height:auto}figure{margin:2rem 0}figcaption{color:#45546a}</style>"
             f"<h1>{html.escape(manifest.get('batch_id', batch.name))}</h1>"
             "<p>Authoritative CSV preview only; not robot execution validation. "
             "Display height h = -plane1 Z (mm). Dashed grey curves are unwarped "
             "prototypes, not alternative execution sources. "
             "<a href='plot_provenance.json'>Source and figure SHA256 provenance</a>.</p>")
    for filename, caption in (("fixed_height_overview.png", "Six fixed-height, open routes."),
                              ("one_way_full_height_overview.png", "Six one-way routes spanning 70-200 mm."),
                              ("speed_orientation_profiles.png", "Real CSV speed and tool-axis profiles; mean speed is only a statistic.")):
        index += f"<figure><img src='{filename}' alt='{caption}'><figcaption>{caption}</figcaption></figure>"
    (plots / "index.html").write_text(index + "\n")
    outputs["index.html"] = {"trajectory_ids": [s.trajectory_id for s in sources],
                             "sha256": sha256(plots / "index.html")}
    if sha256(batch / "manifest.json") != manifest_hash:
        raise ValueError("Manifest changed during rendering; regenerate the preview")
    for source in sources:
        if (sha256(source.csv_path) != source.csv_sha256 or
                sha256(source.metadata_path) != source.metadata_sha256):
            raise ValueError(f"Authority changed during rendering: {source.trajectory_id}")
    provenance = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "batch_id": manifest.get("batch_id", batch.name),
        "status": "csv_visualization_only_not_robot_execution_validation",
        "script": "continuous_augmented/plot_preview.py",
        "script_sha256": sha256(Path(__file__)),
        "manifest_sha256": manifest_hash,
        "display_frame": "plane1",
        "display_coordinates": "[X_mm, Y_mm, height_mm=-1000*tip_z_m]",
        "orientation_arrows": "tool +Z = Rotation(q_xyzw)*[0,0,1]; display arrow [ax,ay,-az]",
        "time_color": "original time_s divided by original final time_s",
        "profile_method": "finite differences of source XYZ and SO(3) increments; display diagnostics only",
        "all_csv_vertices_used_in_trajectory_polylines": True,
        "source_files_modified": False,
        "fixed_height_count": len(fixed),
        "full_height_count": len(full),
        "sources": [{"trajectory_id": s.trajectory_id, "csv_file": s.record["csv_file"],
                     "metadata_file": s.record["metadata_file"], "csv_sha256": s.csv_sha256,
                     "metadata_sha256": s.metadata_sha256, "samples": len(s.time_s),
                     "duration_s": float(s.time_s[-1]),
                     "height_mm": [float(s.display_mm[:, 2].min()), float(s.display_mm[:, 2].max())],
                     "endpoint_distance_mm": float(np.linalg.norm(s.position_m[-1] - s.position_m[0]) * 1000)}
                    for s in sources],
        "outputs": outputs,
    }
    (plots / "plot_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return {"batch": str(batch), "outputs": [str(plots / name) for name in outputs],
            "provenance": str(plots / "plot_provenance.json"),
            "sources_verified": len(sources)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--expected-count", type=int, default=12)
    args = parser.parse_args()
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.titleweight": "bold", "axes.edgecolor": "#637080"})
    print(json.dumps(render(args.batch.resolve(), args.expected_count), indent=2))


if __name__ == "__main__":
    main()
