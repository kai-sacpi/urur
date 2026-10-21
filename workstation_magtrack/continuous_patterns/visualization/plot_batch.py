#!/usr/bin/env python3
"""Render a frozen continuous_patterns batch without changing its authorities.

Usage:
    python visualization/plot_batch.py --batch batches/BATCH_ID

The only output directory is BATCH/plots. All geometry figures read the exact
CSV positions; stride reduction is confined to drawing pixels. The default
motion profiles use the offline analytic source and label it explicitly. A
CSV finite-difference fallback is illustrative and is never a certificate.
No ROS, robot, GPU, network, runtime resampling, or playback is used here.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
import math
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


SOURCE_ROOT = Path(__file__).resolve().parents[1]
HEIGHT_NORM = Normalize(70.0, 200.0)
CMAP = plt.get_cmap("viridis")
COLORS = plt.get_cmap("tab10").colors
GROUP_LABELS = {
    "stratified": "Fixed height layers",
    "full_height": "Full height · 70–200 mm",
    "full_height_reverse": "Full height reverse · 70–200 mm",
    "guarded_full_height": "Separate companion · 75–200 mm",
}
FAMILY_LABELS = {
    "woven_lissajous": "Woven Lissajous",
    "trefoil_projection": "Trefoil projection",
    "torus_knot_2_5": "Torus knot 2:5",
    "hypotrochoid_3_2": "Hypotrochoid 3:2",
    "braided_spiral": "Braided spiral",
    "spherical_sweep": "Spherical sweep",
    "compound_harmonograph": "Compound harmonograph",
    "bent_tube_braid": "Bent tube braid",
    "rounded_square_shear": "Rounded square shear",
    "celtic_weave": "Celtic weave",
}
FOOTER = (
    "PLANNED nominal rod_tip · magnet/axis calibration pending · plane1: h = −z · "
    "190 mm rod · CPU source view; robot simulation NOT RUN"
)
CSV_COLUMNS = ("time_s", "tip_x_m", "tip_y_m", "tip_z_m", "qx", "qy", "qz", "qw")


@dataclass
class Record:
    entry: dict[str, Any]
    meta: dict[str, Any]
    csv_path: Path
    metadata_path: Path
    stem: str
    summary: dict[str, Any] = field(default_factory=dict)
    audit: dict[str, Any] = field(default_factory=dict)

    @property
    def ident(self) -> str:
        return str(self.meta["trajectory_id"])

    @property
    def group(self) -> str:
        return str(self.meta.get("group_id", "unclassified"))

    @property
    def family(self) -> str:
        return str(self.meta.get("family_id", "unknown"))

    @property
    def constant(self) -> bool:
        return self.meta["height_profile"]["mode"] == "constant"

    @property
    def lower(self) -> float:
        return float(self.meta["height_profile"]["lower_m"])

    @property
    def upper(self) -> float:
        return float(self.meta["height_profile"]["upper_m"])


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def batch_file(batch: Path, relative: str) -> Path:
    path = (batch / relative).resolve()
    if not path.is_relative_to(batch):
        raise ValueError(f"manifest path leaves the batch: {relative}")
    return path


def load_records(batch: Path) -> tuple[dict[str, Any], list[Record]]:
    manifest = json.loads((batch / "manifest.json").read_text(encoding="utf-8"))
    entries = manifest.get("records", manifest.get("trajectories", []))
    if not entries:
        raise ValueError("manifest has no records; finish generation before rendering")
    records = []
    stems = set()
    audit_path = batch / "reports" / "source_audit.json"
    audit_records = {}
    if audit_path.exists():
        audit_report = json.loads(audit_path.read_text(encoding="utf-8"))
        audit_records = {item["trajectory_id"]: item for item in audit_report.get("trajectories", [])}
    for entry in entries:
        csv_path = batch_file(batch, entry["csv_file"])
        metadata_path = batch_file(batch, entry["metadata_file"])
        for path, key in ((csv_path, "sha256"), (metadata_path, "metadata_sha256")):
            expected = entry.get(key)
            if expected and digest(path) != expected:
                raise ValueError(f"authority hash mismatch for {path}")
        meta = json.loads(metadata_path.read_text(encoding="utf-8"))
        stem = re.sub(r"[^a-zA-Z0-9_.-]+", "_", str(meta["trajectory_id"]))
        if stem in stems:
            raise ValueError(f"duplicate figure stem: {stem}")
        stems.add(stem)
        record = Record(entry, meta, csv_path, metadata_path, stem)
        if record.ident in audit_records:
            audit = audit_records[record.ident]
            authority = audit.get("authority", {})
            if authority.get("sha256") != entry.get("sha256") or authority.get("metadata_sha256") != entry.get("metadata_sha256"):
                raise ValueError(f"source audit is not bound to the current authority: {record.ident}")
            record.audit = audit
        records.append(record)
    return manifest, records


def load_csv(record: Record) -> np.ndarray:
    with record.csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        indexes = [header.index(name) for name in CSV_COLUMNS]
        data = np.asarray([[float(row[i]) for i in indexes] for row in reader], dtype=float)
    if len(data) < 3 or not np.all(np.isfinite(data)) or not np.all(np.diff(data[:, 0]) > 0):
        raise ValueError(f"invalid finite, increasing CSV for plotting: {record.csv_path}")
    return data


def get_status(record: Record) -> dict[str, str]:
    meta = record.meta
    collision = record.audit.get("nominal_geometry_precheck") or next((meta[key] for key in (
        "rod_collision_precheck", "rod_precheck", "clearance_precheck", "clearance", "collision_precheck"
    ) if isinstance(meta.get(key), dict)), {})
    rod = str(collision.get("status", meta.get("rod_precheck_status", "not_reported")))
    source = meta.get("source_validation_status", meta.get("source_validation",
                      meta.get("source_design_status", "not_reported")))
    if isinstance(source, dict):
        source = source.get("status", "not_reported")
    if record.audit:
        source = "source_audit_passed" if record.audit.get("source_level_pass") else "source_audit_failed"
    # A retained 70 mm design is never shown as released, even before a report
    # has been attached. The figure makes the unchanged-guard gap explicit.
    if "reject" in rod.lower():
        status = "rod_rejected"
        label = "ROD ENVELOPE REJECTED · arm/IK pending"
    elif record.lower < 0.075 - 1e-12:
        status = "70mm_guard_gap"
        label = "70 mm CANDIDATE · guard gap / rejection retained"
    elif "pass" in rod.lower():
        status = "rod_subset_only_arm_pending"
        label = "ROD SUBSET ONLY · arm/IK and robot simulation pending"
    else:
        status = "precheck_pending"
        label = "PRECHECK PENDING · robot simulation NOT RUN"
    return {"status": status, "label": label, "source": str(source), "rod": rod,
            "robot_simulation": "not_run", "magnet_calibration": "not_accepted"}


def display_height(record: Record) -> str:
    lo, hi = 1000 * record.lower, 1000 * record.upper
    return f"{lo:g} mm layer" if record.constant else f"{lo:g}–{hi:g} mm sweep"


def choose_representative(records: list[Record]) -> Record:
    return min(records, key=lambda r: (
        r.meta.get("v_tier") != "V2", r.meta.get("orientation_mode") != "fixed",
        r.meta.get("direction") == "reverse", r.ident))


def subset_for_plot(data: np.ndarray, maximum: int = 3000) -> np.ndarray:
    if len(data) <= maximum:
        return data
    # Display-only thinning. Neither times nor CSVs are ever written here.
    indexes = np.unique(np.r_[np.linspace(0, len(data) - 1, maximum).astype(int), len(data) - 1])
    return data[indexes]


def configure_3d(ax: Any, *, compact: bool = False) -> None:
    ax.set_xlim(-130, 130)
    ax.set_ylim(-130, 130)
    ax.set_zlim(70, 200)
    # Identical millimeter scale in x, y and h, across every figure.
    ax.set_box_aspect((260, 260, 130))
    ax.view_init(elev=27, azim=-55)
    ticks = [-120, 0, 120] if compact else [-120, -60, 0, 60, 120]
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_zticks([70, 100, 150, 200])
    font = 7 if compact else 9
    pad = -1 if compact else 2
    ax.set_xlabel("x (mm)", fontsize=font, labelpad=pad)
    ax.set_ylabel("y (mm)", fontsize=font, labelpad=pad)
    ax.set_zlabel("h = −z (mm)", fontsize=font, labelpad=1 if compact else 4)
    ax.tick_params(labelsize=font, pad=0)
    ax.grid(True, alpha=0.2)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((0.97, 0.98, 0.99, 1))


def draw_curve(ax: Any, data: np.ndarray, *, color: Any = None, label: str | None = None,
               linewidth: float = 1.5, endpoints: bool = True) -> None:
    sampled = subset_for_plot(data)
    xyz = sampled[:, 1:4] * [1000., 1000., -1000.]
    if color is not None:
        ax.plot(*xyz.T, color=color, linewidth=linewidth, label=label)
    else:
        segments = np.stack([xyz[:-1], xyz[1:]], axis=1)
        collection = Line3DCollection(segments, cmap=CMAP, norm=HEIGHT_NORM, linewidths=linewidth)
        collection.set_array((xyz[:-1, 2] + xyz[1:, 2]) / 2)
        ax.add_collection3d(collection)
    if endpoints:
        ax.scatter(*xyz[0], color="#152331", marker="o", s=13, depthshade=False, zorder=5)
        ax.scatter(*xyz[-1], color="#b32f36", marker="x", s=24, depthshade=False, zorder=6)


def annotate_footer(fig: Any, text: str = FOOTER) -> None:
    fig.text(.5, .013, text, ha="center", va="bottom", fontsize=7, color="#526174")


def save_figure(fig: Any, target: Path, *, pdf: bool = False, dpi: int = 160) -> None:
    fig.savefig(target.with_suffix(".png"), dpi=dpi, facecolor="white", bbox_inches="tight")
    if pdf:
        fig.savefig(target.with_suffix(".pdf"), facecolor="white", bbox_inches="tight",
                    metadata={"CreationDate": None, "ModDate": None,
                              "Creator": "continuous_patterns offline renderer"})
    plt.close(fig)


def plot_trajectory(record: Record, data: np.ndarray, output: Path, pdf: bool) -> None:
    fig = plt.figure(figsize=(7.1, 6.2))
    ax = fig.add_axes([.025, .15, .78, .68], projection="3d")
    configure_3d(ax)
    draw_curve(ax, data, linewidth=1.6)
    cax = fig.add_axes([.875, .31, .021, .40])
    colorbar = fig.colorbar(ScalarMappable(norm=HEIGHT_NORM, cmap=CMAP), cax=cax)
    colorbar.set_label("Nominal height (mm)", fontsize=9)
    colorbar.ax.tick_params(labelsize=8)
    fig.suptitle(FAMILY_LABELS.get(record.family, record.family), y=.975, fontsize=12, fontweight="bold")
    fig.text(.5, .895, f"{record.ident}\n{display_height(record)} · "
             f"{record.meta.get('v_tier', '')} · {record.meta.get('orientation_mode', '')} pose · "
             f"{record.meta.get('direction', 'forward')}", ha="center", fontsize=8)
    status = get_status(record)
    color = "#a62b35" if status["status"] in ("rod_rejected", "70mm_guard_gap") else "#8a5b0b"
    fig.text(.5, .092, status["label"], color=color, fontsize=8, ha="center", fontweight="bold")
    nature = "Fixed height: planar centerline; torsion is zero." if record.constant else "Full height: continuous descent and ascent; physical axis scales match."
    fig.text(.5, .062, nature + "  Start ○  End ×", ha="center", fontsize=7.8)
    annotate_footer(fig)
    save_figure(fig, output / "trajectories" / record.stem, pdf=pdf, dpi=170)


def resolve_source() -> tuple[Any, str]:
    sys.path.insert(0, str(SOURCE_ROOT))
    try:
        return importlib.import_module("src.trajectory"), "analytic offline source"
    except ImportError as error:
        return None, f"CSV finite differences, illustrative only ({type(error).__name__})"


def motion_profile(record: Record, data: np.ndarray, source: Any) -> dict[str, Any]:
    meta = record.meta
    start = float(meta["draw_start_s"])
    end = float(meta.get("draw_end_s", start + meta["draw_duration_s"]))
    duration = float(meta["draw_duration_s"])
    if source is not None:
        t = np.unique(np.r_[data[0, 0], start, np.linspace(start, end, 4097), end, data[-1, 0]])
        s = np.clip((t - start) / duration, 0., 1.)
        derivatives = source.position_derivatives(meta, s)
        angular = source.orientation_diagnostics(meta, s)
        q = source.orientation(meta, s)
        velocity, acceleration, jerk = [np.linalg.norm(derivatives[d], axis=1) / duration**d for d in (1, 2, 3)]
        omega, alpha, angular_jerk = [np.rad2deg(np.linalg.norm(angular[key], axis=1))
                                     for key in ("omega_rad_s", "alpha_rad_s2", "jerk_rad_s3")]
        outside = (t <= start) | (t >= end)
        for values in (velocity, acceleration, jerk, omega, alpha, angular_jerk):
            values[outside] = 0.
        method = "analytic offline source samples; continuous certificates remain in metadata"
        sampled = subset_for_plot(data, 4097)
        compare_s = np.clip((sampled[:, 0] - start) / duration, 0., 1.)
        p_expected = source.position_derivatives(meta, compare_s)[0]
        q_expected = source.orientation(meta, compare_s)
        q_csv = sampled[:, 4:8] / np.linalg.norm(sampled[:, 4:8], axis=1)[:, None]
        comparison = {"sampled_csv_rows_compared": len(sampled),
                      "max_abs_position_source_csv_difference_m": float(np.max(np.abs(p_expected - sampled[:, 1:4]))),
                      "max_quaternion_abs_dot_difference": float(np.max(np.abs(1. - np.abs(np.sum(q_expected * q_csv, axis=1))))),
                      "scope": "sampled display cross-check only, not the source validation certificate"}
    else:
        t, p, q = data[:, 0], data[:, 1:4], data[:, 4:8]
        v = np.gradient(p, t, axis=0, edge_order=2)
        a = np.gradient(v, t, axis=0, edge_order=2)
        j = np.gradient(a, t, axis=0, edge_order=2)
        q = q / np.linalg.norm(q, axis=1)[:, None]
        q = q * np.where(np.r_[1., np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1., 1.))] < 0, -1., 1.)[:, None]
        dq = np.gradient(q, t, axis=0, edge_order=2)
        # Norm of body/spatial omega is 2*|qdot| for a unit quaternion.
        omega = np.rad2deg(2 * np.linalg.norm(dq, axis=1))
        alpha = np.abs(np.gradient(omega, t, edge_order=2))
        angular_jerk = np.abs(np.gradient(np.gradient(omega, t, edge_order=2), t, edge_order=2))
        velocity, acceleration, jerk = [np.linalg.norm(value, axis=1) for value in (v, a, j)]
        method = "CSV finite-difference norms; illustrative only, not full-vector angular certificates"
        comparison = {"scope": "CSV fallback; no analytic source cross-check"}
    angle = np.rad2deg(2 * np.arccos(np.clip(np.abs(q @ q[0]), 0., 1.)))
    return {"time": t, "speed": velocity, "acceleration": acceleration, "jerk": jerk,
            "angle": angle, "omega": omega, "alpha": alpha, "angular_jerk": angular_jerk,
            "method": method, "source_csv_comparison": comparison}


def plot_profile(record: Record, data: np.ndarray, profile: dict[str, Any], output: Path, pdf: bool) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(11, 10), sharex=True)
    fig.subplots_adjust(left=.09, right=.97, bottom=.13, top=.88, hspace=.29, wspace=.28)
    t = profile["time"]
    meta = record.meta
    start = float(meta["draw_start_s"])
    end = float(meta.get("draw_end_s", start + meta["draw_duration_s"]))
    speed_limit = {"V1": 10., "V2": 20., "V3": 50.}.get(str(meta.get("v_tier")))
    plots = [
        (axes[0, 0], data[:, 0], -1000 * data[:, 3], "Nominal height (mm)", None, True),
        (axes[1, 0], t, 1000 * profile["speed"], "Speed norm (mm/s)", speed_limit, False),
        (axes[2, 0], t, 1000 * profile["acceleration"], "Acceleration norm (mm/s²)", 50., False),
        (axes[3, 0], t, 1000 * profile["jerk"], "Jerk norm (mm/s³)", 250., False),
        (axes[0, 1], t, profile["angle"], "Angle from start (deg)", None, False),
        (axes[1, 1], t, profile["omega"], "Angular speed norm (deg/s)", None, False),
        (axes[2, 1], t, profile["alpha"], "Angular acceleration (deg/s²)", 30., False),
        (axes[3, 1], t, profile["angular_jerk"], "Angular jerk (deg/s³)", 150., False),
    ]
    for ax, tx, values, ylabel, limit, is_height in plots:
        plotted = subset_for_plot(np.c_[tx, values], 5000)
        ax.plot(plotted[:, 0], plotted[:, 1], color="#24578a", linewidth=.95)
        ax.axvspan(data[0, 0], start, color="#b8c4d0", alpha=.25)
        ax.axvspan(end, data[-1, 0], color="#b8c4d0", alpha=.25)
        ax.axvline(start, color="#94a3b8", linewidth=.6, linestyle=":")
        ax.axvline(end, color="#94a3b8", linewidth=.6, linestyle=":")
        if limit is not None:
            ax.axhline(limit, color="#af343a", linewidth=.8, linestyle="--")
            ax.text(.995, .91, f"limit {limit:g}", transform=ax.transAxes, ha="right", fontsize=7.5, color="#af343a")
            ax.set_ylim(bottom=0., top=max(limit * 1.12, float(np.max(values)) * 1.15, .1))
        elif is_height:
            ax.set_ylim(65, 205)
            ax.axhspan(150, 200, color="#dfead0", alpha=.4)
        else:
            ax.set_ylim(bottom=0., top=max(float(np.max(values)) * 1.15, .1))
        if not is_height:
            ax.text(.012, .91, f"display max {float(np.max(values)):.4g}",
                    transform=ax.transAxes, fontsize=7, color="#526174")
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=.22)
        ax.tick_params(labelsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Authoritative time_s (s)", fontsize=9)
    fig.suptitle(f"{record.ident}\n{display_height(record)} · {meta.get('v_tier', '')} · {meta.get('orientation_mode', '')} pose",
                 fontsize=11, y=.97)
    fig.text(.5, .91, profile["method"], ha="center", fontsize=8, color="#526174")
    fig.text(.5, .062, "Height: original CSV samples. Gray: endpoint hold. Norms include coupled vector components; curves are display samples.",
             ha="center", fontsize=7.8)
    annotate_footer(fig)
    save_figure(fig, output / "profiles" / record.stem, pdf=pdf, dpi=145)


def summarize_csv(record: Record, data: np.ndarray) -> None:
    start = float(record.meta["draw_start_s"])
    end = float(record.meta.get("draw_end_s", start + record.meta["draw_duration_s"]))
    dt = np.maximum(0., np.minimum(data[1:, 0], end) - np.maximum(data[:-1, 0], start))
    h = -500 * (data[:-1, 3] + data[1:, 3])
    xy = 500 * (data[:-1, 1:3] + data[1:, 1:3])
    # Time-weighted interval midpoints are a descriptive CSV occupancy estimate.
    height_time, _ = np.histogram(h, bins=np.arange(70., 200.1, 10.), weights=dt)
    xy_time, _, _ = np.histogram2d(xy[:, 0], xy[:, 1], bins=np.linspace(-130, 130, 27), weights=dt)
    status = get_status(record)
    record.summary.update({
        "trajectory_id": record.ident, "group_id": record.group, "family_id": record.family,
        "family_label": FAMILY_LABELS.get(record.family, record.family),
        "geometry_id": record.meta.get("geometry_id", record.ident),
        "height": display_height(record), "height_mode": "constant" if record.constant else "sweep",
        "lower_height_mm": 1000 * record.lower, "upper_height_mm": 1000 * record.upper,
        "v_tier": record.meta.get("v_tier", ""), "orientation_mode": record.meta.get("orientation_mode", ""),
        "direction": record.meta.get("direction", "forward"), "w_tier": record.meta.get("w_tier", ""),
        "draw_duration_s": float(record.meta["draw_duration_s"]), "csv_sample_count": len(data),
        "csv_time_span_s": [float(data[0, 0]), float(data[-1, 0])],
        "csv_min_height_mm": float(-1000 * data[:, 3].max()),
        "csv_max_height_mm": float(-1000 * data[:, 3].min()),
        "height_time_bin_edges_mm": list(range(70, 201, 10)),
        "draw_time_by_height_bin_s": height_time.tolist(),
        "xy_draw_time_bin_s": xy_time.tolist(),
        "status": status["status"], "status_label": status["label"],
        "source_status": status["source"], "rod_status": status["rod"],
        "robot_simulation": "not_run", "reference_center": "PLANNED nominal rod_tip; calibration pending",
        "csv_href": "../" + record.entry["csv_file"],
        "metadata_href": "../" + record.entry["metadata_file"],
        "csv_sha256": record.entry.get("sha256", digest(record.csv_path)),
        "metadata_sha256": record.entry.get("metadata_sha256", digest(record.metadata_path)),
        "plot_href": "trajectories/" + record.stem + ".png",
        "profile_href": "profiles/" + record.stem + ".png",
        "seed": record.meta.get("seed", record.meta.get("geometry_seed", "see metadata")),
        "xy": record.meta.get("xy", {}),
    })


def plot_layer_overview(records: list[Record], output: Path) -> str | None:
    by_height: dict[float, list[Record]] = defaultdict(list)
    for record in records:
        if record.constant:
            by_height[round(record.lower, 9)].append(record)
    if not by_height:
        return None
    ncols = 4
    nrows = math.ceil(len(by_height) / ncols)
    fig = plt.figure(figsize=(14.4, nrows * 3.25 + 1.1))
    grid = fig.add_gridspec(nrows, ncols, left=.025, right=.97, bottom=.07, top=.89, wspace=.08, hspace=.20)
    for index, (height, pool) in enumerate(sorted(by_height.items())):
        ax = fig.add_subplot(grid[index // ncols, index % ncols], projection="3d")
        configure_3d(ax, compact=True)
        geometries: dict[str, list[Record]] = defaultdict(list)
        for record in pool:
            geometries[str(record.meta.get("geometry_id", record.family))].append(record)
        selected = [choose_representative(items) for _, items in sorted(geometries.items())]
        names = []
        for j, record in enumerate(selected):
            draw_curve(ax, load_csv(record), color=COLORS[j % len(COLORS)], linewidth=1.0, endpoints=False)
            names.append(FAMILY_LABELS.get(record.family, record.family))
        suffix = " · GUARD REJECT" if height < .075 - 1e-12 else ""
        ax.set_title(f"{height * 1000:g} mm{suffix}\n" + " / ".join(names), fontsize=7.7,
                     color="#a62b35" if suffix else "#23334a", pad=0)
    fig.suptitle("Fixed height layers · independent randomized XY geometries at every height", fontsize=15, y=.972, fontweight="bold")
    fig.text(.5, .93, "Each panel overlays unique geometry representatives (prefer V2, fixed pose). Height is constant; centerlines are planar and torsion is zero.\n"
             "All axes use the same physical millimeter scale and the same 70–200 mm display range. 70 mm candidates remain rejected/pending release.",
             ha="center", fontsize=9, color="#526174")
    annotate_footer(fig)
    name = "fixed_height_layers_overview"
    save_figure(fig, output / name, pdf=True, dpi=180)
    return name + ".png"


def plot_full_overviews(records: list[Record], output: Path) -> list[str]:
    groups: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        if not record.constant:
            groups[record.group].append(record)
    names = []
    for group, pool in sorted(groups.items()):
        families: dict[str, list[Record]] = defaultdict(list)
        for record in pool:
            families[record.family].append(record)
        selected = [choose_representative(items) for _, items in sorted(families.items())]
        ncols, nrows = 4, math.ceil(len(selected) / 4)
        fig = plt.figure(figsize=(14.4, nrows * 3.65 + 1.15))
        grid = fig.add_gridspec(nrows, ncols, left=.025, right=.92, bottom=.075, top=.88,
                                wspace=.08, hspace=.20)
        for index, record in enumerate(selected):
            ax = fig.add_subplot(grid[index // ncols, index % ncols], projection="3d")
            configure_3d(ax, compact=True)
            draw_curve(ax, load_csv(record), linewidth=1.05)
            ax.set_title(f"{FAMILY_LABELS.get(record.family, record.family)}\n"
                         f"{record.meta.get('v_tier', '')} · {record.meta.get('orientation_mode', '')} pose · "
                         f"{record.meta.get('draw_duration_s', 0):.1f} s draw", fontsize=8.3, pad=0)
        cax = fig.add_axes([.95, .26, .012, .43])
        cbar = fig.colorbar(ScalarMappable(norm=HEIGHT_NORM, cmap=CMAP), cax=cax)
        cbar.set_label("Nominal height (mm)", fontsize=9)
        cbar.ax.tick_params(labelsize=8)
        fig.suptitle(GROUP_LABELS.get(group, group) + " · geometry overview", fontsize=15, y=.97, fontweight="bold")
        lower = min(r.lower for r in pool) * 1000
        gate = "70 mm designs retained with guard rejection." if lower < 75 - 1e-8 else "75 mm companion is a separate group; arm and scene simulation remain pending."
        fig.text(.5, .925, "Same physical axis scales: x/y ±130 mm, h 70–200 mm. Color shows height; ○ start, × end.\n"
                 "One representative per family; prefer V2 and fixed pose. " + gate, ha="center", fontsize=9, color="#526174")
        annotate_footer(fig)
        name = re.sub(r"[^a-zA-Z0-9_.-]+", "_", group) + "_overview"
        save_figure(fig, output / name, pdf=True, dpi=180)
        names.append(name + ".png")
    return names


def plot_coverage(records: list[Record], output: Path) -> str:
    fig, axes = plt.subplots(2, 2, figsize=(13.3, 9.4))
    fig.subplots_adjust(left=.08, right=.97, bottom=.11, top=.88, hspace=.40, wspace=.28)
    layers = [r for r in records if r.constant]
    heights = sorted(set(round(r.lower * 1000, 6) for r in layers))
    counts = [sum(abs(r.lower * 1000 - h) < 1e-4 for r in layers) for h in heights]
    unique = [len(set(str(r.meta.get("geometry_id", r.family)) for r in layers if abs(r.lower * 1000 - h) < 1e-4)) for h in heights]
    ax = axes[0, 0]
    ax.bar(np.asarray(heights) - 1.4, counts, width=2.8, color="#24578a", label="CSV authorities")
    ax.bar(np.asarray(heights) + 1.4, unique, width=2.8, color="#d0912f", label="Unique XY geometries")
    ax.set_xticks(heights)
    ax.tick_params(axis="x", labelrotation=45)
    ax.set_xlabel("Constant height (mm)")
    ax.set_ylabel("Count")
    ax.set_title("Fixed layers have their own trajectories", fontsize=11)
    ax.legend(fontsize=8, frameon=False)
    ax.grid(axis="y", alpha=.20)
    ax = axes[0, 1]
    groups: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        if not record.constant:
            groups[record.group].append(record)
    centers = np.arange(75, 196, 10)
    for index, (group, pool) in enumerate(sorted(groups.items())):
        dwell = np.sum([r.summary["draw_time_by_height_bin_s"] for r in pool], axis=0)
        fraction = 100. * dwell / dwell.sum()
        ax.plot(centers, fraction, marker="o", markersize=3, linewidth=1.2, color=COLORS[index],
                label=GROUP_LABELS.get(group, group))
    ax.axvspan(150, 200, color="#dfead0", alpha=.45)
    ax.axhline(12., color="#526174", linewidth=.7, linestyle="--", label="12% per high 1 cm bin (60% / 5)")
    ax.set_xlim(70, 200)
    ax.set_xlabel("Height bin midpoint (mm); 10 mm bins")
    ax.set_ylabel("Pooled draw time (%)")
    ax.set_title("Full height occupancy · CSV interval time weights", fontsize=11)
    ax.legend(fontsize=6.9, frameon=False)
    ax.grid(True, alpha=.20)
    ax = axes[1, 0]
    grid = np.sum([np.asarray(r.summary["xy_draw_time_bin_s"]) for r in records], axis=0)
    image = ax.imshow(np.log10(1. + grid.T), extent=(-130, 130, -130, 130), origin="lower",
                      cmap="cividis", aspect="equal")
    fig.colorbar(image, ax=ax, label="log10(1 + planned draw seconds)", shrink=.86)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("XY coverage · all planned CSVs", fontsize=11)
    ax = axes[1, 1]
    grouped: dict[str, Counter] = defaultdict(Counter)
    for record in records:
        grouped[record.group][f"{record.meta.get('v_tier', '')}/{record.meta.get('orientation_mode', '')}"] += 1
    labels = sorted(set(key for value in grouped.values() for key in value))
    xpos = np.arange(len(grouped))
    bottom = np.zeros(len(grouped))
    for index, label in enumerate(labels):
        values = np.array([value[label] for value in grouped.values()])
        ax.bar(xpos, values, bottom=bottom, label=label, color=COLORS[index % len(COLORS)])
        bottom += values
    ax.set_xticks(xpos, [name.replace("_", "\n") for name in grouped], fontsize=8)
    ax.set_ylabel("CSV authorities")
    ax.set_title("Speed tier and pose mode stay explicit", fontsize=11)
    ax.legend(fontsize=7, frameon=False, ncols=2)
    ax.grid(axis="y", alpha=.20)
    fig.suptitle("Coverage of the planned batch · layers and full height are separate designs", fontsize=14, y=.97, fontweight="bold")
    fig.text(.5, .925, "Time weights exclude endpoint holds. Pooled planned time includes pose/speed/direction controls; it is not recorded experimental data.",
             ha="center", fontsize=8.5, color="#526174")
    annotate_footer(fig)
    save_figure(fig, output / "coverage_summary", pdf=True, dpi=180)
    return "coverage_summary.png"


def plot_height_profiles(records: list[Record], output: Path) -> str:
    fig, axes = plt.subplots(1, 3, figsize=(15.1, 4.9))
    fig.subplots_adjust(left=.065, right=.98, bottom=.18, top=.75, wspace=.32)
    layers: dict[float, list[Record]] = defaultdict(list)
    for record in records:
        if record.constant:
            layers[record.lower].append(record)
    for height, pool in sorted(layers.items()):
        record = choose_representative(pool)
        data = subset_for_plot(load_csv(record), 2000)
        axes[0].plot(data[:, 0] - float(record.meta["draw_start_s"]), -1000 * data[:, 3],
                     color=CMAP(HEIGHT_NORM(height * 1000)), linewidth=1.0)
    axes[0].set_title("Fixed height layers · representative elapsed time", fontsize=10)
    axes[0].set_xlabel("Time relative to draw start (s)")
    axes[0].set_ylabel("Nominal height (mm)")
    full: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        if not record.constant and record.group == "full_height":
            full[record.family].append(record)
    for index, (family, pool) in enumerate(sorted(full.items())):
        record = choose_representative(pool)
        data = subset_for_plot(load_csv(record), 3000)
        axes[1].plot(data[:, 0] - float(record.meta["draw_start_s"]), -1000 * data[:, 3],
                     linewidth=.9, color=COLORS[index % len(COLORS)], label=FAMILY_LABELS.get(family, family))
    axes[1].set_title("Full height · V2 fixed pose, by family", fontsize=10)
    axes[1].set_xlabel("Time relative to draw start (s)")
    axes[1].legend(fontsize=5.8, frameon=False, ncols=2, loc="upper right")
    groups: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        if not record.constant:
            groups[record.group].append(record)
    for index, (group, pool) in enumerate(sorted(groups.items())):
        record = choose_representative(pool)
        data = subset_for_plot(load_csv(record), 3000)
        progress = (data[:, 0] - float(record.meta["draw_start_s"])) / float(record.meta["draw_duration_s"]) * 100
        axes[2].plot(progress, -1000 * data[:, 3], linewidth=1.2, color=COLORS[index],
                     linestyle="--" if "reverse" in group else "-", label=GROUP_LABELS.get(group, group))
    axes[2].set_xlim(-1, 101)
    axes[2].set_title("Sweep shape · normalized draw time", fontsize=10)
    axes[2].set_xlabel("Draw time (%)")
    axes[2].legend(fontsize=6.2, frameon=False)
    for ax in axes:
        ax.set_ylim(65, 205)
        ax.axhspan(150, 200, color="#dfead0", alpha=.35)
        ax.grid(True, alpha=.20)
        ax.tick_params(labelsize=8)
    fig.suptitle("Height versus authoritative time · separate fixed layers and whole-height sweeps", fontsize=14, y=.96, fontweight="bold")
    fig.text(.5, .84, "CSV positions and time_s are plotted unchanged. High band 150–200 mm is shaded; sweep design allocates 60% of draw time to its five bins.",
             ha="center", fontsize=8.5, color="#526174")
    annotate_footer(fig)
    save_figure(fig, output / "height_time_profiles", pdf=True, dpi=180)
    return "height_time_profiles.png"


HTML_TEMPLATE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Planned continuous trajectory gallery</title>
<style>
:root{--ink:#182b40;--muted:#5c6f80;--paper:#f3f5f7;--line:#d5dde5;--warn:#9c6417;--red:#9f2d39;--blue:#24578a}*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:14px/1.5 system-ui,-apple-system,sans-serif}header{background:#122b43;color:#fff;padding:32px max(24px,calc((100vw - 1500px)/2));}header p{color:#d7e4ef;max-width:1040px;margin:10px 0 0}h1{font-size:28px;line-height:1.2;margin:4px 0 14px;letter-spacing:-.4px}h2{font-size:20px;margin:0 0 14px}main{max-width:1548px;margin:auto;padding:25px 24px}.notice{padding:14px 18px;background:#fff6e6;border-left:4px solid #b27823;margin:0 0 22px}.stats{display:flex;gap:12px;flex-wrap:wrap;margin:18px 0 0}.stat{padding:7px 13px;border:1px solid #6b859d;border-radius:5px;color:#eef4f9}.overview{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:16px;margin-bottom:28px}.overview a{display:block;color:var(--ink);background:#fff;text-decoration:none;border:1px solid var(--line);border-radius:7px;overflow:hidden}.overview img{display:block;width:100%;aspect-ratio:1.3;object-fit:contain;background:#fff}.overview span{display:block;padding:9px 12px;border-top:1px solid var(--line);font-weight:600}.filters{position:sticky;top:0;z-index:2;padding:16px;background:#edf1f5ee;backdrop-filter:blur(8px);border:1px solid var(--line);border-radius:7px;display:flex;flex-wrap:wrap;gap:10px;margin-bottom:16px}.filters label{display:flex;flex-direction:column;font-size:11px;color:var(--muted);gap:3px;flex:1;min-width:135px}.filters label.search{flex:1.5;min-width:240px}select,input,button{font:inherit;color:var(--ink);border:1px solid #bdcad7;border-radius:4px;padding:7px;background:#fff}button{cursor:pointer}#count{margin:12px 0;color:var(--muted)}.gallery{display:grid;grid-template-columns:repeat(auto-fill,minmax(340px,1fr));gap:17px}.card{background:#fff;border:1px solid var(--line);border-radius:8px;overflow:hidden}.card>button{padding:0;width:100%;border:0;border-radius:0;display:block}.card img{width:100%;height:auto;display:block;aspect-ratio:1.14;object-fit:contain;background:#fff}.body{padding:15px}.body h3{font-size:15px;line-height:1.35;margin:0 0 8px}.id{font:11px ui-monospace,monospace;color:var(--muted);word-break:break-all}.tags{display:flex;gap:5px;flex-wrap:wrap;margin:10px 0}.tag{font-size:11px;background:#edf2f7;padding:3px 6px;border-radius:3px}.gate{font-size:11px;color:var(--warn);background:#fff7e7;padding:7px;border-radius:3px}.gate.reject{color:var(--red);background:#fff0f1}.links{display:flex;flex-wrap:wrap;gap:12px;margin-top:12px}.links a,a{color:var(--blue)}details{margin-top:10px;font-size:12px;color:var(--muted)}pre{white-space:pre-wrap;word-break:break-all;font:10px/1.6 ui-monospace,monospace;background:#f6f8fa;padding:9px;border-radius:4px}dialog{width:min(1280px,96vw);max-height:94vh;padding:0;border:1px solid #b7c4cf;border-radius:8px;color:var(--ink)}dialog::backdrop{background:#102335bb}.modal-head{position:sticky;top:0;background:#fff;display:flex;justify-content:space-between;align-items:center;padding:12px 18px;border-bottom:1px solid var(--line);z-index:2}.modal-head h3{margin:0;font-size:15px}.modal-content{padding:14px}.modal-content img{width:100%;height:auto;display:block}.modal-content p{margin:5px 0 15px;color:var(--muted)}footer{padding:28px 0;color:var(--muted);font-size:12px}noscript{display:block;padding:20px;background:#ffecce}@media(max-width:600px){header{padding:24px}h1{font-size:23px}main{padding:20px 12px}.gallery{grid-template-columns:1fr}.filters{position:static}.overview{grid-template-columns:1fr}}
</style></head><body>
<header><p>CONTINUOUS PATTERNS · OFFLINE CANDIDATES</p><h1>Fixed height layers, plus whole-height trajectories</h1><p>Independent randomized XY geometry at 14 fixed heights; a separate set descends and rises across the full range. Source CSV waypoint/time_s authority stays unchanged. All views show <strong>PLANNED nominal rod_tip</strong> in plane1, h = −z; magnet center and axis calibration are pending.</p><div class="stats" id="stats"></div></header>
<main><div class="notice"><strong>CPU source and rod-subset prechecks only. Robot simulation has NOT RUN.</strong> The 70 mm candidates are retained with the unchanged guard-envelope rejection visible. The 75–200 mm companion is a separate group. Every candidate still requires accepted calibration, robot/scene checks and a release gate.</div>
<h2>Batch overviews</h2><section class="overview" id="overviews"></section>
<h2>Trajectory catalogue</h2><div class="filters" id="filters"><label class="search">Search ID, family or geometry<input id="search" type="search" placeholder="Search trajectories"></label></div><p id="count"></p><section class="gallery" id="gallery"></section><noscript>Enable JavaScript to use the offline filters. PNG/PDF files and original CSV/metadata remain in the adjacent directories.</noscript>
<footer>This is an offline gallery, not a robot command interface. PNG curves may use display-only thinning; the authoritative files are linked unchanged. Motion norms use analytic offline source samples when available; finite-difference fallback is labelled illustrative. Continuous derivative certificates and detailed source-level validation remain in the batch metadata/reports. Fixed height centerlines are planar; their torsion is zero. Rod length: 190 mm; diameter: 5 mm; environment: mla_short / UR5. No GPU, ROS or device is used by this renderer.</footer></main>
<dialog id="detail"><div class="modal-head"><h3 id="modal-title"></h3><button id="close-modal" type="button">Close</button></div><div class="modal-content"><p id="modal-gate"></p><img id="modal-plot" alt="Three dimensional nominal centerline"><p id="modal-method"></p><img id="modal-profile" alt="Height and motion derivative profiles"></div></dialog>
<script id="catalog-data" type="application/json">__DATA__</script>
<script>
"use strict";
const catalogue = JSON.parse(document.getElementById("catalog-data").textContent), records = catalogue.records;
const make = (tag, cls, value) => {const el = document.createElement(tag);if(cls)el.className=cls;if(value!==undefined)el.textContent=value;return el;};
const stats = document.getElementById("stats");
for(const value of [`${records.length} CSV authorities`,`${new Set(records.filter(r=>r.height_mode==="constant").map(r=>r.lower_height_mm)).size} fixed height layers`,`${new Set(records.map(r=>r.family_id)).size} geometry families`,"70–200 mm requested range","Robot simulation: not run"]){stats.append(make("span","stat",value));}
for(const item of catalogue.overviews){const link=make("a");link.href=item.file;link.target="_blank";const image=make("img");image.src=item.file;image.alt=item.label;image.loading="lazy";link.append(image,make("span","",item.label));document.getElementById("overviews").append(link);}
const dimensions=[["group_id","Group"],["height","Height coverage"],["family_label","Family"],["v_tier","Speed tier"],["orientation_mode","Pose mode"],["status","Precheck status"]],selectors={};
for(const [key,label] of dimensions){const wrapper=make("label","",label),select=make("select");select.append(new Option("All",""));for(const value of [...new Set(records.map(r=>String(r[key])))].sort((a,b)=>a.localeCompare(b,undefined,{numeric:true}))){select.append(new Option(value.replaceAll("_"," "),value));}wrapper.append(select);document.getElementById("filters").append(wrapper);selectors[key]=select;select.addEventListener("change",render);}
const reset=make("button","","Reset");reset.type="button";reset.addEventListener("click",()=>{for(const selector of Object.values(selectors))selector.value="";document.getElementById("search").value="";render();});document.getElementById("filters").append(reset);
function link(label,href){const value=make("a","",label);value.href=href;value.target="_blank";return value;}
function openDetail(record){document.getElementById("modal-title").textContent=record.trajectory_id;document.getElementById("modal-gate").textContent=record.status_label+" · PLANNED nominal rod_tip";document.getElementById("modal-plot").src=record.plot_href;document.getElementById("modal-profile").src=record.profile_href;document.getElementById("modal-method").textContent=record.profile_method;document.getElementById("detail").showModal();}
function card(record){const el=make("article","card"),button=make("button");button.type="button";button.title="Open geometry and motion profiles";button.addEventListener("click",()=>openDetail(record));const image=make("img");image.loading="lazy";image.src=record.plot_href;image.alt=record.family_label+", "+record.height;button.append(image);const body=make("div","body");body.append(make("h3","",record.family_label+" · "+record.height),make("div","id",record.trajectory_id));const tags=make("div","tags");for(const value of [record.group_id.replaceAll("_"," "),record.v_tier,record.orientation_mode+" pose",record.direction,record.draw_duration_s.toFixed(1)+" s draw"]){tags.append(make("span","tag",value));}body.append(tags,make("div","gate"+(["rod_rejected","70mm_guard_gap"].includes(record.status)?" reject":""),record.status_label));const links=make("div","links");links.append(link("3D PNG",record.plot_href),link("Motion profile",record.profile_href),link("CSV",record.csv_href),link("Metadata",record.metadata_href));if(record.pdf_href)links.append(link("3D PDF",record.pdf_href));body.append(links);const details=make("details");details.append(make("summary","","Source, geometry and hashes"));details.append(make("pre","",JSON.stringify({source_status:record.source_status,rod_status:record.rod_status,robot_simulation:record.robot_simulation,seed:record.seed,geometry_id:record.geometry_id,xy:record.xy,csv_sample_count:record.csv_sample_count,csv_sha256:record.csv_sha256,metadata_sha256:record.metadata_sha256,profile_method:record.profile_method,source_csv_comparison:record.source_csv_comparison},null,2)));body.append(details);el.append(button,body);return el;}
function render(){const search=document.getElementById("search").value.toLowerCase().trim();const selected=records.filter(record=>dimensions.every(([key])=>!selectors[key].value||String(record[key])===selectors[key].value)&&(!search||[record.trajectory_id,record.family_label,record.geometry_id,record.group_id].join(" ").toLowerCase().includes(search)));const root=document.getElementById("gallery");root.replaceChildren();const fragment=document.createDocumentFragment();for(const record of selected)fragment.append(card(record));root.append(fragment);document.getElementById("count").textContent=`Showing ${selected.length} of ${records.length} trajectories. Geometry figures use the same physical axis scale.`;}
document.getElementById("search").addEventListener("input",render);document.getElementById("close-modal").addEventListener("click",()=>document.getElementById("detail").close());document.getElementById("detail").addEventListener("click",event=>{if(event.target===document.getElementById("detail"))event.target.close();});render();
</script></body></html>'''


def write_gallery(records: list[Record], overview_names: list[str], manifest: dict[str, Any], output: Path) -> None:
    labels = {
        "fixed_height_layers_overview.png": "Fixed height layers · 14 layer comparison",
        "full_height_overview.png": "Full height · 10 geometry families, 70–200 mm",
        "full_height_reverse_overview.png": "Reverse direction controls · 70–200 mm",
        "guarded_full_height_overview.png": "Separate companion group · 75–200 mm",
        "coverage_summary.png": "Height / XY / speed / pose coverage",
        "height_time_profiles.png": "Fixed layers and full height over authoritative time",
    }
    serializable = []
    for record in records:
        item = {key: value for key, value in record.summary.items() if key != "xy_draw_time_bin_s"}
        serializable.append(item)
    payload = {"batch_id": manifest.get("batch_id", "see manifest"), "records": serializable,
               "overviews": [{"file": name, "label": labels.get(name, name)} for name in overview_names],
               "reference_center": "PLANNED nominal rod_tip; magnet/axis calibration pending",
               "robot_simulation": "not_run", "figure_axis_units": "millimeters",
               "csv_authorities_modified": False,
               "occupancy_method": "time-weighted CSV segment midpoints within draw intervals; endpoint holds excluded"}
    encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).replace("<", chr(92) + "u003c")
    (output / "index.html").write_text(HTML_TEMPLATE.replace("__DATA__", encoded), encoding="utf-8")
    (output / "plot_catalog.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--batch", type=Path, required=True, help="Completed batch directory containing manifest.json")
    parser.add_argument("--csv-kinematics", action="store_true", help="Use illustrative CSV differences instead of the analytic source")
    args = parser.parse_args()
    batch = args.batch.expanduser().resolve()
    manifest, records = load_records(batch)
    output = batch / "plots"
    for relative in ("", "trajectories", "profiles"):
        (output / relative).mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.titlesize": 11,
                         "axes.labelsize": 9, "pdf.fonttype": 42, "ps.fonttype": 42,
                         "savefig.facecolor": "white"})
    source, source_label = resolve_source() if not args.csv_kinematics else (None, "CSV differences requested")
    print(f"Rendering {len(records)} authorities with {source_label}", flush=True)
    # One fixed-pose V2 representative per geometry gets vector PDF copies.
    by_geometry: dict[str, list[Record]] = defaultdict(list)
    for record in records:
        by_geometry[str(record.meta.get("geometry_id", record.ident))].append(record)
    pdf_ids = {choose_representative(pool).ident for pool in by_geometry.values()}
    for index, record in enumerate(records, 1):
        data = load_csv(record)
        summarize_csv(record, data)
        is_pdf = record.ident in pdf_ids
        plot_trajectory(record, data, output, is_pdf)
        profile = motion_profile(record, data, source)
        plot_profile(record, data, profile, output, is_pdf)
        record.summary.update({"profile_method": profile["method"],
                               "source_csv_comparison": profile["source_csv_comparison"]})
        if is_pdf:
            record.summary["pdf_href"] = "trajectories/" + record.stem + ".pdf"
        if index == 1 or index % 10 == 0 or index == len(records):
            print(f"Rendered {index}/{len(records)}: {record.ident}", flush=True)
    overview_names = []
    layers = plot_layer_overview(records, output)
    if layers:
        overview_names.append(layers)
    overview_names.extend(plot_full_overviews(records, output))
    overview_names.extend([plot_coverage(records, output), plot_height_profiles(records, output)])
    write_gallery(records, overview_names, manifest, output)
    # Only output files are indexed; the authority manifest is never rewritten.
    files = sorted(path for path in output.rglob("*") if path.is_file() and path.name != "rendered_files_sha256.json")
    rendered = {str(path.relative_to(output)): digest(path) for path in files}
    (output / "rendered_files_sha256.json").write_text(json.dumps(rendered, indent=2) + "\n", encoding="utf-8")
    print(f"Gallery complete: {output / 'index.html'} ({len(records)} trajectories, {len(files)} rendered files)", flush=True)


if __name__ == "__main__":
    main()
