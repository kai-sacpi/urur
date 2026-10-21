#!/usr/bin/env python3
"""Small representative previews and a complete source index for batch120.

All 120 CSV/metadata pairs are hash-verified. Only six representatives from
each route group are drawn; the HTML table links every authoritative source.
This is source visualization, not a GPU/robot/replay validation program.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import html
import json
import os
from pathlib import Path
import sys

if __package__:
    from . import plot_preview as preview
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import plot_preview as preview

import matplotlib.pyplot as plt
import numpy as np


FAMILIES = ("open_line", "gentle_s", "strong_s", "c_curve", "slanted_s", "tube_bend")
FIXED_REPRESENTATIVE_HEIGHTS_MM = (70, 90, 120, 150, 180, 200)
DEFAULT_RUNTIME_BATCH = "ca_batch120_20261016_v2"


def draw_bounds(source: preview.Source) -> tuple[float, float]:
    """Use hash-verified metadata; boundary phase labels are not durations."""
    start = float(source.metadata["hold_start_s"])
    end = start + float(source.metadata["draw_duration_s"])
    if not np.isclose(end + float(source.metadata["hold_end_s"]), source.time_s[-1], atol=1e-8, rtol=0):
        raise ValueError(f"Timing metadata disagrees with CSV: {source.trajectory_id}")
    return start, end


def profile_display_source(source: preview.Source) -> preview.Source:
    """Derive a display-only hold mask, without editing any source sample."""
    start, end = draw_bounds(source)
    phase = np.where((source.time_s > start + 1e-9) & (source.time_s < end - 1e-9), "draw", "hold")
    return replace(source, phase=phase)


def representatives(sources: list[preview.Source]) -> tuple[list[preview.Source], list[preview.Source]]:
    fixed, full = [], []
    if {s.metadata["family_id"] for s in sources} != set(FAMILIES):
        raise ValueError("Unexpected family set: expected the six open-route families")
    for i, (family, height) in enumerate(zip(FAMILIES, FIXED_REPRESENTATIVE_HEIGHTS_MM)):
        candidates = [s for s in sources if s.fixed_height and s.metadata["family_id"] == family]
        chosen = min(candidates, key=lambda s: abs(float(s.display_mm[0, 2]) - height))
        if abs(float(chosen.display_mm[0, 2]) - height) > 1e-6:
            raise ValueError(f"Missing representative {family}, height {height} mm")
        fixed.append(chosen)
        ascending = i % 2 == 0
        candidates = sorted([s for s in sources if not s.fixed_height
                             and s.metadata["family_id"] == family
                             and bool(s.display_mm[-1, 2] > s.display_mm[0, 2]) == ascending],
                            key=lambda s: s.trajectory_id)
        if len(candidates) != 3:
            raise ValueError(f"Expected three {family} full-height variants in each direction")
        full.append(candidates[(i // 2) % len(candidates)])
    return fixed, full


def coverage_summary(sources: list[preview.Source]) -> dict:
    heights = np.arange(70, 201, 10)
    fixed_counts = np.zeros(len(heights), dtype=int)
    bins = np.arange(70, 201, 10, dtype=float)
    histogram_bins = bins.copy()
    histogram_bins[0] -= 1e-7
    histogram_bins[-1] += 1e-7
    draw_time = np.zeros(len(bins) - 1)
    full_direction_counts = {family: {"ascending": 0, "descending": 0} for family in FAMILIES}
    for source in sources:
        h = source.display_mm[:, 2]
        if source.fixed_height:
            index = int(np.argmin(np.abs(heights - h[0])))
            if abs(heights[index] - h[0]) > 1e-6:
                raise ValueError(f"Unexpected fixed height: {source.trajectory_id}")
            fixed_counts[index] += 1
        else:
            direction = "ascending" if h[-1] > h[0] else "descending"
            full_direction_counts[source.metadata["family_id"]][direction] += 1
        start, end = draw_bounds(source)
        # Time weights only: intersect each unchanged CSV interval with the
        # metadata draw window. This is not execution clipping or retiming.
        draw_dt = np.maximum(0.0, np.minimum(source.time_s[1:], end) -
                             np.maximum(source.time_s[:-1], start))
        intervals = draw_dt > 0.0
        dt = draw_dt[intervals]
        midpoint_height = ((h[:-1] + h[1:]) / 2)[intervals]
        weight, _ = np.histogram(midpoint_height, bins=histogram_bins, weights=dt)
        if not np.isclose(weight.sum(), dt.sum(), atol=1e-7, rtol=0):
            raise ValueError(f"Source height escaped coverage bounds: {source.trajectory_id}")
        draw_time += weight
    return {"fixed_heights_mm": heights.tolist(), "fixed_count_by_height": fixed_counts.tolist(),
            "full_height_direction_counts_by_family": full_direction_counts,
            "height_bin_edges_mm": bins.tolist(), "draw_time_by_height_bin_s": draw_time.tolist(),
            "height_histogram_method": "CSV interval midpoint height, weighted by original interval duration within metadata draw bounds; holds excluded",
            "total_source_duration_s": float(sum(s.time_s[-1] for s in sources)),
            "total_draw_duration_s": float(draw_time.sum())}


def plot_coverage(summary: dict, output: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.6, 4.7), constrained_layout=True)
    x = summary["fixed_heights_mm"]
    axes[0].bar(x, summary["fixed_count_by_height"], width=7, color="#167d9a")
    axes[0].set_xticks(x)
    axes[0].set_xlabel("Fixed route height h = -Z (mm)")
    axes[0].set_ylabel("Authoritative source files")
    axes[0].set_title("84 fixed-height routes: six families at each height")
    axes[0].set_ylim(0, max(summary["fixed_count_by_height"]) * 1.2)
    edges = np.asarray(summary["height_bin_edges_mm"])
    seconds = np.asarray(summary["draw_time_by_height_bin_s"])
    axes[1].bar((edges[:-1] + edges[1:]) / 2, seconds / 60, width=8, color="#7456a5")
    axes[1].set_xticks(edges)
    axes[1].set_xlabel("Source height bin (mm); rightmost includes 200 mm")
    axes[1].set_ylabel("Draw-phase time (min)")
    axes[1].set_title("All 120 routes: original-time height coverage")
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(True, axis="y", alpha=0.18)
        ax.tick_params(axis="x", labelsize=8)
        ax.set_axisbelow(True)
    fig.suptitle("Source-bank coverage | 36 one-way routes: 3 up + 3 down per family", fontsize=15)
    preview.save_figure(fig, output)


def relative_link(target: Path, from_directory: Path) -> str:
    return Path(os.path.relpath(target, from_directory)).as_posix()


def html_index(batch: Path, sources: list[preview.Source], outputs: dict,
               replay_status: Path, summary: dict) -> str:
    escape = html.escape
    status_href = escape(relative_link(replay_status, batch / "plots"), quote=True)
    status_csv_href = escape(relative_link(replay_status.with_suffix(".csv"), batch / "plots"), quote=True)
    result = (
        "<!doctype html><meta charset='utf-8'><title>Open augmented batch120</title>"
        "<style>body{font:15px system-ui;max-width:1650px;margin:2rem auto;padding:0 1rem;color:#243247}"
        "img{max-width:100%;height:auto}figure{margin:2.5rem 0}figcaption{color:#526176}"
        "table{border-collapse:collapse;width:100%;font-size:12px}td,th{padding:.4rem;border-bottom:1px solid #dde3e9;text-align:left}"
        "td:first-child{overflow-wrap:anywhere;max-width:400px}th{background:#eff3f6}.notice{background:#f1f6fa;padding:1rem}</style>"
        f"<h1>{escape(batch.name)}</h1><p class='notice'>All {len(sources)} source CSV/metadata pairs "
        "were verified against the frozen manifest. Figures show representatives, not all trajectories. "
        "This page is visualization only and does not certify IK, collision, controller tracking or replay. "
        "Display height h = -plane1 Z (mm). Only new open, one-pass sources are indexed here.</p>"
        "<p><a href='../manifest.json'>Source manifest</a> · "
        "<a href='../reports/source_audit.json'>Source audit JSON</a> · "
        "<a href='../reports/source_audit.md'>Source audit report</a> · "
        f"<a href='{status_href}'>Current replay status (separate runtime evidence)</a> · "
        f"<a href='{status_csv_href}'>Runtime status CSV</a> · "
        "<a href='../reports/replay_index.json'>Sealed replay index JSON</a> · "
        "<a href='../reports/replay_index.csv'>Sealed replay index CSV</a> · "
        "<a href='plot_provenance.json'>Plot/source SHA256 provenance</a></p>"
        f"<p>Source duration including holds: {summary['total_source_duration_s'] / 60:.2f} min; "
        f"draw phase: {summary['total_draw_duration_s'] / 60:.2f} min. "
        "Runtime status is intentionally not frozen or inferred in this page. "
        "Missing audit/replay links mean pending evidence, never a ready result.</p>")
    captions = {
        "representative_fixed_height.png": "One fixed-height representative per family; 70, 90, 120, 150, 180 and 200 mm. Dashed grey: unwarped prototype, not an execution source.",
        "representative_one_way_full_height.png": "One full-height representative per family; alternating ascent/descent. All trajectories remain open and one-pass.",
        "representative_speed_orientation.png": "Three representative CSV-derived speed and rod-axis profiles. Dashed speed is a statistical mean, not a timing baseline.",
        "source_height_coverage.png": "Coverage of all 120 authoritative source files, not a robot-execution summary.",
    }
    for filename, caption in captions.items():
        result += (f"<figure><img src='{filename}' alt='{escape(caption, quote=True)}'>"
                   f"<figcaption>{escape(caption)}</figcaption></figure>")
    result += "<h2>All authoritative sources</h2><table><thead><tr><th>Trajectory ID</th><th>Family</th><th>Height (mm)</th><th>Direction</th><th>Variant</th><th>Duration (s)</th><th>Authority files</th></tr></thead><tbody>"
    for source in sources:
        meta, record = source.metadata, source.record
        h = source.display_mm[:, 2]
        height = f"{h[0]:.0f}" if source.fixed_height else f"{h[0]:.0f} → {h[-1]:.0f}"
        direction = "fixed height" if source.fixed_height else ("ascending" if h[-1] > h[0] else "descending")
        csv_href, metadata_href = ("../" + escape(record[key], quote=True) for key in ("csv_file", "metadata_file"))
        result += (f"<tr><td>{escape(source.trajectory_id)}</td><td>{escape(meta['family_id'])}</td>"
                   f"<td>{height}</td><td>{direction}</td><td>{escape(str(meta.get('variant_index', record.get('variant_index', ''))))}</td>"
                   f"<td>{source.time_s[-1]:.2f}</td><td><a href='{csv_href}' title='SHA256 {source.csv_sha256}'>CSV</a> · "
                   f"<a href='{metadata_href}' title='SHA256 {source.metadata_sha256}'>Metadata</a></td></tr>")
    return result + "</tbody></table>\n"


def render(batch: Path, replay_status: Path | None = None) -> dict:
    if "continuous_augmented" not in batch.parts or batch.name == "ca_open_elastic_preview_20261016_v1":
        raise ValueError("plot_batch must target a new augmented batch, never the frozen 12-source preview")
    manifest_hash = preview.sha256(batch / "manifest.json")
    manifest, sources = preview.load_sources(batch, expected_count=120)
    if manifest.get("batch_id") != batch.name:
        raise ValueError("Manifest batch ID does not match its directory")
    if any(s.metadata.get("orientation", {}).get("mode") != "smooth_multi_axis" for s in sources):
        raise ValueError("Batch120 expects independently varied multi-axis orientation on every route")
    fixed, full = representatives(sources)
    summary = coverage_summary(sources)
    if summary["fixed_count_by_height"] != [6] * 14:
        raise ValueError("Expected exactly six fixed-height routes in each of 14 layers")
    if any(d != {"ascending": 3, "descending": 3} for d in summary["full_height_direction_counts_by_family"].values()):
        raise ValueError("Expected three up and three down routes per family")
    plots = batch / "plots"
    plots.mkdir(exist_ok=True)
    outputs = {}
    specs = ((fixed, "representative_fixed_height.png", "Fixed-height representatives | 6 of 84 open routes"),
             (full, "representative_one_way_full_height.png", "One-way 70-200 mm representatives | 6 of 36 open routes"))
    for selected, filename, title in specs:
        outputs[filename] = {"trajectory_ids": preview.plot_overview(selected, plots / filename, title),
                             "sha256": preview.sha256(plots / filename)}
    filename = "representative_speed_orientation.png"
    outputs[filename] = {"trajectory_ids": preview.plot_profiles([profile_display_source(s) for s in fixed + full], plots / filename),
                         "sha256": preview.sha256(plots / filename)}
    filename = "source_height_coverage.png"
    plot_coverage(summary, plots / filename)
    outputs[filename] = {"trajectory_ids": [s.trajectory_id for s in sources],
                         "sha256": preview.sha256(plots / filename)}
    if replay_status is None:
        replay_status = Path(__file__).resolve().parent.parent / "runs" / DEFAULT_RUNTIME_BATCH / "status.json"
    replay_status = replay_status.resolve()
    (plots / "index.html").write_text(html_index(batch, sources, outputs, replay_status, summary))
    outputs["index.html"] = {"trajectory_ids": [s.trajectory_id for s in sources],
                             "sha256": preview.sha256(plots / "index.html")}
    if preview.sha256(batch / "manifest.json") != manifest_hash:
        raise ValueError("Manifest changed during rendering")
    for source in sources:
        if (preview.sha256(source.csv_path) != source.csv_sha256 or
                preview.sha256(source.metadata_path) != source.metadata_sha256):
            raise ValueError(f"Authority changed during rendering: {source.trajectory_id}")
    provenance = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "batch_id": batch.name, "source_count": len(sources), "fixed_count": 84, "full_height_count": 36,
        "status": "source_visualization_only_not_robot_execution_validation",
        "script_sha256": preview.sha256(Path(__file__)),
        "plot_preview_helper_sha256": preview.sha256(Path(preview.__file__)),
        "manifest_sha256": manifest_hash,
        "display_coordinates": "[1000*plane1.x,1000*plane1.y,-1000*plane1.z] mm",
        "orientation_arrows": "tool +Z; vector z sign inverted only for height display",
        "hold_display_and_mean_speed": "derive display-only hold mask from authoritative metadata draw bounds; never alter CSV phase, XYZ, quaternion or time_s",
        "all_selected_csv_vertices_used_in_polylines": True, "source_files_modified": False,
        "representative_policy": "one per family in each group; six dispersed fixed heights; alternating full-height directions",
        "coverage": summary,
        "replay_status_link": {"relative_to_plots": relative_link(replay_status, plots),
                               "exists_at_render_time": replay_status.is_file(),
                               "replay_pass_not_inferred": True},
        "sources": [{"trajectory_id": s.trajectory_id, "csv_file": s.record["csv_file"],
                     "metadata_file": s.record["metadata_file"], "csv_sha256": s.csv_sha256,
                     "metadata_sha256": s.metadata_sha256, "samples": len(s.time_s)} for s in sources],
        "outputs": outputs,
    }
    (plots / "plot_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return {"batch": str(batch), "sources_verified": len(sources),
            "outputs": [str(plots / filename) for filename in outputs],
            "provenance": str(plots / "plot_provenance.json")}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--replay-status", type=Path,
                        help="Link the separate runtime status JSON; no pass state is inferred")
    args = parser.parse_args()
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.titleweight": "bold", "axes.edgecolor": "#637080"})
    print(json.dumps(render(args.batch.resolve(), args.replay_status), indent=2))


if __name__ == "__main__":
    main()
