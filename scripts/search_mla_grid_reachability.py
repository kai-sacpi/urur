#!/usr/bin/env python3
"""GPU-search the fixed MLA 54-point grid and emit immutable static authorities."""

from __future__ import annotations

import argparse
from pathlib import Path

from hx_waypoint_tools.mla_grid import MlaGridError, run_search, utc_run_id


def _positive_integer(raw: str) -> int:
    try:
        value = int(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=repository_root)
    parser.add_argument(
        "--output-directory",
        type=Path,
        help=(
            "new evidence directory; default is "
            "<workspace>/artifacts/mla_grid_54/<UTC run id>"
        ),
    )
    parser.add_argument("--azimuth-step-deg", type=_positive_integer, default=30)
    parser.add_argument("--candidates-per-pose", type=_positive_integer, default=32)
    arguments = parser.parse_args()
    workspace = arguments.workspace.expanduser().resolve()
    output_directory = arguments.output_directory or (
        workspace / "artifacts" / "mla_grid_54" / utc_run_id()
    )
    try:
        reachability, attempts, manifest = run_search(
            workspace=workspace,
            output_directory=output_directory,
            azimuth_step_deg=arguments.azimuth_step_deg,
            candidates_per_pose=arguments.candidates_per_pose,
        )
    except (MlaGridError, OSError, ValueError) as error:
        parser.exit(2, f"search_mla_grid_reachability.py: {error}\n")
    print(f"reachability: {reachability}")
    print(f"attempts: {attempts}")
    print(f"manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

