#!/usr/bin/env python3
"""Translate the selected zplus26mm authorities by -50 mm in plane1 Z."""

from __future__ import annotations

import argparse
from pathlib import Path

from hx_waypoint_tools.trajectory_generator import (
    generate_new_board_continuous_zplus26_up50_trajectories,
)


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "output_directory",
        nargs="?",
        type=Path,
        default=(
            repository_root
            / "environments"
            / "new_board_4cm_rod33365"
            / "trajectories"
        ),
    )
    arguments = parser.parse_args()
    for path in generate_new_board_continuous_zplus26_up50_trajectories(
        arguments.output_directory
    ):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
