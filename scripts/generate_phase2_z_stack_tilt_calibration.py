#!/usr/bin/env python3
"""Generate the two authoritative Phase-2 Z-stack-plus-tilt experiments."""

from __future__ import annotations

import argparse
from pathlib import Path

from hx_waypoint_tools.z_stack_calibration import (
    Z_STACK_ENVIRONMENT_ID,
    generate_z_stack_tilt_experiments,
)


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--environment",
        type=Path,
        default=repository_root / "environments" / Z_STACK_ENVIRONMENT_ID,
        help="authoritative Phase-2 environment directory",
    )
    arguments = parser.parse_args()
    trajectories, capture_windows = generate_z_stack_tilt_experiments(
        arguments.environment
    )
    for path in trajectories:
        print(path)
    print(capture_windows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
