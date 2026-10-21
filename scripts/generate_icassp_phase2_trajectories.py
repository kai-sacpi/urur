#!/usr/bin/env python3
"""Generate registration-bound ICASSP Phase-2 static authorities."""

from __future__ import annotations

import argparse
from pathlib import Path

from hx_waypoint_tools.icassp_phase2 import (
    Phase2GenerationError,
    generate_phase2_authorities,
)


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
    default_environment = (
        repository_root
        / "environments"
        / "new_board_4cm_rod33365_up50_icassp_phase2"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("conditions_csv", type=Path)
    parser.add_argument("registration_yaml", type=Path)
    parser.add_argument(
        "output_root",
        type=Path,
        nargs="?",
        default=default_environment / "trajectories" / "icassp_phase2",
        help="Mode-specific formal/ or exploratory/ is always appended here.",
    )
    parser.add_argument(
        "--environment-directory",
        type=Path,
        default=default_environment,
    )
    parser.add_argument(
        "--expected-condition-count",
        type=_positive_integer,
        required=True,
        help="Hard condition-count gate; use 94 only for the audited nominal preview.",
    )
    parser.add_argument(
        "--exploratory",
        action="store_true",
        help=(
            "Allow preview/non-frozen conditions, isolate them under exploratory/, "
            "and force use_in_main_statistics=false."
        ),
    )
    arguments = parser.parse_args()

    try:
        result = generate_phase2_authorities(
            conditions_path=arguments.conditions_csv,
            registration_path=arguments.registration_yaml,
            environment_directory=arguments.environment_directory,
            output_root=arguments.output_root,
            expected_condition_count=arguments.expected_condition_count,
            exploratory=arguments.exploratory,
        )
    except (FileExistsError, OSError, Phase2GenerationError) as error:
        parser.exit(2, f"generate_icassp_phase2_trajectories.py: {error}\n")
    print(f"mode: {result.mode}")
    print(f"condition_count: {len(result.authority_paths)}")
    print(f"authority_index: {result.index_path}")
    print(f"manifest: {result.manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
