from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest


def _load_generator(repository_root: Path):
    scripts = str(repository_root / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    path = repository_root / "scripts" / "generate_icassp_f64_13_rest_trajectories.py"
    spec = importlib.util.spec_from_file_location("f64_rest_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rest_authorities_preserve_period_and_geometry(
    repository_root: Path, tmp_path: Path
) -> None:
    generator = _load_generator(repository_root)
    output = tmp_path / "trajectories"
    manifest_path = tmp_path / "manifest.json"
    paths = generator.generate(
        repository_root / "environments/mla/trajectories/paper_f64_v1_source",
        output,
        manifest_path,
    )
    assert len(paths) == 13
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["route_count"] == 13
    assert manifest["orientation"]["implemented_moment_sign"] == 1
    for route, path in zip(generator.reference.ROUTE_ORDER, paths, strict=True):
        with path.open("r", encoding="ascii", newline="") as stream:
            rows = list(csv.DictReader(stream))
        expected_count = int(round(generator.reference.ROUTE_DURATIONS_S[route] / 0.02)) + 1
        assert len(rows) == expected_count
        assert rows[0]["time_s"] == "0.000000000"
        assert float(rows[-1]["time_s"]) == pytest.approx(generator.reference.ROUTE_DURATIONS_S[route])
        assert all(row["phase"] == "draw" for row in rows)
        x = np.asarray([float(row["tip_x_m"]) for row in rows])
        y = np.asarray([float(row["tip_y_m"]) for row in rows])
        z = np.asarray([float(row["tip_z_m"]) for row in rows])
        assert x[0] == pytest.approx(x[-1], abs=1.0e-10)
        assert y[0] == pytest.approx(y[-1], abs=1.0e-10)
        assert z[0] == pytest.approx(z[-1], abs=1.0e-10)
        if route.startswith("line_x"):
            assert np.max(np.abs(x)) == pytest.approx(0.15, abs=2.0e-6)
            assert np.max(np.abs(y)) < 2.0e-6
        elif route.startswith("line_y"):
            assert np.max(np.abs(y)) == pytest.approx(0.15, abs=2.0e-6)
            assert np.max(np.abs(x)) < 2.0e-6
        else:
            assert np.max(np.abs(x)) == pytest.approx(0.10, abs=2.0e-6)
            assert np.max(np.abs(y)) == pytest.approx(0.10, abs=2.0e-6)


def test_rest_phase_has_zero_endpoint_finite_differences(
    repository_root: Path, tmp_path: Path
) -> None:
    generator = _load_generator(repository_root)
    output = tmp_path / "trajectories"
    paths = generator.generate(
        repository_root / "environments/mla/trajectories/paper_f64_v1_source",
        output,
        tmp_path / "manifest.json",
    )
    for path in paths:
        with path.open("r", encoding="ascii", newline="") as stream:
            rows = list(csv.DictReader(stream))
        p = np.asarray(
            [[float(row[field]) for field in ("tip_x_m", "tip_y_m", "tip_z_m")] for row in rows]
        )
        # A quintic phase makes the first finite-difference velocity O(dt^2)
        # and acceleration O(dt), rather than the non-zero paper-rate value.
        v0 = np.linalg.norm((p[1] - p[0]) / 0.02)
        v1 = np.linalg.norm((p[-1] - p[-2]) / 0.02)
        assert v0 < 1.0e-3
        assert v1 < 1.0e-3


def test_flipped_sign_is_a_single_continuous_policy(repository_root: Path) -> None:
    generator = _load_generator(repository_root)
    nominal = generator.tip_quaternion_from_moment(0.0, 0.0, 1.0, moment_sign=1)
    flipped = generator.tip_quaternion_from_moment(0.0, 0.0, 1.0, moment_sign=-1)
    assert nominal == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)
    assert flipped == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1.0e-12)
