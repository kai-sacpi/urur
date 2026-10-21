from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import pytest


def _load_generator(repository_root: Path):
    path = repository_root / "scripts" / "generate_icassp_f64_13_trajectories.py"
    spec = importlib.util.spec_from_file_location("f64_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_f64_source_converts_to_13_authority_routes(
    repository_root: Path, tmp_path: Path
) -> None:
    generator = _load_generator(repository_root)
    output = tmp_path / "trajectories"
    manifest = tmp_path / "manifest.json"
    paths = generator.generate(
        repository_root / "environments" / "mla" / "trajectories" / "paper_f64_v1_source",
        output,
        manifest,
    )

    assert len(paths) == 13
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["route_count"] == 13
    assert payload["timing"]["source_timestamps_preserved"] is True
    assert payload["orientation"]["rod_axis_mapping"] == (
        "rod_tip +Z in plane1 = (-qx,+qy,+qz)"
    )
    for route, path in zip(generator.ROUTE_ORDER, paths, strict=True):
        with path.open("r", encoding="ascii", newline="") as stream:
            rows = list(csv.DictReader(stream))
        expected_count = int(round(generator.ROUTE_DURATIONS_S[route] / generator.SOURCE_DT_S)) + 1
        assert len(rows) == expected_count
        assert rows[0]["time_s"] == "0.000000000"
        assert float(rows[-1]["time_s"]) == pytest.approx(generator.ROUTE_DURATIONS_S[route])
        assert {row["frame_id"] for row in rows} == {"plane1"}
        assert {row["phase"] for row in rows} == {"draw"}
        assert rows[0]["tip_x_m"] == rows[-1]["tip_x_m"]
        assert rows[0]["tip_y_m"] == rows[-1]["tip_y_m"]
        assert rows[0]["tip_z_m"] == rows[-1]["tip_z_m"]


def test_moment_to_tip_axis_uses_frozen_mla_registration(
    repository_root: Path,
) -> None:
    generator = _load_generator(repository_root)
    quaternion = generator.tip_quaternion_from_moment(2.0**-0.5, 0.0, 2.0**-0.5)
    # For q_A=(+x,0,+z), the rod axis in plane1 is (-x,0,+z).  The shortest
    # rotation around -Y has quaternion (0,-sin(22.5°),0,cos(22.5°)).
    assert quaternion == pytest.approx(
        (0.0, -0.3826834323650898, 0.0, 0.9238795325112867), abs=1.0e-12
    )


def test_f64_generator_refuses_to_overwrite_different_authority(
    repository_root: Path, tmp_path: Path
) -> None:
    generator = _load_generator(repository_root)
    source = repository_root / "environments" / "mla" / "trajectories" / "paper_f64_v1_source"
    output = tmp_path / "trajectories"
    manifest = tmp_path / "manifest.json"
    generator.generate(source, output, manifest)
    target = output / "icassp_f64_line_x_vertical_z09.csv"
    target.write_text(target.read_text(encoding="ascii") + "# changed\n", encoding="ascii")
    with pytest.raises(FileExistsError, match="refusing to overwrite differing authority"):
        generator.generate(source, output, manifest)
