from __future__ import annotations

import math

import pytest

from hx_waypoint_tools.static_grid_capture import build_serpentine_grid


def test_mla_7_by_7_by_5_grid_is_complete_unique_and_physically_raised() -> None:
    points = build_serpentine_grid()

    assert len(points) == 245
    assert len({point.point_id for point in points}) == 245
    expected_coordinates = [-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15]
    assert sorted({point.x_m for point in points}) == pytest.approx(expected_coordinates)
    assert sorted({point.y_m for point in points}) == pytest.approx(expected_coordinates)
    assert sorted({point.height_m for point in points}) == pytest.approx(
        [0.07, 0.09, 0.11, 0.13, 0.15]
    )
    assert all(point.plane1_position_m[2] == -point.height_m for point in points)
    assert all(point.vertical_command.rotation_vector_rad == (0.0, 0.0, 0.0) for point in points)


def test_serpentine_order_uses_only_one_axis_step_between_targets() -> None:
    points = build_serpentine_grid()

    for left, right in zip(points, points[1:], strict=False):
        delta = tuple(
            abs(a - b)
            for a, b in zip(left.plane1_position_m, right.plane1_position_m, strict=True)
        )
        nonzero = [value for value in delta if value > 1e-12]
        assert len(nonzero) == 1
        assert nonzero[0] == pytest.approx(
            0.02 if left.height_m != right.height_m else 0.05
        )
        assert math.dist(left.plane1_position_m, right.plane1_position_m) <= 0.05 + 1e-12


def test_xy20_supplement_excludes_previously_captured_inner_grid() -> None:
    points = build_serpentine_grid(
        (-20, -15, -10, -5, 0, 5, 10, 15, 20),
        (7, 9, 11, 13, 15),
        exclude_interior_cm=15,
    )

    assert len(points) == 160
    assert all(abs(point.x_m) == pytest.approx(0.20) or abs(point.y_m) == pytest.approx(0.20) for point in points)
    assert len({(point.x_m, point.y_m, point.height_m) for point in points}) == 160
    for left, right in zip(points, points[1:], strict=False):
        delta = tuple(
            abs(a - b)
            for a, b in zip(left.plane1_position_m, right.plane1_position_m, strict=True)
        )
        nonzero = [value for value in delta if value > 1e-12]
        assert len(nonzero) == 1
        assert nonzero[0] == pytest.approx(
            0.02 if left.height_m != right.height_m else 0.05
        )
