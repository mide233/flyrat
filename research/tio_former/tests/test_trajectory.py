"""Tests for trajectory generation."""

from __future__ import annotations

import numpy as np

from tio_former.sim.trajectory import (
    canonical_trajectory,
    load_csv,
    random_trajectory,
)

BOUNDS = {"x": [-5.0, 5.0], "y": [-3.0, 3.0], "z": [0.15, 2.5]}


def _make(shape: str, duration: float = 3.0):
    return canonical_trajectory(
        shape,
        BOUNDS,
        sample_rate_hz=20.0,
        speed_mps=0.8,
        altitude_m=1.0,
        clearance_m=0.3,
        radius_m=1.0,
        duration_s=duration,
    )


def test_canonical_shapes() -> None:
    for shape in ("circle", "figure_eight", "square", "straight"):
        trajectory = _make(shape)
        assert trajectory.t.size >= 2
        assert np.all(np.diff(trajectory.t) > 0.0)
        assert trajectory.path_length > 0.0
        assert np.all(trajectory.position[:, 2] >= 0.45)
        assert np.all(trajectory.position[:, 2] <= 2.2)
        assert np.all(np.isfinite(trajectory.position))
        assert np.all(np.isfinite(trajectory.quat))


def test_csv_roundtrip(tmp_path) -> None:
    trajectory = _make("circle")
    path = trajectory.to_csv(tmp_path / "trajectory.csv")
    loaded = load_csv(path)
    np.testing.assert_allclose(loaded.t, trajectory.t, atol=1e-9)
    np.testing.assert_allclose(loaded.position, trajectory.position, atol=1e-8)
    np.testing.assert_allclose(loaded.quat, trajectory.quat, atol=1e-8)


def test_random_trajectory_reproducible() -> None:
    waypoints = [[-2.0, -1.0], [1.0, -0.5], [2.0, 1.0], [0.0, 1.5]]
    first = random_trajectory(
        BOUNDS,
        rng=np.random.default_rng(0),
        sample_rate_hz=20.0,
        duration_s=4.0,
        altitude_m=1.0,
        clearance_m=0.3,
        waypoint_count=4,
        speed_mps=0.8,
        waypoint_xy=waypoints,
    )
    second = random_trajectory(
        BOUNDS,
        rng=np.random.default_rng(0),
        sample_rate_hz=20.0,
        duration_s=4.0,
        altitude_m=1.0,
        clearance_m=0.3,
        waypoint_count=4,
        speed_mps=0.8,
        waypoint_xy=waypoints,
    )
    np.testing.assert_allclose(first.position, second.position)
    assert first.duration == second.duration == 4.0
