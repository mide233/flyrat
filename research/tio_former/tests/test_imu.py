"""Tests for analytic IMU generation and ToF-window resampling."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from tio_former.data.imu import generate_imu, resample_to_tof_windows
from tio_former.sim.trajectory import canonical_trajectory
from tio_former.utils.config import load_yaml

ROOT = Path(__file__).resolve().parents[1]


def _sensors() -> dict:
    return load_yaml(ROOT / "configs/sim/sensors.yaml")


def test_generate_imu_shapes_and_finiteness() -> None:
    sensors = _sensors()
    scene_bounds = {
        "x": [-5.0, 5.0],
        "y": [-2.0, 2.0],
        "z": [0.15, 2.5],
    }
    trajectory = canonical_trajectory(
        "straight",
        scene_bounds,
        sample_rate_hz=100.0,
        speed_mps=1.0,
        altitude_m=1.0,
        clearance_m=0.3,
        radius_m=1.0,
        duration_s=3.0,
    )
    imu = generate_imu(trajectory, sensors["imu"], np.random.default_rng(0))
    assert imu.t.ndim == 1
    assert imu.accel.shape == (imu.t.size, 3)
    assert imu.gyro.shape == (imu.t.size, 3)
    assert imu.bias_acc.shape == imu.accel.shape
    assert imu.bias_gyro.shape == imu.gyro.shape
    assert np.all(np.diff(imu.t) > 0.0)
    assert np.all(np.isfinite(imu.accel))
    assert np.all(np.isfinite(imu.gyro))
    assert abs(imu.t.size - round(trajectory.duration * sensors["imu"]["rate_hz"])) <= 1


def test_resample_to_windows() -> None:
    sensors = _sensors()
    scene_bounds = {"x": [-5.0, 5.0], "y": [-2.0, 2.0], "z": [0.15, 2.5]}
    trajectory = canonical_trajectory(
        "circle",
        scene_bounds,
        sample_rate_hz=100.0,
        speed_mps=0.8,
        altitude_m=1.0,
        clearance_m=0.3,
        radius_m=1.0,
        duration_s=2.0,
    )
    imu = generate_imu(trajectory, sensors["imu"], np.random.default_rng(1))
    tof_times = np.arange(0.0, trajectory.duration, 1.0 / 15.0)
    windows, window_times = resample_to_tof_windows(imu, tof_times, 16)
    assert windows.shape == (tof_times.size, 16, 6)
    assert window_times.shape == (tof_times.size, 16)
    assert np.all(np.isfinite(windows))
    assert np.all(window_times[:, 0] < tof_times + 1e-9)
    assert np.all(window_times[:, -1] < tof_times + 1e-9)
