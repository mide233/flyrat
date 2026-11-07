"""Analytic IMU generation from a ground-truth trajectory.

The paper consumes 16 IMU samples per 15 Hz ToF frame (200 Hz IMU resampled
into the interval between two range frames).  In simulation we know the pose
trajectory exactly, so we differentiate it, add the configured bias/noise, and
resample into fixed-size windows.  This avoids dependending on Gazebo's IMU
sensor and keeps the dataset deterministic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from tio_former.sim.trajectory import Trajectory
from tio_former.utils.se3 import (
    interpolate_pose,
    matrix_to_pose,
    pose_to_matrix,
    quat_slerp,
    so3_log,
)


@dataclass(frozen=True)
class ImuSequence:
    t: np.ndarray
    accel: np.ndarray
    gyro: np.ndarray
    bias_acc: np.ndarray
    bias_gyro: np.ndarray


def _resample_poses(trajectory: Trajectory, t_out: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate a pose trajectory at arbitrary times."""
    t_src = trajectory.t
    positions = np.empty((t_out.size, 3), dtype=np.float64)
    quats = np.empty((t_out.size, 4), dtype=np.float64)
    for i, value in enumerate(t_out):
        clipped = float(np.clip(value, t_src[0], t_src[-1]))
        index = int(np.searchsorted(t_src, clipped, side="right") - 1)
        index = min(max(index, 0), t_src.size - 2)
        t0, t1 = t_src[index], t_src[index + 1]
        alpha = 0.0 if t1 <= t0 else (clipped - t0) / (t1 - t0)
        p0, q0 = matrix_to_pose(pose_to_matrix(trajectory.position[index], trajectory.quat[index]))
        p1, q1 = matrix_to_pose(pose_to_matrix(trajectory.position[index + 1], trajectory.quat[index + 1]))
        positions[i] = p0 + alpha * (p1 - p0)
        quats[i] = quat_slerp(q0, q1, float(alpha))
    return positions, quats


def generate_imu(
    trajectory: Trajectory,
    config: dict[str, Any],
    rng: np.random.Generator | None = None,
) -> ImuSequence:
    """Generate noisy 200 Hz IMU measurements from a pose trajectory.

    ``config`` is the ``imu`` block of ``configs/sim/sensors.yaml``.
    """
    rng = np.random.default_rng() if rng is None else rng
    rate = float(config.get("rate_hz", 200.0))
    if rate <= 0.0:
        raise ValueError("IMU rate must be positive")
    duration = trajectory.duration
    count = max(2, int(round(duration * rate)) + 1)
    t = np.linspace(0.0, duration, count, dtype=np.float64)
    position, quat = _resample_poses(trajectory, t)

    dt = np.diff(t)
    if dt.size == 0:
        dt = np.array([1e-9], dtype=np.float64)
    # Keep one interval per sample: the final sample reuses the last step.
    dt = np.concatenate([dt, dt[-1:]])
    dt = np.where(dt > 1e-9, dt, 1e-9)
    velocity = np.gradient(position, t, axis=0)
    acceleration_world = np.gradient(velocity, t, axis=0)
    rotations = np.stack(
        [
            pose_to_matrix(position[i], quat[i])[:3, :3]
            for i in range(t.size)
        ],
        axis=0,
    )

    # Body angular velocity from R_i^T R_{i+1}.
    gyro_true = np.zeros((t.size, 3), dtype=np.float64)
    for i in range(t.size - 1):
        delta = rotations[i].T @ rotations[i + 1]
        gyro_true[i] = so3_log(delta) / dt[i]
    if t.size > 1:
        gyro_true[-1] = gyro_true[-2]

    gravity = np.asarray(config.get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)
    # Specific force measured by an accelerometer: R^T (a - g).
    specific_force = np.einsum(
        "nij,nj->ni", rotations.transpose(0, 2, 1), acceleration_world - gravity
    )

    # Bias random walk and white measurement noise, with Kalibr density values.
    noise_gyro = float(config.get("noise_gyro", 0.0))
    noise_acc = float(config.get("noise_acc", 0.0))
    gyro_walk = float(config.get("gyro_walk", 0.0))
    acc_walk = float(config.get("acc_walk", 0.0))
    bias_acc = np.empty_like(specific_force)
    bias_gyro = np.empty_like(gyro_true)
    bias_acc[0] = rng.normal(0.0, float(config.get("init_bias_acc", 0.0)), size=3)
    bias_gyro[0] = rng.normal(0.0, float(config.get("init_bias_gyro", 0.0)), size=3)
    for i in range(1, t.size):
        step = np.sqrt(dt[i - 1])
        bias_acc[i] = bias_acc[i - 1] + acc_walk * step * rng.normal(size=3)
        bias_gyro[i] = bias_gyro[i - 1] + gyro_walk * step * rng.normal(size=3)

    accel_meas = specific_force + bias_acc + noise_acc / np.sqrt(dt)[:, None] * rng.normal(size=specific_force.shape)
    gyro_meas = gyro_true + bias_gyro + noise_gyro / np.sqrt(dt)[:, None] * rng.normal(size=gyro_true.shape)

    return ImuSequence(
        t=t,
        accel=accel_meas.astype(np.float32),
        gyro=gyro_meas.astype(np.float32),
        bias_acc=bias_acc.astype(np.float32),
        bias_gyro=bias_gyro.astype(np.float32),
    )


def resample_to_tof_windows(
    imu: ImuSequence,
    tof_times: np.ndarray,
    samples_per_window: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Resample an IMU sequence into ``[T, K, 6]`` windows.

    The interval associated with ToF frame ``i`` is ``(tau_{i-1}, tau_i]``;
    the first frame reuses the ToF period before the first timestamp.  This
    mirrors the paper's ``I_t in R^{16x6}`` input.
    """
    tof_times = np.asarray(tof_times, dtype=np.float64)
    if tof_times.ndim != 1 or tof_times.size == 0:
        raise ValueError("tof_times must be a non-empty 1-D array")
    if samples_per_window <= 0:
        raise ValueError("samples_per_window must be positive")
    if tof_times.size > 1:
        period = float(np.median(np.diff(tof_times)))
    else:
        period = 1.0 / 15.0
    windows = np.empty((tof_times.size, samples_per_window, 6), dtype=np.float32)
    times = np.empty((tof_times.size, samples_per_window), dtype=np.float64)
    for i, end in enumerate(tof_times):
        start = float(tof_times[i - 1]) if i > 0 else float(end - period)
        # Endpoint=False avoids duplicating the boundary sample between frames.
        t_window = np.linspace(start, end, samples_per_window, endpoint=False)
        times[i] = t_window
        for channel in range(3):
            windows[i, :, channel] = np.interp(
                t_window, imu.t, imu.accel[:, channel]
            )
            windows[i, :, channel + 3] = np.interp(
                t_window, imu.t, imu.gyro[:, channel]
            )
    return windows, times
