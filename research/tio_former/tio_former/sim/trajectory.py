"""Offline trajectory generation for the TIO-Former simulation data pipeline.

The simulator is used as a ray-caster only.  We therefore generate smooth
body trajectories first, replay them in Gazebo with a tiny system plugin, and
derive the 200 Hz IMU analytically from the same trajectory.  This keeps the
ToF images and the ground-truth pose/IMU perfectly aligned in simulation time.

The implementation only depends on NumPy.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from tio_former.utils.se3 import yaw_to_quat

_TWO_PI = 2.0 * np.pi


@dataclass(frozen=True)
class Trajectory:
    """Sampled trajectory in the world frame (FLU / ENU-like Gazebo world)."""

    t: np.ndarray
    position: np.ndarray
    quat: np.ndarray

    def __post_init__(self) -> None:
        t = np.asarray(self.t, dtype=np.float64)
        position = np.asarray(self.position, dtype=np.float64)
        quat = np.asarray(self.quat, dtype=np.float64)
        if t.ndim != 1:
            raise ValueError("t must be 1-D")
        if position.shape != (t.size, 3):
            raise ValueError(f"position must have shape {(t.size, 3)}, got {position.shape}")
        if quat.shape != (t.size, 4):
            raise ValueError(f"quat must have shape {(t.size, 4)}, got {quat.shape}")
        if t.size < 2:
            raise ValueError("a trajectory needs at least two samples")
        if not np.all(np.diff(t) > 0.0):
            raise ValueError("trajectory timestamps must be strictly increasing")

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    @property
    def path_length(self) -> float:
        return float(np.sum(np.linalg.norm(np.diff(self.position, axis=0), axis=1)))

    def to_csv(self, path: str | Path) -> Path:
        """Write ``t,x,y,z,qw,qx,qy,qz`` (quaternion scalar first)."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = np.concatenate([self.t[:, None], self.position, self.quat], axis=1)
        np.savetxt(path, data, delimiter=",", fmt="%.10f")
        return path


def load_csv(path: str | Path) -> Trajectory:
    path = Path(path)
    data = np.loadtxt(path, delimiter=",", ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f"{path}: expected 8 columns, got {data.shape[1]}")
    return Trajectory(data[:, 0], data[:, 1:4], data[:, 4:8])


def _bounds(scene_bounds: dict[str, Iterable[float]]) -> tuple[np.ndarray, np.ndarray]:
    low = np.array(
        [float(scene_bounds[axis][0]) for axis in ("x", "y", "z")], dtype=np.float64
    )
    high = np.array(
        [float(scene_bounds[axis][1]) for axis in ("x", "y", "z")], dtype=np.float64
    )
    if np.any(high <= low):
        raise ValueError(f"invalid scene bounds: {scene_bounds}")
    return low, high


def _clip_altitude(altitude: float, low: np.ndarray, high: np.ndarray, clearance: float) -> float:
    return float(np.clip(altitude, low[2] + clearance, high[2] - clearance))


def _positions_to_yaw_quat(position: np.ndarray) -> np.ndarray:
    velocity = np.gradient(position, axis=0)
    yaw = np.arctan2(velocity[:, 1], velocity[:, 0])
    # Keep the heading well-defined when the trajectory momentarily stops.
    for i in range(1, yaw.size):
        if np.linalg.norm(velocity[i, :2]) < 1e-9:
            yaw[i] = yaw[i - 1]
    return np.stack([yaw_to_quat(float(a)) for a in yaw], axis=0)


def canonical_trajectory(
    shape: str,
    scene_bounds: dict[str, Iterable[float]],
    *,
    sample_rate_hz: float,
    speed_mps: float,
    altitude_m: float,
    clearance_m: float,
    radius_m: float,
    center_xy: Iterable[float] | None = None,
    duration_s: float | None = None,
) -> Trajectory:
    """Generate one of the four canonical shapes used in the paper."""
    if sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    if speed_mps <= 0.0:
        raise ValueError("speed_mps must be positive")
    low, high = _bounds(scene_bounds)
    center = np.array(
        center_xy if center_xy is not None else 0.5 * (low[:2] + high[:2]),
        dtype=np.float64,
    )
    clearance = max(0.0, float(clearance_m))
    max_radius = float(
        min(high[0] - center[0], center[0] - low[0], high[1] - center[1], center[1] - low[1])
        - clearance
    )
    if max_radius <= 0.05:
        raise ValueError("scene is too small for the requested clearance")
    radius = float(np.clip(radius_m, 0.05, max_radius))
    altitude = _clip_altitude(float(altitude_m), low, high, clearance)

    shape = shape.lower().replace("-", "_")
    if shape == "straight":
        length = 2.0 * radius
        duration = length / speed_mps
    elif shape in {"circle", "figure_eight", "square"}:
        # Approximate perimeter; duration only controls sampling density.
        if shape == "circle":
            length = _TWO_PI * radius
        elif shape == "figure_eight":
            length = 4.0 * radius
        else:
            length = 8.0 * radius
        duration = max(length / speed_mps, 1.0)
    else:
        raise ValueError(f"unknown canonical shape: {shape!r}")

    if duration_s is not None:
        duration = float(duration_s)
        if duration <= 0.0:
            raise ValueError("duration_s must be positive")

    count = max(2, int(round(duration * sample_rate_hz)) + 1)
    t = np.linspace(0.0, duration, count)
    s = np.linspace(0.0, 1.0, count, endpoint=True)
    position = np.zeros((count, 3), dtype=np.float64)
    position[:, 2] = altitude

    if shape == "circle":
        theta = _TWO_PI * s
        position[:, 0] = center[0] + radius * np.cos(theta)
        position[:, 1] = center[1] + radius * np.sin(theta)
    elif shape == "figure_eight":
        theta = _TWO_PI * s
        position[:, 0] = center[0] + radius * np.cos(theta)
        position[:, 1] = center[1] + 0.5 * radius * np.sin(2.0 * theta)
    elif shape == "square":
        # Rounded square via a smoothed L-infinity radius.
        k = 3.0
        theta = _TWO_PI * s
        cx = np.tanh(k * np.cos(theta)) / np.tanh(k)
        cy = np.tanh(k * np.sin(theta)) / np.tanh(k)
        position[:, 0] = center[0] + radius * cx
        position[:, 1] = center[1] + radius * cy
    else:  # straight
        position[:, 0] = center[0] - radius + 2.0 * radius * s

    quat = _positions_to_yaw_quat(position)
    _check_bounds(position, low, high, clearance)
    return Trajectory(t, position, quat)


def random_trajectory(
    scene_bounds: dict[str, Iterable[float]],
    *,
    rng: np.random.Generator,
    sample_rate_hz: float,
    duration_s: float,
    altitude_m: float,
    clearance_m: float,
    waypoint_count: int,
    speed_mps: float,
    waypoint_xy: Iterable[Iterable[float]] | None = None,
) -> Trajectory:
    """Generate a smooth random 3-D trajectory through 4-10 waypoints.

    The path is a non-uniform Catmull-Rom spline.  It is C1-continuous, which is
    enough for finite-difference IMU generation and avoids the stop-and-go
    behaviour of a piecewise-linear path.
    """
    if sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    low, high = _bounds(scene_bounds)
    clearance = max(0.0, float(clearance_m))
    inner_low = low + clearance
    inner_high = high - clearance
    if np.any(inner_high <= inner_low):
        raise ValueError("scene is too small for the requested clearance")

    altitude = _clip_altitude(float(altitude_m), low, high, clearance)
    count = max(3, int(waypoint_count))
    if waypoint_xy is not None:
        xy = np.asarray(list(waypoint_xy), dtype=np.float64)
        if xy.ndim != 2 or xy.shape[1] != 2:
            raise ValueError("waypoint_xy must be a sequence of 2-D points")
    else:
        xy = np.empty((count, 2), dtype=np.float64)
        xy[0] = 0.5 * (inner_low[:2] + inner_high[:2])
        for i in range(1, count):
            # Rejection-sample a waypoint that is not too close to the previous one.
            for _ in range(100):
                candidate = rng.uniform(inner_low[:2], inner_high[:2])
                if np.linalg.norm(candidate - xy[i - 1]) >= 0.5:
                    xy[i] = candidate
                    break
            else:
                xy[i] = candidate
    count = xy.shape[0]
    if count < 2:
        raise ValueError("at least two waypoints are required")

    # Give every waypoint a small vertical offset while staying inside bounds.
    z = np.clip(
        altitude + rng.normal(0.0, 0.12, size=count),
        inner_low[2],
        inner_high[2],
    )
    waypoints = np.column_stack([xy, z])

    chord = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    if np.any(chord < 1e-6):
        raise ValueError("duplicate waypoints would create a zero-length segment")
    # Segment durations proportional to length at the nominal speed; then scale
    # the whole timeline to the requested duration.
    segment_time = chord / max(float(speed_mps), 1e-3)
    segment_time *= duration_s / float(np.sum(segment_time))
    knot_t = np.concatenate([[0.0], np.cumsum(segment_time)])
    sample_count = max(2, int(round(duration_s * sample_rate_hz)) + 1)
    t = np.linspace(0.0, duration_s, sample_count)
    position = _catmull_rom(waypoints, knot_t, t)

    # Re-anchor the first sample at the actual requested start and keep the
    # altitude inside bounds after spline overshoot.
    position[:, 2] = np.clip(position[:, 2], inner_low[2], inner_high[2])
    _check_bounds(position, low, high, clearance)
    quat = _positions_to_yaw_quat(position)
    return Trajectory(t, position, quat)


def _catmull_rom(points: np.ndarray, knot_t: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Evaluate a non-uniform Catmull-Rom spline at times ``t``."""
    out = np.empty((t.size, 3), dtype=np.float64)
    for i, value in enumerate(t):
        # Find the segment containing this time.
        segment = int(np.searchsorted(knot_t, value, side="right") - 1)
        segment = min(max(segment, 0), points.shape[0] - 2)
        t0 = knot_t[segment]
        t1 = knot_t[segment + 1]
        u = 0.0 if t1 <= t0 else float(np.clip((value - t0) / (t1 - t0), 0.0, 1.0))
        p0 = points[segment]
        p1 = points[segment + 1]
        p_prev = points[max(segment - 1, 0)]
        p_next = points[min(segment + 2, points.shape[0] - 1)]
        # Catmull-Rom tangents in the local segment parameterisation.
        m0 = 0.5 * (p1 - p_prev)
        m1 = 0.5 * (p_next - p0)
        h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
        h10 = u**3 - 2.0 * u**2 + u
        h01 = -2.0 * u**3 + 3.0 * u**2
        h11 = u**3 - u**2
        out[i] = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
    return out


def _check_bounds(position: np.ndarray, low: np.ndarray, high: np.ndarray, clearance: float) -> None:
    margin = max(0.0, float(clearance))
    if np.any(position < low + margin) or np.any(position > high - margin):
        raise ValueError("trajectory leaves the free-flight bounds")


def generate_sequence(
    rng: np.random.Generator,
    scene_bounds: dict[str, Iterable[float]],
    trajectory_config: dict[str, Any],
    *,
    kind: str,
    duration_s: float | None = None,
    shape: str | None = None,
) -> Trajectory:
    """Generate either a canonical or a random trajectory from YAML config."""
    cfg = trajectory_config.get("trajectory", trajectory_config)
    canonical_cfg = trajectory_config.get("canonical", {})
    random_cfg = trajectory_config.get("random", {})
    sample_rate = float(cfg.get("sample_rate_hz", 100.0))
    clearance = float(cfg.get("min_clearance_m", 0.3))
    altitude = float(cfg.get("altitude_m", 1.0))
    speed = float(cfg.get("speed_mps", 0.8))

    if kind == "canonical":
        shapes = canonical_cfg.get("shapes", ["circle", "figure_eight", "square", "straight"])
        chosen = shape or str(rng.choice(shapes))
        radius_range = canonical_cfg.get("radius_m", [1.0, 2.0])
        speed_range = canonical_cfg.get("speed_mps", [0.4, 1.2])
        radius = float(rng.uniform(*radius_range))
        canonical_speed = float(rng.uniform(*speed_range))
        return canonical_trajectory(
            chosen,
            scene_bounds,
            sample_rate_hz=sample_rate,
            speed_mps=canonical_speed,
            altitude_m=altitude,
            clearance_m=clearance,
            radius_m=radius,
        )

    if kind == "random":
        if duration_s is None:
            length_range = cfg.get("test_length_s", [8.0, 20.0])
            duration_s = float(rng.uniform(*length_range))
        waypoint_range = random_cfg.get("waypoint_count", [4, 10])
        speed_range = random_cfg.get("speed_mps", [0.5, 2.0])
        return random_trajectory(
            scene_bounds,
            rng=rng,
            sample_rate_hz=sample_rate,
            duration_s=float(duration_s),
            altitude_m=altitude,
            clearance_m=clearance,
            waypoint_count=int(rng.integers(waypoint_range[0], waypoint_range[1] + 1)),
            speed_mps=float(rng.uniform(*speed_range)),
        )

    raise ValueError(f"unknown trajectory kind: {kind!r}")
