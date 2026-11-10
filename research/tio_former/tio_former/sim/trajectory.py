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

from tio_former.utils.se3 import quat_slerp, yaw_to_quat

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


def retime_to_speed(
    trajectory: Trajectory,
    target_speed_mps: float,
    *,
    sample_rate_hz: float = 100.0,
) -> Trajectory:
    """Stretch a trajectory to an exact average speed without changing its path.

    The input path is preserved; only the time parameterisation changes.  This
    is used by ``generate_dataset.py`` to make every split share one speed
    distribution instead of relying on the duration ranges of the raw
    generator.
    """
    if target_speed_mps <= 0.0:
        raise ValueError("target_speed_mps must be positive")
    if sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    path_length = trajectory.path_length
    if path_length <= 1e-6:
        raise ValueError("cannot retime a zero-length trajectory")

    duration = path_length / target_speed_mps
    count = max(2, int(round(duration * sample_rate_hz)) + 1)
    t = np.linspace(0.0, duration, count)
    source_t = trajectory.t - trajectory.t[0]
    old_duration = float(source_t[-1])
    if old_duration <= 0.0:
        raise ValueError("trajectory has zero duration")
    query = (t / duration) * old_duration
    position = np.column_stack(
        [np.interp(query, source_t, trajectory.position[:, axis]) for axis in range(3)]
    )

    indices = np.searchsorted(source_t, query, side="right") - 1
    indices = np.clip(indices, 0, source_t.size - 2)
    denominator = source_t[indices + 1] - source_t[indices]
    alpha = np.where(denominator > 0.0, (query - source_t[indices]) / denominator, 0.0)
    quat = np.stack(
        [
            quat_slerp(trajectory.quat[i], trajectory.quat[i + 1], float(a))
            for i, a in zip(indices, alpha)
        ],
        axis=0,
    )
    return Trajectory(t, position, quat)


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
    obstacles: list[dict[str, Any]] | None = None,
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
    if obstacles:
        _check_obstacles(position, obstacles, clearance)
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
    vertical_jitter_m: float = 0.12,
    waypoint_step_m: Iterable[float] | None = None,
    obstacles: list[dict[str, Any]] | None = None,
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
        # Start somewhere in the free volume and take bounded random steps.
        # This produces local, drone-like motion instead of long chords across
        # the whole room, which keeps the average speed near the paper's dataset.
        xy[0] = rng.uniform(inner_low[:2], inner_high[:2])
        steps = waypoint_step_m if waypoint_step_m is not None else (0.6, 2.0)
        step_low, step_high = (float(v) for v in steps)
        for i in range(1, count):
            for _ in range(200):
                angle = float(rng.uniform(0.0, _TWO_PI))
                step = float(rng.uniform(step_low, step_high))
                candidate = xy[i - 1] + step * np.array([np.cos(angle), np.sin(angle)])
                if np.all(candidate >= inner_low[:2]) and np.all(candidate <= inner_high[:2]):
                    xy[i] = candidate
                    break
            else:
                # Fall back to a clipped step when the walk reaches a corner.
                xy[i] = np.clip(xy[i - 1] + step * np.array([np.cos(angle), np.sin(angle)]),
                                inner_low[:2], inner_high[:2])
    count = xy.shape[0]
    if count < 2:
        raise ValueError("at least two waypoints are required")

    # Give every waypoint a small vertical offset while staying inside bounds.
    z = np.clip(
        altitude + rng.normal(0.0, max(0.0, float(vertical_jitter_m)), size=count),
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
    if obstacles:
        _check_obstacles(position, obstacles, clearance)
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


def _check_obstacles(
    position: np.ndarray,
    obstacles: list[dict[str, Any]],
    clearance: float,
) -> None:
    """Reject a trajectory that enters a conservative obstacle AABB."""
    margin = max(0.0, float(clearance))
    for obstacle in obstacles:
        center = np.asarray(obstacle["center"], dtype=np.float64)
        half = 0.5 * np.asarray(obstacle["size"], dtype=np.float64) + margin
        if center.shape != (3,) or half.shape != (3,):
            raise ValueError("obstacle center/size must be 3-D")
        if np.any(np.all(np.abs(position - center) <= half, axis=1)):
            raise ValueError(f"trajectory intersects obstacle {obstacle.get('name', '?')}")


def generate_sequence(
    rng: np.random.Generator,
    scene_bounds: dict[str, Iterable[float]],
    trajectory_config: dict[str, Any],
    *,
    kind: str,
    duration_s: float | None = None,
    shape: str | None = None,
    obstacles: list[dict[str, Any]] | None = None,
    radius_range: Iterable[float] | None = None,
    speed_range: Iterable[float] | None = None,
    altitude_m: float | None = None,
    vertical_jitter_m: float | None = None,
    waypoint_step_m: Iterable[float] | None = None,
    waypoint_count: Iterable[int] | None = None,
    attempts: int = 80,
) -> Trajectory:
    """Generate either a canonical or a random trajectory from YAML config.

    ``obstacles`` is a list of conservative AABBs (``center``/``size``) from
    :func:`tio_former.sim.scene.scene_obstacles`.  Generation retries with new
    random parameters until the sampled path clears every obstacle.
    """
    cfg = trajectory_config.get("trajectory", trajectory_config)
    canonical_cfg = trajectory_config.get("canonical", {})
    random_cfg = trajectory_config.get("random", {})
    sample_rate = float(cfg.get("sample_rate_hz", 100.0))
    clearance = float(cfg.get("min_clearance_m", 0.3))
    altitude = float(altitude_m if altitude_m is not None else cfg.get("altitude_m", 1.0))
    jitter = float(
        vertical_jitter_m
        if vertical_jitter_m is not None
        else random_cfg.get("vertical_jitter_m", 0.12)
    )

    if kind not in {"canonical", "random"}:
        raise ValueError(f"unknown trajectory kind: {kind!r}")

    last_error: Exception | None = None
    for _attempt in range(max(1, int(attempts))):
        try:
            attempt_altitude = altitude + float(rng.uniform(-0.15, 0.15))
            if kind == "canonical":
                shapes = canonical_cfg.get(
                    "shapes", ["circle", "figure_eight", "square", "straight"]
                )
                chosen = shape or str(rng.choice(shapes))
                radii = radius_range or canonical_cfg.get("radius_m", [1.0, 2.0])
                speeds = speed_range or canonical_cfg.get("speed_mps", [0.4, 1.2])
                radius = float(rng.uniform(*radii))
                canonical_speed = float(rng.uniform(*speeds))
                low, high = _bounds(scene_bounds)
                margin = clearance + radius
                if np.any(high[:2] - low[:2] <= 2.0 * margin):
                    center_xy = 0.5 * (low[:2] + high[:2])
                else:
                    center_xy = rng.uniform(low[:2] + margin, high[:2] - margin)
                return canonical_trajectory(
                    chosen,
                    scene_bounds,
                    sample_rate_hz=sample_rate,
                    speed_mps=canonical_speed,
                    altitude_m=attempt_altitude,
                    clearance_m=clearance,
                    radius_m=radius,
                    center_xy=center_xy,
                    duration_s=duration_s,
                    obstacles=obstacles,
                )

            if duration_s is None:
                test_lengths = cfg.get("test_length_s", [8.0, 20.0])
                # Callers generating training data pass duration explicitly;
                # the default follows the test-length distribution.
                duration = float(rng.uniform(*test_lengths))
            else:
                duration = float(duration_s)
            waypoint_range = waypoint_count or random_cfg.get("waypoint_count", [4, 10])
            speeds = speed_range or random_cfg.get("speed_mps", [0.5, 2.0])
            return random_trajectory(
                scene_bounds,
                rng=rng,
                sample_rate_hz=sample_rate,
                duration_s=duration,
                altitude_m=attempt_altitude,
                clearance_m=clearance,
                waypoint_count=int(rng.integers(waypoint_range[0], waypoint_range[1] + 1)),
                speed_mps=float(rng.uniform(*speeds)),
                vertical_jitter_m=jitter,
                waypoint_step_m=waypoint_step_m,
                obstacles=obstacles,
            )
        except ValueError as exc:
            last_error = exc

    raise ValueError(
        f"failed to generate a collision-free {kind} trajectory after {attempts} attempts: {last_error}"
    )


def _min_jerk(alpha: np.ndarray) -> np.ndarray:
    """Zero-velocity/zero-acceleration interpolation between two keyframes."""
    return alpha * alpha * alpha * (10.0 + alpha * (-15.0 + 6.0 * alpha))


def inspection_trajectory(
    scene_bounds: dict[str, Iterable[float]],
    *,
    rng: np.random.Generator,
    sample_rate_hz: float,
    duration_s: float,
    altitude_m: float,
    clearance_m: float,
    obstacles: list[dict[str, Any]] | None = None,
    motion_mix: dict[str, float] | None = None,
    lateral_excursion_m: float = 0.4,
    vertical_excursion_m: float = 0.25,
    max_speed_mps: float = 1.0,
    return_to_start: bool = True,
) -> Trajectory:
    """Stop-and-go inspection motion with real static dwell.

    The sequence is a chain of holds and smooth moves.  Moves use a minimum
    jerk profile, so velocity and acceleration are exactly zero at every hold
    boundary; this gives the model genuine stationary samples instead of the
    old constant 0.3 m/s crawl that caused the real-vehicle velocity bias.
    """
    if sample_rate_hz <= 0.0 or duration_s <= 0.0:
        raise ValueError("sample_rate_hz and duration_s must be positive")
    low, high = _bounds(scene_bounds)
    clearance = max(0.0, float(clearance_m))
    inner_low = low + clearance
    inner_high = high - clearance
    if np.any(inner_high <= inner_low):
        raise ValueError("scene is too small for the requested clearance")

    mix = motion_mix or {"static": 0.3, "micro": 0.2, "low": 0.3, "normal": 0.2}
    names = list(mix)
    weights = np.asarray([max(0.0, float(mix[name])) for name in names], dtype=np.float64)
    if weights.sum() <= 0.0:
        raise ValueError("motion_mix must have positive total weight")
    weights /= weights.sum()

    start = np.array(
        [
            rng.uniform(max(inner_low[0], -lateral_excursion_m), min(inner_high[0], lateral_excursion_m)),
            rng.uniform(max(inner_low[1], -lateral_excursion_m), min(inner_high[1], lateral_excursion_m)),
            np.clip(altitude_m, inner_low[2], inner_high[2]),
        ],
        dtype=np.float64,
    )
    lateral = max(0.05, float(lateral_excursion_m))
    vertical = max(0.03, float(vertical_excursion_m))

    key_t: list[float] = [0.0]
    key_p: list[np.ndarray] = [start.copy()]
    t = 0.0
    position = start.copy()
    stagnant = 0
    while t < duration_s - 3.0:
        name = str(rng.choice(names, p=weights))
        remaining = duration_s - t
        if name == "static":
            length = float(rng.uniform(0.8, 2.0))
            length = min(length, remaining)
            t += length
            key_t.append(t)
            key_p.append(position.copy())
            continue
        if name == "micro":
            length = float(rng.uniform(1.5, 3.5))
            distance = float(rng.uniform(0.02, 0.12))
        elif name == "low":
            speed = float(rng.uniform(0.05, 0.2))
            length = float(rng.uniform(2.0, 5.0))
            distance = speed * length / 1.875
        else:
            speed = float(rng.uniform(0.25, max_speed_mps))
            length = float(rng.uniform(1.5, 4.0))
            # Minimum-jerk peak speed is 1.875x the mean, so budget the
            # distance from the mean instead of the requested peak.
            distance = speed * length / 1.875
        planned = length
        if return_to_start:
            # Keep enough time for a final return that does not exceed the
            # speed limit (minimum-jerk peak is 1.875x the mean speed).
            needed = (float(np.linalg.norm(position - start)) + distance) / 0.5
            if t + length + needed > duration_s - 1.0:
                break
        length = min(length, remaining)
        if length < 0.4:
            break
        # Reserve the end of the sequence for a final return-to-start move.
        if return_to_start and t + length > duration_s - 1.2:
            length = max(0.4, duration_s - 1.2 - t)
        distance *= length / max(planned, 1e-6)
        target = position.copy()
        for _ in range(60):
            angle = float(rng.uniform(-np.pi, np.pi))
            candidate = position + distance * np.array([np.cos(angle), np.sin(angle), 0.0])
            candidate[2] = np.clip(
                candidate[2] + float(rng.uniform(-vertical, vertical)),
                start[2] - vertical,
                start[2] + vertical,
            )
            offsets = candidate[:2] - start[:2]
            if np.linalg.norm(offsets) <= lateral:
                target = candidate
                break
        else:
            target = position.copy()
        if np.linalg.norm(target - position) < 1e-3:
            # The excursion bound rejected every direction; try another phase
            # instead of inserting an unplanned hold.
            stagnant += 1
            if stagnant > 12:
                break
            continue
        position = target
        stagnant = 0
        t += length
        key_t.append(t)
        key_p.append(position.copy())

    if return_to_start and np.linalg.norm(position - start) > 1e-3:
        distance = float(np.linalg.norm(position - start))
        required = distance * 1.875 / max(0.1, max_speed_mps)
        length = float(np.clip(max(distance / 0.20, required), 1.0, 8.0))
        available = max(0.6, duration_s - t)
        if length > available:
            length = available
        t += length
        key_t.append(t)
        key_p.append(start.copy())
    if t < duration_s:
        key_t.append(duration_s)
        key_p.append(key_p[-1].copy())

    sample_count = max(2, int(round(duration_s * sample_rate_hz)) + 1)
    times = np.linspace(0.0, duration_s, sample_count)
    knots = np.asarray(key_t, dtype=np.float64)
    points = np.asarray(key_p, dtype=np.float64)
    position_out = np.empty((times.size, 3), dtype=np.float64)
    segment = 0
    for index, value in enumerate(times):
        while segment + 1 < knots.size - 1 and value > knots[segment + 1]:
            segment += 1
        t0, t1 = knots[segment], knots[segment + 1]
        alpha = 0.0 if t1 <= t0 else float(np.clip((value - t0) / (t1 - t0), 0.0, 1.0))
        position_out[index] = points[segment] + _min_jerk(np.asarray(alpha)) * (
            points[segment + 1] - points[segment]
        )

    position_out[:, 0] = np.clip(position_out[:, 0], inner_low[0], inner_high[0])
    position_out[:, 1] = np.clip(position_out[:, 1], inner_low[1], inner_high[1])
    position_out[:, 2] = np.clip(position_out[:, 2], inner_low[2], inner_high[2])
    _check_bounds(position_out, low, high, clearance)
    if obstacles:
        _check_obstacles(position_out, obstacles, clearance)
    quat = _positions_to_yaw_quat(position_out)
    return Trajectory(times, position_out, quat)
