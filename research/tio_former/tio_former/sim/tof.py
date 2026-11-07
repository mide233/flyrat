"""Physical ToF simulation and reliability frontend utilities.

Gazebo renders one depth image per view.  This module converts the optical-axis
depth to radial range, aggregates the 64x64 render into the physical 8x8 grid,
and applies the paper's noise/status/degradation model.

Nothing here is model-specific and everything is NumPy-only so it can run on
the data-generation node without PyTorch.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def _focal_lengths(width: int, height: int, hfov: float, vfov: float) -> tuple[float, float, float, float]:
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not (0.0 < hfov < np.pi) or not (0.0 < vfov < np.pi):
        raise ValueError("field of view must be in (0, pi)")
    fx = 0.5 * width / np.tan(0.5 * hfov)
    fy = 0.5 * height / np.tan(0.5 * vfov)
    cx = 0.5 * (width - 1)
    cy = 0.5 * (height - 1)
    return fx, fy, cx, cy


def depth_to_radial_range(
    depth: np.ndarray,
    *,
    hfov: float,
    vfov: float,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Convert an optical-axis depth image (metres) to radial range.

    Gazebo's depth camera returns the Z coordinate in the camera optical frame.
    The physical ToF returns the Euclidean distance to the target, so each pixel
    is scaled by ``||(x/fx, y/fy, 1)||``.
    """
    depth = np.asarray(depth, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError(f"depth must be 2-D, got shape {depth.shape}")
    height, width = depth.shape
    fx, fy, cx, cy = _focal_lengths(width, height, hfov, vfov)
    u = np.arange(width, dtype=np.float64)[None, :]
    v = np.arange(height, dtype=np.float64)[:, None]
    scale = np.sqrt(((u - cx) / fx) ** 2 + ((v - cy) / fy) ** 2 + 1.0)
    ranges = depth * scale
    invalid = ~np.isfinite(ranges) | (ranges < near_m) | (ranges > far_m)
    ranges = np.where(invalid, np.nan, ranges)
    return ranges


def aggregate_grid(
    ranges: np.ndarray,
    *,
    grid_width: int = 8,
    grid_height: int = 8,
) -> np.ndarray:
    """Aggregate a dense range image to ``grid_height x grid_width`` zones.

    The per-zone median is robust to depth discontinuities inside one FoV zone,
    which is a reasonable stand-in for a multi-zone ToF module.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    if ranges.ndim != 2:
        raise ValueError(f"ranges must be 2-D, got shape {ranges.shape}")
    if grid_width <= 0 or grid_height <= 0:
        raise ValueError("grid dimensions must be positive")
    height, width = ranges.shape
    out = np.full((grid_height, grid_width), np.nan, dtype=np.float64)
    for gy in range(grid_height):
        y0 = int(round(gy * height / grid_height))
        y1 = int(round((gy + 1) * height / grid_height))
        for gx in range(grid_width):
            x0 = int(round(gx * width / grid_width))
            x1 = int(round((gx + 1) * width / grid_width))
            block = ranges[y0:y1, x0:x1]
            valid = block[np.isfinite(block)]
            if valid.size:
                out[gy, gx] = float(np.median(valid))
    return out


def status_to_weight(status: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    """Map sensor status codes to the paper's reliability weights."""
    status = np.asarray(status, dtype=np.uint8)
    valid = np.isin(status, np.asarray(config.get("valid_target_status", [5, 9]), dtype=np.uint8))
    new_target = status == int(config.get("new_target_status", 10))
    weight = np.zeros(status.shape, dtype=np.float64)
    weight[valid] = float(config.get("valid_weight", 1.0))
    weight[new_target] = float(config.get("new_target_weight", 0.5))
    return weight


def add_ar1_noise(
    ranges: np.ndarray,
    valid: np.ndarray,
    rng: np.random.Generator,
    config: dict[str, Any],
    previous_noise: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply coloured AR(1) range noise to valid zones only."""
    ranges = np.asarray(ranges, dtype=np.float64)
    valid = np.asarray(valid, dtype=bool)
    gamma = float(config.get("ar1_gamma", 0.9))
    sigma = float(config.get("ar1_sigma_m", 0.01))
    if not (0.0 <= gamma < 1.0):
        raise ValueError("ar1_gamma must be in [0, 1)")
    if sigma < 0.0:
        raise ValueError("ar1_sigma_m must be non-negative")
    if previous_noise is None:
        previous = np.zeros(ranges.shape, dtype=np.float64)
    else:
        previous = np.asarray(previous_noise, dtype=np.float64)
        if previous.shape != ranges.shape:
            raise ValueError("previous_noise shape does not match ranges")
    innovation = rng.normal(0.0, sigma * np.sqrt(max(1.0 - gamma * gamma, 0.0)), size=ranges.shape)
    noise = gamma * previous + innovation
    out = np.where(valid, ranges + noise, 0.0)
    noise = np.where(valid, noise, 0.0)
    return out, noise


def simulate_frame(
    ideal_range: np.ndarray,
    rng: np.random.Generator,
    config: dict[str, Any],
    previous_noise: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Simulate one ToF acquisition from ideal 8x8 ranges.

    Returns ``range``, ``status``, ``weight``, ``valid`` and ``noise`` arrays.
    Out-of-range zones, random dropouts and specular dropouts become invalid
    (status 0, weight 0).  A small fraction of otherwise valid zones is marked
    as a newly acquired target (status 10, weight 0.5), mimicking the paper's
    TOFSense-M temporal-continuity flag.
    """
    ideal = np.asarray(ideal_range, dtype=np.float64)
    if ideal.ndim < 2:
        raise ValueError("ideal_range must have at least two dimensions")
    near = float(config.get("near_m", 0.015))
    far = float(config.get("far_m", 4.0))
    valid = np.isfinite(ideal) & (ideal >= near) & (ideal <= far)
    status = np.zeros(ideal.shape, dtype=np.uint8)
    status[valid] = 5

    dropout_probability = float(config.get("dropout_probability", 0.0))
    reflectivity_dropout = float(config.get("reflectivity_dropout", 0.0))
    if dropout_probability > 0.0:
        drop = rng.random(ideal.shape) < dropout_probability
        valid &= ~drop
    if reflectivity_dropout > 0.0:
        drop = rng.random(ideal.shape) < reflectivity_dropout
        valid &= ~drop
    status[~valid] = 0

    new_target_probability = float(config.get("new_target_probability", 0.02))
    if new_target_probability > 0.0:
        new_target = valid & (rng.random(ideal.shape) < new_target_probability)
        status[new_target] = int(config.get("new_target_status", 10))

    weight = status_to_weight(status, config)
    range_with_noise, noise = add_ar1_noise(ideal, valid, rng, config, previous_noise)
    # Invalid zones carry no return.  Keep the ideal array separate for
    # diagnostics and for the reliability-only ablations.  Physical returns are
    # bounded by the sensor clip range even after noise.
    range_with_noise = np.where(valid, np.clip(range_with_noise, near, far), 0.0)
    return {
        "range": range_with_noise.astype(np.float32),
        "ideal": np.where(np.isfinite(ideal), ideal, 0.0).astype(np.float32),
        "status": status,
        "weight": weight.astype(np.float32),
        "valid": valid,
        "noise": noise.astype(np.float32),
    }


def apply_missing_mask(
    ranges: np.ndarray,
    weights: np.ndarray,
    rng: np.random.Generator,
    *,
    ratio: float,
    mode: str = "spatial_patch",
) -> tuple[np.ndarray, np.ndarray]:
    """Zero a fraction of currently valid zones (training augmentation)."""
    if not (0.0 <= ratio <= 1.0):
        raise ValueError("ratio must be in [0, 1]")
    ranges = np.asarray(ranges, dtype=np.float32).copy()
    weights = np.asarray(weights, dtype=np.float32).copy()
    valid = np.argwhere(weights > 0.0)
    if valid.size == 0 or ratio == 0.0:
        return ranges, weights
    count = max(1, int(np.floor(ratio * valid.shape[0])))
    count = min(count, valid.shape[0])
    if mode == "discrete":
        chosen = valid[rng.choice(valid.shape[0], size=count, replace=False)]
    elif mode == "temporal_burst":
        # Drop a contiguous time-like slice if a leading axis exists.
        flat = valid.reshape(-1, valid.shape[-1]) if valid.ndim > 1 else valid
        chosen = valid[rng.choice(valid.shape[0], size=count, replace=False)]
    else:  # spatial_patch
        # Pick a seed zone, then take the nearest valid zones in grid space.
        seed = valid[rng.integers(valid.shape[0])]
        order = np.argsort(np.sum((valid - seed) ** 2, axis=1))
        chosen = valid[order[:count]]
    for index in chosen:
        ranges[tuple(index)] = 0.0
        weights[tuple(index)] = 0.0
    return ranges, weights


def causal_impute(
    frames: np.ndarray,
    weights: np.ndarray,
) -> np.ndarray:
    """Causal spatial/temporal imputation of invalid zones.

    ``frames`` has shape ``[T, ...]``.  Invalid zones are filled from the same
    zone in the previous frame when available, then from the median of valid
    spatial neighbours in the current frame.  The reliability weights are left
    untouched, matching the paper's "imputed but untrusted" convention.
    """
    frames = np.asarray(frames, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    if frames.shape != weights.shape:
        raise ValueError("frames and weights must have the same shape")
    out = frames.copy()
    if out.ndim < 3:
        raise ValueError("expected [T, H, W] frames")
    for t in range(out.shape[0]):
        current = out[t]
        if t > 0:
            missing = weights[t] <= 0.0
            previous = out[t - 1]
            current = np.where(missing, previous, current)
        missing = weights[t] <= 0.0
        if np.any(missing):
            h, w = current.shape
            filled = current.copy()
            for y, x in np.argwhere(missing):
                y0, y1 = max(0, y - 1), min(h, y + 2)
                x0, x1 = max(0, x - 1), min(w, x + 2)
                neighbours = current[y0:y1, x0:x1][weights[t, y0:y1, x0:x1] > 0.0]
                neighbours = neighbours[np.isfinite(neighbours)]
                filled[y, x] = float(np.median(neighbours)) if neighbours.size else 0.0
            current = filled
        out[t] = current
    return out
