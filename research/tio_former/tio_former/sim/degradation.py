"""Sensor degradation models matched to the real FlyRat sensor suite.

The simulator produces clean, well-lit, always-valid sensors.  The real vehicle
sees 54% valid ToF zones, a dark and motion-blurred camera, and a biased MEMS
IMU.  Training only on clean data makes the model over-confident and lets it
ignore whichever branch is degraded.  This module injects the measured real
statistics into each sequence while keeping the ground truth untouched.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def _uniform(rng: np.random.Generator, config: Any, default: tuple[float, float]) -> float:
    if config is None:
        return float(rng.uniform(*default))
    if isinstance(config, (int, float)):
        return float(config)
    values = [float(value) for value in config]
    return float(rng.uniform(values[0], values[1]))


def apply_tof_degradation(
    tof: np.ndarray,
    tof_ideal: np.ndarray,
    tof_status: np.ndarray,
    tof_weight: np.ndarray,
    rng: np.random.Generator,
    config: dict[str, Any] | None,
) -> dict[str, Any]:
    """Corrupt a ``[T, V, 8, 8]`` ToF stack in place.

    Models three real failure modes: per-view long-term health (one sensor can
    be mostly dead for a whole flight), fixed dead zones inside a view, and
    per-frame region/whole-frame dropouts.  Valid returns also get range noise
    and a small scale error.
    """
    if not config:
        return {"enabled": False}
    frames, views, height, width = tof.shape
    health_cfg = config.get("view_health", {})
    weak_probability = float(health_cfg.get("weak_probability", 0.15))
    weak_range = health_cfg.get("weak_range", [0.0, 0.2])
    normal_range = health_cfg.get("normal_range", [0.85, 1.0])
    health = np.ones(views, dtype=np.float64)
    for view in range(views):
        if float(rng.random()) < weak_probability:
            health[view] = float(rng.uniform(*weak_range))
        else:
            health[view] = float(rng.uniform(*normal_range))

    dead_cfg = config.get("dead_zones", {})
    dead_count = int(rng.integers(*dead_cfg.get("count", [0, 2])))
    dead_size = dead_cfg.get("size_cells", [1, 3])
    dead_masks = np.zeros((views, height, width), dtype=bool)
    for _ in range(dead_count):
        view = int(rng.integers(0, views))
        rows = int(rng.integers(dead_size[0], dead_size[1] + 1))
        cols = int(rng.integers(dead_size[0], dead_size[1] + 1))
        top = int(rng.integers(0, max(1, height - rows + 1)))
        left = int(rng.integers(0, max(1, width - cols + 1)))
        dead_masks[view, top : top + rows, left : left + cols] = True

    frame_cfg = config.get("frame_dropout", {})
    whole_probability = float(frame_cfg.get("whole_frame_probability", 0.06))
    block_probability = float(frame_cfg.get("block_probability", 0.25))
    block_ratio = frame_cfg.get("block_ratio", [0.25, 0.55])
    block_cells = frame_cfg.get("block_size_cells", [2, 4])

    scale = _uniform(rng, config.get("range_scale", [0.97, 1.03]), (0.97, 1.03))
    noise_std = float(config.get("range_noise_std_m", 0.02))
    min_range = float(config.get("min_range_m", 0.15))

    valid_before = float((tof_weight > 0.0).mean())
    for frame in range(frames):
        for view in range(views):
            valid = tof_weight[frame, view] > 0.0
            if not np.any(valid):
                continue
            if dead_masks[view].any():
                dead = dead_masks[view]
                tof[frame, view][dead] = 0.0
                tof_weight[frame, view][dead] = 0.0
                tof_status[frame, view][dead] = 0
                valid = tof_weight[frame, view] > 0.0
            if float(rng.random()) < whole_probability:
                tof[frame, view] = 0.0
                tof_weight[frame, view] = 0.0
                tof_status[frame, view] = 0
                continue
            # Long-term health scales how often zones survive frame to frame.
            if health[view] < 1.0:
                keep = rng.random((height, width)) < health[view]
                drop = valid & ~keep
                tof[frame, view][drop] = 0.0
                tof_weight[frame, view][drop] = 0.0
                tof_status[frame, view][drop] = 0
                valid = tof_weight[frame, view] > 0.0
            if float(rng.random()) < block_probability and np.any(valid):
                ratio = float(rng.uniform(*block_ratio))
                rows = int(rng.integers(block_cells[0], block_cells[1] + 1))
                cols = int(rng.integers(block_cells[0], block_cells[1] + 1))
                top = int(rng.integers(0, max(1, height - rows + 1)))
                left = int(rng.integers(0, max(1, width - cols + 1)))
                block = np.zeros((height, width), dtype=bool)
                block[top : top + rows, left : left + cols] = True
                if ratio < 1.0:
                    block |= rng.random((height, width)) < (ratio * 0.25)
                drop = valid & block
                tof[frame, view][drop] = 0.0
                tof_weight[frame, view][drop] = 0.0
                tof_status[frame, view][drop] = 0
            valid = tof_weight[frame, view] > 0.0
            if np.any(valid):
                updated = tof[frame, view][valid] * scale + rng.normal(0.0, noise_std, valid.sum())
                tof[frame, view][valid] = np.maximum(min_range, updated)

    valid_after = float((tof_weight > 0.0).mean())
    return {
        "enabled": True,
        "view_health": [round(float(value), 3) for value in health],
        "range_scale": round(scale, 4),
        "valid_before": round(valid_before, 4),
        "valid_after": round(valid_after, 4),
        "dead_cells": int(dead_masks.sum()),
    }


def apply_camera_degradation(
    image: np.ndarray,
    angular_speed: float,
    rng: np.random.Generator,
    config: dict[str, Any] | None,
    state: dict[str, float],
) -> np.ndarray:
    """Darken, blur, vignette and noise one mono frame.

    ``state`` carries per-sequence values (gain, brightness, vignette) sampled
    once so consecutive frames stay photometrically consistent.
    """
    if not config:
        return image

    output = image.astype(np.float32) / 255.0
    output = output * state["brightness"] * state["gain"]
    # One auto-exposure decision per sequence (the real camera runs fixed
    # exposure, so scene/lighting changes must remain visible frame to frame).
    if "exposure_scale" not in state:
        target = float(state.get("target_brightness", 30.0)) / 255.0
        current = float(output.mean())
        state["exposure_scale"] = (
            float(np.clip(target / current, 0.25, 4.0)) if current > 1e-4 else 1.0
        )
    output = output * state["exposure_scale"]
    blur_gain = float(config.get("blur_gain", 1.0))
    exposure = float(config.get("exposure_s", 0.005))
    focal = float(config.get("focal_px", 140.0))
    blur_px = abs(float(angular_speed)) * exposure * focal * blur_gain
    blur_px = float(np.clip(blur_px, 0.0, float(config.get("blur_max_px", 8.0))))
    if blur_px > 0.4:
        radius = 0.5 * blur_px
        try:
            import cv2
        except ImportError:  # pragma: no cover - PIL fallback
            from PIL import Image, ImageFilter

            output = np.asarray(
                Image.fromarray(np.clip(output * 255.0, 0, 255).astype(np.uint8)).filter(
                    ImageFilter.GaussianBlur(radius=radius)
                ),
                dtype=np.float32,
            ) / 255.0
        else:
            ksize = int(max(3, 2 * round(radius) + 1))
            output = cv2.GaussianBlur(output, (ksize, ksize), sigmaX=radius, sigmaY=radius)
    rows, cols = output.shape
    yy = np.linspace(-1.0, 1.0, rows)[:, None]
    xx = np.linspace(-1.0, 1.0, cols)[None, :]
    radius_sq = yy * yy + xx * xx
    vignette = 1.0 - state["vignette"] * np.clip(radius_sq / 2.0, 0.0, 1.0)
    output = output * vignette
    read_noise = float(state["read_noise"])
    output = output + rng.normal(0.0, read_noise, output.shape)
    output = np.clip(output, 0.0, 1.0)
    hot_ratio = float(config.get("hot_pixel_ratio", 0.0001))
    if hot_ratio > 0.0:
        count = int(hot_ratio * output.size)
        if count > 0:
            flat = output.reshape(-1)
            index = rng.integers(0, flat.size, count)
            flat[index] = 1.0
    return (output * 255.0).astype(np.uint8)


def camera_state(rng: np.random.Generator, config: dict[str, Any] | None) -> dict[str, float]:
    if not config:
        return {"brightness": 1.0, "gain": 1.0, "vignette": 0.0, "read_noise": 0.0}
    gain_jitter = float(config.get("gain_jitter", 0.2))
    return {
        "brightness": _uniform(rng, config.get("brightness", [0.5, 0.9]), (0.5, 0.9)),
        "gain": float(1.0 + rng.uniform(-gain_jitter, gain_jitter)),
        "vignette": _uniform(rng, config.get("vignette", [0.1, 0.25]), (0.1, 0.25)),
        "read_noise": _uniform(rng, config.get("read_noise", [0.008, 0.02]), (0.008, 0.02)),
        "target_brightness": (
            float(rng.uniform(75.0, 100.0))
            if rng.random() < 0.15
            else float(rng.uniform(50.0, 75.0))
        ),
    }


def apply_imu_degradation(
    accel: np.ndarray,
    gyro: np.ndarray,
    rng: np.random.Generator,
    config: dict[str, Any] | None,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Add per-sequence scale error, axis misalignment and residual offset."""
    if not config:
        return accel, gyro, {"enabled": False}
    accel_scale = _uniform(rng, config.get("accel_scale", [0.995, 1.005]), (0.995, 1.005))
    gyro_scale = _uniform(rng, config.get("gyro_scale", [0.99, 1.01]), (0.99, 1.01))
    tilt = float(np.radians(_uniform(rng, config.get("misalignment_deg", [0.2, 1.0]), (0.2, 1.0))))
    axis = rng.normal(size=3)
    axis /= max(1e-9, float(np.linalg.norm(axis)))
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    rotation = np.eye(3) + np.sin(tilt) * skew + (1.0 - np.cos(tilt)) * (skew @ skew)
    offset_acc = rng.normal(0.0, float(config.get("extra_accel_bias", 0.03)), size=3)
    offset_gyro = rng.normal(0.0, float(config.get("extra_gyro_bias", 0.003)), size=3)
    accel_out = (accel @ rotation.T) * accel_scale + offset_acc
    gyro_out = (gyro @ rotation.T) * gyro_scale + offset_gyro
    return accel_out, gyro_out, {
        "enabled": True,
        "accel_scale": round(accel_scale, 5),
        "gyro_scale": round(gyro_scale, 5),
        "misalignment_deg": round(float(np.degrees(tilt)), 3),
        "extra_accel_bias": [round(float(value), 4) for value in offset_acc],
        "extra_gyro_bias": [round(float(value), 5) for value in offset_gyro],
    }
