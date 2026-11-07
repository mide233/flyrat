"""Tests for the physical ToF frontend."""

from __future__ import annotations

import numpy as np

from tio_former.sim.tof import (
    add_ar1_noise,
    aggregate_grid,
    apply_missing_mask,
    causal_impute,
    depth_to_radial_range,
    simulate_frame,
    status_to_weight,
)


def _tof_config() -> dict:
    return {
        "horizontal_fov": 0.7853981633974483,
        "vertical_fov": 0.7853981633974483,
        "near_m": 0.015,
        "far_m": 4.0,
        "ar1_gamma": 0.9,
        "ar1_sigma_m": 0.01,
        "valid_target_status": [5, 9],
        "new_target_status": 10,
        "valid_weight": 1.0,
        "new_target_weight": 0.5,
        "dropout_probability": 0.0,
        "reflectivity_dropout": 0.0,
        "new_target_probability": 0.0,
    }


def test_depth_to_radial_range() -> None:
    depth = np.full((64, 64), 2.0, dtype=np.float32)
    ranges = depth_to_radial_range(
        depth,
        hfov=0.7853981633974483,
        vfov=0.7853981633974483,
        near_m=0.015,
        far_m=4.0,
    )
    assert ranges.shape == depth.shape
    assert np.isclose(ranges[32, 32], 2.0, atol=0.02)
    assert ranges[0, 0] > ranges[32, 32]
    assert np.all(np.isfinite(ranges))


def test_depth_out_of_range_is_invalid() -> None:
    depth = np.full((8, 8), 10.0, dtype=np.float32)
    ranges = depth_to_radial_range(depth, hfov=1.0, vfov=1.0, near_m=0.015, far_m=4.0)
    assert np.all(np.isnan(ranges))


def test_aggregate_grid_median() -> None:
    ranges = np.zeros((64, 64), dtype=np.float64)
    ranges[:, :] = np.arange(64)[None, :]
    grid = aggregate_grid(ranges, grid_width=8, grid_height=8)
    assert grid.shape == (8, 8)
    assert np.allclose(grid[:, 0], 3.5)
    assert np.allclose(grid[:, -1], 59.5)


def test_status_weight_mapping() -> None:
    status = np.array([[5, 9, 10, 0]], dtype=np.uint8)
    weights = status_to_weight(status, _tof_config())
    np.testing.assert_allclose(weights, [[1.0, 1.0, 0.5, 0.0]])


def test_ar1_noise_statistics() -> None:
    rng = np.random.default_rng(0)
    ranges = np.full((8, 8), 2.0)
    valid = np.ones((8, 8), dtype=bool)
    noisy, noise = add_ar1_noise(ranges, valid, rng, _tof_config())
    assert noisy.shape == ranges.shape
    assert np.all(np.isfinite(noisy))
    assert np.std(noise) > 0.0


def test_simulate_frame_shapes_and_statuses() -> None:
    rng = np.random.default_rng(0)
    ideal = np.full((8, 8), 2.0)
    result = simulate_frame(ideal, rng, _tof_config())
    assert result["range"].shape == (8, 8)
    assert result["status"].dtype == np.uint8
    assert set(np.unique(result["status"])).issubset({5, 10})
    assert np.all(result["weight"] > 0.0)


def test_missing_mask_and_causal_imputation() -> None:
    rng = np.random.default_rng(0)
    ranges = np.full((2, 8, 8), 2.0)
    weights = np.ones((2, 8, 8))
    masked, masked_weights = apply_missing_mask(
        ranges[0], weights[0], rng, ratio=0.25, mode="spatial_patch"
    )
    assert np.count_nonzero(masked_weights == 0.0) == 16
    frames = np.stack([masked, np.full((8, 8), 2.0)])
    weights = np.stack([masked_weights, np.ones((8, 8))])
    imputed = causal_impute(frames, weights)
    assert imputed.shape == frames.shape
    assert np.all(np.isfinite(imputed))
    valid = masked_weights > 0.0
    np.testing.assert_allclose(imputed[0][valid], masked[valid])
