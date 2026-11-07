"""Tests for the NPZ dataset container."""

from __future__ import annotations

import numpy as np
import pytest

from tio_former.data.dataset import (
    load_sequence,
    save_sequence,
    split_records,
    validate_sequence,
)


def _arrays(frames: int = 5) -> dict[str, np.ndarray]:
    return {
        "tof": np.zeros((frames, 4, 8, 8), dtype=np.float32),
        "tof_ideal": np.zeros((frames, 4, 8, 8), dtype=np.float32),
        "tof_status": np.zeros((frames, 4, 8, 8), dtype=np.uint8),
        "tof_weight": np.zeros((frames, 4, 8, 8), dtype=np.float32),
        "gt_pose": np.zeros((frames, 7), dtype=np.float32),
        "gt_twist": np.zeros((frames, 6), dtype=np.float32),
        "imu": np.zeros((frames, 16, 6), dtype=np.float32),
        "t_tof": np.arange(frames, dtype=np.float64) / 15.0,
        "t_imu": np.arange(frames)[:, None] + np.linspace(0.0, 1.0, 16)[None, :],
    }


def test_save_load_validate(tmp_path) -> None:
    arrays = _arrays()
    arrays["gt_pose"][:, 3] = 1.0
    path = save_sequence(tmp_path / "seq.npz", arrays, {"scene": "corridor"})
    loaded, metadata = load_sequence(path)
    validate_sequence(loaded)
    assert metadata["scene"] == "corridor"
    for key in arrays:
        np.testing.assert_array_equal(loaded[key], arrays[key])


def test_invalid_dtype_rejected() -> None:
    arrays = _arrays()
    arrays["tof"] = arrays["tof"].astype(np.float64)
    with pytest.raises(ValueError, match="dtype"):
        validate_sequence(arrays)


def test_non_monotonic_time_rejected() -> None:
    arrays = _arrays()
    arrays["t_tof"][2] = arrays["t_tof"][1]
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_sequence(arrays)


def test_split_records() -> None:
    records = [{"id": index} for index in range(10)]
    splits = split_records(records, seed=0)
    assert len(splits["train"]) == 7
    assert len(splits["val"]) == 2
    assert len(splits["test"]) == 1
    ids = [item["id"] for group in splits.values() for item in group]
    assert sorted(ids) == list(range(10))
