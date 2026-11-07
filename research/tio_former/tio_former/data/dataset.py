"""Dataset container and validation for simulation sequences (M1: NPZ)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np

REQUIRED_ARRAYS = {
    "tof": ((None, 4, 8, 8), np.float32),
    "tof_ideal": ((None, 4, 8, 8), np.float32),
    "tof_status": ((None, 4, 8, 8), np.uint8),
    "tof_weight": ((None, 4, 8, 8), np.float32),
    "gt_pose": ((None, 7), np.float32),
    "gt_twist": ((None, 6), np.float32),
    "imu": ((None, 16, 6), np.float32),
    "t_tof": ((None,), np.float64),
    "t_imu": ((None, 16), np.float64),
}


def save_sequence(
    path: str | Path,
    arrays: dict[str, np.ndarray],
    metadata: dict[str, Any],
) -> Path:
    """Save one sequence as compressed NPZ plus a JSON sidecar."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    validate_sequence(arrays)
    np.savez_compressed(path, **arrays)
    meta_path = path.with_suffix(".json")
    meta_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    return path


def load_sequence(path: str | Path) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    path = Path(path)
    with np.load(path, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    meta_path = path.with_suffix(".json")
    metadata = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
    return arrays, metadata


def validate_sequence(arrays: dict[str, np.ndarray]) -> None:
    if not arrays:
        raise ValueError("sequence is empty")
    for key, (shape, dtype) in REQUIRED_ARRAYS.items():
        if key not in arrays:
            raise ValueError(f"missing array {key!r}")
        value = np.asarray(arrays[key])
        if value.dtype != dtype:
            raise ValueError(f"{key}: expected dtype {dtype}, got {value.dtype}")
        if value.ndim != len(shape):
            raise ValueError(f"{key}: expected {len(shape)} dims, got {value.ndim}")
        if shape[0] is not None and value.shape[0] != shape[0]:
            raise ValueError(f"{key}: expected first dim {shape[0]}, got {value.shape[0]}")
        for axis, expected in enumerate(shape):
            if expected is not None and value.shape[axis] != expected:
                raise ValueError(f"{key}: axis {axis} expected {expected}, got {value.shape[axis]}")
        if not np.issubdtype(value.dtype, np.integer) and not np.all(np.isfinite(value)):
            raise ValueError(f"{key}: contains non-finite values")
    t_tof = arrays["t_tof"]
    if np.any(np.diff(t_tof) <= 0.0):
        raise ValueError("t_tof must be strictly increasing")
    t_imu = arrays["t_imu"]
    if np.any(np.diff(t_imu, axis=1) <= 0.0):
        raise ValueError("t_imu windows must be strictly increasing")
    if arrays["tof"].shape[0] != t_tof.size:
        raise ValueError("tof and t_tof lengths differ")
    if arrays["gt_pose"].shape[0] != t_tof.size:
        raise ValueError("gt_pose and t_tof lengths differ")


def inspect_sequence(path: str | Path) -> str:
    arrays, metadata = load_sequence(path)
    validate_sequence(arrays)
    lines = [f"sequence: {path}"]
    if metadata:
        for key in ("scene", "kind", "shape", "seed", "duration_s", "path_length_m"):
            if key in metadata:
                lines.append(f"{key}: {metadata[key]}")
    lines.append(f"frames: {arrays['tof'].shape[0]}")
    lines.append(f"tof shape: {arrays['tof'].shape[1:]}")
    lines.append(f"imu window: {arrays['imu'].shape[1:]}")
    lines.append(
        f"duration: {float(arrays['t_tof'][-1] - arrays['t_tof'][0]):.3f} s"
    )
    valid = arrays["tof_status"] > 0
    lines.append(f"valid zone ratio: {float(valid.mean()):.3f}")
    if arrays["tof"].size:
        values = arrays["tof"][arrays["tof_weight"] > 0.0]
        if values.size:
            lines.append(
                f"range: min {float(values.min()):.3f} m, max {float(values.max()):.3f} m"
            )
    lines.append(f"gt start: {np.round(arrays['gt_pose'][0, :3], 3).tolist()}")
    lines.append(f"gt end: {np.round(arrays['gt_pose'][-1, :3], 3).tolist()}")
    return "\n".join(lines)


def split_records(
    records: Iterable[dict[str, Any]],
    *,
    seed: int = 0,
    train: float = 0.70,
    val: float = 0.15,
    test: float = 0.15,
) -> dict[str, list[dict[str, Any]]]:
    """Random sequence-level split with deterministic shuffling."""
    records = list(records)
    if abs(train + val + test - 1.0) > 1e-6:
        raise ValueError("train + val + test must equal 1")
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(records))
    n_train = int(round(train * len(records)))
    n_val = int(round(val * len(records)))
    train_records = [records[i] for i in order[:n_train]]
    val_records = [records[i] for i in order[n_train : n_train + n_val]]
    test_records = [records[i] for i in order[n_train + n_val :]]
    return {"train": train_records, "val": val_records, "test": test_records}


def leave_one_scene_out(records: Iterable[dict[str, Any]]) -> dict[str, dict[str, list[dict[str, Any]]]]:
    """Return one split per held-out scene."""
    records = list(records)
    scenes = sorted({str(record.get("scene", "")) for record in records})
    splits: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for scene in scenes:
        test = [record for record in records if str(record.get("scene", "")) == scene]
        rest = [record for record in records if str(record.get("scene", "")) != scene]
        splits[scene] = {"train": rest, "val": [], "test": test}
    return splits
