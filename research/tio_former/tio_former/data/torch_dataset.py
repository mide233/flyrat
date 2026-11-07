"""PyTorch dataset over processed TIO-Former NPZ sequences."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import torch
from torch.utils.data import Dataset

from tio_former.data.dataset import load_sequence
from tio_former.utils.se3 import quat_to_matrix, so3_log

VIEWS = ("front", "left", "right", "down")


def _increments(poses: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Body-frame translation and rotation increments from absolute poses."""
    positions = poses[:, :3].astype(np.float64)
    quats = poses[:, 3:7].astype(np.float64)
    rotations = np.stack([quat_to_matrix(q) for q in quats], axis=0)
    delta_p = np.zeros((poses.shape[0], 3), dtype=np.float32)
    delta_phi = np.zeros((poses.shape[0], 3), dtype=np.float32)
    for index in range(1, poses.shape[0]):
        previous_rotation = rotations[index - 1].T
        delta_p[index] = (previous_rotation @ (positions[index] - positions[index - 1])).astype(np.float32)
        delta_phi[index] = so3_log(previous_rotation @ rotations[index]).astype(np.float32)
    return delta_p, delta_phi


def _augment_missing(
    tof: np.ndarray,
    weight: np.ndarray,
    rng: np.random.Generator,
    *,
    probability: float,
    ratios: list[float],
) -> None:
    if rng.random() > probability:
        return
    frames, views, height, width = tof.shape
    ratio = float(rng.choice(ratios))
    zones = height * width
    count = max(1, int(round(ratio * zones)))
    for frame in range(frames):
        for view in range(views):
            valid = np.flatnonzero(weight[frame, view].reshape(-1) > 0.0)
            if valid.size == 0:
                continue
            chosen = rng.choice(valid, size=min(count, valid.size), replace=False)
            tof[frame, view].reshape(-1)[chosen] = 0.0
            weight[frame, view].reshape(-1)[chosen] = 0.0


class SequenceWindows(Dataset):
    """Windowed sequences from one or more processed NPZ files.

    Each item contains the current and previous range images, reliability
    weights, IMU windows and body-frame increments aligned with the current
    ToF frame.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        split: str | None = None,
        window_frames: int | None = 64,
        stride: int | None = None,
        seed: int = 0,
        augment: bool = False,
        missing_probability: float = 0.6,
        missing_ratios: list[float] | None = None,
        max_sequences: int | None = None,
        files: list[str | Path] | None = None,
    ) -> None:
        self.root = Path(root)
        self.window_frames = window_frames
        self.stride = stride or (window_frames if window_frames else 1)
        self.augment = augment
        self.missing_probability = missing_probability
        self.missing_ratios = missing_ratios or [0.1, 0.3, 0.5]
        self.seed = seed
        self.files: list[Path] = []
        if files is not None:
            self.files = [Path(path).expanduser().resolve() for path in files]
        else:
            for path in sorted(self.root.rglob("*.npz")):
                meta = self._metadata(path)
                if split is None or str(meta.get("split", "")) == split:
                    self.files.append(path)
        if max_sequences is not None:
            self.files = self.files[:max_sequences]
        if not self.files:
            raise FileNotFoundError(f"no NPZ sequences under {self.root} for split {split!r}")
        self.windows: list[tuple[int, int, int]] = []
        for file_index, path in enumerate(self.files):
            with np.load(path, allow_pickle=False) as data:
                length = int(data["tof"].shape[0])
            if window_frames is None or length <= window_frames:
                self.windows.append((file_index, 0, length))
            else:
                for start in range(0, length - window_frames + 1, self.stride):
                    self.windows.append((file_index, start, window_frames))

    @staticmethod
    def _metadata(path: Path) -> dict[str, Any]:
        sidecar = path.with_suffix(".json")
        if not sidecar.exists():
            return {}
        return json.loads(sidecar.read_text(encoding="utf-8"))

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        file_index, start, length = self.windows[index]
        arrays, _ = load_sequence(self.files[file_index])
        end = start + length
        tof = arrays["tof"][start:end].astype(np.float32)
        weight = arrays["tof_weight"][start:end].astype(np.float32)
        imu = arrays["imu"][start:end].astype(np.float32)
        poses = arrays["gt_pose"].astype(np.float64)
        delta_p, delta_phi = _increments(poses)
        previous = np.empty_like(tof)
        previous_weight = np.empty_like(weight)
        if start == 0:
            previous[0] = tof[0]
            previous_weight[0] = weight[0]
        else:
            previous[0] = arrays["tof"][start - 1]
            previous_weight[0] = arrays["tof_weight"][start - 1]
        previous[1:] = tof[:-1]
        previous_weight[1:] = weight[:-1]
        if self.augment:
            rng = np.random.default_rng(self.seed + index)
            _augment_missing(
                tof,
                weight,
                rng,
                probability=self.missing_probability,
                ratios=self.missing_ratios,
            )
        return {
            "tof": torch.from_numpy(tof),
            "previous_tof": torch.from_numpy(previous),
            "weight": torch.from_numpy(weight),
            "previous_weight": torch.from_numpy(previous_weight),
            "imu": torch.from_numpy(imu),
            "delta_p": torch.from_numpy(delta_p[start:end]),
            "delta_phi": torch.from_numpy(delta_phi[start:end]),
            "gt_pose": torch.from_numpy(poses[start:end].astype(np.float32)),
        }


def iterate_batches(
    dataset: Dataset,
    batch_size: int,
    shuffle: bool = True,
    generator: torch.Generator | None = None,
) -> Iterator[dict[str, torch.Tensor]]:
    from torch.utils.data import DataLoader

    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        drop_last=False,
        generator=generator,
    )
    yield from loader
