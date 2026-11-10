"""PyTorch dataset over processed TIO-Former NPZ sequences."""

from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import torch
from PIL import Image
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


def _decode_camera(
    blob: np.ndarray,
    offsets: np.ndarray,
    start: int,
    end: int,
    fallback: tuple[int, int],
) -> np.ndarray:
    """Decode JPEG slices into a [T, H, W] uint8 grayscale window."""
    height, width = fallback
    frames: list[np.ndarray] = []
    for index in range(start, end):
        begin, finish = int(offsets[index]), int(offsets[index + 1])
        if finish > begin:
            with Image.open(io.BytesIO(blob[begin:finish].tobytes())) as image:
                array = np.asarray(image.convert("L"), dtype=np.uint8)
            if array.shape[:2] != (height, width):
                array = np.asarray(
                    Image.fromarray(array).resize((width, height), Image.BILINEAR),
                    dtype=np.uint8,
                )
            frames.append(array)
        else:
            frames.append(np.zeros((height, width), dtype=np.uint8))
    return np.stack(frames, axis=0)


def _augment_missing(
    tof: np.ndarray,
    weight: np.ndarray,
    rng: np.random.Generator,
    *,
    probability: float,
    ratios: list[float],
    view_dropout: float = 0.0,
    block_probability: float = 0.0,
    degraded_probability: float = 0.0,
    degraded_ratio: float = 0.6,
    degraded_view_dropout: float = 0.5,
) -> None:
    degraded = degraded_probability > 0.0 and rng.random() < degraded_probability
    if not degraded and rng.random() > probability:
        return
    if degraded:
        view_dropout = max(view_dropout, degraded_view_dropout)
        ratios = [degraded_ratio]
    frames, views, height, width = tof.shape
    if view_dropout > 0.0:
        for view in range(views):
            if rng.random() < view_dropout:
                tof[:, view] = 0.0
                weight[:, view] = 0.0
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
    if block_probability > 0.0 and rng.random() < block_probability:
        block_h = int(rng.integers(2, min(5, height) + 1))
        block_w = int(rng.integers(2, min(5, width) + 1))
        top = int(rng.integers(0, height - block_h + 1))
        left = int(rng.integers(0, width - block_w + 1))
        view = int(rng.integers(0, views))
        tof[:, view, top : top + block_h, left : left + block_w] = 0.0
        weight[:, view, top : top + block_h, left : left + block_w] = 0.0


def _augment_camera(
    camera: np.ndarray,
    rng: np.random.Generator,
    *,
    brightness: float = 0.2,
    contrast: float = 0.2,
    gamma: float = 0.2,
    noise: float = 0.03,
) -> np.ndarray:
    image = camera.astype(np.float32) / 255.0
    if brightness > 0.0:
        image = image + float(rng.uniform(-brightness, brightness))
    if contrast > 0.0:
        factor = float(rng.uniform(1.0 - contrast, 1.0 + contrast))
        image = (image - 0.5) * factor + 0.5
    if gamma > 0.0:
        exponent = float(np.exp(rng.uniform(-gamma, gamma)))
        image = np.clip(image, 1e-4, 1.0) ** exponent
    if noise > 0.0:
        image = image + rng.normal(0.0, noise, image.shape).astype(np.float32)
    return np.clip(image, 0.0, 1.0)


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
        cache: bool = True,
        splits: list[str] | None = None,
        scenes: list[str] | None = None,
        exclude_scenes: list[str] | None = None,
        use_camera: bool = False,
        view_dropout: float = 0.0,
        block_probability: float = 0.0,
        camera_augment: bool = False,
        tof_degraded_probability: float = 0.0,
        tof_degraded_ratio: float = 0.6,
        tof_degraded_view_dropout: float = 0.5,
    ) -> None:
        self.root = Path(root)
        self.window_frames = window_frames
        self.stride = stride or (window_frames if window_frames else 1)
        self.augment = augment
        self.missing_probability = missing_probability
        self.missing_ratios = missing_ratios or [0.1, 0.3, 0.5]
        self.seed = seed
        self.use_camera = use_camera
        self.view_dropout = view_dropout
        self.block_probability = block_probability
        self.camera_augment = camera_augment
        self.tof_degraded_probability = tof_degraded_probability
        self.tof_degraded_ratio = tof_degraded_ratio
        self.tof_degraded_view_dropout = tof_degraded_view_dropout
        self.camera_size = (120, 160)
        self.skipped_short_sequences = 0
        self.files: list[Path] = []
        if files is not None:
            self.files = [Path(path).expanduser().resolve() for path in files]
        else:
            for path in sorted(self.root.rglob("*.npz")):
                meta = self._metadata(path)
                meta_split = str(meta.get("split", ""))
                meta_scene = str(meta.get("scene", ""))
                if splits is not None:
                    if meta_split not in splits:
                        continue
                elif split is not None and meta_split != split:
                    continue
                if scenes is not None and meta_scene not in scenes:
                    continue
                if exclude_scenes is not None and meta_scene in exclude_scenes:
                    continue
                self.files.append(path)
        if max_sequences is not None:
            self.files = self.files[:max_sequences]
        if not self.files:
            raise FileNotFoundError(f"no NPZ sequences under {self.root} for split {split!r}")
        self.windows: list[tuple[int, int, int]] = []
        self.cache: list[dict[str, np.ndarray]] | None = [] if cache else None
        for file_index, path in enumerate(self.files):
            with np.load(path, allow_pickle=False) as data:
                length = int(data["tof"].shape[0])
                if window_frames is not None and length < window_frames:
                    self.skipped_short_sequences += 1
                    if cache:
                        assert self.cache is not None
                        self.cache.append({})
                    continue
                if cache:
                    tof = data["tof"].astype(np.float32)
                    weight = data["tof_weight"].astype(np.float32)
                    imu = data["imu"].astype(np.float32)
                    poses = data["gt_pose"].astype(np.float32)
                    delta_p, delta_phi = _increments(poses.astype(np.float64))
                    entry = {
                        "tof": tof,
                        "weight": weight,
                        "imu": imu,
                        "poses": poses,
                        "delta_p": delta_p,
                        "delta_phi": delta_phi,
                    }
                    if self.use_camera and "camera_jpeg" in data:
                        entry["camera_jpeg"] = data["camera_jpeg"]
                        entry["camera_offset"] = data["camera_offset"]
                        if "camera_height" in data and "camera_width" in data:
                            self.camera_size = (
                                int(np.asarray(data["camera_height"]).reshape(-1)[0]),
                                int(np.asarray(data["camera_width"]).reshape(-1)[0]),
                            )
                    assert self.cache is not None
                    self.cache.append(entry)
            if window_frames is None:
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
        end = start + length
        if self.cache is not None:
            cached = self.cache[file_index]
            tof = cached["tof"][start:end].copy()
            weight = cached["weight"][start:end].copy()
            imu = cached["imu"][start:end]
            poses = cached["poses"]
            delta_p = cached["delta_p"]
            delta_phi = cached["delta_phi"]
            camera_blob = cached.get("camera_jpeg")
            camera_offset = cached.get("camera_offset")
            if start == 0:
                previous_frame = tof[0]
                previous_weight_frame = weight[0]
            else:
                previous_frame = cached["tof"][start - 1]
                previous_weight_frame = cached["weight"][start - 1]
        else:
            arrays, _ = load_sequence(self.files[file_index])
            tof = arrays["tof"][start:end].astype(np.float32)
            weight = arrays["tof_weight"][start:end].astype(np.float32)
            imu = arrays["imu"][start:end].astype(np.float32)
            poses = arrays["gt_pose"].astype(np.float32)
            delta_p, delta_phi = _increments(poses.astype(np.float64))
            camera_blob = arrays.get("camera_jpeg")
            camera_offset = arrays.get("camera_offset")
            if start == 0:
                previous_frame = tof[0]
                previous_weight_frame = weight[0]
            else:
                previous_frame = arrays["tof"][start - 1].astype(np.float32)
                previous_weight_frame = arrays["tof_weight"][start - 1].astype(np.float32)

        if self.augment:
            rng = np.random.default_rng(self.seed + index)
            _augment_missing(
                tof,
                weight,
                rng,
                probability=self.missing_probability,
                ratios=self.missing_ratios,
                view_dropout=self.view_dropout,
                block_probability=self.block_probability,
                degraded_probability=self.tof_degraded_probability,
                degraded_ratio=self.tof_degraded_ratio,
                degraded_view_dropout=self.tof_degraded_view_dropout,
            )

        previous = np.empty_like(tof)
        previous_weight = np.empty_like(weight)
        previous[0] = previous_frame if start else tof[0]
        previous_weight[0] = previous_weight_frame if start else weight[0]
        previous[1:] = tof[:-1]
        previous_weight[1:] = weight[:-1]
        item = {
            "tof": torch.from_numpy(tof),
            "previous_tof": torch.from_numpy(previous),
            "weight": torch.from_numpy(weight),
            "previous_weight": torch.from_numpy(previous_weight),
            "imu": torch.from_numpy(imu),
            "delta_p": torch.from_numpy(delta_p[start:end]),
            "delta_phi": torch.from_numpy(delta_phi[start:end]),
            "gt_pose": torch.from_numpy(poses[start:end].astype(np.float32)),
        }
        if self.use_camera:
            if camera_blob is None or camera_offset is None:
                camera = np.zeros((length, *self.camera_size), dtype=np.uint8)
            else:
                camera = _decode_camera(camera_blob, camera_offset, start, end, self.camera_size)
            if self.augment and self.camera_augment:
                camera = _augment_camera(
                    camera, np.random.default_rng(self.seed + index + 7919)
                ) * 255.0
            item["camera"] = torch.from_numpy(camera.astype(np.float32)).div_(255.0)
        return item


def _collate(batch: list[dict[str, torch.Tensor]]) -> dict[str, torch.Tensor]:
    return {key: torch.stack([item[key] for item in batch], dim=0) for key in batch[0]}


def iterate_batches(
    dataset: Dataset,
    batch_size: int,
    shuffle: bool = True,
    generator: torch.Generator | None = None,
    *,
    num_workers: int = 0,
    pin_memory: bool = False,
    persistent_workers: bool = False,
) -> Iterator[dict[str, torch.Tensor]]:
    from torch.utils.data import DataLoader

    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        drop_last=False,
        generator=generator,
        num_workers=num_workers,
        pin_memory=pin_memory,
        persistent_workers=persistent_workers and num_workers > 0,
        collate_fn=_collate,
    )
    yield from loader
