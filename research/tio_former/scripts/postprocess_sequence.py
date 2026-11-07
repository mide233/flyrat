#!/usr/bin/env python3
"""Convert a raw Gazebo recording into the TIO-Former M1 dataset format.

The raw recording contains one 64x64 optical-axis depth image per view, plus
15 Hz ground-truth poses published by the trajectory plugin.  This script
converts depth to radial range, aggregates to the physical 8x8 grid, simulates
sensor status/noise/dropouts, derives a 200 Hz IMU analytically, and writes the
validated NPZ sequence consumed by the training code.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import save_sequence  # noqa: E402
from tio_former.data.imu import generate_imu, resample_to_tof_windows  # noqa: E402
from tio_former.sim.tof import (  # noqa: E402
    aggregate_grid,
    depth_to_radial_range,
    simulate_frame,
)
from tio_former.sim.trajectory import Trajectory  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402
from tio_former.utils.se3 import quat_to_matrix, so3_log  # noqa: E402


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", required=True, help="Raw NPZ from record_sequence.py")
    parser.add_argument("--out", required=True, help="Output dataset .npz")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--kind", default="demo")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--sensors-config",
        default=str(ROOT / "configs/sim/sensors.yaml"),
    )
    return parser.parse_args()


def _nearest_indices(source: np.ndarray, target: np.ndarray, tolerance: float) -> np.ndarray:
    """Return source index nearest to each target, or -1 when too far."""
    order = np.argsort(source)
    source = source[order]
    right = np.searchsorted(source, target, side="left")
    right = np.clip(right, 0, source.size - 1)
    left = np.clip(right - 1, 0, source.size - 1)
    choose_left = np.abs(target - source[left]) <= np.abs(target - source[right])
    chosen = np.where(choose_left, left, right)
    distance = np.abs(target - source[chosen])
    chosen = np.where(distance <= tolerance, order[chosen], -1)
    return chosen.astype(np.int64)


def _quat_normalize(quat: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat, axis=1, keepdims=True)
    return quat / np.maximum(norm, 1e-12)


def _ground_truth_twist(t: np.ndarray, position: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Body-frame linear/angular velocity from world poses."""
    count = t.size
    twist = np.zeros((count, 6), dtype=np.float64)
    if count < 2:
        return twist
    rotations = np.stack([quat_to_matrix(q) for q in quat], axis=0)
    dt = np.diff(t)
    dt = np.maximum(dt, 1e-9)
    for i in range(1, count):
        twist[i, :3] = rotations[i - 1].T @ (position[i] - position[i - 1]) / dt[i - 1]
        twist[i, 3:] = so3_log(rotations[i - 1].T @ rotations[i]) / dt[i - 1]
    twist[0] = twist[1]
    return twist


def main() -> int:
    args = _parse_args()
    logger = get_logger("postprocess_sequence")
    sensors = load_yaml(args.sensors_config)
    tof_cfg = sensors["tof"]
    tof_sim_cfg = dict(tof_cfg)
    tof_sim_cfg.update(sensors.get("tof_sim", {}))
    views = list(tof_cfg["views"])
    rate = float(tof_cfg["rate_hz"])
    rng = np.random.default_rng(args.seed)

    with np.load(args.raw, allow_pickle=False) as raw:
        arrays = {key: raw[key] for key in raw.files}
    if "gt_stamp" not in arrays or "gt_pose" not in arrays:
        raise ValueError("raw recording has no gt_stamp/gt_pose")

    gt_stamp = np.asarray(arrays["gt_stamp"], dtype=np.float64)
    gt_pose = np.asarray(arrays["gt_pose"], dtype=np.float64)
    if gt_stamp.size < 2:
        raise ValueError("not enough ground-truth poses")

    tolerance = 0.75 / rate
    indices = np.stack(
        [
            _nearest_indices(
                np.asarray(arrays[f"t_{view}"], dtype=np.float64), gt_stamp, tolerance
            )
            for view in views
        ],
        axis=1,
    )
    keep = np.all(indices >= 0, axis=1)
    if not np.any(keep):
        raise ValueError("no timestamp-aligned frame across all views")
    dropped = int((~keep).sum())
    if dropped:
        logger.warning("dropped %d/%d frames with missing/aligned views", dropped, keep.size)
    indices = indices[keep]
    t_tof = gt_stamp[keep] - gt_stamp[keep][0]
    if t_tof.size < 2:
        raise ValueError("fewer than two aligned frames")

    ranges = np.full((t_tof.size, len(views), 8, 8), np.nan, dtype=np.float64)
    for view_index, view in enumerate(views):
        depth_frames = np.asarray(arrays[f"depth_{view}"], dtype=np.float32)[indices[:, view_index]]
        for frame_index, depth in enumerate(depth_frames):
            radial = depth_to_radial_range(
                depth,
                hfov=float(tof_cfg["horizontal_fov"]),
                vfov=float(tof_cfg["vertical_fov"]),
                near_m=float(tof_cfg["near_m"]),
                far_m=float(tof_cfg["far_m"]),
            )
            ranges[frame_index, view_index] = aggregate_grid(
                radial,
                grid_width=int(tof_cfg["grid_width"]),
                grid_height=int(tof_cfg["grid_height"]),
            )

    tof = np.zeros_like(ranges, dtype=np.float32)
    tof_ideal = np.zeros_like(ranges, dtype=np.float32)
    tof_status = np.zeros_like(ranges, dtype=np.uint8)
    tof_weight = np.zeros_like(ranges, dtype=np.float32)
    previous_noise = {view: None for view in views}
    for frame_index in range(t_tof.size):
        for view_index, view in enumerate(views):
            result = simulate_frame(
                ranges[frame_index, view_index],
                rng,
                tof_sim_cfg,
                previous_noise[view],
            )
            previous_noise[view] = result["noise"]
            tof[frame_index, view_index] = result["range"]
            tof_ideal[frame_index, view_index] = result["ideal"]
            tof_status[frame_index, view_index] = result["status"]
            tof_weight[frame_index, view_index] = result["weight"]

    position = gt_pose[keep, :3]
    quat = _quat_normalize(gt_pose[keep, 3:7])
    trajectory = Trajectory(t=t_tof, position=position, quat=quat)
    gt_twist = _ground_truth_twist(t_tof, position, quat).astype(np.float32)

    imu = generate_imu(trajectory, sensors["imu"], np.random.default_rng(args.seed + 1))
    imu_windows, imu_times = resample_to_tof_windows(
        imu,
        t_tof,
        int(sensors["imu"].get("samples_per_tof", 16)),
    )

    out_arrays = {
        "tof": tof.astype(np.float32),
        "tof_ideal": tof_ideal.astype(np.float32),
        "tof_status": tof_status.astype(np.uint8),
        "tof_weight": tof_weight.astype(np.float32),
        "gt_pose": np.concatenate([position, quat], axis=1).astype(np.float32),
        "gt_twist": gt_twist,
        "imu": imu_windows.astype(np.float32),
        "t_tof": np.asarray(t_tof, dtype=np.float64),
        "t_imu": np.asarray(imu_times, dtype=np.float64),
    }
    metadata = {
        "scene": args.scene,
        "kind": args.kind,
        "seed": args.seed,
        "source_raw": str(Path(args.raw).resolve()),
        "views": views,
        "grid": [int(tof_cfg["grid_height"]), int(tof_cfg["grid_width"])],
        "rate_hz": rate,
        "frames": int(t_tof.size),
        "dropped_frames": dropped,
        "duration_s": float(t_tof[-1] - t_tof[0]),
        "path_length_m": float(trajectory.path_length),
        "sensors_config": str(Path(args.sensors_config).resolve()),
    }
    save_sequence(args.out, out_arrays, metadata)
    logger.info(
        "wrote %s: %d frames, %.2f s, %.2f m",
        args.out,
        t_tof.size,
        metadata["duration_s"],
        metadata["path_length_m"],
    )
    print(json.dumps({k: metadata[k] for k in ("frames", "duration_s", "path_length_m")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
