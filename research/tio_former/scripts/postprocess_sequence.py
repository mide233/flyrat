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


def _resize_gray(image: np.ndarray, width: int, height: int) -> np.ndarray:
    if image.shape[1] == width and image.shape[0] == height:
        return image
    try:
        import cv2

        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    except ImportError:
        from PIL import Image

        return np.asarray(Image.fromarray(image, mode="L").resize((width, height), Image.BILINEAR))


def _encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    """Encode a mono image, preferring OpenCV (system Python) over Pillow."""
    try:
        import cv2

        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise ValueError("cv2.imencode failed")
        return bytes(buffer.tobytes())
    except ImportError:
        from PIL import Image

        import io

        buffer = io.BytesIO()
        Image.fromarray(image, mode="L").save(buffer, format="JPEG", quality=quality)
        return buffer.getvalue()

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import save_sequence  # noqa: E402
from tio_former.data.imu import generate_imu, resample_to_tof_windows  # noqa: E402
from tio_former.sim.degradation import (  # noqa: E402
    apply_camera_degradation,
    apply_imu_degradation,
    apply_tof_degradation,
    camera_state,
)
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
    parser.add_argument("--split", default="demo")
    parser.add_argument("--seq-id", default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--sensors-config",
        default=str(ROOT / "configs/sim/sensors.yaml"),
    )
    parser.add_argument("--camera-quality", type=int, default=85)
    parser.add_argument("--camera-width", type=int, default=160)
    parser.add_argument("--camera-height", type=int, default=120)
    parser.add_argument(
        "--scene-spec",
        default=None,
        help="Optional JSON with the sampled random-scene parameters.",
    )
    parser.add_argument(
        "--camera",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Encode the recorded forward camera into the dataset.",
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
    # Gazebo occasionally republishes a pose with the same stamp when a
    # worker is starved; keep the first sample so the timeline is monotonic.
    unique = np.concatenate([[True], np.diff(gt_stamp) > 0])
    gt_stamp = gt_stamp[unique]
    gt_pose = gt_pose[unique]

    # A dropped frame can leave one view with fewer depth images than stamps.
    for view in views:
        stamps = np.asarray(arrays[f"t_{view}"], dtype=np.float64)
        depth = np.asarray(arrays[f"depth_{view}"], dtype=np.float32)
        count = min(stamps.size, depth.shape[0])
        if count < 2:
            raise ValueError(f"view {view} has too few frames")
        arrays[f"t_{view}"] = stamps[:count]
        arrays[f"depth_{view}"] = depth[:count]

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
    aligned_stamp = gt_stamp[keep]
    monotonic = np.concatenate([[True], np.diff(aligned_stamp) > 0])
    indices = indices[monotonic]
    aligned_stamp = aligned_stamp[monotonic]
    keep = np.flatnonzero(keep)[monotonic]
    t_tof = aligned_stamp - aligned_stamp[0]
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

    tof_degradation = apply_tof_degradation(
        tof,
        tof_ideal,
        tof_status,
        tof_weight,
        np.random.default_rng(args.seed + 3),
        sensors.get("tof_degradation"),
    )

    position = gt_pose[keep, :3]
    quat = _quat_normalize(gt_pose[keep, 3:7])
    trajectory = Trajectory(t=t_tof, position=position, quat=quat)
    gt_twist = _ground_truth_twist(t_tof, position, quat).astype(np.float32)

    imu = generate_imu(trajectory, sensors["imu"], np.random.default_rng(args.seed + 1))
    accel_degraded, gyro_degraded, imu_degradation = apply_imu_degradation(
        imu.accel,
        imu.gyro,
        np.random.default_rng(args.seed + 4),
        sensors.get("imu_degradation"),
    )
    imu = type(imu)(t=imu.t, accel=accel_degraded, gyro=gyro_degraded,
                    bias_acc=imu.bias_acc, bias_gyro=imu.bias_gyro)
    imu_windows, imu_times = resample_to_tof_windows(
        imu,
        t_tof,
        int(sensors["imu"].get("samples_per_tof", 16)),
    )

    camera_arrays: dict[str, np.ndarray] = {}
    camera_meta: dict[str, object] = {"camera": False}
    if args.camera and "camera_image" in arrays and "t_camera" in arrays:
        camera_images = np.asarray(arrays["camera_image"], dtype=np.uint8)
        camera_stamp = np.asarray(arrays["t_camera"], dtype=np.float64)
        camera_count = min(camera_stamp.size, camera_images.shape[0])
        camera_stamp = camera_stamp[:camera_count]
        camera_images = camera_images[:camera_count]
        camera_indices = _nearest_indices(camera_stamp, aligned_stamp, tolerance)
        valid_camera = camera_indices >= 0
        if not np.all(valid_camera):
            logger.warning(
                "missing camera frame for %d/%d ToF frames", int((~valid_camera).sum()), valid_camera.size
            )
        camera_cfg = sensors.get("camera_degradation")
        camera_rng = np.random.default_rng(args.seed + 5)
        camera_values = camera_state(camera_rng, camera_cfg)
        quality_range = camera_cfg.get("jpeg_quality", [args.camera_quality, args.camera_quality]) if camera_cfg else None
        if quality_range:
            quality = int(np.clip(int(camera_rng.integers(quality_range[0], quality_range[1] + 1)), 40, 95))
        else:
            quality = int(args.camera_quality)
        encoded: list[bytes] = []
        offsets = [0]
        for frame_index, camera_index in enumerate(camera_indices):
            if camera_index < 0:
                encoded.append(b"")
            else:
                frame = camera_images[camera_index]
                if camera_cfg:
                    angular = float(np.linalg.norm(gt_twist[frame_index, 3:]))
                    frame = apply_camera_degradation(
                        frame, angular, camera_rng, camera_cfg, camera_values
                    )
                resized = _resize_gray(frame, int(args.camera_width), int(args.camera_height))
                encoded.append(_encode_jpeg(resized, quality))
            offsets.append(offsets[-1] + len(encoded[-1]))
        blob = np.frombuffer(b"".join(encoded), dtype=np.uint8)
        camera_arrays = {
            "camera_jpeg": blob,
            "camera_offset": np.asarray(offsets, dtype=np.int64),
            "camera_frame_valid": valid_camera.astype(np.uint8),
        }
        camera_meta = {
            "camera": True,
            "camera_source_width": int(camera_images.shape[2]),
            "camera_source_height": int(camera_images.shape[1]),
            "camera_width": int(args.camera_width),
            "camera_height": int(args.camera_height),
            "camera_jpeg_quality": int(quality),
            "camera_degradation": camera_values,
            "camera_blur_enabled": bool(sensors.get("camera_degradation")),
            "camera_frames": int(len(camera_images)),
            "camera_missing_frames": int((~valid_camera).sum()),
        }

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
    out_arrays.update(camera_arrays)
    metadata = {
        "scene": args.scene,
        "kind": args.kind,
        "split": args.split,
        "seq_id": args.seq_id or Path(args.out).stem,
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
        "tof_degradation": tof_degradation,
        "imu_degradation": imu_degradation,
        "camera_degradation": camera_meta.get("camera_degradation"),
    }
    if args.scene_spec and Path(args.scene_spec).exists():
        metadata["scene_spec"] = json.loads(Path(args.scene_spec).read_text(encoding="utf-8"))
    metadata.update(camera_meta)
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
