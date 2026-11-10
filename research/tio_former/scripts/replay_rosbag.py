#!/usr/bin/env python3
"""Replay an extracted FlyRat rosbag through the TIO-Former offline.

Input is the ``bag.npz`` written by ``inspect_rosbag.py``.  The preprocessing
mirrors ``tio_odometry_node`` / ``TioOdometry`` so the resulting trajectory is
directly comparable with the live node:

* ToF views are already in model order (front, left, right, down).
* IMU windows are 16 samples over ``(t_{i-1}, t_i]``.
* Camera frames are paired to the nearest ToF stamp within ``--camera-skew``.
* A 64-frame window is left-padded with zeros.

Example::

    .venv/bin/python scripts/replay_rosbag.py \
        --extracted /tmp/opencode/issue6/bag.npz \
        --checkpoint checkpoints_cam_full/seed0.pt \
        --out runs_bag/issue6 --plot
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extracted", default="/tmp/opencode/issue6/bag.npz")
    parser.add_argument("--checkpoint", default="checkpoints_cam_full/seed0.pt")
    parser.add_argument("--out", default="runs_bag/issue6")
    parser.add_argument("--window", type=int, default=64)
    parser.add_argument("--camera-skew", type=float, default=0.08)
    parser.add_argument("--zero-camera", action="store_true")
    parser.add_argument("--drop-tof", action="store_true")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--plot", action="store_true")
    return parser.parse_args()


def _imu_windows(imu_t: np.ndarray, imu: np.ndarray, tof_t: np.ndarray, samples: int = 16):
    period = float(np.median(np.diff(tof_t))) if tof_t.size > 1 else 1.0 / 15.0
    windows = np.empty((tof_t.size, samples, 6), dtype=np.float32)
    for index, end in enumerate(tof_t):
        start = float(tof_t[index - 1]) if index > 0 else float(end - period)
        times = np.linspace(start, end, samples, endpoint=False)
        for channel in range(6):
            windows[index, :, channel] = np.interp(times, imu_t, imu[:, channel])
    return windows


def _decode_camera(arrays: dict, tof_t: np.ndarray, skew: float) -> np.ndarray:
    import io

    from PIL import Image

    camera_t = arrays["camera_t"]
    offsets = arrays["camera_offset"]
    blob = arrays["camera_jpeg"]
    frames = np.zeros((tof_t.size, 120, 160), dtype=np.float32)
    matched = 0
    for index, stamp in enumerate(tof_t):
        if camera_t.size == 0:
            continue
        position = int(np.argmin(np.abs(camera_t - stamp)))
        if abs(camera_t[position] - stamp) > skew:
            continue
        encoded = blob[offsets[position] : offsets[position + 1]].tobytes()
        image = np.asarray(Image.open(io.BytesIO(encoded)).convert("L"), dtype=np.float32)
        frames[index] = image / 255.0
        matched += 1
    return frames, matched


def main() -> int:
    args = _args()
    from tio_former.models.geometry import integrate_increments
    from tio_former.models.tio_former import ModelConfig, TioFormer

    extracted = Path(args.extracted).resolve()
    with np.load(extracted, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}

    tof = np.asarray(arrays["tof"], dtype=np.float32)
    weight = np.asarray(arrays["tof_weight"], dtype=np.float32)
    tof_t = np.asarray(arrays["tof_t"], dtype=np.float64)
    if args.drop_tof:
        tof = np.zeros_like(tof)
        weight = np.zeros_like(weight)
    imu = _imu_windows(arrays["imu_t"], arrays["imu"], tof_t)
    camera, matched = _decode_camera(arrays, tof_t, args.camera_skew)

    state = torch.load((ROOT / args.checkpoint).resolve(), map_location="cpu", weights_only=False)
    model = TioFormer(ModelConfig.from_dict(state["model_config"]))
    model.load_state_dict(state["model"])
    model.eval().to(args.device)

    window = args.window
    zeros_tof = np.zeros((window, 4, 8, 8), dtype=np.float32)
    zeros_imu = np.zeros((window, 16, 6), dtype=np.float32)
    zeros_camera = np.zeros((window, 120, 160), dtype=np.float32)
    deltas_p: list[np.ndarray] = []
    deltas_phi: list[np.ndarray] = []
    gates: list[np.ndarray] = []
    for index in range(tof_t.size):
        start = max(0, index + 1 - window)
        count = index + 1 - start
        tof_buffer = zeros_tof.copy()
        weight_buffer = zeros_tof.copy()
        imu_buffer = zeros_imu.copy()
        camera_buffer = zeros_camera.copy()
        tof_buffer[window - count :] = tof[start : index + 1]
        weight_buffer[window - count :] = weight[start : index + 1]
        imu_buffer[window - count :] = imu[start : index + 1]
        camera_buffer[window - count :] = camera[start : index + 1]
        with torch.no_grad():
            output = model(
                torch.from_numpy(tof_buffer).unsqueeze(0).to(args.device),
                torch.from_numpy(imu_buffer).unsqueeze(0).to(args.device),
                weight=torch.from_numpy(weight_buffer).unsqueeze(0).to(args.device),
                previous_tof=torch.from_numpy(np.concatenate([tof_buffer[:1], tof_buffer[:-1]])[None]).to(args.device),
                previous_weight=torch.from_numpy(
                    np.concatenate([weight_buffer[:1], weight_buffer[:-1]])[None]
                ).to(args.device),
                camera=(
                    None
                    if args.zero_camera or not model.config.camera
                    else torch.from_numpy(camera_buffer).unsqueeze(0).to(args.device)
                ),
            )
        deltas_p.append(output["delta_p"][0, -1].cpu().numpy())
        deltas_phi.append(output["delta_phi"][0, -1].cpu().numpy())
        if "gate_weights" in output:
            gates.append(output["gate_weights"][0, -1].cpu().numpy())

    delta_p = torch.from_numpy(np.stack(deltas_p)[None]).to(args.device)
    delta_phi = torch.from_numpy(np.stack(deltas_phi)[None]).to(args.device)
    positions = integrate_increments(delta_p, delta_phi)[0].cpu().numpy()
    relative = positions - positions[0]
    radius = np.linalg.norm(relative[:, :2], axis=1)
    path_length = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())

    summary = {
        "frames": int(tof_t.size),
        "duration_s": float(tof_t[-1] - tof_t[0]),
        "camera_frames_used": int(matched),
        "zero_camera": bool(args.zero_camera),
        "drop_tof": bool(args.drop_tof),
        "final_position_m": relative[-1].tolist(),
        "final_displacement_m": float(np.linalg.norm(relative[-1])),
        "final_horizontal_m": float(np.linalg.norm(relative[-1, :2])),
        "max_radius_m": float(radius.max()),
        "z_min_m": float(relative[:, 2].min()),
        "z_max_m": float(relative[:, 2].max()),
        "z_range_m": float(relative[:, 2].max() - relative[:, 2].min()),
        "path_length_m": path_length,
        "position_std_m": relative.std(axis=0).tolist(),
    }
    if gates:
        gate = np.stack(gates)
        summary["gate_usage"] = {
            "imu": float(gate[:, 0].mean()),
            "camera": float(gate[:, 1].mean()),
            "tof": float(gate[:, 2].mean()),
        }

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out / "replay.npz",
        positions=positions,
        relative=relative,
        tof_t=tof_t,
        delta_p=np.stack(deltas_p),
        delta_phi=np.stack(deltas_phi),
        gate=np.stack(gates) if gates else np.zeros((0, 3), dtype=np.float32),
    )
    with (out / "replay.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "z", "radius_xy"])
        for index, stamp in enumerate(tof_t):
            writer.writerow(
                [
                    f"{stamp - tof_t[0]:.6f}",
                    f"{relative[index, 0]:.6f}",
                    f"{relative[index, 1]:.6f}",
                    f"{relative[index, 2]:.6f}",
                    f"{radius[index]:.6f}",
                ]
            )
    (out / "replay.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        axes[0].plot(relative[:, 0], relative[:, 1], "-", label="xy")
        axes[0].scatter([0], [0], c="g", marker="o", label="start")
        axes[0].scatter([relative[-1, 0]], [relative[-1, 1]], c="r", marker="x", label="end")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        axes[0].set_title("horizontal trajectory")
        axes[0].axis("equal")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].plot(tof_t - tof_t[0], relative[:, 2], label="z")
        axes[1].plot(tof_t - tof_t[0], radius, label="radius xy")
        axes[1].set_xlabel("time [s]")
        axes[1].set_ylabel("m")
        axes[1].set_title("height and horizontal radius")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        figure.tight_layout()
        figure.savefig(out / "replay.png", dpi=150)
        print(f"wrote {out / 'replay.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
