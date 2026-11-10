#!/usr/bin/env python3
"""Evaluate a trained TIO-Former checkpoint on processed sequences.

The model predicts body-frame increments.  We integrate them into a world
trajectory, align the result to ground truth with an SE(3) transform (no scale,
because the ToF ranges are metric), and report ATE / RPE / end-drift error.
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
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import load_sequence  # noqa: E402
from tio_former.models.geometry import cumulative_rotation, integrate_increments  # noqa: E402
from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--eval-config", default="configs/eval/main.yaml")
    parser.add_argument("--processed", default="data/processed")
    parser.add_argument("--split", default=None, help="Override eval config split")
    parser.add_argument("--scenes", default="", help="comma-separated scene filter")
    parser.add_argument("--limit", type=int, default=0, help="Evaluate at most N sequences")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--out", default=None)
    parser.add_argument("--no-save-trajectories", action="store_true")
    parser.add_argument("--drop-views", nargs="*", default=None, help="View names to drop, e.g. front right")
    parser.add_argument("--tof-noise-m", type=float, default=0.0, help="Extra Gaussian range noise (m)")
    parser.add_argument("--status-dropout", type=float, default=0.0, help="Fraction of valid zones to drop")
    parser.add_argument("--perturb-seed", type=int, default=0)
    parser.add_argument(
        "--zero-camera",
        action="store_true",
        help="Feed zero camera frames to measure the camera branch contribution.",
    )
    return parser.parse_args()


def _perturb_arrays(arrays: dict, metadata: dict, args: argparse.Namespace) -> None:
    """Apply deterministic view/noise/status perturbations in place."""
    tof = np.asarray(arrays["tof"], dtype=np.float32)
    weight = np.asarray(arrays["tof_weight"], dtype=np.float32)
    views = list(metadata.get("views") or ["front", "left", "right", "down"])
    rng = np.random.default_rng(args.perturb_seed)

    for name in args.drop_views or []:
        if name in views:
            tof[:, views.index(name)] = 0.0
            weight[:, views.index(name)] = 0.0

    if args.tof_noise_m > 0.0:
        noise = rng.normal(0.0, args.tof_noise_m, size=tof.shape).astype(np.float32)
        valid = weight > 0.0
        tof = np.where(valid, tof + noise, tof)
        tof = np.where(valid, np.clip(tof, 0.015, 4.0), 0.0)

    if args.status_dropout > 0.0:
        drop = rng.random(weight.shape) < args.status_dropout
        tof = np.where(drop, 0.0, tof)
        weight = np.where(drop, 0.0, weight)

    arrays["tof"] = tof
    arrays["tof_weight"] = weight


def _list_sequences(root: Path, split: str | None, scenes: list[str] | None = None) -> list[Path]:
    files = sorted(root.rglob("*.npz"))
    selected = []
    for path in files:
        sidecar = path.with_suffix(".json")
        if not sidecar.exists():
            continue
        metadata = json.loads(sidecar.read_text(encoding="utf-8"))
        if split is not None and metadata.get("split") != split:
            continue
        if scenes is not None and metadata.get("scene") not in scenes:
            continue
        selected.append(path)
    return selected


def _umeyama_no_scale(predicted: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return rotation/translation aligning predicted to target (no scale)."""
    mu_p = predicted.mean(axis=0)
    mu_t = target.mean(axis=0)
    covariance = (predicted - mu_p).T @ (target - mu_t)
    u, _s, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = mu_t - rotation @ mu_p
    return rotation, translation


def _align(predicted: np.ndarray, target: np.ndarray) -> np.ndarray:
    rotation, translation = _umeyama_no_scale(predicted, target)
    return predicted @ rotation.T + translation


def _rotation_angle(rotation: np.ndarray) -> float:
    value = (np.trace(rotation) - 1.0) / 2.0
    return float(np.arccos(np.clip(value, -1.0, 1.0)))


def _metrics(predicted: np.ndarray, target: np.ndarray, rotations: np.ndarray, target_rotations: np.ndarray, step: int) -> dict:
    aligned = _align(predicted, target)
    position_error = aligned - target
    ate = float(np.sqrt(np.mean(np.sum(position_error**2, axis=1))))
    path_length = float(np.sum(np.linalg.norm(np.diff(target, axis=0), axis=1)))
    end_drift = float(np.linalg.norm(aligned[-1] - target[-1]) / max(path_length, 1e-6) * 100.0)

    rpe_p: list[float] = []
    rpe_r: list[float] = []
    for index in range(0, len(predicted) - step):
        rel_p = rotations[index].T @ (predicted[index + step] - predicted[index])
        rel_gt_p = target_rotations[index].T @ (target[index + step] - target[index])
        rpe_p.append(float(np.linalg.norm(rel_p - rel_gt_p)))
        rel_r = rotations[index].T @ rotations[index + step]
        rel_gt_r = target_rotations[index].T @ target_rotations[index + step]
        rpe_r.append(_rotation_angle(rel_r.T @ rel_gt_r))
    rpe_p_rmse = float(np.sqrt(np.mean(np.square(rpe_p)))) if rpe_p else float("nan")
    rpe_r_rmse = float(np.sqrt(np.mean(np.square(rpe_r)))) if rpe_r else float("nan")
    return {
        "ate_m": ate,
        "rpe_position_m": rpe_p_rmse,
        "rpe_rotation_deg": float(np.degrees(rpe_r_rmse)),
        "end_drift_percent": end_drift,
        "path_length_m": path_length,
    }


def _load_model(checkpoint: Path, device: str) -> TioFormer:
    state = torch.load(checkpoint, map_location=device, weights_only=False)
    config = state.get("model_config")
    if config is None:
        raise ValueError(f"checkpoint {checkpoint} has no model_config")
    model = TioFormer(ModelConfig.from_dict(config)).to(device)
    model.load_state_dict(state["model"])
    model.eval()
    return model


def _predict(model: TioFormer, arrays: dict, device: str, zero_camera: bool = False):
    tof = torch.from_numpy(np.asarray(arrays["tof"], dtype=np.float32)).unsqueeze(0).to(device)
    weight = torch.from_numpy(np.asarray(arrays["tof_weight"], dtype=np.float32)).unsqueeze(0).to(device)
    imu = torch.from_numpy(np.asarray(arrays["imu"], dtype=np.float32)).unsqueeze(0).to(device)
    previous = torch.cat([tof[:, :1], tof[:, :-1]], dim=1)
    previous_weight = torch.cat([weight[:, :1], weight[:, :-1]], dim=1)
    camera = None
    if model.config.camera:
        from tio_former.data.torch_dataset import _decode_camera

        height, width = model.config.camera_height, model.config.camera_width
        if not zero_camera and "camera_jpeg" in arrays and "camera_offset" in arrays:
            frames = _decode_camera(
                np.asarray(arrays["camera_jpeg"]),
                np.asarray(arrays["camera_offset"]),
                0,
                tof.shape[1],
                (height, width),
            )
        else:
            frames = np.zeros((tof.shape[1], height, width), dtype=np.uint8)
        camera = torch.from_numpy(frames).float().div_(255.0).unsqueeze(0).to(device)
    with torch.no_grad():
        output = model(
            tof,
            imu,
            weight=weight,
            previous_tof=previous,
            previous_weight=previous_weight,
            camera=camera,
        )
    gate = None
    if "gate_weights" in output:
        gate = output["gate_weights"].float().mean(dim=(0, 1)).cpu().tolist()
    return output["delta_p"].cpu().numpy(), output["delta_phi"].cpu().numpy(), gate


def _aggregate(rows: list[dict], keys: tuple[str, ...]) -> list[dict]:
    groups: dict[tuple, list[dict]] = {}
    for row in rows:
        groups.setdefault(tuple(row[key] for key in keys), []).append(row)
    result = []
    for key, items in sorted(groups.items()):
        entry = {name: value for name, value in zip(keys, key)}
        entry["sequences"] = len(items)
        entry["duration_s"] = float(sum(item["duration_s"] for item in items))
        entry["path_length_m"] = float(sum(item["path_length_m"] for item in items))
        for metric in ("ate_m", "rpe_position_m", "rpe_rotation_deg", "end_drift_percent"):
            entry[metric] = float(np.mean([item[metric] for item in items]))
        result.append(entry)
    return result


def _markdown(rows: list[dict], groups: list[dict], title: str) -> str:
    lines = [f"# {title}", "", f"Sequences: {len(rows)}", ""]
    lines.extend(["| group | seq | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |", "|---|---:|---:|---:|---:|---:|"])
    for entry in groups:
        label = " / ".join(str(entry[key]) for key in ("scene", "kind") if key in entry)
        lines.append(
            f"| {label} | {entry['sequences']} | {entry['ate_m']:.4f} | "
            f"{entry['rpe_position_m']:.4f} | {entry['rpe_rotation_deg']:.3f} | "
            f"{entry['end_drift_percent']:.2f} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = _args()
    logger = get_logger("evaluate")
    eval_config = load_yaml(ROOT / args.eval_config)
    split = args.split if args.split is not None else eval_config.get("split", "test")
    files = _list_sequences(
        ROOT / args.processed,
        split,
        [item for item in args.scenes.split(",") if item] or None,
    )
    if args.limit:
        files = files[: args.limit]
    if not files:
        logger.error("no processed sequences for split=%s", split)
        return 2

    model = _load_model((ROOT / args.checkpoint).resolve(), args.device)
    out_dir = Path(args.out) if args.out else ROOT / eval_config.get("output_dir", "runs/eval")
    out_dir = (ROOT / out_dir).resolve() if not out_dir.is_absolute() else out_dir
    trajectory_dir = out_dir / "trajectories"
    if not args.no_save_trajectories:
        trajectory_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for index, path in enumerate(files, start=1):
        arrays, _ = load_sequence(path)
        metadata = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
        _perturb_arrays(arrays, metadata, args)
        delta_p, delta_phi, gate = _predict(model, arrays, args.device, args.zero_camera)
        delta_p_t = torch.from_numpy(np.asarray(delta_p)).to(args.device)
        delta_phi_t = torch.from_numpy(np.asarray(delta_phi)).to(args.device)
        predicted = integrate_increments(delta_p_t, delta_phi_t)[0].cpu().numpy()
        rotations = cumulative_rotation(delta_phi_t)[0].cpu().numpy()

        gt = np.asarray(arrays["gt_pose"], dtype=np.float64)
        gt_positions = gt[:, :3]
        gt_positions = gt_positions - gt_positions[0]
        gt_quat = gt[:, 3:7]
        from tio_former.utils.se3 import quat_to_matrix  # local import keeps startup light

        gt_rotations = np.stack([quat_to_matrix(q) for q in gt_quat])
        # Predicted poses live in the initial body frame (FLU); rotate world GT
        # into that frame, and express orientations relative to the first frame.
        gt_positions = gt_positions @ gt_rotations[0]
        gt_rotations = np.einsum("ij,tjk->tik", gt_rotations[0].T, gt_rotations)
        frame_step = max(1, int(round(float(eval_config.get("rpe_delta_s", 0.0667)) * 15.0)))
        metrics = _metrics(predicted, gt_positions, rotations, gt_rotations, frame_step)
        row = {
            "seq_id": metadata.get("seq_id", path.stem),
            "scene": metadata.get("scene", "unknown"),
            "split": metadata.get("split", split),
            "kind": metadata.get("kind", "unknown"),
            "duration_s": float(metadata.get("duration_s", len(predicted) / 15.0)),
            **metrics,
        }
        if gate is not None:
            row["gate_imu"], row["gate_camera"], row["gate_tof"] = gate
        rows.append(row)
        if not args.no_save_trajectories:
            np.savez_compressed(
                trajectory_dir / f"{row['seq_id']}.npz",
                predicted=predicted.astype(np.float32),
                predicted_rotations=rotations.astype(np.float32),
                ground_truth=gt_positions.astype(np.float32),
                ground_truth_rotations=gt_rotations.astype(np.float32),
            )
        if index == 1 or index % 10 == 0 or index == len(files):
            logger.info("evaluated %d/%d (%s: ATE %.4f m)", index, len(files), row["seq_id"], metrics["ate_m"])

    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    by_scene_kind = _aggregate(rows, ("scene", "kind"))
    by_scene = _aggregate(rows, ("scene",))
    summary = {
        "split": split,
        "checkpoint": str(Path(args.checkpoint)),
        "camera": bool(model.config.camera),
        "zero_camera": bool(args.zero_camera),
        "sequences": len(rows),
        "duration_s": float(sum(row["duration_s"] for row in rows)),
        "path_length_m": float(sum(row["path_length_m"] for row in rows)),
        "metrics": {
            key: float(np.mean([row[key] for row in rows]))
            for key in ("ate_m", "rpe_position_m", "rpe_rotation_deg", "end_drift_percent")
        },
        "by_scene": by_scene,
        "by_scene_kind": by_scene_kind,
        "gate_usage": (
            {
                "imu": float(np.mean([row["gate_imu"] for row in rows])),
                "camera": float(np.mean([row["gate_camera"] for row in rows])),
                "tof": float(np.mean([row["gate_tof"] for row in rows])),
            }
            if "gate_imu" in rows[0]
            else None
        ),
        "perturbations": {
            "drop_views": list(args.drop_views or []),
            "tof_noise_m": float(args.tof_noise_m),
            "status_dropout": float(args.status_dropout),
            "perturb_seed": int(args.perturb_seed),
        },
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out_dir / "report.md").write_text(
        _markdown(rows, by_scene_kind, f"TIO-Former evaluation ({split})"), encoding="utf-8"
    )
    logger.info(
        "done: %d sequences | ATE %.4f m | RPEp %.4f m | RPER %.3f deg | EDR %.2f%%",
        len(rows),
        summary["metrics"]["ate_m"],
        summary["metrics"]["rpe_position_m"],
        summary["metrics"]["rpe_rotation_deg"],
        summary["metrics"]["end_drift_percent"],
    )
    if summary["gate_usage"] is not None:
        logger.info(
            "gate usage: imu %.3f | camera %.3f | tof %.3f",
            summary["gate_usage"]["imu"],
            summary["gate_usage"]["camera"],
            summary["gate_usage"]["tof"],
        )
    logger.info("wrote %s", out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
