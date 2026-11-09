#!/usr/bin/env python3
"""Visualise and diagnose the best TIO-Former test run.

The script reads an evaluation directory produced by ``scripts/evaluate.py``
and writes trajectory overlays, accuracy plots, scale-bias diagnostics and a
Chinese markdown report under ``report/best_model``.

It intentionally uses only NumPy/Matplotlib (no pandas) so it runs in the
project's lightweight Python environment.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCENES = ("corridor", "office", "pantry", "meeting_room")
SCENE_COLORS = {
    "corridor": "#3b6ea8",
    "office": "#c46a3a",
    "pantry": "#587d5a",
    "meeting_room": "#8a5a9e",
}
KIND_COLORS = {"random": "#3b6ea8", "canonical": "#c46a3a"}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--eval",
        default="runs/eval_main_imu_seed1",
        help="Evaluation directory for the model to present (default: best seed).",
    )
    parser.add_argument(
        "--processed",
        default="data/processed",
        help="Processed NPZ directory with ToF/IMU/GT arrays.",
    )
    parser.add_argument(
        "--stats",
        default="data/stats.json",
        help="Dataset statistics from build_dataset_index.py.",
    )
    parser.add_argument(
        "--split-eval",
        action="append",
        default=[],
        metavar="NAME=DIR",
        help="Optional in-distribution split evaluation, e.g. train=runs/eval_best_train.",
    )
    parser.add_argument("--out", default="report/best_model")
    return parser.parse_args()


def _read_metrics(eval_dir: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with (eval_dir / "metrics.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            for key in (
                "duration_s",
                "ate_m",
                "rpe_position_m",
                "rpe_rotation_deg",
                "end_drift_percent",
                "path_length_m",
            ):
                row[key] = float(row[key])
            rows.append(row)
    return rows


def _summary(eval_dir: Path) -> dict[str, Any]:
    return json.loads((eval_dir / "summary.json").read_text(encoding="utf-8"))


def _load_traj(eval_dir: Path, seq_id: str) -> tuple[np.ndarray, ...]:
    data = np.load(eval_dir / "trajectories" / f"{seq_id}.npz")
    return (
        data["predicted"].astype(np.float64),
        data["ground_truth"].astype(np.float64),
        data["predicted_rotations"].astype(np.float64),
        data["ground_truth_rotations"].astype(np.float64),
    )


def _align_no_scale(predicted: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Kabsch alignment matching scripts/evaluate.py."""
    mu_p = predicted.mean(axis=0)
    mu_t = target.mean(axis=0)
    covariance = (predicted - mu_p).T @ (target - mu_t)
    u, _s, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    return (predicted - mu_p) @ rotation.T + mu_t


def _optimal_scale(predicted: np.ndarray, target: np.ndarray) -> tuple[float, np.ndarray]:
    """Return per-sequence optimal scale and scaled-aligned prediction."""
    mu_p = predicted.mean(axis=0)
    mu_t = target.mean(axis=0)
    p = predicted - mu_p
    t = target - mu_t
    covariance = p.T @ t
    u, singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
        singular[-1] *= -1.0
    variance = float((p**2).sum())
    scale = float(singular.sum() / max(variance, 1e-12))
    return scale, (p * scale) @ rotation.T + mu_t


def _rmse(predicted: np.ndarray, target: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((predicted - target) ** 2, axis=1))))


def _path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _yaw_from_quat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _relative_angle(rotation: np.ndarray, target: np.ndarray) -> float:
    value = (np.trace(rotation.T @ target) - 1.0) / 2.0
    return float(math.degrees(math.acos(float(np.clip(value, -1.0, 1.0)))))


def _record(row: dict[str, Any], eval_dir: Path, processed_dir: Path) -> dict[str, Any]:
    seq_id = row["seq_id"]
    arrays = np.load(processed_dir / f"{seq_id}.npz")
    tof = arrays["tof"].astype(np.float64)
    weight = arrays["tof_weight"].astype(np.float64)
    t_tof = arrays["t_tof"].astype(np.float64)
    gt_pose = arrays["gt_pose"].astype(np.float64)
    positions = gt_pose[:, :3]
    quat = gt_pose[:, 3:7]
    dt = float(np.median(np.diff(t_tof))) if len(t_tof) > 1 else 1.0 / 15.0

    t_weight = weight.reshape(len(weight), 4, -1)
    t_range = tof.reshape(len(tof), 4, -1)
    view_valid = t_weight.mean(axis=(0, 2))
    near = ((t_range > 0.015) & (t_range < 0.3)).mean(axis=(0, 2))
    mean_range = []
    for view in range(4):
        mask = t_weight[:, view] > 0
        mean_range.append(float(t_range[:, view][mask].mean()) if mask.any() else float("nan"))

    yaw = np.unwrap(np.array([_yaw_from_quat(q) for q in quat]))
    yaw_rate = float(np.mean(np.abs(np.diff(yaw)) / max(dt, 1e-9)))
    velocity = np.linalg.norm(np.diff(positions, axis=0), axis=1) / max(dt, 1e-9)

    predicted, ground_truth, predicted_rot, ground_rot = _load_traj(eval_dir, seq_id)
    aligned = _align_no_scale(predicted, ground_truth)
    error = np.linalg.norm(aligned - ground_truth, axis=1)
    scale, scaled = _optimal_scale(predicted, ground_truth)

    gt_path = _path_length(ground_truth)
    pred_path = _path_length(predicted)
    gt_step = gt_path / max(len(ground_truth) - 1, 1)
    pred_step = pred_path / max(len(predicted) - 1, 1)
    final_error = float(error[-1])
    if len(error) >= 3:
        slope = float(np.polyfit(t_tof - t_tof[0], error, 1)[0])
    else:
        slope = 0.0

    return {
        "seq_id": seq_id,
        "scene": row["scene"],
        "kind": row["kind"],
        "duration_s": row["duration_s"],
        "path_length_m": row["path_length_m"],
        "ate_m": row["ate_m"],
        "rpe_position_m": row["rpe_position_m"],
        "rpe_rotation_deg": row["rpe_rotation_deg"],
        "edr": row["end_drift_percent"],
        "speed_mps": row["path_length_m"] / max(row["duration_s"], 1e-9),
        "gt_path_m": gt_path,
        "pred_path_m": pred_path,
        "path_ratio": pred_path / max(gt_path, 1e-9),
        "gt_step_cm": 100.0 * gt_step,
        "pred_step_cm": 100.0 * pred_step,
        "opt_scale": scale,
        "ate_scaled_m": _rmse(scaled, ground_truth),
        "final_error_m": final_error,
        "error_slope_mps": slope,
        "heading_error_deg": _relative_angle(predicted_rot[-1], ground_rot[-1]),
        "valid_ratio": float(t_weight.mean()),
        "min_view_valid": float(view_valid.min()),
        "front_valid": float(view_valid[0]),
        "left_valid": float(view_valid[1]),
        "right_valid": float(view_valid[2]),
        "down_valid": float(view_valid[3]),
        "front_near_ratio": float(near[0]),
        "down_near_ratio": float(near[3]),
        "front_range_m": mean_range[0],
        "left_range_m": mean_range[1],
        "right_range_m": mean_range[2],
        "down_range_m": mean_range[3],
        "yaw_rate_deg_s": float(np.degrees(yaw_rate)),
        "vertical_range_m": float(np.ptp(positions[:, 2])),
        "z_std_m": float(np.std(positions[:, 2])),
        "mean_speed_from_gt_mps": float(velocity.mean()),
        "time_s": t_tof - t_tof[0],
        "error_m": error,
        "aligned": aligned,
        "ground_truth": ground_truth,
        "predicted": predicted,
    }


def _split_summary(name: str, eval_dir: Path) -> dict[str, Any]:
    rows = _read_metrics(eval_dir)
    gt_steps, pred_steps, ratios = [], [], []
    for row in rows:
        predicted, truth, _pr, _tr = _load_traj(eval_dir, row["seq_id"])
        gt_step = _path_length(truth) / max(len(truth) - 1, 1)
        pred_step = _path_length(predicted) / max(len(predicted) - 1, 1)
        gt_steps.append(100.0 * gt_step)
        pred_steps.append(100.0 * pred_step)
        ratios.append(pred_step / max(gt_step, 1e-12))
    return {
        "name": name,
        "dir": str(eval_dir),
        "sequences": len(rows),
        "ate_m": float(np.mean([r["ate_m"] for r in rows])),
        "rpe_position_m": float(np.mean([r["rpe_position_m"] for r in rows])),
        "rpe_rotation_deg": float(np.mean([r["rpe_rotation_deg"] for r in rows])),
        "edr": float(np.mean([r["end_drift_percent"] for r in rows])),
        "mean_step_gt_cm": float(np.mean(gt_steps)),
        "mean_step_pred_cm": float(np.mean(pred_steps)),
        "path_ratio": float(np.mean(ratios)),
        "speed_mps": float(
            sum(r["path_length_m"] for r in rows) / max(sum(r["duration_s"] for r in rows), 1e-9)
        ),
        "points": list(zip(gt_steps, pred_steps)),
    }


def _pearson(records: list[dict[str, Any]], key: str) -> float:
    x = np.array([r[key] for r in records], dtype=float)
    y = np.array([r["ate_m"] for r in records], dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    return float(np.corrcoef(x[mask], y[mask])[0, 1])


def _partial_pearson(records: list[dict[str, Any]], key: str, control: str = "speed_mps") -> float:
    x = np.array([r[key] for r in records], dtype=float)
    y = np.array([r["ate_m"] for r in records], dtype=float)
    z = np.array([r[control] for r in records], dtype=float)
    mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[mask], y[mask], z[mask]
    design = np.column_stack([np.ones_like(z), z])
    rx = x - design @ np.linalg.lstsq(design, x, rcond=None)[0]
    ry = y - design @ np.linalg.lstsq(design, y, rcond=None)[0]
    return float(np.corrcoef(rx, ry)[0, 1])


def _speed_bins(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    edges = [(0.0, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 2.0)]
    result = []
    for low, high in edges:
        selected = [r for r in records if low <= r["speed_mps"] < high]
        if not selected:
            continue
        result.append(
            {
                "label": f"{low:.1f}-{high:.1f}",
                "n": len(selected),
                "ate_m": float(np.mean([r["ate_m"] for r in selected])),
                "ate_scaled_m": float(np.mean([r["ate_scaled_m"] for r in selected])),
                "path_ratio": float(np.mean([r["path_ratio"] for r in selected])),
                "opt_scale": float(np.mean([r["opt_scale"] for r in selected])),
            }
        )
    return result


def _save(fig: plt.Figure, path: Path, dpi: int = 170) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def _plot_trajectory_gallery(records: list[dict[str, Any]], out: Path) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(11.5, 15.0))
    for row_index, scene in enumerate(SCENES):
        selected = sorted([r for r in records if r["scene"] == scene], key=lambda r: r["ate_m"])
        for col_index, record in enumerate((selected[0], selected[-1])):
            ax = axes[row_index, col_index]
            truth = record["ground_truth"]
            predicted = record["aligned"]
            ax.plot(truth[:, 0], truth[:, 1], "--", color="#222222", lw=1.5, label="GT")
            ax.plot(predicted[:, 0], predicted[:, 1], "-", color=SCENE_COLORS[scene], lw=2.0, label="TIO-Former")
            ax.scatter(truth[0, 0], truth[0, 1], marker="o", s=38, color="#2e8b57", zorder=5)
            ax.scatter(truth[-1, 0], truth[-1, 1], marker="s", s=38, color="#222222", zorder=5)
            ax.scatter(predicted[-1, 0], predicted[-1, 1], marker="x", s=42, color="#b22222", zorder=6)
            ax.set_title(
                f"{scene} / {record['kind']} / {record['seq_id']}\n"
                f"ATE {record['ate_m']:.3f} m  EDR {record['edr']:.1f}%  "
                f"v {record['speed_mps']:.2f} m/s  ratio {record['path_ratio']:.2f}",
                fontsize=8.5,
            )
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.grid(alpha=0.25)
            ax.set_aspect("equal", adjustable="datalim")
            if row_index == 0 and col_index == 0:
                ax.legend(frameon=False, fontsize=8, loc="best")
    fig.suptitle("Best/worst test trajectories per scene (SE(3)-aligned, no scale)", fontsize=13)
    _save(fig, out / "trajectory_gallery.png")


def _plot_trajectory_3d(records: list[dict[str, Any]], out: Path) -> None:
    ordered = sorted(records, key=lambda r: r["ate_m"])
    selected = [ordered[0], ordered[len(ordered) // 2], ordered[-1]]
    fig = plt.figure(figsize=(15.0, 4.8))
    for index, record in enumerate(selected):
        ax = fig.add_subplot(1, 3, index + 1, projection="3d")
        truth = record["ground_truth"]
        predicted = record["aligned"]
        ax.plot(truth[:, 0], truth[:, 1], truth[:, 2], "--", color="#222222", lw=1.5, label="GT")
        ax.plot(
            predicted[:, 0],
            predicted[:, 1],
            predicted[:, 2],
            color=SCENE_COLORS[record["scene"]],
            lw=2.0,
            label="TIO-Former",
        )
        ranges = np.ptp(np.vstack([truth, predicted]), axis=0)
        ax.set_box_aspect(tuple(np.maximum(ranges, 0.15)))
        ax.set_title(
            f"{record['seq_id']}\nATE {record['ate_m']:.3f} m  EDR {record['edr']:.1f}%",
            fontsize=9,
        )
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        if index == 0:
            ax.legend(frameon=False, fontsize=8)
    fig.suptitle("3D view: best / median / worst test sequence", fontsize=13)
    _save(fig, out / "trajectory_3d.png")


def _plot_error_curves(records: list[dict[str, Any]], out: Path) -> None:
    ordered = sorted(records, key=lambda r: r["ate_m"])
    worst_canonical = max(
        [r for r in records if r["kind"] == "canonical"], key=lambda r: r["ate_m"]
    )
    selected = [ordered[0], ordered[len(ordered) // 2], ordered[-1], worst_canonical]
    titles = ["best overall", "median", "worst overall", "worst canonical"]
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.5))
    for ax, record, label in zip(axes.ravel(), selected, titles):
        ax.plot(record["time_s"], record["error_m"], color=SCENE_COLORS[record["scene"]], lw=2.0)
        ax.fill_between(record["time_s"], 0.0, record["error_m"], color=SCENE_COLORS[record["scene"]], alpha=0.12)
        ax.set_title(
            f"{label}: {record['seq_id']}\nATE {record['ate_m']:.3f} m  "
            f"EDR {record['edr']:.1f}%  final {record['final_error_m']:.3f} m",
            fontsize=9,
        )
        ax.set_xlabel("time (s)")
        ax.set_ylabel("position error (m)")
        ax.grid(alpha=0.25)
    fig.suptitle("Position error over time (after SE(3) alignment)", fontsize=13)
    _save(fig, out / "error_over_time.png")


def _plot_metrics_overview(records: list[dict[str, Any]], out: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(14.0, 7.8))
    ate = np.array([r["ate_m"] for r in records])
    edr = np.array([r["edr"] for r in records])
    axes[0, 0].hist(ate, bins=16, color="#3b6ea8", alpha=0.85)
    axes[0, 0].axvline(ate.mean(), color="#c46a3a", ls="--", label=f"mean {ate.mean():.3f}")
    axes[0, 0].set_title("ATE distribution")
    axes[0, 0].set_xlabel("ATE (m)")
    axes[0, 0].set_ylabel("sequences")
    axes[0, 0].legend(frameon=False)

    axes[0, 1].hist(edr, bins=16, color="#587d5a", alpha=0.85)
    axes[0, 1].axvline(edr.mean(), color="#c46a3a", ls="--", label=f"mean {edr.mean():.1f}%")
    axes[0, 1].set_title("End-drift distribution")
    axes[0, 1].set_xlabel("EDR (%)")
    axes[0, 1].set_ylabel("sequences")
    axes[0, 1].legend(frameon=False)

    scene_values = [[r["ate_m"] for r in records if r["scene"] == scene] for scene in SCENES]
    axes[0, 2].boxplot(scene_values, tick_labels=list(SCENES), patch_artist=True)
    for patch, scene in zip(axes[0, 2].patches, SCENES):
        patch.set_facecolor(SCENE_COLORS[scene])
        patch.set_alpha(0.6)
    axes[0, 2].set_title("ATE by scene")
    axes[0, 2].set_ylabel("ATE (m)")
    axes[0, 2].tick_params(axis="x", rotation=20)

    kind_values = [[r["ate_m"] for r in records if r["kind"] == kind] for kind in ("random", "canonical")]
    axes[1, 0].boxplot(kind_values, tick_labels=["random", "canonical"], patch_artist=True)
    for patch, kind in zip(axes[1, 0].patches, ("random", "canonical")):
        patch.set_facecolor(KIND_COLORS[kind])
        patch.set_alpha(0.6)
    axes[1, 0].set_title("ATE by trajectory type")
    axes[1, 0].set_ylabel("ATE (m)")

    for scene in SCENES:
        values = np.sort([r["ate_m"] for r in records if r["scene"] == scene])
        axes[1, 1].plot(values, np.linspace(0, 1, len(values)), color=SCENE_COLORS[scene], label=scene)
    axes[1, 1].set_title("CDF of ATE")
    axes[1, 1].set_xlabel("ATE (m)")
    axes[1, 1].set_ylabel("cumulative fraction")
    axes[1, 1].legend(frameon=False, fontsize=8)
    axes[1, 1].grid(alpha=0.25)

    for scene in SCENES:
        selected = [r for r in records if r["scene"] == scene]
        axes[1, 2].scatter(
            [r["rpe_rotation_deg"] for r in selected],
            [r["rpe_position_m"] for r in selected],
            s=24,
            color=SCENE_COLORS[scene],
            label=scene,
            alpha=0.85,
        )
    axes[1, 2].set_title("RPE rotation vs translation")
    axes[1, 2].set_xlabel("RPE rotation (deg)")
    axes[1, 2].set_ylabel("RPE position (m)")
    axes[1, 2].grid(alpha=0.25)
    axes[1, 2].legend(frameon=False, fontsize=8)
    for ax in axes.ravel():
        ax.tick_params(labelsize=8)
    fig.suptitle("Best model test accuracy overview", fontsize=13)
    _save(fig, out / "metrics_overview.png")


def _plot_scene_kind(records: list[dict[str, Any]], out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 4.6))
    x = np.arange(len(SCENES))
    width = 0.36
    for ax, metric, label in zip(axes, ("ate_m", "edr"), ("ATE (m)", "EDR (%)")):
        for offset, kind in ((-width / 2, "random"), (width / 2, "canonical")):
            values = [
                np.mean([r[metric] for r in records if r["scene"] == scene and r["kind"] == kind])
                if any(r["scene"] == scene and r["kind"] == kind for r in records)
                else 0.0
                for scene in SCENES
            ]
            ax.bar(x + offset, values, width, color=KIND_COLORS[kind], alpha=0.85, label=kind)
        ax.set_xticks(x)
        ax.set_xticklabels(SCENES, rotation=20)
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.25)
        ax.legend(frameon=False)
        ax.set_title(f"{label} by scene and trajectory type")
    fig.suptitle("Where the error concentrates", fontsize=13)
    _save(fig, out / "scene_kind.png")


def _plot_speed_scale(
    records: list[dict[str, Any]],
    split_summaries: dict[str, dict[str, Any]],
    out: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 9.0))
    ax = axes[0, 0]
    for scene in SCENES:
        selected = [r for r in records if r["scene"] == scene]
        ax.scatter(
            [r["speed_mps"] for r in selected],
            [r["path_ratio"] for r in selected],
            color=SCENE_COLORS[scene],
            s=24,
            alpha=0.85,
            label=scene,
        )
    x = np.array([r["speed_mps"] for r in records])
    y = np.array([r["path_ratio"] for r in records])
    slope, intercept = np.polyfit(x, y, 1)
    xx = np.linspace(x.min(), x.max(), 100)
    ax.plot(xx, slope * xx + intercept, color="#222222", ls="--", lw=1.2)
    ax.set_title(f"Speed vs predicted path ratio (r={np.corrcoef(x, y)[0, 1]:+.3f})")
    ax.set_xlabel("GT speed (m/s)")
    ax.set_ylabel("predicted / GT path length")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False, fontsize=8)

    ax = axes[0, 1]
    colors = {"train": "#587d5a", "val": "#c46a3a", "test": "#3b6ea8"}
    for name, summary in split_summaries.items():
        points = np.array(summary["points"])
        ax.scatter(
            points[:, 0],
            points[:, 1],
            s=18,
            alpha=0.6,
            color=colors.get(name, "#777777"),
            label=f"{name} (n={len(points)})",
        )
    all_points = np.vstack([np.array(s["points"]) for s in split_summaries.values()])
    upper = max(all_points.max(), 4.0)
    ax.plot([0, upper], [0, upper], color="#222222", ls="--", lw=1.0, label="ideal")
    ax.set_title("GT vs predicted mean step length")
    ax.set_xlabel("GT step (cm)")
    ax.set_ylabel("predicted step (cm)")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False, fontsize=8)

    ax = axes[1, 0]
    names = list(split_summaries)
    xpos = np.arange(len(names))
    width = 0.36
    ax.bar(
        xpos - width / 2,
        [split_summaries[name]["mean_step_gt_cm"] for name in names],
        width,
        color="#555555",
        label="GT",
    )
    ax.bar(
        xpos + width / 2,
        [split_summaries[name]["mean_step_pred_cm"] for name in names],
        width,
        color="#3b6ea8",
        label="predicted",
    )
    for index, name in enumerate(names):
        entry = split_summaries[name]
        step_ratio = entry["mean_step_pred_cm"] / max(entry["mean_step_gt_cm"], 1e-9)
        ax.text(
            index,
            max(entry["mean_step_gt_cm"], entry["mean_step_pred_cm"]) + 0.06,
            f"step ratio {step_ratio:.2f}",
            ha="center",
            fontsize=8,
        )
    ax.set_xticks(xpos)
    ax.set_xticklabels(names)
    ax.set_ylabel("mean step (cm)")
    ax.set_title("Model predicts a nearly constant step length")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)

    ax = axes[1, 1]
    bins = _speed_bins(records)
    labels = [entry["label"] for entry in bins]
    xpos = np.arange(len(bins))
    width = 0.36
    ax.bar(xpos - width / 2, [entry["ate_m"] for entry in bins], width, color="#c46a3a", label="no scale")
    ax.bar(
        xpos + width / 2,
        [entry["ate_scaled_m"] for entry in bins],
        width,
        color="#587d5a",
        label="oracle per-seq scale",
    )
    for index, entry in enumerate(bins):
        ax.text(
            index,
            max(entry["ate_m"], entry["ate_scaled_m"]) + 0.012,
            f"ratio {entry['path_ratio']:.2f}",
            ha="center",
            fontsize=8,
        )
    ax.set_xticks(xpos)
    ax.set_xticklabels(labels)
    ax.set_xlabel("GT speed (m/s)")
    ax.set_ylabel("mean ATE (m)")
    ax.set_title("Scale correction removes much of the error")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)

    fig.suptitle("Diagnosis: speed-dependent scale collapse", fontsize=13)
    _save(fig, out / "speed_scale.png")


def _plot_correlations(records: list[dict[str, Any]], out: Path) -> None:
    entries = [
        ("speed", "speed_mps"),
        ("path length", "path_length_m"),
        ("duration", "duration_s"),
        ("valid ToF ratio", "valid_ratio"),
        ("min view valid", "min_view_valid"),
        ("left view valid", "left_valid"),
        ("vertical range", "vertical_range_m"),
        ("yaw rate", "yaw_rate_deg_s"),
        ("path ratio", "path_ratio"),
        ("front range", "front_range_m"),
        ("down range", "down_range_m"),
        ("RPE rotation", "rpe_rotation_deg"),
    ]
    values = [(label, _pearson(records, key)) for label, key in entries]
    values.sort(key=lambda item: item[1])
    labels = [item[0] for item in values]
    correlations = [item[1] for item in values]
    fig, ax = plt.subplots(figsize=(8.0, 6.2))
    colors = ["#c46a3a" if value >= 0 else "#3b6ea8" for value in correlations]
    ax.barh(labels, correlations, color=colors, alpha=0.85)
    ax.axvline(0, color="#222222", lw=0.8)
    for index, value in enumerate(correlations):
        ax.text(
            value + (0.015 if value >= 0 else -0.015),
            index,
            f"{value:+.3f}",
            va="center",
            ha="left" if value >= 0 else "right",
            fontsize=8,
        )
    ax.set_xlim(min(-0.95, min(correlations) - 0.12), max(0.85, max(correlations) + 0.12))
    ax.set_xlabel("Pearson correlation with ATE")
    ax.set_title("Univariate correlation with ATE")
    ax.grid(axis="x", alpha=0.25)
    _save(fig, out / "feature_correlations.png")


def _plot_view_validity(records: list[dict[str, Any]], out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 4.6))
    x = np.arange(len(SCENES))
    width = 0.19
    views = ("front", "left", "right", "down")
    colors = ("#3b6ea8", "#c46a3a", "#587d5a", "#8a5a9e")
    for offset, view, color in zip(np.linspace(-1.5 * width, 1.5 * width, 4), views, colors):
        values = [
            np.mean([r[f"{view}_valid"] for r in records if r["scene"] == scene])
            for scene in SCENES
        ]
        axes[0].bar(x + offset, values, width, color=color, alpha=0.85, label=view)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(SCENES, rotation=20)
    axes[0].set_ylabel("valid zone ratio")
    axes[0].set_title("ToF validity by view and scene")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False, fontsize=8)

    for scene in SCENES:
        selected = [r for r in records if r["scene"] == scene]
        axes[1].scatter(
            [r["left_valid"] for r in selected],
            [r["ate_m"] for r in selected],
            s=24,
            color=SCENE_COLORS[scene],
            alpha=0.85,
            label=scene,
        )
    xv = np.array([r["left_valid"] for r in records])
    yv = np.array([r["ate_m"] for r in records])
    slope, intercept = np.polyfit(xv, yv, 1)
    xx = np.linspace(xv.min(), xv.max(), 100)
    axes[1].plot(xx, slope * xx + intercept, color="#222222", ls="--", lw=1.2)
    axes[1].set_xlabel("left-view valid ratio")
    axes[1].set_ylabel("ATE (m)")
    axes[1].set_title(f"Occlusion vs error (r={np.corrcoef(xv, yv)[0, 1]:+.3f})")
    axes[1].grid(alpha=0.25)
    axes[1].legend(frameon=False, fontsize=8)
    fig.suptitle("ToF visibility is a secondary factor", fontsize=13)
    _save(fig, out / "view_validity.png")


def _plot_ablation(records: list[dict[str, Any]], out: Path) -> None:
    baseline = _summary(ROOT / "runs/eval_main_imu")
    best = _summary(ROOT / "runs/eval_main_imu_seed1")
    hybrid = _summary(ROOT / "runs/eval_main_v2")
    drop_all = _summary(ROOT / "runs/eval_main_imu_dropall")
    ablations = json.loads((ROOT / "runs/ablations_imu/summary.json").read_text(encoding="utf-8"))
    ablations = {row["variant"]: row for row in ablations}

    rows = [
        ("best seed1", best["metrics"]["ate_m"], best["metrics"]["end_drift_percent"], "best"),
        ("full seed0", baseline["metrics"]["ate_m"], baseline["metrics"]["end_drift_percent"], "full"),
        ("hybrid_gated", hybrid["metrics"]["ate_m"], hybrid["metrics"]["end_drift_percent"], "variant"),
        ("naive_concat", ablations["naive_concat"]["ate_m_mean"], ablations["naive_concat"]["end_drift_percent_mean"], "ablation"),
        ("kv_only_no_fifo", ablations["kv_only_no_fifo"]["ate_m_mean"], ablations["kv_only_no_fifo"]["end_drift_percent_mean"], "ablation"),
        ("imu_only", ablations["imu_only"]["ate_m_mean"], ablations["imu_only"]["end_drift_percent_mean"], "ablation"),
        ("memoryless", ablations["memoryless"]["ate_m_mean"], ablations["memoryless"]["end_drift_percent_mean"], "ablation"),
        ("tof_only", ablations["tof_only"]["ate_m_mean"], ablations["tof_only"]["end_drift_percent_mean"], "ablation"),
        ("no ToF (seed0)", drop_all["metrics"]["ate_m"], drop_all["metrics"]["end_drift_percent"], "drop"),
    ]
    palette = {
        "best": "#2e8b57",
        "full": "#3b6ea8",
        "variant": "#8a5a9e",
        "ablation": "#c46a3a",
        "drop": "#777777",
    }
    labels = [row[0] for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(14.0, 5.2))
    for ax, metric_index, ylabel in zip(axes, (1, 2), ("ATE (m)", "EDR (%)")):
        values = [row[metric_index] for row in rows]
        ax.bar(labels, values, color=[palette[row[3]] for row in rows], alpha=0.9)
        for index, value in enumerate(values):
            ax.text(index, value + (0.008 if metric_index == 1 else 0.15), f"{value:.3f}" if metric_index == 1 else f"{value:.1f}", ha="center", fontsize=7.5)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", rotation=35, labelsize=8)
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_title("ATE: best model vs ablations")
    axes[1].set_title("EDR: best model vs ablations")
    fig.suptitle("Architecture is not the bottleneck (ablations are 15 epochs unless noted)", fontsize=12)
    _save(fig, out / "ablation.png")


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def _write_report(
    records: list[dict[str, Any]],
    summary: dict[str, Any],
    split_summaries: dict[str, dict[str, Any]],
    out: Path,
) -> None:
    overall_ate = float(np.mean([r["ate_m"] for r in records]))
    overall_edr = float(np.mean([r["edr"] for r in records]))
    overall_ratio = float(np.mean([r["path_ratio"] for r in records]))
    overall_scaled = float(np.mean([r["ate_scaled_m"] for r in records]))
    overall_scale = float(np.mean([r["opt_scale"] for r in records]))
    speed_bins = _speed_bins(records)
    speed_values = np.array([r["speed_mps"] for r in records])
    ratio_values = np.array([r["path_ratio"] for r in records])
    speed_ratio_corr = float(np.corrcoef(speed_values, ratio_values)[0, 1])
    ratio_ate_corr = _pearson(records, "path_ratio")

    scene_rows = []
    for scene in SCENES:
        selected = [r for r in records if r["scene"] == scene]
        scene_rows.append(
            [
                scene,
                len(selected),
                f"{np.mean([r['ate_m'] for r in selected]):.3f}",
                f"{np.mean([r['rpe_position_m'] for r in selected]):.4f}",
                f"{np.mean([r['rpe_rotation_deg'] for r in selected]):.3f}",
                f"{np.mean([r['edr'] for r in selected]):.2f}",
                f"{np.mean([r['speed_mps'] for r in selected]):.3f}",
                f"{np.mean([r['valid_ratio'] for r in selected]):.3f}",
                f"{np.mean([r['path_ratio'] for r in selected]):.3f}",
            ]
        )

    split_rows = []
    for name in ("train", "val", "test"):
        if name not in split_summaries:
            continue
        entry = split_summaries[name]
        split_rows.append(
            [
                name,
                entry["sequences"],
                f"{entry['ate_m']:.3f}",
                f"{entry['rpe_position_m']:.4f}",
                f"{entry['rpe_rotation_deg']:.3f}",
                f"{entry['edr']:.2f}",
                f"{entry['speed_mps']:.3f}",
                f"{entry['mean_step_gt_cm']:.2f}",
                f"{entry['mean_step_pred_cm']:.2f}",
                f"{entry['path_ratio']:.3f}",
            ]
        )

    speed_rows = [
        [
            entry["label"],
            entry["n"],
            f"{entry['ate_m']:.3f}",
            f"{entry['ate_scaled_m']:.3f}",
            f"{entry['path_ratio']:.3f}",
            f"{entry['opt_scale']:.2f}",
        ]
        for entry in speed_bins
    ]

    correlations = [
        ("speed", _pearson(records, "speed_mps")),
        ("path length", _pearson(records, "path_length_m")),
        ("duration", _pearson(records, "duration_s")),
        ("valid ToF ratio", _pearson(records, "valid_ratio")),
        ("left view valid", _pearson(records, "left_valid")),
        ("vertical range", _pearson(records, "vertical_range_m")),
        ("yaw rate", _pearson(records, "yaw_rate_deg_s")),
        ("pred/GT path ratio", _pearson(records, "path_ratio")),
    ]
    corr_rows = [[label, f"{value:+.3f}"] for label, value in correlations]

    text = f"""# TIO-Former 最佳模型：轨迹、精度与失效因素分析

生成脚本：`scripts/analyze_best_model.py`  
模型检查点：`{summary['checkpoint']}`  
分支：`dev/odom-tio`  
说明：这里展示 3 个 seed 中测试 ATE 最好的 seed 1；总体泛化结论仍以
3-seed 均值 **0.4073 ± 0.0218 m** 为准，不能把 best-of-3 当作无偏指标。

## 1. 结论摘要

- seed 1 在 **103 条 test 序列、1,468.6 s、562.7 m** 上：
  **ATE {overall_ate:.4f} m、RPEp {summary['metrics']['rpe_position_m']:.4f} m、
  RPER {summary['metrics']['rpe_rotation_deg']:.3f}°、EDR {overall_edr:.2f}%**。
- 位置误差的主要来源不是旋转，而是**平移尺度塌缩**：
  预测轨迹长度只有 GT 的 **{overall_ratio:.3f}**；对每条序列做 oracle 尺度修正后，
  ATE 从 **{overall_ate:.3f} m 降到 {overall_scaled:.3f} m（约
  {100 * (1 - overall_scaled / overall_ate):.0f}%）**。
- 模型对速度的响应几乎饱和。train / val / test 的真实平均步长分别约为
  {split_summaries.get('train', {}).get('mean_step_gt_cm', float('nan')):.2f} /
  {split_summaries.get('val', {}).get('mean_step_gt_cm', float('nan')):.2f} /
  {split_summaries.get('test', {}).get('mean_step_gt_cm', float('nan')):.2f} cm，
  但模型预测的平均步长只有
  {split_summaries.get('train', {}).get('mean_step_pred_cm', float('nan')):.2f} /
  {split_summaries.get('val', {}).get('mean_step_pred_cm', float('nan')):.2f} /
  {split_summaries.get('test', {}).get('mean_step_pred_cm', float('nan')):.2f} cm，
  几乎不随真实步长变化。
  也就是说，模型学到的是训练集的“平均步长先验”，而不是从 ToF/IMU 中恢复真实尺度。
- 这解释了为什么 test 越快、ATE 越大：test 平均速度 {split_summaries['test']['speed_mps']:.3f} m/s，
  train 只有 {split_summaries['train']['speed_mps']:.3f} m/s。test 被系统性低估；
  train 上预测步长 {split_summaries['train']['mean_step_pred_cm']:.2f} cm 也略高于
  GT 的 {split_summaries['train']['mean_step_gt_cm']:.2f} cm，且序列更长，
  因而累积 EDR 达到 {split_summaries['train']['edr']:.1f}%。

## 2. 轨迹叠加

![best/worst per scene](figures/trajectory_gallery.png)

每个场景给出 ATE 最好和最差的测试序列。GT 为黑色虚线，TIO-Former 为彩色实线，
绿色圆点为起点，黑色方块为 GT 终点，红色叉号为预测终点。较好的序列能保持轨迹形状，
但普遍“缩水”；较差的办公室 canonical 序列在转弯后明显丢失平移尺度。

![3D best/median/worst](figures/trajectory_3d.png)

![position error over time](figures/error_over_time.png)

## 3. 精度统计

![accuracy overview](figures/metrics_overview.png)

{_table(['scene', 'n', 'ATE m', 'RPEp m', 'RPER deg', 'EDR %', 'speed m/s', 'valid ratio', 'pred/GT path'], scene_rows)}

![scene and trajectory type](figures/scene_kind.png)

{_table(['split', 'n', 'ATE m', 'RPEp m', 'RPER deg', 'EDR %', 'speed m/s', 'GT step cm', 'pred step cm', 'pred/GT path'], split_rows)}

注：`pred/GT path` 是逐序列路径长度比的平均值，近静止序列会放大该值；
平均步长比见图 `speed_scale.png` 左下角。test 的两种统计都表明明显欠尺度。

canonical 轨迹的 ATE 明显高于 random：canonical 平均
**{np.mean([r['ate_m'] for r in records if r['kind'] == 'canonical']):.3f} m**，
random 平均 **{np.mean([r['ate_m'] for r in records if r['kind'] == 'random']):.3f} m**。
canonical 的旋转误差反而更低（RPER
{np.mean([r['rpe_rotation_deg'] for r in records if r['kind'] == 'canonical']):.3f}°），
说明问题主要是平移尺度，而不是朝向估计。

## 4. 关键诊断：尺度塌缩

![speed and scale diagnosis](figures/speed_scale.png)

- 测试集 speed 与 ATE 的相关系数为 **{_pearson(records, 'speed_mps'):+.3f}**；
  speed 与 pred/GT path ratio 的相关系数为 **{speed_ratio_corr:+.3f}**：
  越快的轨迹，预测越“缩水”。
- path ratio 与 ATE 的相关系数为 **{ratio_ate_corr:+.3f}**：
  尺度误差几乎直接决定 ATE。
- 平均最优尺度为 **{overall_scale:.2f}×**，说明模型平均只走了真实位移的
  {overall_ratio * 100:.1f}%。
- 分速度段：

{_table(['speed m/s', 'n', 'ATE m', 'oracle-scale ATE m', 'pred/GT path', 'mean optimal scale'], speed_rows)}

在低速段模型甚至过冲，在高速段严重欠冲；这不是简单的“整体偏小”，而是
**速度相关增益缺失**。

### 为什么会出现尺度塌缩

1. **ToF 对平移尺度的可观测性不足。** 4 个 8×8、45° FoV 的视角（缺后/上），
   8×8 像素角分辨率约 5.6°；在 1 m 距离上约对应 10 cm 的横向分辨，
   而 0.2–0.8 m/s、15 Hz 下单帧位移只有 1.3–5.3 cm。单帧 ToF 差分量与
   1 cm 级测距噪声、8×8 稀疏采样同量级，模型很难从中稳定读出速度。
2. **损失函数鼓励回归到条件中位数。** 每步位移用米制 Smooth L1 监督；
   当输入无法确定尺度时，最优策略是输出训练分布的平均步长。
   论文的 scale loss 权重只有 0.005，且是 64 帧窗口内的总位移比，
   不足以阻止尺度塌缩。
3. **训练/测试速度分布不一致。** 数据生成时 train 为 20–60 s，
   test 被限制为 4–7 m / 8–20 s，导致 test 平均速度约为 train 的
   {split_summaries['test']['speed_mps'] / split_summaries['train']['speed_mps']:.1f} 倍。
   test 因此落在模型“平均步长”先验的右侧，系统性欠冲。
4. **训练窗口太短，缺少长时一致性。** 训练样本是独立的 64 帧窗口（约 4.3 s），
   推理却跑完整 8–20 s（train 20–60 s）序列，没有跨窗口的尺度/漂移约束。
   train 上 ATE 反而更高、EDR {split_summaries['train']['edr']:.1f}%，说明
   长时累积误差同样是问题。

## 5. 其他因素

![feature correlations](figures/feature_correlations.png)

{_table(['feature', 'Pearson r with ATE'], corr_rows)}

- **ToF 有效率是次要因素。** office 的有效 zone 比例最低（约 74%），
  ATE 最高（{np.mean([r['ate_m'] for r in records if r['scene'] == 'office']):.3f} m）；
  但 valid ratio 与 ATE 的相关系数只有 {_pearson(records, 'valid_ratio'):+.3f}，
  控制速度后的偏相关约 {_partial_pearson(records, 'valid_ratio'):+.3f}。
  遮挡会加剧误差，但不是主因。
- **旋转不是主因。** RPER 平均只有
  {np.mean([r['rpe_rotation_deg'] for r in records]):.3f}°，
  最终朝向误差虽会累积，但轨迹误差的主要投影仍然是平移尺度。
- **垂直运动与误差负相关**（r={_pearson(records, 'vertical_range_m'):+.3f}），
  这不是“垂直运动更容易”，而是垂直运动较多的序列往往更慢、更短，
  是速度混杂因素。

![view validity](figures/view_validity.png)

## 6. 架构与消融

![ablations](figures/ablation.png)

- full 主模型（seed0）ATE 0.4316 m；seed1 0.3788 m；seed2 0.4116 m。
- `naive_concat` 0.4013 m、`kv_only_no_fifo` 0.3925 m 仍优于 full；
  `imu_only` 0.4464 m，`tof_only` 0.6636 m。
- 丢弃全部 ToF 视角时 ATE 0.4257 m，和 full 0.4316 m 几乎相同：
  当前 4 向仿真数据下，ToF 分支没有提供稳定的尺度/漂移校正。
- 这说明瓶颈不是“cross-attention 不够复杂”，而是**输入可观测性 + 训练目标**
  没有给模型提供可学习的尺度信号。继续加结构很可能只会过拟合。

## 7. 建议的优先级

1. **先修数据分布**：让 train/val/test 的速度、步长分布一致；不要用
   “test 4–7 m / 8–20 s”这种会系统性提高 test 速度的采样方式。
2. **显式处理尺度**：增加序列级/长时尺度损失，或输出显式速度/尺度状态；
   不要只依赖窗口内的弱 scale loss。
3. **提高 ToF 可观测性**：补后/上视角，或评估 6 向布局；在 4 向硬件上，
   考虑用陀螺去旋转后的 range flow / 几何残差，而不是纯黑盒。
4. **做长时训练**：跨窗口状态传递或 TBPTT，加入绝对轨迹/漂移约束。
5. **真机前先做净空静态和匀速直线测试**；当前旧布局真机 20 s 漂移约 0.98 m，
   新布局尚未复测，不能用于闭环飞行。

## 8. 方法说明与限制

- 轨迹图为 SE(3) no-scale 对齐后的可视化；ATE 是位置 RMSE，EDR 是末端误差
  除以 GT 路径长度。
- “oracle per-seq scale”只是诊断上界，不是可部署方法；它说明尺度误差占多大比例。
- 所有结论基于 Gazebo 4 向 ToF 仿真，不能直接等同于论文的 6 向真实飞行结果。
- seed1 是 best-of-3，报告顶部同时给出 3-seed 均值以避免选择性偏差。
"""
    (out / "analysis.md").write_text(text, encoding="utf-8")


def main() -> int:
    args = _args()
    eval_dir = (ROOT / args.eval).resolve()
    out = (ROOT / args.out).resolve()
    figure_dir = out / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    summary = _summary(eval_dir)
    rows = _read_metrics(eval_dir)
    records = [_record(row, eval_dir, (ROOT / args.processed).resolve()) for row in rows]

    split_dirs: dict[str, Path] = {}
    for item in args.split_eval:
        if "=" not in item:
            raise SystemExit(f"--split-eval expects NAME=DIR, got {item!r}")
        name, directory = item.split("=", 1)
        path = (ROOT / directory).resolve()
        if (path / "metrics.csv").exists():
            split_dirs[name] = path
    split_summaries = {name: _split_summary(name, path) for name, path in split_dirs.items()}
    if "test" not in split_summaries:
        split_summaries["test"] = _split_summary("test", eval_dir)

    _plot_trajectory_gallery(records, figure_dir)
    _plot_trajectory_3d(records, figure_dir)
    _plot_error_curves(records, figure_dir)
    _plot_metrics_overview(records, figure_dir)
    _plot_scene_kind(records, figure_dir)
    _plot_speed_scale(records, split_summaries, figure_dir)
    _plot_correlations(records, figure_dir)
    _plot_view_validity(records, figure_dir)
    _plot_ablation(records, figure_dir)
    _write_report(records, summary, split_summaries, out)

    serialisable = {
        "eval": str(eval_dir),
        "summary": summary,
        "overall": {
            "ate_m": float(np.mean([r["ate_m"] for r in records])),
            "edr": float(np.mean([r["edr"] for r in records])),
            "path_ratio": float(np.mean([r["path_ratio"] for r in records])),
            "ate_scaled_m": float(np.mean([r["ate_scaled_m"] for r in records])),
            "opt_scale": float(np.mean([r["opt_scale"] for r in records])),
        },
        "splits": {
            name: {key: value for key, value in entry.items() if key != "points"}
            for name, entry in split_summaries.items()
        },
        "speed_bins": _speed_bins(records),
        "records": [
            {
                key: value
                for key, value in record.items()
                if key not in {"time_s", "error_m", "aligned", "ground_truth", "predicted"}
            }
            for record in records
        ],
    }
    (out / "analysis.json").write_text(
        json.dumps(serialisable, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"wrote {out}")
    print(
        "best model: ATE %.4f m, EDR %.2f%%, pred/GT path %.3f, oracle-scale ATE %.4f m"
        % (
            serialisable["overall"]["ate_m"],
            serialisable["overall"]["edr"],
            serialisable["overall"]["path_ratio"],
            serialisable["overall"]["ate_scaled_m"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
