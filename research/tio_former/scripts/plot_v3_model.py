#!/usr/bin/env python3
"""Plot v3 test-set metrics: per-scene ATE, speed-bin ATE and path ratio."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval", default="runs_v3/eval_seed0_fixed_ep60")
    parser.add_argument("--out", default="report/v3_model")
    return parser.parse_args()


def main() -> int:
    args = _args()
    eval_dir = (ROOT / args.eval).resolve()
    out_dir = (ROOT / args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = json.loads((eval_dir / "summary.json").read_text(encoding="utf-8"))

    rows = []
    with (eval_dir / "metrics.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            trajectory = np.load(eval_dir / "trajectories" / f"{row['seq_id']}.npz")
            gt_path = _path_length(trajectory["ground_truth"].astype(np.float64))
            pred_path = _path_length(trajectory["predicted"].astype(np.float64))
            duration = float(row["duration_s"])
            rows.append(
                {
                    "scene": row["scene"],
                    "speed": float(row["path_length_m"]) / max(duration, 1e-9),
                    "ate": float(row["ate_m"]),
                    "ratio": pred_path / max(gt_path, 1e-9),
                }
            )

    scenes = [row["scene"] for row in summary["by_scene"]]
    scene_ate = [row["ate_m"] for row in summary["by_scene"]]
    scene_edr = [row["end_drift_percent"] for row in summary["by_scene"]]

    bins = [(0.0, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 2.0)]
    bin_labels, bin_ate, bin_ratio, bin_count = [], [], [], []
    speeds = np.array([row["speed"] for row in rows])
    ates = np.array([row["ate"] for row in rows])
    ratios = np.array([row["ratio"] for row in rows])
    for lo, hi in bins:
        mask = (speeds >= lo) & (speeds < hi)
        bin_labels.append(f"{lo:.1f}-{hi:.1f}" if hi < 2.0 else f"≥{lo:.1f}")
        bin_ate.append(float(ates[mask].mean()) if mask.any() else np.nan)
        bin_ratio.append(float(ratios[mask].mean()) if mask.any() else np.nan)
        bin_count.append(int(mask.sum()))

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    x = np.arange(len(scenes))
    axes[0].bar(x, scene_ate, color="#3b6ea5")
    axes[0].set_xticks(x, scenes, rotation=20)
    axes[0].set_ylabel("ATE (m)")
    axes[0].set_title("Test ATE by scene")
    for i, value in enumerate(scene_ate):
        axes[0].text(i, value, f"{value:.3f}", ha="center", va="bottom", fontsize=8)

    x = np.arange(len(bin_labels))
    axes[1].bar(x, bin_ate, color="#c26b3a")
    axes[1].set_xticks(x, bin_labels)
    axes[1].set_xlabel("mean speed (m/s)")
    axes[1].set_ylabel("ATE (m)")
    axes[1].set_title("Test ATE by speed bin")

    axes[2].plot(x, bin_ratio, marker="o", color="#4f8f5a")
    axes[2].axhline(1.0, color="black", linewidth=0.8, linestyle="--")
    axes[2].axhspan(0.8, 1.2, color="#4f8f5a", alpha=0.12)
    axes[2].set_xticks(x, bin_labels)
    axes[2].set_ylim(0.6, 1.4)
    axes[2].set_xlabel("mean speed (m/s)")
    axes[2].set_ylabel("predicted / GT path length")
    axes[2].set_title("Scale ratio by speed bin")

    fig.suptitle(
        f"v3.1 seed 0: ATE {summary['metrics']['ate_m']:.4f} m, "
        f"EDR {summary['metrics']['end_drift_percent']:.2f}% "
        f"({summary['sequences']} test sequences)"
    )
    fig.tight_layout()
    figure = out_dir / "seed0_metrics.png"
    fig.savefig(figure, dpi=160)
    plt.close(fig)

    print("scene ATE/EDR:")
    for name, ate, edr in zip(scenes, scene_ate, scene_edr):
        print(f"  {name}: ATE {ate:.4f} m, EDR {edr:.2f}%")
    print("speed bins:")
    for label, ate, ratio, count in zip(bin_labels, bin_ate, bin_ratio, bin_count):
        print(f"  {label}: n={count}, ATE {ate:.4f} m, ratio {ratio:.3f}")
    print(f"wrote {figure}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
