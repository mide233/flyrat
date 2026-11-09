#!/usr/bin/env python3
"""Plot v3 dataset split distributions for the speed/step fix."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
COLORS = {"train": "#3b6ea8", "val": "#c46a3a", "test": "#587d5a"}
SPEED_EDGES = [0.10, 0.20, 0.30, 0.40, 0.50, 0.65]


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", default="data_v3/index.csv")
    parser.add_argument("--out", default="report/v3_dataset")
    return parser.parse_args()


def _read(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            duration = float(row["duration_s"])
            path_length = float(row["path_length_m"])
            frames = max(int(row["frames"]), 2)
            rows.append(
                {
                    "split": row["split"],
                    "scene": row["scene"],
                    "speed": path_length / max(duration, 1e-9),
                    "step_cm": 100.0 * path_length / (frames - 1),
                    "duration": duration,
                    "path": path_length,
                }
            )
    return rows


def _save(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = _args()
    rows = _read((ROOT / args.index).resolve())
    out = (ROOT / args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.0))
    bins = np.linspace(0.05, 0.70, 40)
    for split in ("train", "val", "test"):
        values = [float(row["speed"]) for row in rows if row["split"] == split]
        axes[0, 0].hist(values, bins=bins, density=True, histtype="step", lw=1.8, color=COLORS[split], label=f"{split} (n={len(values)})")
    axes[0, 0].set_title("Speed distribution (v2 train/test mean: 0.174 / 0.412 m/s)")
    axes[0, 0].set_xlabel("average speed (m/s)")
    axes[0, 0].set_ylabel("density")
    axes[0, 0].legend(frameon=False)
    axes[0, 0].grid(alpha=0.25)

    width = 0.26
    x = np.arange(len(SPEED_EDGES) - 1)
    for offset, split in zip((-width, 0.0, width), ("train", "val", "test")):
        values = []
        for low, high in zip(SPEED_EDGES[:-1], SPEED_EDGES[1:]):
            values.append(
                sum(
                    1
                    for row in rows
                    if row["split"] == split and low <= float(row["speed"]) < high
                )
            )
        axes[0, 1].bar(x + offset, values, width, color=COLORS[split], alpha=0.85, label=split)
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels([f"{low:.2f}-{high:.2f}" for low, high in zip(SPEED_EDGES[:-1], SPEED_EDGES[1:])], rotation=20)
    axes[0, 1].set_title("Speed-bin counts")
    axes[0, 1].set_ylabel("sequences")
    axes[0, 1].legend(frameon=False)
    axes[0, 1].grid(axis="y", alpha=0.25)

    step_bins = np.linspace(0.5, 6.0, 40)
    for split in ("train", "val", "test"):
        values = [float(row["step_cm"]) for row in rows if row["split"] == split]
        axes[1, 0].hist(values, bins=step_bins, density=True, histtype="step", lw=1.8, color=COLORS[split], label=split)
    axes[1, 0].set_title("Per-frame translation step")
    axes[1, 0].set_xlabel("mean step (cm)")
    axes[1, 0].set_ylabel("density")
    axes[1, 0].legend(frameon=False)
    axes[1, 0].grid(alpha=0.25)

    for split in ("train", "val", "test"):
        selected = [row for row in rows if row["split"] == split]
        axes[1, 1].scatter(
            [float(row["duration"]) for row in selected],
            [float(row["path"]) for row in selected],
            s=14,
            alpha=0.55,
            color=COLORS[split],
            label=split,
        )
    axes[1, 1].set_title("Duration vs path length")
    axes[1, 1].set_xlabel("duration (s)")
    axes[1, 1].set_ylabel("path length (m)")
    axes[1, 1].legend(frameon=False)
    axes[1, 1].grid(alpha=0.25)

    fig.suptitle("TIO-Former v3.1 dataset: common speed distribution across splits", fontsize=13)
    _save(fig, out / "dataset_distribution.png")

    print(f"wrote {out / 'dataset_distribution.png'}")
    for split in ("train", "val", "test"):
        values = [float(row["speed"]) for row in rows if row["split"] == split]
        steps = [float(row["step_cm"]) for row in rows if row["split"] == split]
        print(
            f"{split}: n={len(values)} speed mean={np.mean(values):.3f} "
            f"p05-p95={np.percentile(values, 5):.3f}-{np.percentile(values, 95):.3f} "
            f"step={np.mean(steps):.2f} cm"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
