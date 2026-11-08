#!/usr/bin/env python3
"""Create M5 seed and LOSO figures from evaluation summaries."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="runs/seeds_summary.json")
    parser.add_argument("--loso", default="runs/loso/summary.json")
    parser.add_argument("--out", default="report/figures")
    return parser.parse_args()


def _plot_seeds(path: Path, out: Path) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["runs"]
    values = [row["ate_m"] for row in rows]
    labels = [Path(row["run"]).name.replace("eval_main_imu", "seed0") for row in rows]
    labels = [f"seed{index}" for index in range(len(rows))]
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    ax.bar(labels, values, color="#3b6ea8")
    mean = data["aggregate"]["ate_m"]["mean"]
    ax.axhline(mean, color="#c46a3a", linestyle="--", linewidth=1.2, label=f"mean {mean:.4f}")
    for index, value in enumerate(values):
        ax.text(index, value + 0.005, f"{value:.3f}", ha="center", fontsize=8)
    ax.set_ylabel("ATE (m)")
    ax.set_title("TIO-Former simulation: 3 seeds")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out / "seeds_ate.png", dpi=160)
    plt.close(fig)


def _plot_loso(path: Path, out: Path) -> None:
    rows = json.loads(path.read_text(encoding="utf-8"))
    labels = [row["scene"] for row in rows]
    values = [row["metrics"]["ate_m"] for row in rows]
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    ax.bar(labels, values, color="#587d5a")
    for index, value in enumerate(values):
        ax.text(index, value + 0.005, f"{value:.3f}", ha="center", fontsize=8)
    ax.set_ylabel("ATE (m)")
    ax.set_title("Leave-one-scene-out")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out / "loso_ate.png", dpi=160)
    plt.close(fig)


def main() -> int:
    args = _args()
    out = (ROOT / args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    seeds = (ROOT / args.seeds).resolve()
    loso = (ROOT / args.loso).resolve()
    if seeds.exists():
        _plot_seeds(seeds, out)
    if loso.exists():
        _plot_loso(loso, out)
    print(f"wrote figures to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
