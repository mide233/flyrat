#!/usr/bin/env python3
"""Build paper-style tables and figures from evaluation outputs."""

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
    parser.add_argument("--main", default="runs/eval_main/summary.json")
    parser.add_argument("--ablations", default="runs/ablations/summary.json")
    parser.add_argument("--train-metrics", default="checkpoints/main/seed0/metrics.jsonl")
    parser.add_argument("--robustness", default="runs/robustness/summary.json")
    parser.add_argument("--out", default="report")
    return parser.parse_args()


def _read_json(path: Path):
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _main_table(summary: dict) -> str:
    metrics = summary["metrics"]
    lines = [
        "### 主结果（test split）",
        "",
        "| 指标 | 值 |",
        "|---|---:|",
        f"| 序列数 | {summary['sequences']} |",
        f"| ATE position (m) | {metrics['ate_m']:.4f} |",
        f"| RPE position (m) | {metrics['rpe_position_m']:.4f} |",
        f"| RPE rotation (deg) | {metrics['rpe_rotation_deg']:.3f} |",
        f"| End drift (%) | {metrics['end_drift_percent']:.2f} |",
        "",
        "| 场景 | 序列 | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    by_scene = summary.get("by_scene", [])
    if isinstance(by_scene, dict):
        by_scene = [{"scene": key, **value} for key, value in by_scene.items()]
    for row in by_scene:
        values = row.get("metrics", row)
        scene = row.get("scene", "?")
        lines.append(
            f"| {scene} | {row.get('sequences', '-')} | "
            f"{values['ate_m']:.4f} | {values['rpe_position_m']:.4f} | "
            f"{values['rpe_rotation_deg']:.3f} | {values['end_drift_percent']:.2f} |"
        )
    return "\n".join(lines)


def _ablation_table(summary: list[dict]) -> str:
    lines = [
        "### 核心消融",
        "",
        "| 变体 | seeds | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for entry in summary:
        lines.append(
            f"| {entry['variant']} | {','.join(map(str, entry['seeds']))} | "
            f"{entry['ate_m_mean']:.4f} ± {entry['ate_m_std']:.4f} | "
            f"{entry['rpe_position_m_mean']:.4f} ± {entry['rpe_position_m_std']:.4f} | "
            f"{entry['rpe_rotation_deg_mean']:.3f} ± {entry['rpe_rotation_deg_std']:.3f} | "
            f"{entry['end_drift_percent_mean']:.2f} ± {entry['end_drift_percent_std']:.2f} |"
        )
    return "\n".join(lines)


def _plot_ablations(summary: list[dict], out: Path) -> None:
    names = [entry["variant"] for entry in summary]
    values = [entry["ate_m_mean"] for entry in summary]
    errors = [entry["ate_m_std"] for entry in summary]
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    ax.bar(range(len(names)), values, yerr=errors, color="#3b6ea8", capsize=3)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=20, ha="right")
    ax.set_ylabel("ATE (m)")
    ax.set_title("TIO-Former simulation ablations")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out / "ablations_ate.png", dpi=160)
    plt.close(fig)


def _plot_training(metrics_path: Path, out: Path) -> None:
    if not metrics_path.exists():
        return
    rows = [json.loads(line) for line in metrics_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        return
    epochs = [row["epoch"] for row in rows if "train_loss" in row]
    train = [row["train_loss"] for row in rows if "train_loss" in row]
    val_rows = [row for row in rows if "val_loss" in row]
    val_epochs = [row["epoch"] for row in val_rows]
    val = [row["val_loss"] for row in val_rows]
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    ax.plot(epochs, train, label="train", color="#3b6ea8")
    ax.plot(val_epochs, val, label="val", color="#c46a3a")
    ax.set_yscale("log")
    ax.set_xlabel("epoch")
    ax.set_ylabel("loss")
    ax.set_title("Training curve")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "training_curve.png", dpi=160)
    plt.close(fig)


def _robust_row(entry: dict) -> tuple[str, dict, object]:
    name = entry.get("condition", entry.get("name", "?"))
    metrics = entry.get("metrics", entry)
    return name, metrics, entry.get("sequences", "-")


def _plot_robustness(summary: list[dict], out: Path) -> None:
    if not summary:
        return
    names = [_robust_row(entry)[0] for entry in summary]
    values = [_robust_row(entry)[1]["ate_m"] for entry in summary]
    fig, ax = plt.subplots(figsize=(8.0, 3.6))
    ax.bar(range(len(names)), values, color="#587d5a")
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=25, ha="right")
    ax.set_ylabel("ATE (m)")
    ax.set_title("Robustness to degraded ToF input")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out / "robustness_ate.png", dpi=160)
    plt.close(fig)


def main() -> int:
    args = _args()
    out = (ROOT / args.out).resolve()
    figures = out / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    sections = ["# TIO-Former M4 结果", "", "本文件由 `scripts/make_report.py` 自动生成。", ""]
    main_summary = _read_json((ROOT / args.main).resolve())
    if main_summary:
        sections.append(_main_table(main_summary))
        sections.append("")
    ablation_summary = _read_json((ROOT / args.ablations).resolve())
    if ablation_summary:
        sections.append(_ablation_table(ablation_summary))
        sections.append("")
        _plot_ablations(ablation_summary, figures)
    robustness = _read_json((ROOT / args.robustness).resolve())
    if robustness:
        lines = [
            "### 鲁棒性",
            "",
            "| 条件 | 序列 | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
            "|---|---:|---:|---:|---:|---:|",
        ]
        for entry in robustness:
            name, metrics, sequences = _robust_row(entry)
            lines.append(
                f"| {name} | {sequences} | {metrics['ate_m']:.4f} | "
                f"{metrics['rpe_position_m']:.4f} | {metrics['rpe_rotation_deg']:.3f} | "
                f"{metrics['end_drift_percent']:.2f} |"
            )
        sections.append("\n".join(lines))
        sections.append("")
        _plot_robustness(robustness, figures)
    _plot_training((ROOT / args.train_metrics).resolve(), figures)

    (out / "m4_results.md").write_text("\n".join(sections) + "\n", encoding="utf-8")
    print(f"wrote {out / 'm4_results.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
