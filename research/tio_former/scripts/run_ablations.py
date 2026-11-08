#!/usr/bin/env python3
"""Train and evaluate the core TIO-Former ablation variants.

This is a lightweight orchestration helper, not a distributed trainer.  It runs
one process per variant/seed, which is sufficient for the single-GPU
reproduction.  Results are written under gitignored ``checkpoints/`` and
``runs/``.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable

VARIANTS = (
    "full",
    "imu_only",
    "tof_only",
    "naive_concat",
    "memoryless",
    "kv_only_no_fifo",
    "hybrid",
    "gated_fifo",
    "hybrid_gated",
    "hybrid_split",
)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS))
    parser.add_argument("--seeds", nargs="+", type=int, default=[0])
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--checkpoint-root", default="checkpoints/ablations")
    parser.add_argument("--eval-root", default="runs/ablations")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--eval-only", action="store_true")
    parser.add_argument(
        "--main-checkpoint",
        default="checkpoints/main_imu/seed0/best.pt",
        help="Use the main run as the full-model seed 0 entry",
    )
    parser.add_argument("--train-full", action="store_true", help="Retrain full instead of reusing main")
    return parser.parse_args()


def _run(command: list[str], env: dict[str, str]) -> None:
    print("$", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def main() -> int:
    args = _args()
    env = os.environ.copy()
    env.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
    checkpoint_root = (ROOT / args.checkpoint_root).resolve()
    eval_root = (ROOT / args.eval_root).resolve()

    results: list[dict] = []
    for variant in args.variants:
        if variant not in VARIANTS:
            raise ValueError(f"unknown variant {variant}")
        for seed in args.seeds:
            run_name = f"{variant}_s{seed}"
            run_dir = checkpoint_root / run_name
            best = run_dir / "best.pt"
            output_dir = eval_root / run_name
            main_checkpoint = (ROOT / args.main_checkpoint).resolve()
            use_main = (
                variant == "full"
                and seed == 0
                and not args.train_full
                and main_checkpoint.exists()
            )
            if use_main:
                best = main_checkpoint
                print(f"use main checkpoint for {run_name}: {best}")
            elif not args.eval_only:
                if args.skip_existing and best.exists():
                    print(f"skip training {run_name} (checkpoint exists)")
                else:
                    _run(
                        [
                            PYTHON,
                            "scripts/train.py",
                            "--variant",
                            variant,
                            "--seed",
                            str(seed),
                            "--epochs",
                            str(args.epochs),
                            "--workers",
                            str(args.workers),
                            "--batch-size",
                            str(args.batch_size),
                            "--out",
                            str(checkpoint_root),
                            "--run-name",
                            run_name,
                        ],
                        env,
                    )
            if not best.exists():
                print(f"warning: missing checkpoint {best}", file=sys.stderr)
                continue
            _run(
                [
                    PYTHON,
                    "scripts/evaluate.py",
                    "--checkpoint",
                    str(best),
                    "--split",
                    "test",
                    "--out",
                    str(output_dir),
                ],
                env,
            )
            summary_path = output_dir / "summary.json"
            if summary_path.exists():
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                results.append(
                    {
                        "variant": variant,
                        "seed": seed,
                        "metrics": summary["metrics"],
                        "sequences": summary["sequences"],
                    }
                )

    eval_root.mkdir(parents=True, exist_ok=True)
    aggregate = []
    for variant in args.variants:
        rows = [row for row in results if row["variant"] == variant]
        if not rows:
            continue
        entry = {"variant": variant, "seeds": [row["seed"] for row in rows], "sequences": rows[0]["sequences"]}
        for key in ("ate_m", "rpe_position_m", "rpe_rotation_deg", "end_drift_percent"):
            values = [row["metrics"][key] for row in rows]
            entry[f"{key}_mean"] = float(statistics.mean(values))
            entry[f"{key}_std"] = float(statistics.pstdev(values)) if len(values) > 1 else 0.0
        aggregate.append(entry)
    (eval_root / "summary.json").write_text(json.dumps(aggregate, indent=2), encoding="utf-8")

    lines = [
        "# TIO-Former ablations",
        "",
        "| variant | seeds | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for entry in aggregate:
        lines.append(
            f"| {entry['variant']} | {','.join(map(str, entry['seeds']))} | "
            f"{entry['ate_m_mean']:.4f} ± {entry['ate_m_std']:.4f} | "
            f"{entry['rpe_position_m_mean']:.4f} ± {entry['rpe_position_m_std']:.4f} | "
            f"{entry['rpe_rotation_deg_mean']:.3f} ± {entry['rpe_rotation_deg_std']:.3f} | "
            f"{entry['end_drift_percent_mean']:.2f} ± {entry['end_drift_percent_std']:.2f} |"
        )
    (eval_root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {eval_root / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
