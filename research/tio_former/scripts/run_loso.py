#!/usr/bin/env python3
"""Leave-one-scene-out evaluation for TIO-Former.

For each scene, train on the train split excluding that scene, validate on the
val split excluding it, then evaluate on the held-out scene's test sequences.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default="corridor,office,pantry,meeting_room")
    parser.add_argument("--epochs", type=int, default=15)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--checkpoint-root", default="checkpoints/loso")
    parser.add_argument("--eval-root", default="runs/loso")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--eval-only", action="store_true")
    return parser.parse_args()


def _run(command: list[str], env: dict[str, str]) -> None:
    print("$", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def main() -> int:
    args = _args()
    env = os.environ.copy()
    env.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
    scenes = [item for item in args.scenes.split(",") if item]
    checkpoint_root = ROOT / args.checkpoint_root
    eval_root = ROOT / args.eval_root
    rows = []
    for scene in scenes:
        run_name = f"without_{scene}"
        best = checkpoint_root / run_name / "best.pt"
        out_dir = eval_root / scene
        if not args.eval_only and not (args.skip_existing and best.exists()):
            _run(
                [
                    PYTHON,
                    "scripts/train.py",
                    "--exclude-scenes",
                    scene,
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
            print(f"skip {scene}: missing {best}")
            continue
        _run(
            [
                PYTHON,
                "scripts/evaluate.py",
                "--checkpoint",
                str(best),
                "--split",
                "test",
                "--scenes",
                scene,
                "--out",
                str(out_dir),
                "--no-save-trajectories",
            ],
            env,
        )
        summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
        rows.append(
            {
                "scene": scene,
                "sequences": summary["sequences"],
                "metrics": summary["metrics"],
            }
        )

    eval_root.mkdir(parents=True, exist_ok=True)
    (eval_root / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    lines = [
        "# TIO-Former leave-one-scene-out",
        "",
        "| held-out scene | sequences | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        m = row["metrics"]
        lines.append(
            f"| {row['scene']} | {row['sequences']} | {m['ate_m']:.4f} | "
            f"{m['rpe_position_m']:.4f} | {m['rpe_rotation_deg']:.3f} | "
            f"{m['end_drift_percent']:.2f} |"
        )
    (eval_root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {eval_root / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
