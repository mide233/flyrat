#!/usr/bin/env python3
"""Run the M4 robustness matrix for a trained checkpoint.

Each condition invokes ``evaluate.py`` with a perturbation flag.  By default a
subset of the test split is used (``--limit 40``) so the matrix stays cheap;
set ``--limit 0`` for the full split.
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
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--limit", type=int, default=40)
    parser.add_argument("--split", default="test")
    parser.add_argument("--out", default="runs/robustness")
    parser.add_argument("--perturb-seed", type=int, default=0)
    return parser.parse_args()


CONDITIONS = {
    "nominal": [],
    "missing_1_front": ["--drop-views", "front"],
    "missing_1_down": ["--drop-views", "down"],
    "missing_2_front_down": ["--drop-views", "front", "down"],
    "missing_3_front_left_right": ["--drop-views", "front", "left", "right"],
    "noise_0.02": ["--tof-noise-m", "0.02"],
    "noise_0.05": ["--tof-noise-m", "0.05"],
    "status_drop_10": ["--status-dropout", "0.10"],
    "status_drop_30": ["--status-dropout", "0.30"],
    "status_drop_50": ["--status-dropout", "0.50"],
}


def main() -> int:
    args = _args()
    env = os.environ.copy()
    env.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
    out_root = (ROOT / args.out).resolve()
    rows = []
    for name, extra in CONDITIONS.items():
        out_dir = out_root / name
        command = [
            PYTHON,
            "scripts/evaluate.py",
            "--checkpoint",
            str(Path(args.checkpoint).resolve()),
            "--split",
            args.split,
            "--out",
            str(out_dir),
            "--perturb-seed",
            str(args.perturb_seed),
            "--no-save-trajectories",
        ]
        if args.limit > 0:
            command += ["--limit", str(args.limit)]
        command += extra
        print("$", " ".join(command), flush=True)
        subprocess.run(command, cwd=ROOT, env=env, check=True)
        summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
        rows.append({"condition": name, "metrics": summary["metrics"], "sequences": summary["sequences"]})

    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    lines = [
        "# TIO-Former robustness",
        "",
        "| condition | sequences | ATE (m) | RPEp (m) | RPER (deg) | EDR (%) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        m = row["metrics"]
        lines.append(
            f"| {row['condition']} | {row['sequences']} | {m['ate_m']:.4f} | "
            f"{m['rpe_position_m']:.4f} | {m['rpe_rotation_deg']:.3f} | {m['end_drift_percent']:.2f} |"
        )
    (out_root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out_root / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
