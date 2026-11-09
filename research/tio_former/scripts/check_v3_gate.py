#!/usr/bin/env python3
"""Evaluate the v3.1 seed-0 gate from an evaluation directory.

The gate is deliberately simple: it reads metrics.csv and the saved
trajectories, computes the predicted/GT path-length ratio and the
speed--ATE correlation, then prints PASS/FAIL against the v3.1 targets.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TARGETS = {
    "ate_m": 0.34,
    "path_ratio_min": 0.80,
    "path_ratio_max": 1.20,
    "speed_ate_correlation": 0.50,
}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval", default="runs_v3/eval_seed0")
    parser.add_argument("--out", default="runs_v3/gate_seed0.json")
    return parser.parse_args()


def _path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def main() -> int:
    args = _args()
    eval_dir = (ROOT / args.eval).resolve()
    summary = json.loads((eval_dir / "summary.json").read_text(encoding="utf-8"))
    rows: list[dict[str, float]] = []
    with (eval_dir / "metrics.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            trajectory = np.load(eval_dir / "trajectories" / f"{row['seq_id']}.npz")
            predicted = trajectory["predicted"]
            ground_truth = trajectory["ground_truth"]
            gt_path = _path_length(ground_truth.astype(np.float64))
            pred_path = _path_length(predicted.astype(np.float64))
            duration = float(row["duration_s"])
            rows.append(
                {
                    "path_ratio": pred_path / max(gt_path, 1e-9),
                    "speed_mps": float(row["path_length_m"]) / max(duration, 1e-9),
                    "ate_m": float(row["ate_m"]),
                    "end_drift_percent": float(row["end_drift_percent"]),
                }
            )

    ate = float(summary["metrics"]["ate_m"])
    ratio = float(np.mean([row["path_ratio"] for row in rows]))
    speed = np.array([row["speed_mps"] for row in rows])
    speed_ate_corr = float(np.corrcoef(speed, [row["ate_m"] for row in rows])[0, 1])
    speed_ate_abs = abs(speed_ate_corr)
    result = {
        "eval": str(eval_dir),
        "sequences": len(rows),
        "ate_m": ate,
        "path_ratio": ratio,
        "speed_ate_correlation": speed_ate_abs,
        "speed_ate_correlation_signed": speed_ate_corr,
        "end_drift_percent": float(summary["metrics"]["end_drift_percent"]),
        "targets": TARGETS,
        "pass": {
            "ate": ate <= TARGETS["ate_m"],
            "path_ratio": TARGETS["path_ratio_min"] <= ratio <= TARGETS["path_ratio_max"],
            "speed_ate_correlation": speed_ate_abs <= TARGETS["speed_ate_correlation"],
        },
    }
    result["pass"]["all"] = all(result["pass"].values())
    out = (ROOT / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(
        f"sequences={len(rows)} ATE={ate:.4f} m path_ratio={ratio:.3f} "
        f"speed-ATE |r|={speed_ate_abs:.3f} (signed {speed_ate_corr:+.3f})"
    )
    for name, passed in result["pass"].items():
        print(f"  {name}: {'PASS' if passed else 'FAIL'}")
    print(f"wrote {out}")
    return 0 if result["pass"]["all"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
