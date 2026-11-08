#!/usr/bin/env python3
"""Aggregate multiple evaluation summary.json files into mean/std."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
METRICS = ("ate_m", "rpe_position_m", "rpe_rotation_deg", "end_drift_percent")


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", required=True)
    parser.add_argument("--out", default="runs/seeds_summary.json")
    return parser.parse_args()


def main() -> int:
    args = _args()
    rows = []
    for run in args.runs:
        path = (ROOT / run / "summary.json").resolve()
        summary = json.loads(path.read_text(encoding="utf-8"))
        row = {"run": str(run), "sequences": summary["sequences"]}
        row.update({key: float(summary["metrics"][key]) for key in METRICS})
        rows.append(row)
    aggregate = {}
    for key in METRICS:
        values = [row[key] for row in rows]
        aggregate[key] = {
            "mean": statistics.fmean(values),
            "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "min": min(values),
            "max": max(values),
        }
    result = {"runs": rows, "aggregate": aggregate}
    out = (ROOT / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
