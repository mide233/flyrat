#!/usr/bin/env python3
"""Audit a TIO-Former dataset before training.

Checks the properties that caused the v2 scale-collapse failure:

* common speed/step distribution across splits (KS distance);
* per-split speed-bin coverage;
* test path range;
* acceleration/jerk/yaw/vertical motion;
* ToF validity, status and dropout statistics;
* duplicate trajectories and scene/kind coverage.

Only NumPy and PyYAML are required.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.utils.config import load_yaml  # noqa: E402

VIEWS = ("front", "left", "right", "down")


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generation", default="configs/data/generation_v3.yaml")
    parser.add_argument("--processed", default="data_v3/processed")
    parser.add_argument("--out", default="data_v3/audit")
    parser.add_argument("--limit", type=int, default=0, help="Optional sequence limit for a quick check")
    return parser.parse_args()


def _quantile(values: np.ndarray, q: float) -> float:
    return float(np.quantile(values, q)) if values.size else float("nan")


def _ks_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Two-sample Kolmogorov-Smirnov distance without SciPy."""
    if a.size == 0 or b.size == 0:
        return float("nan")
    x = np.sort(np.concatenate([a, b]))
    cdf_a = np.searchsorted(np.sort(a), x, side="right") / a.size
    cdf_b = np.searchsorted(np.sort(b), x, side="right") / b.size
    return float(np.max(np.abs(cdf_a - cdf_b)))


def _yaw_rate(quat: np.ndarray, t: np.ndarray) -> float:
    w, x, y, z = quat.T
    yaw = np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    dt = np.diff(t)
    if dt.size == 0:
        return 0.0
    return float(np.degrees(np.abs(np.diff(yaw)) / np.maximum(dt, 1e-9)).mean())


def _collect(processed: Path, limit: int = 0) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    files = sorted(processed.glob("*.npz"))
    if limit:
        files = files[:limit]
    for path in files:
        meta_path = path.with_suffix(".json")
        if not meta_path.exists():
            continue
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        arrays = np.load(path)
        tof = arrays["tof"]
        weight = arrays["tof_weight"]
        status = arrays["tof_status"]
        ideal = arrays["tof_ideal"]
        poses = arrays["gt_pose"]
        t = arrays["t_tof"]
        position = poses[:, :3]
        dt = np.median(np.diff(t)) if len(t) > 1 else 1.0 / 15.0
        velocity = np.diff(position, axis=0) / max(dt, 1e-9)
        speed = np.linalg.norm(velocity, axis=1) if len(velocity) else np.array([0.0])
        acceleration = (
            np.linalg.norm(np.diff(position, n=2, axis=0), axis=1) / max(dt * dt, 1e-9)
            if len(position) >= 3
            else np.array([0.0])
        )
        jerk = (
            np.linalg.norm(np.diff(position, n=3, axis=0), axis=1) / max(dt**3, 1e-9)
            if len(position) >= 4
            else np.array([0.0])
        )
        durations = float(meta.get("duration_s", t[-1] - t[0] if len(t) else 0.0))
        path_length_m = float(meta.get("path_length_m", 0.0))
        frames = int(meta.get("frames", len(t)))
        view_valid = weight.reshape(len(weight), 4, -1).mean(axis=(0, 2))
        range_valid = (ideal > 0.015) & (ideal <= 4.0)
        dropped = (range_valid & (weight <= 0.0)).mean()
        status_counts = dict(Counter(int(v) for v in np.unique(status)))
        digest = hashlib.sha1(np.round(position, 2).tobytes()).hexdigest()
        records.append(
            {
                "seq_id": path.stem,
                "split": str(meta.get("split", "unknown")),
                "scene": str(meta.get("scene", "unknown")),
                "kind": str(meta.get("kind", "unknown")),
                "duration_s": durations,
                "path_length_m": path_length_m,
                "frames": frames,
                "speed_mps": path_length_m / max(durations, 1e-9),
                "step_cm": 100.0 * path_length_m / max(frames - 1, 1),
                "speed_median_mps": float(np.median(speed)),
                "speed_p95_mps": _quantile(speed, 0.95),
                "accel_mean_mps2": float(np.mean(acceleration)),
                "accel_p95_mps2": _quantile(acceleration, 0.95),
                "jerk_p95_mps3": _quantile(jerk, 0.95),
                "yaw_rate_deg_s": _yaw_rate(poses[:, 3:7], t),
                "z_range_m": float(np.ptp(position[:, 2])),
                "valid_ratio": float(weight.mean()),
                "view_valid": [float(v) for v in view_valid],
                "dropped_ratio": float(dropped),
                "status_counts": status_counts,
                "digest": digest,
            }
        )
    return records


def _group(records: list[dict[str, Any]], split: str) -> dict[str, Any]:
    selected = [r for r in records if r["split"] == split]
    if not selected:
        return {"sequences": 0}
    speeds = np.array([r["speed_mps"] for r in selected])
    steps = np.array([r["step_cm"] for r in selected])
    return {
        "sequences": len(selected),
        "seconds": float(sum(r["duration_s"] for r in selected)),
        "path_m": float(sum(r["path_length_m"] for r in selected)),
        "speed_mean": float(speeds.mean()),
        "speed_median": float(np.median(speeds)),
        "speed_p05": _quantile(speeds, 0.05),
        "speed_p95": _quantile(speeds, 0.95),
        "step_mean_cm": float(steps.mean()),
        "step_median_cm": float(np.median(steps)),
        "accel_mean": float(np.mean([r["accel_mean_mps2"] for r in selected])),
        "accel_p95": float(np.mean([r["accel_p95_mps2"] for r in selected])),
        "jerk_p95": float(np.mean([r["jerk_p95_mps3"] for r in selected])),
        "yaw_rate_mean": float(np.mean([r["yaw_rate_deg_s"] for r in selected])),
        "z_range_mean": float(np.mean([r["z_range_m"] for r in selected])),
        "valid_ratio": float(np.mean([r["valid_ratio"] for r in selected])),
        "view_valid": [
            float(np.mean([r["view_valid"][index] for r in selected])) for index in range(4)
        ],
        "dropped_ratio": float(np.mean([r["dropped_ratio"] for r in selected])),
    }


def _bins(records: list[dict[str, Any]], bins: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for entry in bins:
        low, high = float(entry["min"]), float(entry["max"])
        row = {"label": f"{low:.2f}-{high:.2f}", "total": 0}
        for split in ("train", "val", "test"):
            count = sum(
                1 for r in records if r["split"] == split and low <= r["speed_mps"] < high
            )
            row[split] = count
            row["total"] += count
        result.append(row)
    return result


def _status_summary(records: list[dict[str, Any]]) -> dict[str, dict[str, int]]:
    result: dict[str, dict[str, int]] = {}
    for split in ("train", "val", "test"):
        total: Counter[int] = Counter()
        for record in records:
            if record["split"] == split:
                total.update(record["status_counts"])
        result[split] = dict(sorted(total.items()))
    return result


def _duplicates(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[str, list[str]] = {}
    for record in records:
        groups.setdefault(record["digest"], []).append(record["seq_id"])
    return [
        {"count": len(ids), "sequences": ids}
        for ids in groups.values()
        if len(ids) > 1
    ]


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def _write_report(
    records: list[dict[str, Any]],
    config: dict[str, Any],
    out: Path,
) -> dict[str, Any]:
    by_split = {split: _group(records, split) for split in ("train", "val", "test")}
    bins = _bins(records, config["generation"].get("speed_bins", []))
    duplicates = _duplicates(records)
    status = _status_summary(records)

    speeds = {split: np.array([r["speed_mps"] for r in records if r["split"] == split]) for split in by_split}
    ks = {
        "train_val": _ks_distance(speeds["train"], speeds["val"]),
        "train_test": _ks_distance(speeds["train"], speeds["test"]),
        "val_test": _ks_distance(speeds["val"], speeds["test"]),
    }
    split_means = [by_split[s]["speed_mean"] for s in ("train", "val", "test") if by_split[s]["sequences"]]
    mean_shift = (
        (max(split_means) - min(split_means)) / max(float(np.mean(split_means)), 1e-9)
        if split_means
        else float("nan")
    )
    test_paths = [r["path_length_m"] for r in records if r["split"] == "test"]
    path_ok = bool(test_paths) and all(4.0 <= value <= 7.0 for value in test_paths)
    min_bin = min((row["total"] for row in bins), default=0)
    valid = float(np.mean([r["valid_ratio"] for r in records])) if records else float("nan")
    checks = {
        "common_speed_mean_shift_lt_5pct": bool(mean_shift == mean_shift and mean_shift < 0.05),
        "speed_ks_lt_0_1": bool(max(ks.values()) < 0.1),
        "min_speed_bin_count_ge_50": bool(min_bin >= 50),
        "test_path_4_7m": path_ok,
        "valid_ratio_70_95pct": bool(0.70 <= valid <= 0.95),
        "no_duplicate_trajectories": not duplicates,
    }

    split_rows = [
        [
            split,
            entry["sequences"],
            f"{entry['seconds']:.1f}",
            f"{entry['path_m']:.1f}",
            f"{entry['speed_mean']:.3f}",
            f"{entry['speed_p05']:.3f}-{entry['speed_p95']:.3f}",
            f"{entry['step_mean_cm']:.2f}",
            f"{entry['accel_mean']:.2f}",
            f"{entry['jerk_p95']:.2f}",
            f"{entry['yaw_rate_mean']:.1f}",
            f"{entry['z_range_mean']:.2f}",
            f"{entry['valid_ratio']:.3f}",
        ]
        for split, entry in by_split.items()
        if entry["sequences"]
    ]
    bin_rows = [[row["label"], row["train"], row["val"], row["test"], row["total"]] for row in bins]
    scene_rows = []
    for scene in sorted({r["scene"] for r in records}):
        for split in ("train", "val", "test"):
            selected = [r for r in records if r["scene"] == scene and r["split"] == split]
            if selected:
                scene_rows.append(
                    [
                        scene,
                        split,
                        len(selected),
                        f"{np.mean([r['speed_mps'] for r in selected]):.3f}",
                        f"{np.mean([r['valid_ratio'] for r in selected]):.3f}",
                    ]
                )

    text = f"""# TIO-Former v3 dataset audit

Dataset: `{config['generation']['processed_dir']}`  
Sequences: **{len(records)}**  
Total duration: **{sum(r['duration_s'] for r in records):.1f} s**  
Total path: **{sum(r['path_length_m'] for r in records):.1f} m**  

## 1. Acceptance checks

{_table(['check', 'result'], [[key, 'PASS' if value else 'FAIL'] for key, value in checks.items()])}

- Split mean-speed spread: **{100 * mean_shift:.2f}%**
- Max pairwise KS distance: **{max(ks.values()):.4f}**
- Minimum speed-bin count: **{min_bin}**
- Overall valid ToF ratio: **{valid:.3f}**

## 2. Split summary

{_table(['split', 'n', 'seconds', 'path m', 'mean speed', 'p05-p95', 'step cm', 'accel m/s2', 'jerk p95', 'yaw deg/s', 'z range m', 'valid'], split_rows)}

> v2 had train/test mean speeds of 0.174/0.412 m/s. The target is a <5%
> split mean difference and KS < 0.1.

## 3. Speed-bin coverage

{_table(['bin m/s', 'train', 'val', 'test', 'total'], bin_rows)}

## 4. ToF statistics

- Status counts by split: `{json.dumps(status, sort_keys=True)}`
- Dropped-return ratio (valid ideal but zero weight): per split
  train **{by_split['train'].get('dropped_ratio', float('nan')):.4f}**,
  val **{by_split['val'].get('dropped_ratio', float('nan')):.4f}**,
  test **{by_split['test'].get('dropped_ratio', float('nan')):.4f}**.
- Per-view valid ratios:

{_table(['split', 'front', 'left', 'right', 'down'], [[split, *[f'{v:.3f}' for v in entry['view_valid']]] for split, entry in by_split.items() if entry['sequences']])}

## 5. Scene coverage

{_table(['scene', 'split', 'n', 'mean speed', 'valid'], scene_rows)}

## 6. Duplicates

- Exact/near-duplicate groups (positions rounded to 1 cm): **{len(duplicates)}**
{''.join(f"- {item['count']}x: {', '.join(item['sequences'])}\n" for item in duplicates)}

## 7. Notes

- Duplicate detection rounds positions to 1 cm and hashes; it is conservative
  for detecting accidental repeats, not a perceptual trajectory similarity test.
- `status 10` should be absent (or negligible) in v3 because the real VL53L5CX
  only reports 5/9.
- These checks are generation diagnostics, not model accuracy metrics.
"""
    (out / "audit.md").write_text(text, encoding="utf-8")
    payload = {
        "sequences": len(records),
        "seconds": float(sum(r["duration_s"] for r in records)),
        "path_m": float(sum(r["path_length_m"] for r in records)),
        "by_split": by_split,
        "speed_bins": bins,
        "ks": ks,
        "mean_speed_shift": mean_shift,
        "checks": checks,
        "status": status,
        "duplicates": duplicates,
    }
    (out / "audit.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def main() -> int:
    args = _args()
    config = load_yaml(ROOT / args.generation)
    processed = (ROOT / args.processed).resolve()
    out = (ROOT / args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    records = _collect(processed, limit=args.limit)
    if not records:
        raise SystemExit(f"no processed sequences in {processed}")
    payload = _write_report(records, config, out)
    print(f"wrote {out}")
    print(json.dumps(payload["checks"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
