#!/usr/bin/env python3
"""Build the dataset index, splits, statistics and a short report."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import leave_one_scene_out, split_records  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402

FIELDS = [
    "seq_id",
    "scene",
    "split",
    "kind",
    "shape",
    "duration_s",
    "path_length_m",
    "frames",
    "valid_ratio",
    "range_min_m",
    "range_max_m",
    "file",
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=str(ROOT / "configs/data/dataset.yaml"))
    parser.add_argument("--generation", default=str(ROOT / "configs/data/generation.yaml"))
    parser.add_argument("--processed", default=None)
    parser.add_argument("--out", default=None)
    return parser.parse_args()


def _record(path: Path) -> dict[str, Any]:
    meta_path = path.with_suffix(".json")
    metadata = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
    with np.load(path, allow_pickle=False) as data:
        frames = int(data["tof"].shape[0])
        status = data["tof_status"]
        valid_ratio = float((status > 0).mean()) if status.size else 0.0
        ranges = data["tof"][data["tof_weight"] > 0.0]
        range_min = float(ranges.min()) if ranges.size else 0.0
        range_max = float(ranges.max()) if ranges.size else 0.0
    return {
        "seq_id": metadata.get("seq_id", path.stem),
        "scene": metadata.get("scene", path.parent.name),
        "split": metadata.get("split", "unknown"),
        "kind": metadata.get("kind", "unknown"),
        "shape": metadata.get("shape", ""),
        "duration_s": float(metadata.get("duration_s", 0.0)),
        "path_length_m": float(metadata.get("path_length_m", 0.0)),
        "frames": frames,
        "valid_ratio": valid_ratio,
        "range_min_m": range_min,
        "range_max_m": range_max,
        "file": str(path.relative_to(ROOT)),
    }


def _group(records: list[dict[str, Any]], key: str) -> dict[str, dict[str, float]]:
    groups: dict[str, dict[str, float]] = {}
    for record in records:
        name = str(record.get(key, "unknown"))
        entry = groups.setdefault(
            name,
            {"sequences": 0.0, "seconds": 0.0, "path_m": 0.0, "frames": 0.0, "valid_ratio": 0.0},
        )
        entry["sequences"] += 1
        entry["seconds"] += float(record["duration_s"])
        entry["path_m"] += float(record["path_length_m"])
        entry["frames"] += float(record["frames"])
        entry["valid_ratio"] += float(record["valid_ratio"])
    for entry in groups.values():
        entry["valid_ratio"] /= max(entry["sequences"], 1.0)
    return groups


def main() -> int:
    args = _parse_args()
    logger = get_logger("build_dataset_index")
    dataset = load_yaml(args.dataset)["dataset"]
    generation = load_yaml(args.generation)["generation"]
    processed = Path(args.processed or (ROOT / dataset["processed_dir"]))
    out_path = Path(args.out or (ROOT / generation.get("index_path", "data/index.csv")))
    files = sorted(processed.rglob("*.npz"))
    if not files:
        logger.error("no processed sequences under %s", processed)
        return 2
    records = [_record(path) for path in files]
    records.sort(key=lambda record: record["seq_id"])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(records)

    split_cfg = load_yaml(args.dataset).get("split", {})
    random_split = split_records(
        records,
        seed=int(split_cfg.get("seed", 0)),
        train=float(split_cfg.get("train", 0.70)),
        val=float(split_cfg.get("val", 0.15)),
        test=float(split_cfg.get("test", 0.15)),
    )
    split_payload = {
        "assigned": {
            name: [record["seq_id"] for record in records if record["split"] == name]
            for name in sorted({record["split"] for record in records})
        },
        "random": {
            name: [record["seq_id"] for record in group]
            for name, group in random_split.items()
        },
        "leave_one_scene_out": {
            scene: {name: [record["seq_id"] for record in group] for name, group in folds.items()}
            for scene, folds in leave_one_scene_out(records).items()
        },
    }
    (out_path.parent / "splits.json").write_text(
        json.dumps(split_payload, indent=2, sort_keys=True), encoding="utf-8"
    )

    stats = {
        "sequences": len(records),
        "seconds": float(sum(record["duration_s"] for record in records)),
        "path_m": float(sum(record["path_length_m"] for record in records)),
        "frames": int(sum(record["frames"] for record in records)),
        "valid_ratio": float(np.mean([record["valid_ratio"] for record in records])),
        "by_split": _group(records, "split"),
        "by_scene": _group(records, "scene"),
        "by_scene_split": _group(
            [{**record, "scene_split": f"{record['scene']}/{record['split']}"} for record in records],
            "scene_split",
        ),
    }
    (out_path.parent / "stats.json").write_text(
        json.dumps(stats, indent=2, sort_keys=True), encoding="utf-8"
    )
    _write_report(out_path.parent / "report.md", records, stats)
    logger.info(
        "indexed %d sequences, %.1f s, %.1f m, %d frames",
        stats["sequences"],
        stats["seconds"],
        stats["path_m"],
        stats["frames"],
    )
    return 0


def _write_report(path: Path, records: list[dict[str, Any]], stats: dict[str, Any]) -> None:
    lines = [
        "# TIO-Former M2 dataset",
        "",
        f"- Sequences: {stats['sequences']}",
        f"- Duration: {stats['seconds']:.1f} s",
        f"- Path length: {stats['path_m']:.1f} m",
        f"- Frames: {stats['frames']}",
        f"- Mean valid-zone ratio: {stats['valid_ratio']:.3f}",
        "",
        "## By split",
        "",
        "| split | sequences | seconds | path m | frames | valid |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in sorted(stats["by_split"].items()):
        lines.append(
            f"| {name} | {int(value['sequences'])} | {value['seconds']:.1f} | "
            f"{value['path_m']:.1f} | {int(value['frames'])} | {value['valid_ratio']:.3f} |"
        )
    lines += [
        "",
        "## By scene",
        "",
        "| scene | sequences | seconds | path m | frames | valid |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in sorted(stats["by_scene"].items()):
        lines.append(
            f"| {name} | {int(value['sequences'])} | {value['seconds']:.1f} | "
            f"{value['path_m']:.1f} | {int(value['frames'])} | {value['valid_ratio']:.3f} |"
        )
    lines += ["", "## Trajectory kinds", "", "| kind | sequences |", "|---|---:|"]
    kinds = _group(records, "kind")
    for name, value in sorted(kinds.items()):
        lines.append(f"| {name} | {int(value['sequences'])} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
