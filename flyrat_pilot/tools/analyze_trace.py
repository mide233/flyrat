#!/usr/bin/env python3
"""Summarize the CSVs written by `flyrat_pilot --trace DIR`.

The pilot writes `trace_pilot.csv` with one row per tracked frame using
milliseconds. SVO writes `trace_frontend.csv` with the same frame order but in
seconds. Both files are read independently and aligned by row index.
"""
import csv
import sys
from pathlib import Path

PILOT_COLUMNS = [
    "queue_ms",
    "decode_ms",
    "imu_wait_ms",
    "collect_ms",
    "track_ms",
    "publish_ms",
    "age_ms",
]

# Timers registered by FrameHandlerBase. `reproject_kfs`, `reproject_candidates`
# and `feature_align` are nested inside the timers below, and
# `pyramid_creation` happens before `frontend_time` starts, so only the
# top-level stages are summed here.
FRONTEND_STAGES = [
    "pyramid_creation",
    "sparse_img_align",
    "reproject",
    "pose_optimizer",
    "point_optimizer",
    "local_ba",
]


def load(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def column(rows, name, scale=1.0):
    values = []
    for row in rows:
        try:
            values.append(float(row[name]) * scale)
        except (KeyError, TypeError, ValueError):
            continue
    return values


def report(name, values, unit="ms", indent="  "):
    if not values:
        return
    print(
        f"{indent}{name:<18} p50={percentile(values, 0.50):8.3f} "
        f"p95={percentile(values, 0.95):8.3f} "
        f"p99={percentile(values, 0.99):8.3f} "
        f"max={max(values):8.3f} {unit} n={len(values)}"
    )


def main():
    directory = Path("/tmp/flyrat_trace")
    skip = 0
    args = sys.argv[1:]
    index = 0
    while index < len(args):
        if args[index] == "--skip" and index + 1 < len(args):
            skip = int(args[index + 1])
            index += 2
        else:
            directory = Path(args[index])
            index += 1
    pilot_path = directory / "trace_pilot.csv"
    frontend_path = directory / "trace_frontend.csv"

    if not pilot_path.exists():
        print(f"missing {pilot_path}", file=sys.stderr)
        return 1

    pilot = load(pilot_path)
    print(f"pilot: {pilot_path} rows={len(pilot)}"
          + (f" (skipping first {skip})" if skip else ""))
    if skip:
        pilot = pilot[skip:]
    for name in PILOT_COLUMNS:
        report(name, column(pilot, name))

    if "valid" in pilot:
        valid = sum(1 for row in pilot if row.get("valid") == "1")
        print(f"  valid poses        {valid}/{len(pilot)}")

    if not frontend_path.exists():
        print(f"missing {frontend_path}", file=sys.stderr)
        return 0

    frontend = load(frontend_path)
    print(f"frontend: {frontend_path} rows={len(frontend)}")
    if skip:
        frontend = frontend[skip:]
    for name in FRONTEND_STAGES + ["frontend_time"]:
        report(name, column(frontend, name, 1000.0))

    # Everything the registered timers do not attribute to a stage. Useful for
    # spotting unmeasured work inside addFrameBundle().
    frontend_time = column(frontend, "frontend_time", 1000.0)
    stages = [column(frontend, name, 1000.0) for name in FRONTEND_STAGES[1:]]
    if frontend_time and all(stages):
        unaccounted = [
            total - sum(parts)
            for total, parts in zip(frontend_time, zip(*stages))
        ]
        report("unaccounted", unaccounted)

    # The pilot sees pyramid construction plus the traced frontend as one
    # track() call; show the split so the share of each block is obvious.
    pyramid = column(frontend, "pyramid_creation", 1000.0)
    if pyramid and frontend_time:
        track_model = [p + f for p, f in zip(pyramid, frontend_time)]
        report("pyramid+frontend", track_model)

    if frontend_time:
        means = {name: sum(column(frontend, name, 1000.0)) for name in FRONTEND_STAGES}
        total = sum(means.values())
        if total > 0.0:
            print("frontend stage share (of summed stages):")
            for name in FRONTEND_STAGES:
                print(f"  {name:<18} {100.0 * means[name] / total:5.1f}%"
                      f"  {means[name] / len(frontend):7.3f} ms/frame")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
