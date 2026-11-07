#!/usr/bin/env python3
"""Generate canonical and random trajectory CSV files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.sim.trajectory import (  # noqa: E402
    canonical_trajectory,
    generate_sequence,
    random_trajectory,
)
from tio_former.sim.scene import scene_obstacles  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402
from tio_former.utils.seed import set_seed  # noqa: E402


def _generate_one(
    name: str,
    scene_name: str,
    scene: dict,
    traj_cfg: dict,
    rng,
    *,
    kind: str,
    shape: str | None,
    duration: float | None,
    count: int,
) -> list[dict]:
    records = []
    obstacles = scene_obstacles(scene_name, scene)
    for index in range(count):
        if kind == "demo":
            trajectory = canonical_trajectory(
                shape or "straight",
                scene["bounds"],
                sample_rate_hz=float(traj_cfg["trajectory"]["sample_rate_hz"]),
                speed_mps=float(traj_cfg["trajectory"]["speed_mps"]),
                altitude_m=float(traj_cfg["trajectory"]["altitude_m"]),
                clearance_m=float(traj_cfg["trajectory"]["min_clearance_m"]),
                radius_m=min(2.0, 0.8 * min(
                    scene["bounds"]["x"][1] - scene["bounds"]["x"][0],
                    scene["bounds"]["y"][1] - scene["bounds"]["y"][0],
                ) / 2.0),
                duration_s=duration or 12.0,
                obstacles=obstacles,
            )
            seq_name = name
        else:
            trajectory = generate_sequence(
                rng,
                scene["bounds"],
                traj_cfg,
                kind=kind,
                duration_s=duration,
                shape=shape,
                obstacles=obstacles,
            )
            seq_name = f"{name}_{index:03d}"
        path = Path(traj_cfg.get("_out", ROOT / "data/trajectories")) / f"{seq_name}.csv"
        trajectory.to_csv(path)
        record = {
            "name": seq_name,
            "scene": scene_name,
            "kind": kind,
            "shape": shape,
            "path": str(path),
            "duration_s": round(trajectory.duration, 3),
            "path_length_m": round(trajectory.path_length, 3),
            "samples": int(trajectory.t.size),
        }
        records.append(record)
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=str(ROOT / "configs/sim/scenes.yaml"))
    parser.add_argument("--trajectory", default=str(ROOT / "configs/sim/trajectory.yaml"))
    parser.add_argument("--out", default=str(ROOT / "data/trajectories"))
    parser.add_argument("--kind", choices=["demo", "canonical", "random"], default="demo")
    parser.add_argument("--scene", default="corridor")
    parser.add_argument("--shape", default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--name", default=None)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    logger = get_logger("generate_trajectories")
    rng = set_seed(args.seed)
    scene_config = load_yaml(args.scenes)
    traj_config = load_yaml(args.trajectory)
    traj_config["_out"] = args.out
    scenes = scene_config.get("scenes", {})
    if args.scene not in scenes:
        logger.error("unknown scene %r; choose from %s", args.scene, sorted(scenes))
        return 2
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = args.name or f"{args.scene}_demo"
    records = _generate_one(
        name,
        args.scene,
        scenes[args.scene],
        traj_config,
        rng,
        kind=args.kind,
        shape=args.shape,
        duration=args.duration,
        count=args.count,
    )
    manifest = out_dir / "manifest.json"
    existing: list[dict] = []
    if manifest.exists():
        existing = json.loads(manifest.read_text(encoding="utf-8"))
    existing.extend(records)
    manifest.write_text(json.dumps(existing, indent=2, sort_keys=True), encoding="utf-8")
    for record in records:
        logger.info(
            "wrote %s (%s, %.1f s, %.1f m)",
            record["path"],
            record["scene"],
            record["duration_s"],
            record["path_length_m"],
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
