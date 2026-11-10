#!/usr/bin/env python3
"""Plan and generate the full M2 TIO-Former simulation dataset.

The script is intentionally restartable:

* trajectories are generated first and written under
  ``data/trajectories/<split>/<scene>/``;
* each sequence is recorded with a unique Gazebo partition and converted to
  the processed NPZ format immediately;
* raw depth recordings are deleted after post-processing unless
  ``generation.keep_raw`` is true;
* existing processed sequences are skipped on the next run.

Use ``--dry-run`` to inspect the plan without launching Gazebo.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.sim.random_scene import (  # noqa: E402
    add_path_clutter,
    load_texture_pool,
    sample_scene,
    scene_to_sdf,
)
from tio_former.sim.scene import scene_obstacles  # noqa: E402
from tio_former.sim.trajectory import (  # noqa: E402
    generate_sequence,
    inspection_trajectory,
    retime_to_speed,
)
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402

logger = get_logger("generate_dataset")


@dataclass
class SequenceSpec:
    seq_id: str
    split: str
    scene: str
    kind: str
    seed: int
    duration_s: float
    target_speed_mps: float = 0.0
    speed_bin: str = ""
    csv: str = ""
    path_length_m: float = 0.0
    shape: str | None = None
    world: str = ""
    scene_spec: str = ""


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generation", default=str(ROOT / "configs/data/generation.yaml"))
    parser.add_argument("--scenes", default=str(ROOT / "configs/sim/scenes.yaml"))
    parser.add_argument("--trajectory", default=str(ROOT / "configs/sim/trajectory.yaml"))
    parser.add_argument("--sensors", default=str(ROOT / "configs/sim/sensors_v3.yaml"))
    parser.add_argument("--target-seconds", type=float, default=None)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--split", choices=["train", "val", "test"], default=None)
    parser.add_argument("--scene", default=None)
    parser.add_argument("--smoke", action="store_true", help="Short four-scene smoke run")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--keep-raw", action="store_true")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--plan-only", action="store_true", help="Write trajectories but do not launch Gazebo")
    return parser.parse_args()


def _stable_seed(base: int, *parts: object) -> int:
    payload = "|".join([str(base), *(str(part) for part in parts)]).encode("utf-8")
    return int(zlib.crc32(payload))


def _duration_splits(config: dict[str, Any], total_seconds: float) -> dict[str, float]:
    fractions = config["generation"].get("split_fractions", {"train": 0.7, "val": 0.15, "test": 0.15})
    return {name: total_seconds * float(fraction) for name, fraction in fractions.items()}


def _scene_seconds(config: dict[str, Any], scenes: list[str], split_seconds: float) -> dict[str, float]:
    weights = config["generation"].get("scene_weights", {})
    raw = {scene: float(weights.get(scene, 1.0)) for scene in scenes}
    total = sum(raw.values())
    return {scene: split_seconds * value / total for scene, value in raw.items()}


def _speed_bins(generation: dict[str, Any]) -> list[dict[str, float]]:
    raw = generation.get("speed_bins", [])
    if not raw:
        return []
    normalised: list[dict[str, float]] = []
    for entry in raw:
        low = float(entry["min"])
        high = float(entry["max"])
        weight = float(entry.get("weight", 1.0))
        if not (0.0 < low < high):
            raise ValueError(f"invalid speed bin {entry!r}")
        if weight <= 0.0:
            continue
        normalised.append({"min": low, "max": high, "weight": weight})
    total = sum(entry["weight"] for entry in normalised)
    if total <= 0.0:
        raise ValueError("speed_bins weights must sum to a positive value")
    for entry in normalised:
        entry["weight"] /= total
    return normalised


def _sample_speed(rng: np.random.Generator, bins: list[dict[str, float]]) -> tuple[float, str]:
    draw = float(rng.random())
    cumulative = 0.0
    for entry in bins:
        cumulative += entry["weight"]
        if draw <= cumulative:
            speed = float(rng.uniform(entry["min"], entry["max"]))
            return speed, f"{entry['min']:.2f}-{entry['max']:.2f}"
    entry = bins[-1]
    return float(rng.uniform(entry["min"], entry["max"])), f"{entry['min']:.2f}-{entry['max']:.2f}"


def _plan_specs(
    config: dict[str, Any],
    scene_config: dict[str, Any],
    trajectory_config: dict[str, Any],
    *,
    total_seconds: float,
    split_filter: str | None,
    scene_filter: str | None,
    smoke: bool,
) -> list[SequenceSpec]:
    generation = config["generation"]
    scenes = scene_config["scenes"]
    scene_names = sorted(scenes)
    if scene_filter:
        if scene_filter not in scenes:
            raise ValueError(f"unknown scene {scene_filter!r}")
        scene_names = [scene_filter]
    base_seed = int(generation.get("seed", 0))
    durations = generation["duration_s"]
    speed_bins = _speed_bins(generation)
    nominal_paths = generation.get("nominal_path_m", {})
    canonical_ratio = float(generation.get("canonical_ratio", 0.2))
    attempts = int(generation.get("attempts_per_sequence", 80))
    test_path_range = [float(v) for v in generation.get("test_path_m", [4.0, 7.0])]

    split_seconds = _duration_splits(config, total_seconds)
    specs: list[SequenceSpec] = []
    for split, seconds in split_seconds.items():
        if split_filter and split != split_filter:
            continue
        for scene in scene_names:
            scene_budget = _scene_seconds(config, scene_names, seconds)[scene]
            if smoke:
                scene_budget = 8.0
            rng = np.random.default_rng(_stable_seed(base_seed, "plan", split, scene))
            elapsed = 0.0
            index = 0
            while elapsed < scene_budget:
                duration_range = durations[split]
                if speed_bins:
                    target_speed, speed_bin = _sample_speed(rng, speed_bins)
                    nominal_path = float(nominal_paths.get(split, 6.0))
                    duration = nominal_path / target_speed
                else:
                    target_speed, speed_bin = 0.0, ""
                    duration = float(rng.uniform(*duration_range))
                if smoke:
                    duration = 8.0
                kind = "canonical" if rng.random() < canonical_ratio else "random"
                seed = _stable_seed(base_seed, split, scene, index)
                spec = SequenceSpec(
                    seq_id=f"{split}_{scene}_{index:04d}",
                    split=split,
                    scene=scene,
                    kind=kind,
                    seed=seed,
                    duration_s=duration,
                    target_speed_mps=target_speed,
                    speed_bin=speed_bin,
                )
                specs.append(spec)
                elapsed += duration
                index += 1
    if smoke:
        # One sequence per split/scene is enough for a smoke test.
        unique: dict[tuple[str, str], SequenceSpec] = {}
        for spec in specs:
            unique.setdefault((spec.split, spec.scene), spec)
        specs = list(unique.values())
    return specs


def _make_trajectory(
    spec: SequenceSpec,
    scene_config: dict[str, Any],
    trajectory_config: dict[str, Any],
    *,
    attempts: int,
    test_path_range: list[float],
    test_waypoint_count: list[int],
    test_waypoint_step_m: list[float],
    trajectory_root: Path,
    raw_duration_s: float,
    duration_range: list[float],
    altitude_range: list[float] | None,
    vertical_jitter_m: float,
    max_acceleration_mps2: float,
    sample_rate_hz: float,
) -> SequenceSpec:
    scene = scene_config["scenes"][spec.scene]
    trajectory_dir = trajectory_root / spec.split / spec.scene
    if str(scene.get("style", "")).lower() == "random":
        return _make_random_scene_trajectory(
            spec,
            scene,
            trajectory_config,
            attempts=attempts,
            trajectory_root=trajectory_root,
            raw_duration_s=raw_duration_s,
            duration_range=duration_range,
            sample_rate_hz=sample_rate_hz,
        )
    obstacles = scene_obstacles(spec.scene, scene)
    trajectory_dir.mkdir(parents=True, exist_ok=True)
    last_error: Exception | None = None
    attempts = max(1, attempts)
    for kind in ([spec.kind, "random"] if spec.kind == "canonical" else [spec.kind]):
        for attempt in range(attempts):
            rng = np.random.default_rng(spec.seed + attempt * 1009 + (0 if kind == "canonical" else 7919))
            shape = None
            altitude = None
            if altitude_range:
                altitude = float(rng.uniform(altitude_range[0], altitude_range[1]))
            if kind == "canonical":
                shapes = trajectory_config.get("canonical", {}).get(
                    "shapes", ["circle", "figure_eight", "square", "straight"]
                )
                shape = str(rng.choice(shapes))
            inspection = trajectory_config.get("inspection", {})
            scale = max(0.35, 1.0 - 0.12 * attempt)
            try:
                if inspection.get("enabled", False):
                    cfg = trajectory_config.get("trajectory", trajectory_config)
                    trajectory = inspection_trajectory(
                        scene["bounds"],
                        rng=rng,
                        sample_rate_hz=sample_rate_hz,
                        duration_s=float(rng.uniform(*duration_range)),
                        altitude_m=float(altitude if altitude is not None else cfg.get("altitude_m", 1.0)),
                        clearance_m=float(cfg.get("min_clearance_m", 0.3)),
                        obstacles=obstacles,
                        motion_mix=inspection.get("motion_mix"),
                        lateral_excursion_m=float(inspection.get("lateral_excursion_m", 0.4)) * scale,
                        vertical_excursion_m=float(inspection.get("vertical_excursion_m", 0.2)) * scale,
                        max_speed_mps=float(inspection.get("max_speed_mps", 1.0)),
                        return_to_start=bool(inspection.get("return_to_start", True)),
                    )
                    kind = "inspection"
                else:
                    trajectory = generate_sequence(
                        rng,
                        scene["bounds"],
                        trajectory_config,
                        kind=kind,
                        duration_s=raw_duration_s,
                        shape=shape,
                        obstacles=obstacles,
                        altitude_m=altitude,
                        vertical_jitter_m=vertical_jitter_m,
                        waypoint_count=test_waypoint_count if spec.split == "test" else None,
                        waypoint_step_m=test_waypoint_step_m if spec.split == "test" else None,
                        attempts=1,
                    )
                if spec.split == "test" and kind != "inspection":
                    path_length = trajectory.path_length
                    if not (test_path_range[0] <= path_length <= test_path_range[1]):
                        last_error = ValueError(
                            f"test path {path_length:.2f} m outside {test_path_range}"
                        )
                        continue
                if spec.target_speed_mps > 0.0:
                    trajectory = retime_to_speed(
                        trajectory,
                        spec.target_speed_mps,
                        sample_rate_hz=sample_rate_hz,
                    )
                if not (duration_range[0] <= trajectory.duration <= duration_range[1]):
                    last_error = ValueError(
                        f"duration {trajectory.duration:.2f} s outside {duration_range}"
                    )
                    continue
                if len(trajectory.position) >= 3:
                    dt = float(np.median(np.diff(trajectory.t)))
                    accel = np.linalg.norm(np.diff(trajectory.position, n=2, axis=0), axis=1) / max(
                        dt * dt, 1e-9
                    )
                    peak = float(np.max(accel))
                    if peak > max_acceleration_mps2:
                        last_error = ValueError(
                            f"peak acceleration {peak:.2f} m/s^2 exceeds {max_acceleration_mps2}"
                        )
                        continue
                if spec.target_speed_mps > 0.0:
                    actual_speed = trajectory.path_length / max(trajectory.duration, 1e-9)
                    error = abs(actual_speed / spec.target_speed_mps - 1.0)
                    if error > 0.05:
                        last_error = ValueError(
                            f"retimed speed error {error:.3f} exceeds 0.05"
                        )
                        continue
            except ValueError as exc:
                last_error = exc
                continue
            path = trajectory_dir / f"{spec.seq_id}.csv"
            trajectory.to_csv(path)
            spec.csv = str(path)
            spec.kind = kind
            spec.duration_s = float(trajectory.duration)
            spec.path_length_m = float(trajectory.path_length)
            spec.shape = shape
            return spec
    raise ValueError(f"could not generate {spec.seq_id}: {last_error}")


def _make_random_scene_trajectory(
    spec: SequenceSpec,
    scene: dict[str, Any],
    trajectory_config: dict[str, Any],
    *,
    attempts: int,
    trajectory_root: Path,
    raw_duration_s: float,
    duration_range: list[float],
    sample_rate_hz: float,
) -> SequenceSpec:
    """Sample a textured room and a stop-and-go inspection path in it."""
    manifest = str((ROOT / scene.get("texture_manifest", "configs/sim/textures_manifest.json")).resolve())
    entries = load_texture_pool(manifest)
    texture_root = ROOT / scene.get("texture_root", "assets/textures")
    random_scene = sample_scene(
        spec.seed,
        scene,
        split=spec.split,
        texture_entries=entries,
        texture_root=texture_root,
    )
    world_dir = ROOT / "sim/worlds/generated"
    world_dir.mkdir(parents=True, exist_ok=True)
    world_path = world_dir / f"{spec.seq_id}.sdf"
    spec_path = trajectory_root / spec.split / spec.scene / f"{spec.seq_id}.scene.json"
    spec_path.parent.mkdir(parents=True, exist_ok=True)

    cfg = trajectory_config.get("trajectory", trajectory_config)
    motion = trajectory_config.get("inspection", {})
    last_error: Exception | None = None
    for attempt in range(max(1, attempts)):
        rng = np.random.default_rng(spec.seed + 31 * attempt + 4409)
        duration = float(rng.uniform(*duration_range))
        # Small, cluttered rooms may not fit the nominal excursion; shrink it
        # progressively instead of failing the whole sequence.
        scale = max(0.35, 1.0 - 0.12 * attempt)
        try:
            trajectory = inspection_trajectory(
                random_scene.bounds,
                rng=rng,
                sample_rate_hz=sample_rate_hz,
                duration_s=duration,
                altitude_m=random_scene.spawn[2],
                clearance_m=float(scene.get("clearance_m", cfg.get("min_clearance_m", 0.3))),
                obstacles=random_scene.obstacles,
                motion_mix=motion.get("motion_mix"),
                lateral_excursion_m=float(motion.get("lateral_excursion_m", 0.4)) * scale,
                vertical_excursion_m=float(motion.get("vertical_excursion_m", 0.2)) * scale,
                max_speed_mps=float(motion.get("max_speed_mps", 1.0)),
                return_to_start=bool(motion.get("return_to_start", True)),
            )
            if not (duration_range[0] <= trajectory.duration <= duration_range[1]):
                raise ValueError(
                    f"duration {trajectory.duration:.2f} s outside {duration_range}"
                )
        except ValueError as exc:
            last_error = exc
            continue
        clutter_cfg = scene.get("path_clutter", {})
        if clutter_cfg.get("enabled", True):
            add_path_clutter(
                random_scene,
                trajectory.position,
                np.random.default_rng(spec.seed + 9901),
                count=tuple(clutter_cfg.get("count", [10, 20])),
                min_distance=float(clutter_cfg.get("min_distance_m", 0.25)),
                max_distance=float(clutter_cfg.get("max_distance_m", 0.45)),
                clearance=float(clutter_cfg.get("clearance_m", 0.24)),
                forward_exclusion_deg=float(clutter_cfg.get("forward_exclusion_deg", 50.0)),
            )
        world_path.write_text(scene_to_sdf(random_scene, texture_root=texture_root), encoding="utf-8")
        spec_path.write_text(json.dumps(random_scene.spec(), indent=2), encoding="utf-8")
        path = trajectory_root / spec.split / spec.scene / f"{spec.seq_id}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        trajectory.to_csv(path)
        spec.csv = str(path)
        spec.kind = "inspection"
        spec.duration_s = float(trajectory.duration)
        spec.path_length_m = float(trajectory.path_length)
        spec.shape = "inspection"
        spec.world = str(world_path)
        spec.scene_spec = str(spec_path)
        return spec
    raise ValueError(f"could not generate {spec.seq_id}: {last_error}")


def _run(command: list[str], *, timeout: float) -> None:
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"stdout:\n{completed.stdout[-2000:]}\nstderr:\n{completed.stderr[-2000:]}"
        )


def _sequence_paths(config: dict[str, Any], spec: SequenceSpec) -> tuple[Path, Path]:
    generation = config["generation"]
    raw = ROOT / generation["raw_dir"] / f"{spec.seq_id}.npz"
    processed = ROOT / generation["processed_dir"] / f"{spec.seq_id}.npz"
    return raw, processed


def _run_one(
    spec: SequenceSpec,
    config: dict[str, Any],
    *,
    overwrite: bool,
    keep_raw: bool,
    resume: bool,
    sensors_config: str,
) -> dict[str, Any]:
    raw_path, processed_path = _sequence_paths(config, spec)
    if resume and processed_path.exists() and not overwrite:
        return {"seq_id": spec.seq_id, "status": "skipped", "processed": str(processed_path)}
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    processed_path.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        raw_path.unlink(missing_ok=True)
        processed_path.unlink(missing_ok=True)
        processed_path.with_suffix(".json").unlink(missing_ok=True)

    world = Path(spec.world) if spec.world else ROOT / "sim/worlds/generated" / f"{spec.scene}.sdf"
    if not world.exists():
        raise FileNotFoundError(f"missing world {world}; run scripts/generate_worlds.py")
    timeout = max(180.0, spec.duration_s * 8.0)
    last_error: Exception | None = None
    for attempt in range(3):
        try:
            _record_and_process(
                spec,
                config,
                raw_path=raw_path,
                processed_path=processed_path,
                timeout=timeout,
                sensors_config=sensors_config,
            )
            last_error = None
            break
        except RuntimeError as exc:
            last_error = exc
            raw_path.unlink(missing_ok=True)
            if attempt < 2:
                logger.warning("%s: attempt %d failed, retrying: %s", spec.seq_id, attempt + 1, exc)
    if last_error is not None:
        raise last_error
    if not keep_raw:
        raw_path.unlink(missing_ok=True)
        raw_path.with_suffix(".json").unlink(missing_ok=True)
        raw_path.with_suffix(".gazebo.log").unlink(missing_ok=True)
    return {"seq_id": spec.seq_id, "status": "done", "processed": str(processed_path)}


def _record_and_process(
    spec: SequenceSpec,
    config: dict[str, Any],
    *,
    raw_path: Path,
    processed_path: Path,
    timeout: float,
    sensors_config: str,
) -> None:
    world = Path(spec.world) if spec.world else ROOT / "sim/worlds/generated" / f"{spec.scene}.sdf"
    if not world.exists():
        raise FileNotFoundError(f"missing world {world}; run scripts/generate_worlds.py")
    _run(
        [
            sys.executable,
            str(ROOT / "scripts/record_sequence.py"),
            "--world",
            str(world),
            "--trajectory",
            spec.csv,
            "--out",
            str(raw_path),
            "--duration",
            f"{spec.duration_s:.3f}",
        ],
        timeout=timeout,
    )
    _run(
        [
            sys.executable,
            str(ROOT / "scripts/postprocess_sequence.py"),
            "--raw",
            str(raw_path),
            "--out",
            str(processed_path),
            "--scene",
            spec.scene,
            "--kind",
            spec.kind,
            "--split",
            spec.split,
            "--seq-id",
            spec.seq_id,
            "--seed",
            str(spec.seed),
            "--sensors-config",
            sensors_config,
        ]
        + (["--scene-spec", spec.scene_spec] if spec.scene_spec else []),
        timeout=timeout,
    )


def _summarize(plan: list[SequenceSpec], config: dict[str, Any]) -> dict[str, Any]:
    processed_dir = ROOT / config["generation"]["processed_dir"]
    records: list[dict[str, Any]] = []
    for spec in plan:
        meta_path = processed_dir / f"{spec.seq_id}.json"
        if meta_path.exists():
            records.append(json.loads(meta_path.read_text(encoding="utf-8")))
    seconds = sum(float(record.get("duration_s", 0.0)) for record in records)
    path = sum(float(record.get("path_length_m", 0.0)) for record in records)
    return {
        "sequences": len(records),
        "seconds": seconds,
        "path_m": path,
        "by_split": _group(records, "split"),
        "by_scene": _group(records, "scene"),
    }


def _group(records: list[dict[str, Any]], key: str) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for record in records:
        name = str(record.get(key, "unknown"))
        entry = result.setdefault(name, {"sequences": 0.0, "seconds": 0.0, "path_m": 0.0})
        entry["sequences"] += 1
        entry["seconds"] += float(record.get("duration_s", 0.0))
        entry["path_m"] += float(record.get("path_length_m", 0.0))
    return result


def main() -> int:
    args = _parse_args()
    logger = get_logger("generate_dataset")
    config = load_yaml(args.generation)
    scene_config = load_yaml(args.scenes)
    trajectory_config = load_yaml(args.trajectory)
    generation = config["generation"]
    if args.target_seconds is not None:
        generation["target_seconds"] = float(args.target_seconds)
    if args.workers is not None:
        generation["workers"] = int(args.workers)
    if args.keep_raw:
        generation["keep_raw"] = True
    if args.smoke:
        generation["target_seconds"] = 32.0

    specs = _plan_specs(
        config,
        scene_config,
        trajectory_config,
        total_seconds=float(generation["target_seconds"]),
        split_filter=args.split,
        scene_filter=args.scene,
        smoke=args.smoke,
    )
    logger.info(
        "planned %d sequences: %.1f s",
        len(specs),
        sum(spec.duration_s for spec in specs),
    )

    trajectory_root = ROOT / generation.get("trajectory_dir", "data/trajectories")
    raw_duration_s = float(generation.get("raw_duration_s", 20.0))
    altitude_range = generation.get("altitude_range")
    if altitude_range is not None:
        altitude_range = [float(v) for v in altitude_range]
    max_acceleration = float(generation.get("max_acceleration_mps2", 12.0))
    sample_rate_hz = float(trajectory_config.get("trajectory", {}).get("sample_rate_hz", 100.0))
    sensors_config = str((ROOT / generation.get("sensors_config", args.sensors)).resolve())

    failures: list[str] = []
    made: list[SequenceSpec] = []
    for index, spec in enumerate(specs, start=1):
        try:
            made.append(
                _make_trajectory(
                    spec,
                    scene_config,
                    trajectory_config,
                    attempts=int(generation.get("attempts_per_sequence", 80)),
                    test_path_range=[float(v) for v in generation.get("test_path_m", [4.0, 7.0])],
                    test_waypoint_count=[int(v) for v in generation.get("test_waypoint_count", [3, 7])],
                    test_waypoint_step_m=[
                        float(v) for v in generation.get("test_waypoint_step_m", [0.3, 0.7])
                    ],
                    trajectory_root=trajectory_root,
                    raw_duration_s=raw_duration_s,
                    duration_range=[float(v) for v in generation["duration_s"][spec.split]],
                    altitude_range=altitude_range,
                    vertical_jitter_m=float(generation.get("vertical_jitter_m", 0.15)),
                    max_acceleration_mps2=max_acceleration,
                    sample_rate_hz=sample_rate_hz,
                )
            )
        except ValueError as exc:
            logger.error("[%d/%d] %s", index, len(specs), exc)
            failures.append(spec.seq_id)
    logger.info("generated %d trajectories (%d failures)", len(made), len(failures))

    plan_path = ROOT / generation.get("plan_path", "data/generation_plan.json")
    plan_report_path = ROOT / generation.get("plan_report_path", "data/generation_plan.md")
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps([asdict(spec) for spec in made], indent=2, sort_keys=True), encoding="utf-8"
    )
    total_duration = sum(spec.duration_s for spec in made)
    total_path = sum(spec.path_length_m for spec in made)
    logger.info("planned duration %.1f s, path %.1f m", total_duration, total_path)

    if args.dry_run or args.plan_only:
        _write_plan_report(made, report_path=plan_report_path)
        return 0 if not failures else 1

    workers = int(generation.get("workers", 3))
    if workers <= 0:
        # Each Gazebo server is a single-core process; rendering is the
        # practical limit (8+ concurrent EGL contexts fail on a 6 GB GPU),
        # so cap the automatic value.
        workers = min(6, max(1, (os.cpu_count() or 2) // 2))
    resume = bool(generation.get("resume", True)) and not args.no_resume
    logger.info("recording with %d workers", workers)
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(
                _run_one,
                spec,
                config,
                overwrite=args.overwrite,
                keep_raw=bool(generation.get("keep_raw", False)),
                resume=resume,
                sensors_config=sensors_config,
            ): spec
            for spec in made
        }
        for future in concurrent.futures.as_completed(futures):
            spec = futures[future]
            try:
                result = future.result()
                logger.info("%s: %s", spec.seq_id, result["status"])
            except Exception as exc:  # noqa: BLE001 - keep the batch going
                logger.error("%s failed: %s", spec.seq_id, exc)
                failures.append(spec.seq_id)

    summary = _summarize(made, config)
    logger.info(
        "processed: %d sequences, %.1f s, %.1f m",
        summary["sequences"],
        summary["seconds"],
        summary["path_m"],
    )
    _write_plan_report(
        made,
        summary=summary,
        failures=failures,
        report_path=plan_report_path,
    )
    if failures:
        logger.error("%d sequences failed: %s", len(failures), ", ".join(failures[:10]))
        return 1
    return 0


def _write_plan_report(
    specs: list[SequenceSpec],
    *,
    summary: dict[str, Any] | None = None,
    failures: list[str] | None = None,
    report_path: Path | None = None,
) -> None:
    report = report_path or ROOT / "data/generation_plan.md"
    report.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# TIO-Former generation plan",
        "",
        f"Sequences: {len(specs)}",
        f"Planned duration: {sum(s.duration_s for s in specs):.1f} s",
        f"Planned path: {sum(s.path_length_m for s in specs):.1f} m",
        "",
        "| split | scene | sequences | seconds | path m | mean target speed m/s |",
        "|---|---|---:|---:|---:|---:|",
    ]
    groups: dict[tuple[str, str], list[SequenceSpec]] = {}
    for spec in specs:
        groups.setdefault((spec.split, spec.scene), []).append(spec)
    for (split, scene), group in sorted(groups.items()):
        speeds = [s.target_speed_mps for s in group if s.target_speed_mps > 0.0]
        mean_speed = sum(speeds) / len(speeds) if speeds else 0.0
        lines.append(
            f"| {split} | {scene} | {len(group)} | "
            f"{sum(s.duration_s for s in group):.1f} | "
            f"{sum(s.path_length_m for s in group):.1f} | "
            f"{mean_speed:.3f} |"
        )
    if summary is not None:
        lines += ["", "## Processed", "", f"Sequences: {summary['sequences']}"]
        lines.append(f"Duration: {summary['seconds']:.1f} s")
        lines.append(f"Path: {summary['path_m']:.1f} m")
    if failures:
        lines += ["", "## Failures", "", ", ".join(failures)]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
