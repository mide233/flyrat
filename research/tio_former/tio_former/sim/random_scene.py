"""Randomized textured rooms for domain-randomized TIO training data.

Each scene is sampled deterministically from a seed so the planner (which needs
obstacle AABBs for collision checks) and the recorder (which needs the SDF
world) can build the identical room in different processes.

The room is a rectangular box with textured walls/floor, 8-25 textured boxes,
cylinders and panels, and one directional light whose colour/intensity is
sampled per sequence.  A few panels are placed close to the spawn point so the
range sensors see 0.2-0.7 m returns, matching the real vehicle's ToF
distribution instead of the old 1.5 m median.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np

_TEXTURE_ROOT = "assets/textures"

_HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <scene>
      <ambient>{ambient}</ambient>
      <background>{background}</background>
      <shadows>true</shadows>
    </scene>
    <physics name="physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>{real_time_factor}</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>{light_color} 1</diffuse>
      <specular>{light_specular} 1</specular>
      <direction>{light_direction}</direction>
    </light>
    <light type="point" name="fill">
      <pose>{fill_pose} 0 0 0</pose>
      <diffuse>{fill_color} 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>12</range><constant>0.6</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation>
    </light>
"""

_FOOTER = """  </world>
</sdf>
"""


@dataclass
class RandomScene:
    """Serializable description of one randomized room."""

    name: str
    bounds: dict[str, list[float]]
    spawn: list[float]
    light: dict[str, Any]
    objects: list[dict[str, Any]] = field(default_factory=list)
    obstacles: list[dict[str, Any]] = field(default_factory=list)
    textures: list[str] = field(default_factory=list)

    def spec(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "bounds": self.bounds,
            "spawn": self.spawn,
            "light": self.light,
            "objects": self.objects,
            "obstacles": self.obstacles,
            "textures": self.textures,
        }


def _rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(np.random.SeedSequence(int(seed) & 0xFFFFFFFFFFFF))


def load_texture_pool(manifest: str | Path) -> list[dict[str, Any]]:
    path = Path(manifest)
    if not path.exists():
        return []
    payload = json.loads(path.read_text(encoding="utf-8"))
    entries = []
    for item in payload.get("textures", []):
        if "file" not in item:
            continue
        entries.append(item)
    entries.sort(key=lambda item: str(item["id"]))
    return entries


def unseen_texture_ids(entries: Iterable[dict[str, Any]], fraction: float = 0.2) -> set[str]:
    """Stable 20% hold-out set used only by the test split."""
    ordered = sorted(str(item["id"]) for item in entries)
    if not ordered:
        return set()
    count = max(1, int(round(len(ordered) * fraction)))
    step = max(1, len(ordered) // count)
    return {ordered[index] for index in range(0, len(ordered), step)}


def _kelvin_to_rgb(kelvin: float) -> tuple[float, float, float]:
    """Approximate black-body colour, normalised to a peak of 1."""
    temperature = kelvin / 100.0
    if temperature <= 66.0:
        red = 255.0
        green = 99.4708025861 * math.log(temperature) - 161.1195681661
        blue = 0.0 if temperature <= 19.0 else 138.5177312231 * math.log(temperature - 10.0) - 305.0447927307
    else:
        red = 329.698727446 * ((temperature - 60.0) ** -0.1332047592)
        green = 288.1221695283 * ((temperature - 60.0) ** -0.0755148492)
        blue = 255.0
    values = [min(255.0, max(0.0, value)) for value in (red, green, blue)]
    peak = max(values) or 1.0
    return tuple(round(value / peak, 3) for value in values)  # type: ignore[return-value]


def _texture_material(path: str, tint: float, roughness: float = 0.85) -> str:
    return (
        "<material>"
        f"<ambient>{tint:.3f} {tint:.3f} {tint:.3f} 1</ambient>"
        f"<diffuse>{tint:.3f} {tint:.3f} {tint:.3f} 1</diffuse>"
        "<specular>0.08 0.08 0.08 1</specular>"
        "<pbr><metal>"
        f"<albedo_map>file://{path}</albedo_map>"
        f"<roughness>{roughness:.2f}</roughness>"
        "<metalness>0.0</metalness>"
        "</metal></pbr>"
        "</material>"
    )


def _color_material(color: tuple[float, float, float]) -> str:
    return (
        "<material>"
        f"<ambient>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</ambient>"
        f"<diffuse>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</diffuse>"
        "<specular>0.05 0.05 0.05 1</specular>"
        "</material>"
    )


def _aabb(
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    yaw: float,
) -> dict[str, Any]:
    sx, sy, sz = size
    if abs(yaw) > 1e-6:
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        sx, sy = sx * c + sy * s, sx * s + sy * c
    return {
        "name": name,
        "center": [float(center[0]), float(center[1]), float(center[2])],
        "size": [float(sx), float(sy), float(sz)],
    }


def _overlaps(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    objects: list[dict[str, Any]],
    margin: float = 0.15,
) -> bool:
    for item in objects:
        other = item["center"]
        half = item["size"]
        if (
            abs(center[0] - other[0]) < 0.5 * (size[0] + half[0]) + margin
            and abs(center[1] - other[1]) < 0.5 * (size[1] + half[1]) + margin
            and abs(center[2] - other[2]) < 0.5 * (size[2] + half[2]) + margin
        ):
            return True
    return False


def _pick_texture(
    rng: np.random.Generator,
    entries: list[dict[str, Any]],
    root: Path,
    allow_unseen: bool,
    unseen: set[str],
) -> tuple[str, str]:
    if not entries:
        return "", ""
    if allow_unseen:
        pool = [item for item in entries if str(item["id"]) in unseen] or entries
    else:
        pool = [item for item in entries if str(item["id"]) not in unseen] or entries
    entry = pool[int(rng.integers(0, len(pool)))]
    return str(entry["id"]), str(root / str(entry["file"]))


def sample_scene(
    seed: int,
    config: dict[str, Any],
    *,
    split: str = "train",
    texture_entries: list[dict[str, Any]] | None = None,
    texture_root: str | Path = _TEXTURE_ROOT,
) -> RandomScene:
    """Sample room geometry, clutter, textures and lighting from ``seed``."""
    rng = _rng(seed)
    room_cfg = config.get("room", {})
    width = float(rng.uniform(*room_cfg.get("width_m", [3.0, 8.0])))
    depth = float(rng.uniform(*room_cfg.get("depth_m", [3.0, 8.0])))
    height = float(rng.uniform(*room_cfg.get("height_m", [2.4, 3.2])))
    bounds = {"x": [-0.5 * width, 0.5 * width], "y": [-0.5 * depth, 0.5 * depth], "z": [0.0, height]}
    spawn_cfg = config.get("spawn", {})
    if isinstance(spawn_cfg, dict):
        altitude_range = spawn_cfg.get("altitude_m", config.get("spawn_altitude_m", [0.45, 1.35]))
        low_probability = float(spawn_cfg.get("low_altitude_probability", 0.0))
        low_range = spawn_cfg.get("low_altitude_m", [0.22, 0.35])
    else:
        altitude_range = config.get("spawn_altitude_m", [0.45, 1.35])
        low_probability = 0.0
        low_range = [0.22, 0.35]
    # A low flight line makes the down-looking ToF see the table/floor at
    # 0.2-0.35 m, which is the dominant regime in the real recordings.
    if low_probability > 0.0 and rng.random() < low_probability:
        altitude_range = low_range
    spawn = [0.0, 0.0, float(rng.uniform(*altitude_range))]
    light_cfg = config.get("light", {})
    intensity = float(rng.uniform(*light_cfg.get("intensity", [0.45, 1.15])))
    kelvin = float(rng.uniform(*light_cfg.get("color_temperature_k", [3000.0, 6800.0])))
    color = tuple(min(1.0, round(channel * intensity, 3)) for channel in _kelvin_to_rgb(kelvin))
    ambient_value = float(rng.uniform(*light_cfg.get("ambient", [0.22, 0.6])))
    azimuth = float(rng.uniform(-math.pi, math.pi))
    elevation = float(rng.uniform(0.6, 1.3))
    direction = (
        round(-math.cos(azimuth) * math.cos(elevation), 3),
        round(-math.sin(azimuth) * math.cos(elevation), 3),
        round(-math.sin(elevation), 3),
    )
    fill_pose = (
        round(float(rng.uniform(bounds["x"][0] + 0.5, bounds["x"][1] - 0.5)), 2),
        round(float(rng.uniform(bounds["y"][0] + 0.5, bounds["y"][1] - 0.5)), 2),
        round(height - 0.25, 2),
    )
    light = {
        "color": color,
        "color_temperature_k": round(kelvin, 1),
        "intensity": round(intensity, 3),
        "direction": direction,
        "ambient": [ambient_value] * 3,
        "fill_pose": fill_pose,
        "fill_color": color,
        "drift_amplitude": float(light_cfg.get("drift_amplitude", 0.15)),
        "drift_period_s": float(rng.uniform(*light_cfg.get("drift_period_s", [20.0, 60.0]))),
    }
    background = round(float(rng.uniform(0.35, 0.8)), 3)
    scene = RandomScene(
        name=f"random_{seed:08x}",
        bounds=bounds,
        spawn=spawn,
        light=light,
    )
    scene.light["background"] = [background] * 3

    entries = texture_entries or []
    unseen = unseen_texture_ids(entries) if entries else set()
    allow_unseen = split == "test" and bool(rng.random() < 0.5)
    tex_root = Path(texture_root)
    texture_cfg = config.get("textures", {})
    per_scene = int(rng.integers(*texture_cfg.get("per_scene", [4, 9])))
    chosen: list[tuple[str, str]] = []
    for _ in range(per_scene):
        chosen.append(_pick_texture(rng, entries, tex_root, allow_unseen, unseen))

    def material(index: int) -> str:
        if not chosen or float(rng.random()) < 0.15:
            tint = float(rng.uniform(0.18, 0.8))
            return _color_material((tint, tint * float(rng.uniform(0.85, 1.0)), tint * float(rng.uniform(0.8, 1.0))))
        texture_id, path = chosen[index % len(chosen)]
        tint = float(rng.uniform(0.65, 1.0))
        return _texture_material(path, tint, roughness=float(rng.uniform(0.6, 0.95)))

    # Floor and walls use their own textures.
    floor_id, floor_path = _pick_texture(rng, entries, tex_root, allow_unseen, unseen)
    wall_id, wall_path = _pick_texture(rng, entries, tex_root, allow_unseen, unseen)
    scene.textures = sorted({texture_id for texture_id, _ in chosen if texture_id} | {floor_id, wall_id} - {""})
    scene.objects.append(
        {
            "kind": "floor",
            "name": "floor",
            "center": [0.0, 0.0, -0.04],
            "size": [width + 1.0, depth + 1.0, 0.08],
            "yaw": 0.0,
            "texture": floor_id,
        }
    )
    thickness = 0.06
    walls = [
        ("wall_north", (width, thickness, height), (0.0, 0.5 * depth, 0.5 * height)),
        ("wall_south", (width, thickness, height), (0.0, -0.5 * depth, 0.5 * height)),
        ("wall_east", (thickness, depth, height), (0.5 * width, 0.0, 0.5 * height)),
        ("wall_west", (thickness, depth, height), (-0.5 * width, 0.0, 0.5 * height)),
    ]
    for name, size, center in walls:
        scene.objects.append(
            {
                "kind": "wall",
                "name": name,
                "center": list(center),
                "size": list(size),
                "yaw": 0.0,
                "texture": wall_id,
            }
        )
        scene.obstacles.append(_aabb(name, center, size, 0.0))

    count = int(rng.integers(*config.get("clutter", {}).get("count", [8, 25])))
    near_count = int(rng.integers(*config.get("clutter", {}).get("near_field_count", [2, 4])))
    placed = 0
    attempts = 0
    while placed < count and attempts < count * 30:
        attempts += 1
        kind = str(rng.choice(["box", "box", "box", "cylinder", "panel"]))
        if kind == "panel":
            size = (float(rng.uniform(0.3, 1.1)), float(rng.uniform(0.04, 0.1)), float(rng.uniform(0.5, 1.8)))
        elif kind == "cylinder":
            diameter = float(rng.uniform(0.18, 0.55))
            size = (diameter, diameter, float(rng.uniform(0.3, 1.4)))
        else:
            size = (
                float(rng.uniform(0.15, 0.9)),
                float(rng.uniform(0.15, 0.9)),
                float(rng.uniform(0.15, 1.2)),
            )
        near = placed < near_count and float(rng.random()) < 0.85
        if near:
            radius = float(rng.uniform(0.70, 1.05))
            angle = float(rng.uniform(-math.pi, math.pi))
            center = [
                spawn[0] + radius * math.cos(angle),
                spawn[1] + radius * math.sin(angle),
                min(height - 0.2, spawn[2] + float(rng.uniform(-0.25, 0.35))),
            ]
            size = (min(size[0], 0.45), min(size[1], 0.45), min(size[2], 0.7))
        else:
            center = [
                float(rng.uniform(bounds["x"][0] + 0.25 + 0.5 * size[0], bounds["x"][1] - 0.25 - 0.5 * size[0])),
                float(rng.uniform(bounds["y"][0] + 0.25 + 0.5 * size[1], bounds["y"][1] - 0.25 - 0.5 * size[1])),
                0.5 * size[2] + float(rng.uniform(0.0, 0.15)),
            ]
        yaw = float(rng.uniform(-math.pi, math.pi)) if kind != "cylinder" else 0.0
        if _overlaps(tuple(center), tuple(size), scene.obstacles, margin=0.18):
            continue
        if math.hypot(center[0] - spawn[0], center[1] - spawn[1]) < 0.60:
            continue
        name = f"clutter_{placed}"
        scene.objects.append(
            {
                "kind": kind,
                "name": name,
                "center": [float(value) for value in center],
                "size": [float(value) for value in size],
                "yaw": yaw,
                "texture": chosen[placed % len(chosen)][0] if chosen else "",
                "tint": round(float(rng.uniform(0.65, 1.0)), 3),
            }
        )
        scene.obstacles.append(_aabb(name, tuple(center), tuple(size), yaw))
        placed += 1

    scene.light["floor_texture"] = floor_id
    scene.light["wall_texture"] = wall_id
    return scene


def add_path_clutter(
    scene: "RandomScene",
    positions: np.ndarray,
    rng: np.random.Generator,
    *,
    count: tuple[int, int] = (10, 20),
    min_distance: float = 0.25,
    max_distance: float = 0.45,
    clearance: float = 0.24,
    forward_exclusion_deg: float = 50.0,
) -> int:
    """Place thin near-field panels beside the flight path.

    Real ToF returns in the recorded bag are dominated by surfaces ~0.2 m
    away.  Panels 0.3-0.55 m to the side of the trajectory reproduce that
    without intersecting the path, and they are kept outside a forward cone so
    the camera view is not blocked.
    """
    points = np.asarray(positions, dtype=np.float64)
    if len(points) < 8:
        return 0
    wanted = int(rng.integers(*count))
    height_limit = float(scene.bounds["z"][1])
    placed = 0
    attempts = 0
    while placed < wanted and attempts < wanted * 40:
        attempts += 1
        index = int(rng.integers(6, len(points) - 6))
        heading = points[min(len(points) - 1, index + 6)] - points[max(0, index - 6)]
        if float(np.linalg.norm(heading[:2])) < 1e-6:
            continue
        heading_angle = math.atan2(heading[1], heading[0])
        side = 1.0 if rng.random() < 0.5 else -1.0
        offset_angle = math.radians(float(rng.uniform(forward_exclusion_deg, 130.0)))
        angle = heading_angle + side * offset_angle
        distance = float(rng.uniform(min_distance, max_distance))
        center = [
            float(points[index, 0] + distance * math.cos(angle)),
            float(points[index, 1] + distance * math.sin(angle)),
            float(np.clip(points[index, 2] + float(rng.uniform(-0.15, 0.15)), 0.45, height_limit - 0.3)),
        ]
        size = (
            float(rng.uniform(0.45, 0.8)),
            float(rng.uniform(0.05, 0.08)),
            float(rng.uniform(0.6, 1.1)),
        )
        if not (
            scene.bounds["x"][0] + 0.15 < center[0] < scene.bounds["x"][1] - 0.15
            and scene.bounds["y"][0] + 0.15 < center[1] < scene.bounds["y"][1] - 0.15
        ):
            continue
        if float(np.min(np.linalg.norm(points[:, :2] - np.array(center[:2]), axis=1))) < clearance:
            continue
        if _overlaps(tuple(center), tuple(size), scene.obstacles, margin=0.10):
            continue
        name = f"path_clutter_{placed}"
        scene.objects.append(
            {
                "kind": "panel",
                "name": name,
                "center": [float(value) for value in center],
                "size": [float(value) for value in size],
                "yaw": float(angle + math.pi / 2.0),
                "texture": "",
                "tint": round(float(rng.uniform(0.45, 0.9)), 3),
            }
        )
        scene.obstacles.append(_aabb(name, tuple(center), tuple(size), float(angle + math.pi / 2.0)))
        placed += 1
    return placed


def _object_sdf(
    item: dict[str, Any],
    rng: np.random.Generator,
    entries_by_id: dict[str, str],
) -> str:
    name = str(item["name"])
    center = item["center"]
    size = item["size"]
    yaw = float(item.get("yaw", 0.0))
    texture_path = entries_by_id.get(str(item.get("texture", "")))
    if texture_path:
        color = float(item.get("tint", 1.0))
        material = _texture_material(texture_path, color, roughness=0.8)
    else:
        tint = float(item.get("tint", 0.55))
        material = _color_material((tint, tint * 0.92, tint * 0.85))
    if item["kind"] == "floor":
        return f"""    <model name="{name}">
      <static>true</static>
      <pose>{center[0]} {center[1]} {center[2]} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>{material}</visual>
      </link>
    </model>
"""
    if item["kind"] == "cylinder":
        return f"""    <model name="{name}">
      <static>true</static>
      <pose>{center[0]} {center[1]} {center[2]} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{0.5 * size[0]}</radius><length>{size[2]}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{0.5 * size[0]}</radius><length>{size[2]}</length></cylinder></geometry>{material}</visual>
      </link>
    </model>
"""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{center[0]} {center[1]} {center[2]} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>{material}</visual>
      </link>
    </model>
"""


def scene_to_sdf(
    scene: RandomScene,
    *,
    texture_root: str | Path = _TEXTURE_ROOT,
    model_uri: str = "model://tof_rig",
    real_time_factor: float = 1.0,
) -> str:
    root = Path(texture_root).resolve()
    entries_by_id: dict[str, str] = {}
    manifest = Path(__file__).resolve().parents[2] / "configs/sim/textures_manifest.json"
    if manifest.exists():
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        for item in payload.get("textures", []):
            entries_by_id[str(item["id"])] = str(root / str(item["file"]))
    light = scene.light
    color = light.get("color", (0.9, 0.9, 0.9))
    ambient = light.get("ambient", (0.4, 0.4, 0.4))
    background = light.get("background", (0.6, 0.65, 0.7))
    fill_pose = light.get("fill_pose", (0.0, 0.0, 2.0))
    content = _HEADER.format(
        name=scene.name,
        ambient=" ".join(f"{float(v):.3f}" for v in (*ambient, 1.0)),
        background=" ".join(f"{float(v):.3f}" for v in (*background, 1.0)),
        real_time_factor=float(real_time_factor),
        light_color=" ".join(f"{float(v):.3f}" for v in color),
        light_specular=" ".join(f"{min(1.0, 0.35 * float(v)):.3f}" for v in color),
        light_direction=" ".join(f"{float(v):.3f}" for v in light.get("direction", (-0.4, 0.2, -1.0))),
        fill_pose=" ".join(f"{float(v):.3f}" for v in fill_pose),
        fill_color=" ".join(f"{min(1.0, 0.5 * float(v)):.3f}" for v in color),
    )
    rng = np.random.default_rng(1)
    for item in scene.objects:
        content += _object_sdf(item, rng, entries_by_id)
    content += f"""    <include>
      <uri>{model_uri}</uri>
      <name>tof_rig</name>
      <pose>{scene.spawn[0]} {scene.spawn[1]} {scene.spawn[2]} 0 0 0</pose>
    </include>
"""
    content += _FOOTER
    return content


def write_scene(scene: RandomScene, path: str | Path, **kwargs: Any) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(scene_to_sdf(scene, **kwargs), encoding="utf-8")
    return target
