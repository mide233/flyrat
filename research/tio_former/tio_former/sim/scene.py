"""Procedural primitive scenes for the M1 simulation data pipeline.

Four scene styles mirror the paper's evaluation environments: corridor, office,
pantry and meeting room.  Meshes (Replica / Gazebo Fuel) are intentionally
deferred to M2; the M1 scenes are made of boxes and cylinders so the pipeline
can be validated end-to-end without external assets.
"""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

# SDF 1.9 is the format accepted by Gazebo Harmonic.
_HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <physics name="physics" type="ignored">
      <max_step_size>0.0025</max_step_size>
      <real_time_factor>{real_time_factor}</real_time_factor>
      <real_time_update_rate>400</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1.0</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

_FOOTER = """  </world>
</sdf>
"""


def _material(color: tuple[float, float, float]) -> str:
    return (
        "<material>"
        f"<ambient>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</ambient>"
        f"<diffuse>{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1</diffuse>"
        "<specular>0.05 0.05 0.05 1</specular>"
        "</material>"
    )


def _pose_values(
    pose: tuple[float, ...],
) -> tuple[float, float, float, float, float, float]:
    if len(pose) == 3:
        return float(pose[0]), float(pose[1]), float(pose[2]), 0.0, 0.0, 0.0
    if len(pose) == 6:
        return tuple(float(value) for value in pose)  # type: ignore[return-value]
    raise ValueError(f"pose must have 3 or 6 values, got {len(pose)}")


def _box(
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, ...],
    color: tuple[float, float, float],
    obstacles: list[dict[str, Any]] | None = None,
) -> str:
    sx, sy, sz = size
    x, y, z, roll, pitch, yaw = _pose_values(pose)
    if obstacles is not None:
        _add_obstacle(obstacles, name, (x, y, z), (sx, sy, sz), yaw)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          {_material(color)}
        </visual>
      </link>
    </model>
"""


def _cylinder(
    name: str,
    radius: float,
    height: float,
    pose: tuple[float, ...],
    color: tuple[float, float, float],
    obstacles: list[dict[str, Any]] | None = None,
) -> str:
    x, y, z, roll, pitch, yaw = _pose_values(pose)
    if obstacles is not None:
        _add_obstacle(obstacles, name, (x, y, z), (2.0 * radius, 2.0 * radius, height), yaw)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
          {_material(color)}
        </visual>
      </link>
    </model>
"""


def _add_obstacle(
    obstacles: list[dict[str, Any]],
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    yaw: float = 0.0,
) -> None:
    """Append a conservative axis-aligned box for trajectory collision checks."""
    sx, sy, sz = (float(v) for v in size)
    if abs(yaw) > 1e-6:
        c = abs(float(np.cos(yaw)))
        s = abs(float(np.sin(yaw)))
        sx, sy = sx * c + sy * s, sx * s + sy * c
    obstacles.append(
        {
            "name": name,
            "center": [float(center[0]), float(center[1]), float(center[2])],
            "size": [sx, sy, sz],
        }
    )


def _mesh(
    name: str,
    model: str,
    pose: tuple[float, float, float, float],
    bbox: tuple[float, float, float],
    obstacles: list[dict[str, Any]] | None = None,
    *,
    scale: float = 1.0,
) -> str:
    """Include a Gazebo Fuel mesh model; pose is [x, y, z, yaw]."""
    x, y, z, yaw = (float(v) for v in pose)
    if obstacles is not None:
        _add_obstacle(obstacles, name, (x, y, z + 0.5 * bbox[2]), bbox, yaw)
    scale_xml = f"\n      <scale>{scale} {scale} {scale}</scale>" if scale != 1.0 else ""
    return f"""    <include>
      <uri>model://{model}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>{scale_xml}
    </include>
"""


def _walls(
    bounds: dict[str, Iterable[float]],
    height: float,
    thickness: float = 0.08,
    obstacles: list[dict[str, Any]] | None = None,
) -> str:
    x0, x1 = (float(v) for v in bounds["x"])
    y0, y1 = (float(v) for v in bounds["y"])
    z0 = float(bounds["z"][0])
    z1 = float(bounds["z"][1])
    z = 0.5 * (z0 + z1)
    parts = [
        _box("wall_north", (x1 - x0 + 2 * thickness, thickness, z1 - z0), (0.5 * (x0 + x1), y1, z), (0.72, 0.72, 0.70), obstacles),
        _box("wall_south", (x1 - x0 + 2 * thickness, thickness, z1 - z0), (0.5 * (x0 + x1), y0, z), (0.72, 0.72, 0.70), obstacles),
        _box("wall_east", (thickness, y1 - y0, z1 - z0), (x1, 0.5 * (y0 + y1), z), (0.70, 0.70, 0.68), obstacles),
        _box("wall_west", (thickness, y1 - y0, z1 - z0), (x0, 0.5 * (y0 + y1), z), (0.70, 0.70, 0.68), obstacles),
    ]
    return "".join(parts)


def _office(bounds: dict[str, Iterable[float]], obstacles: list[dict[str, Any]] | None = None) -> str:
    x0, x1 = (float(v) for v in bounds["x"])
    y0, y1 = (float(v) for v in bounds["y"])
    parts: list[str] = []
    # Desks with monitors and chairs on both sides.
    for index, (x, y) in enumerate(
        [(-2.4, -1.8), (0.0, -1.8), (2.4, -1.8), (-2.4, 1.8), (0.0, 1.8), (2.4, 1.8)]
    ):
        parts.append(_box(f"desk_{index}", (1.3, 0.65, 0.06), (x, y, 0.75), (0.35, 0.25, 0.16), obstacles))
        parts.append(_box(f"desk_leg_{index}_a", (0.06, 0.06, 0.72), (x - 0.55, y - 0.25, 0.36), (0.2, 0.2, 0.2), obstacles))
        parts.append(_box(f"desk_leg_{index}_b", (0.06, 0.06, 0.72), (x + 0.55, y - 0.25, 0.36), (0.2, 0.2, 0.2), obstacles))
        parts.append(_box(f"monitor_{index}", (0.45, 0.04, 0.30), (x, y - 0.12, 1.08), (0.05, 0.05, 0.05), obstacles))
        parts.append(_cylinder(f"chair_{index}", 0.22, 0.45, (x, y + 0.55, 0.23), (0.15, 0.2, 0.35), obstacles))
    # Partitions and a cabinet.
    for index, y in enumerate([-0.6, 0.6]):
        parts.append(_box(f"partition_{index}", (2.8, 0.05, 1.2), (0.0, y, 0.6), (0.55, 0.57, 0.60), obstacles))
    parts.append(_box("cabinet", (1.2, 0.4, 1.8), (x1 - 0.8, y1 - 0.7, 0.9), (0.30, 0.30, 0.32), obstacles))
    return "".join(parts)


def _pantry(bounds: dict[str, Iterable[float]], obstacles: list[dict[str, Any]] | None = None) -> str:
    x0, x1 = (float(v) for v in bounds["x"])
    y0, y1 = (float(v) for v in bounds["y"])
    parts: list[str] = []
    # Counters along two walls.
    parts.append(_box("counter_a", (3.2, 0.55, 0.9), (0.0, y1 - 0.55, 0.45), (0.55, 0.38, 0.22), obstacles))
    parts.append(_box("counter_b", (0.55, 2.4, 0.9), (x1 - 0.55, 0.0, 0.45), (0.50, 0.36, 0.20), obstacles))
    # Shelf stack and a fridge.
    parts.append(_box("shelf", (1.5, 0.35, 2.0), (-1.4, y1 - 0.45, 1.0), (0.30, 0.35, 0.40), obstacles))
    parts.append(_box("fridge", (0.7, 0.7, 1.7), (x1 - 0.8, -1.3, 0.85), (0.75, 0.76, 0.78), obstacles))
    parts.append(_box("island", (1.4, 0.7, 0.9), (-0.3, -0.3, 0.45), (0.45, 0.32, 0.18), obstacles))
    return "".join(parts)


def _meeting_room(bounds: dict[str, Iterable[float]], obstacles: list[dict[str, Any]] | None = None) -> str:
    x0, x1 = (float(v) for v in bounds["x"])
    y0, y1 = (float(v) for v in bounds["y"])
    parts: list[str] = []
    parts.append(_box("table", (2.6, 1.2, 0.08), (0.0, 0.0, 0.75), (0.40, 0.28, 0.16), obstacles))
    for index, (x, y) in enumerate(
        [(-0.9, -0.85), (0.0, -0.85), (0.9, -0.85), (-0.9, 0.85), (0.0, 0.85), (0.9, 0.85)]
    ):
        parts.append(_cylinder(f"meeting_chair_{index}", 0.23, 0.5, (x, y, 0.25), (0.12, 0.14, 0.22), obstacles))
    parts.append(_box("screen", (1.6, 0.08, 0.9), (0.0, y1 - 0.12, 1.55), (0.05, 0.05, 0.05), obstacles))
    parts.append(_box("sideboard", (2.2, 0.45, 0.8), (0.0, y0 + 0.35, 0.4), (0.35, 0.25, 0.15), obstacles))
    return "".join(parts)


def _meshes(
    entries: Iterable[dict[str, Any]],
    obstacles: list[dict[str, Any]],
) -> str:
    parts: list[str] = []
    for index, entry in enumerate(entries):
        model = str(entry["model"])
        pose = tuple(float(v) for v in entry.get("pose", [0.0, 0.0, 0.0, 0.0]))
        bbox = tuple(float(v) for v in entry["bbox"])
        name = str(entry.get("name", f"mesh_{index}_{model}"))
        scale = float(entry.get("scale", 1.0))
        parts.append(_mesh(name, model, pose, bbox, obstacles, scale=scale))
    return "".join(parts)


def scene_layout(
    scene_name: str,
    scene: dict[str, Any],
    *,
    model_uri: str = "model://tof_rig",
    real_time_factor: float | None = None,
    include_meshes: bool = True,
) -> tuple[str, list[dict[str, Any]]]:
    """Return the SDF world string and conservative obstacle AABBs."""
    style = str(scene.get("style", scene_name)).lower()
    bounds = scene["bounds"]
    spawn = scene.get("spawn", [0.0, 0.0, 1.0])
    x, y, z = (float(v) for v in spawn)
    if real_time_factor is None:
        real_time_factor = float(scene.get("real_time_factor", 1.0))
    obstacles: list[dict[str, Any]] = []
    content = _HEADER.format(
        name=str(scene.get("world_name", f"tio_{scene_name}")),
        real_time_factor=float(real_time_factor),
    )
    content += _walls(bounds, height=float(bounds["z"][1]), obstacles=obstacles)
    if style == "office":
        content += _office(bounds, obstacles)
    elif style == "pantry":
        content += _pantry(bounds, obstacles)
    elif style == "meeting_room":
        content += _meeting_room(bounds, obstacles)
    elif style == "corridor":
        # Columns give the corridor longitudinal structure and side clutter.
        x0, x1 = (float(v) for v in bounds["x"])
        for index, cx in enumerate([x0 + 4.0, x0 + 9.0, x0 + 14.0, x0 + 19.0, x0 + 24.0]):
            content += _box(f"column_{index}", (0.35, 0.35, 2.5), (cx, 1.15, 1.25), (0.60, 0.60, 0.58), obstacles)
            content += _box(f"column_{index}_b", (0.35, 0.35, 2.5), (cx, -1.15, 1.25), (0.60, 0.60, 0.58), obstacles)
        content += _box("bench", (1.8, 0.45, 0.45), (x0 + 7.0, 1.05, 0.23), (0.30, 0.22, 0.14), obstacles)
    else:
        raise ValueError(f"unknown scene style: {style!r}")

    if include_meshes:
        content += _meshes(scene.get("meshes", []), obstacles)

    content += f"""    <include>
      <uri>{model_uri}</uri>
      <name>tof_rig</name>
      <pose>{x} {y} {z} 0 0 0</pose>
    </include>
"""
    content += _FOOTER
    return content, obstacles


def scene_to_sdf(
    scene_name: str,
    scene: dict[str, Any],
    *,
    model_uri: str = "model://tof_rig",
    real_time_factor: float | None = None,
) -> str:
    """Render a scene configuration into an SDF world string."""
    content, _ = scene_layout(
        scene_name,
        scene,
        model_uri=model_uri,
        real_time_factor=real_time_factor,
    )
    return content


def scene_obstacles(
    scene_name: str,
    scene: dict[str, Any],
    *,
    include_meshes: bool = True,
) -> list[dict[str, Any]]:
    """Return conservative obstacle AABBs for trajectory collision checks."""
    _, obstacles = scene_layout(
        scene_name,
        scene,
        include_meshes=include_meshes,
    )
    return obstacles
