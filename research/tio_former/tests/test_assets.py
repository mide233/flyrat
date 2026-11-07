"""Sanity checks for the research scaffold assets."""

from __future__ import annotations

from pathlib import Path

from tio_former.utils.config import load_yaml

ROOT = Path(__file__).resolve().parents[1]


def test_required_configs_exist() -> None:
    for name in (
        "sim/sensors.yaml",
        "sim/scenes.yaml",
        "sim/trajectory.yaml",
        "data/dataset.yaml",
        "data/generation.yaml",
        "model/base.yaml",
        "train/default.yaml",
        "eval/main.yaml",
        "eval/ablation.yaml",
        "eval/robustness.yaml",
    ):
        path = ROOT / "configs" / name
        assert path.is_file(), path


def test_sensor_views_and_grid() -> None:
    sensors = load_yaml(ROOT / "configs/sim/sensors.yaml")
    assert sensors["tof"]["views"] == ["front", "left", "right", "down"]
    assert sensors["tof"]["grid_width"] == 8
    assert sensors["tof"]["grid_height"] == 8
    assert sensors["tof"]["rate_hz"] == 15.0
    assert sensors["imu"]["rate_hz"] == 200.0


def test_scene_bounds_are_valid() -> None:
    scenes = load_yaml(ROOT / "configs/sim/scenes.yaml")["scenes"]
    assert set(scenes) == {"corridor", "office", "pantry", "meeting_room"}
    for scene in scenes.values():
        for axis in ("x", "y", "z"):
            low, high = scene["bounds"][axis]
            assert high > low


def test_scene_layout_has_obstacles_and_meshes() -> None:
    from tio_former.sim.scene import scene_layout

    scenes = load_yaml(ROOT / "configs/sim/scenes.yaml")["scenes"]
    sdf, obstacles = scene_layout("office", scenes["office"])
    assert "model://Desk" in sdf
    assert "model://tof_rig" in sdf
    assert obstacles
    assert all({"center", "size"} <= set(item) for item in obstacles)
    assert all(len(item["size"]) == 3 for item in obstacles)
