#!/usr/bin/env python3
"""Generate the four M1 primitive Gazebo worlds from ``configs/sim/scenes.yaml``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.sim.scene import scene_to_sdf  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/sim/scenes.yaml"))
    parser.add_argument("--out", default=str(ROOT / "sim/worlds/generated"))
    parser.add_argument("--scene", action="append", help="Generate only this scene (repeatable)")
    args = parser.parse_args()

    logger = get_logger("generate_worlds")
    config = load_yaml(args.config)
    scenes = config.get("scenes", {})
    if not isinstance(scenes, dict) or not scenes:
        logger.error("%s contains no scenes", args.config)
        return 2
    selected = args.scene or sorted(scenes)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in selected:
        if name not in scenes:
            logger.error("unknown scene %r", name)
            return 2
        sdf = scene_to_sdf(name, scenes[name])
        path = out_dir / f"{name}.sdf"
        path.write_text(sdf, encoding="utf-8")
        logger.info("wrote %s", path)
    logger.info(
        "set GZ_SIM_RESOURCE_PATH=%s:%s",
        ROOT / "sim/models",
        out_dir,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
