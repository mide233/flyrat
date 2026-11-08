#!/usr/bin/env python3
"""Regenerate analytic IMU windows for processed sequences.

The M2 dataset stored IMU generated from the same ground-truth trajectory, but
with very small Kalibr noise densities.  That makes the IMU-only baseline
unrealistically strong.  This script re-synthesises IMU from the stored GT
poses with the current ``configs/sim/sensors.yaml`` IMU settings, so the ToF
branch has a realistic role.  ToF arrays and metadata are preserved.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.dataset import load_sequence, save_sequence  # noqa: E402
from tio_former.data.imu import generate_imu, resample_to_tof_windows  # noqa: E402
from tio_former.sim.trajectory import Trajectory  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--processed", default="data/processed")
    parser.add_argument("--sensors-config", default="configs/sim/sensors.yaml")
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = _args()
    sensors = load_yaml(ROOT / args.sensors_config)
    imu_config = sensors["imu"]
    samples = int(imu_config.get("samples_per_tof", 16))
    files = sorted((ROOT / args.processed).rglob("*.npz"))
    if args.limit:
        files = files[: args.limit]
    print(f"regenerating IMU for {len(files)} sequences")
    for index, path in enumerate(files, 1):
        arrays, metadata = load_sequence(path)
        t = arrays["t_tof"].astype(np.float64)
        pose = arrays["gt_pose"].astype(np.float64)
        trajectory = Trajectory(t=t, position=pose[:, :3], quat=pose[:, 3:])
        seed = int(metadata.get("seed", 0)) + 7919
        imu = generate_imu(trajectory, imu_config, np.random.default_rng(seed))
        windows, window_times = resample_to_tof_windows(imu, t, samples)
        arrays["imu"] = windows.astype(np.float32)
        arrays["t_imu"] = window_times.astype(np.float64)
        metadata["imu_config"] = imu_config
        metadata["imu_regenerated"] = True
        save_sequence(path, arrays, metadata)
        if index % 25 == 0 or index == len(files):
            print(f"  {index}/{len(files)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
