#!/usr/bin/env python3
"""Measure single-frame streaming latency of a TIO-Former checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--out", default="runs/latency.json")
    return parser.parse_args()


def main() -> int:
    args = _args()
    checkpoint = torch.load(ROOT / args.checkpoint, map_location="cpu", weights_only=False)
    config = ModelConfig.from_dict(checkpoint["model_config"])
    model = TioFormer(config).to(args.device).eval()
    model.load_state_dict(checkpoint["model"])
    views = config.views
    dtype = torch.float32
    tof = torch.rand(1, 1, views, 8, 8, device=args.device, dtype=dtype)
    weight = torch.ones_like(tof)
    previous_tof = tof.clone()
    previous_weight = weight.clone()
    imu = torch.zeros(1, 1, 16, 6, device=args.device, dtype=dtype)
    state = model.init_state(1, args.device, dtype)
    times: list[float] = []
    with torch.inference_mode():
        for step in range(args.steps + args.warmup):
            if args.device.startswith("cuda"):
                torch.cuda.synchronize()
            start = time.perf_counter()
            _, state = model.forward_stream(
                tof,
                imu,
                states=state,
                weight=weight,
                previous_tof=previous_tof,
                previous_weight=previous_weight,
            )
            if args.device.startswith("cuda"):
                torch.cuda.synchronize()
            elapsed = (time.perf_counter() - start) * 1e3
            if step >= args.warmup:
                times.append(elapsed)
    values = np.asarray(times)
    result = {
        "checkpoint": str(args.checkpoint),
        "device": args.device,
        "parameters": model.count_parameters(),
        "steps": args.steps,
        "mean_ms": float(values.mean()),
        "p50_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "max_ms": float(values.max()),
        "deadline_ms": 1000.0 / 15.0,
        "deadline_overruns": int(np.sum(values > 1000.0 / 15.0)),
    }
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
