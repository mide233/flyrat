#!/usr/bin/env python3
"""Export a trained TIO-Former checkpoint to TorchScript for the C++ backend.

The exported module has the fixed signature used by libflyrat's
``flyrat::tio::TioOdometry``::

    forward(tof, imu, weight, previous_tof, previous_weight)
        -> (delta_p, delta_phi)

with shapes ``[1, T, 4, 8, 8]``, ``[1, T, 16, 6]`` and ``[1, T, 4, 8, 8]``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402


class ExportWrapper(torch.nn.Module):
    def __init__(self, model: TioFormer) -> None:
        super().__init__()
        self.model = model

    def forward(self, tof, imu, weight, previous_tof, previous_weight):
        output = self.model(
            tof,
            imu,
            weight=weight,
            previous_tof=previous_tof,
            previous_weight=previous_weight,
        )
        return output["delta_p"], output["delta_phi"]


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", default="checkpoints/main_imu/seed0/best.pt")
    parser.add_argument("--out", default="../../libflyrat/models/tio_former_window64.pt")
    parser.add_argument("--window", type=int, default=64)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def main() -> int:
    args = _args()
    checkpoint = (ROOT / args.checkpoint).resolve()
    output = (ROOT / args.out).resolve()
    state = torch.load(checkpoint, map_location="cpu", weights_only=False)
    config = state.get("model_config")
    if config is None:
        raise SystemExit(f"checkpoint has no model_config: {checkpoint}")
    model_config = ModelConfig.from_dict(config)
    model = TioFormer(model_config)
    model.load_state_dict(state["model"])
    model.eval()

    tof = torch.zeros(1, args.window, model_config.views, 8, 8)
    imu = torch.zeros(1, args.window, 16, 6)
    weight = torch.zeros_like(tof)
    previous_tof = torch.zeros_like(tof)
    previous_weight = torch.zeros_like(tof)

    wrapper = ExportWrapper(model).eval()
    with torch.no_grad():
        reference = wrapper(tof, imu, weight, previous_tof, previous_weight)
        traced = torch.jit.trace(
            wrapper,
            (tof, imu, weight, previous_tof, previous_weight),
            strict=False,
        )
        exported = traced(tof, imu, weight, previous_tof, previous_weight)
    difference = max(
        float((a - b).abs().max()) for a, b in zip(reference, exported)
    )
    if difference > 1e-5:
        raise SystemExit(f"trace mismatch: max abs difference {difference:.3e}")

    output.parent.mkdir(parents=True, exist_ok=True)
    traced.save(str(output))
    try:
        source_checkpoint = checkpoint.relative_to(ROOT.parents[1])
    except ValueError:
        source_checkpoint = checkpoint
    metadata = {
        "source_checkpoint": str(source_checkpoint),
        "window_frames": args.window,
        "views": model_config.views,
        "grid_size": 8,
        "imu_samples": 16,
        "channels": 6,
        "parameters": model.count_parameters(),
        "trace_max_abs_difference": difference,
        "view_order": ["front", "left", "right", "down"],
    }
    output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    print(f"wrote {output} ({output.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
