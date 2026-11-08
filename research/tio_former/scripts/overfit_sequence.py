#!/usr/bin/env python3
"""Overfit the TIO-Former model to one short sequence.

This is the M3 smoke test for the training path: it verifies that the model,
losses and data windows connect correctly and that the loss decreases.  The
checkpoint is only a debugging artifact and is written under gitignored
``checkpoints/``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.torch_dataset import SequenceWindows  # noqa: E402
from tio_former.models.losses import compute_losses  # noqa: E402
from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sequence",
        default="data/processed/train_corridor_0000.npz",
        help="Processed NPZ sequence used for overfitting",
    )
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--window", type=int, default=64)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--log-every", type=int, default=20)
    parser.add_argument("--out", default="checkpoints/overfit_sequence.pt")
    parser.add_argument("--model-config", default="configs/model/base.yaml")
    parser.add_argument("--train-config", default="configs/train/default.yaml")
    return parser.parse_args()


def main() -> int:
    args = _args()
    logger = get_logger("overfit")
    sequence = (ROOT / args.sequence).resolve()
    if not sequence.exists():
        logger.error("sequence not found: %s", sequence)
        return 2

    dataset = SequenceWindows(
        sequence.parent,
        files=[sequence],
        window_frames=args.window,
        stride=args.window,
    )
    sample = dataset[0]
    window_frames = int(sample["tof"].shape[0])
    batch = {key: value.unsqueeze(0).to(args.device) for key, value in sample.items()}

    model_config = ModelConfig.from_dict(load_yaml(ROOT / args.model_config))
    train_config = load_yaml(ROOT / args.train_config)
    loss_config = train_config.get("loss", train_config)
    model = TioFormer(model_config).to(args.device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.lr,
        weight_decay=float(train_config.get("optimizer", {}).get("weight_decay", 1e-4)),
    )
    logger.info(
        "overfitting %s | %d frames | %d params | device=%s",
        sequence.name,
        window_frames,
        model.count_parameters(),
        args.device,
    )

    model.train()
    history: list[float] = []
    for step in range(1, args.steps + 1):
        optimizer.zero_grad(set_to_none=True)
        output = model(
            batch["tof"],
            batch["imu"],
            weight=batch["weight"],
            previous_tof=batch["previous_tof"],
            previous_weight=batch["previous_weight"],
        )
        total, breakdown = compute_losses(
            output,
            batch["delta_p"],
            batch["delta_phi"],
            loss_config,
        )
        if not torch.isfinite(total):
            logger.error("non-finite loss at step %d: %s", step, breakdown)
            return 2
        total.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        history.append(float(total.detach()))
        if step == 1 or step % args.log_every == 0 or step == args.steps:
            logger.info(
                "step %4d | total %.6f | %s",
                step,
                float(total.detach()),
                " ".join(f"{key}={float(value):.6f}" for key, value in breakdown.items()),
            )

    out_path = (ROOT / args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model": model.state_dict(),
            "model_config": model_config.to_dict(),
            "sequence": str(sequence),
            "steps": args.steps,
            "first_loss": history[0],
            "last_loss": history[-1],
        },
        out_path,
    )
    improvement = 1.0 - history[-1] / max(history[0], 1e-12)
    logger.info("first=%.6f last=%.6f improvement=%.2f%%", history[0], history[-1], 100 * improvement)
    logger.info("wrote %s", out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
