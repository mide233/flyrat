#!/usr/bin/env python3
"""Train the clean-room TIO-Former model.

Single-GPU training with optional bfloat16 autocast, gradient accumulation,
cosine schedule, validation and early stopping.  The script is intentionally
small so the research code stays easy to audit; launch it with ``torchrun``
later if multi-GPU training is needed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.data.torch_dataset import SequenceWindows, iterate_batches  # noqa: E402
from tio_former.models.losses import compute_losses  # noqa: E402
from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402
from tio_former.utils.logging import get_logger  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-config", default="configs/train/default.yaml")
    parser.add_argument("--model-config", default="configs/model/base.yaml")
    parser.add_argument("--processed", default="data/processed")
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--out", default="checkpoints/train")
    parser.add_argument("--run-name", default="default")
    parser.add_argument("--limit-train", type=int, default=None)
    parser.add_argument("--limit-val", type=int, default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument(
        "--exclude-scenes",
        default="",
        help="comma-separated scene names to exclude from train/val",
    )
    parser.add_argument(
        "--variant",
        default="full",
        help="full | imu_only | tof_only | naive_concat | memoryless | kv_only_no_fifo | hybrid | gated_fifo | hybrid_gated | hybrid_split",
    )
    return parser.parse_args()


def _lr_lambda(step: int, *, warmup_steps: int, total_steps: int, min_ratio: float) -> float:
    if warmup_steps > 0 and step < warmup_steps:
        return (step + 1) / warmup_steps
    progress = min(1.0, max(0.0, (step - warmup_steps) / max(1, total_steps - warmup_steps)))
    cosine = 0.5 * (1.0 + math.cos(math.pi * progress))
    return min_ratio + (1.0 - min_ratio) * cosine


@torch.no_grad()
def _validate(model, loader, loss_config, device, amp_dtype) -> float:
    model.eval()
    total_loss = 0.0
    batches = 0
    for batch in loader:
        batch = {key: value.to(device) for key, value in batch.items()}
        with torch.autocast(device_type="cuda", dtype=amp_dtype, enabled=amp_dtype is not None):
            output = model(
                batch["tof"],
                batch["imu"],
                weight=batch["weight"],
                previous_tof=batch["previous_tof"],
                previous_weight=batch["previous_weight"],
            )
        loss, _ = compute_losses(
            {key: value.float() for key, value in output.items()},
            batch["delta_p"].float(),
            batch["delta_phi"].float(),
            loss_config,
        )
        total_loss += float(loss)
        batches += 1
    model.train()
    return total_loss / max(1, batches)


def main() -> int:
    args = _args()
    logger = get_logger("train")
    train_config = load_yaml(ROOT / args.train_config)
    model_dict = load_yaml(ROOT / args.model_config)
    model_dict.setdefault("model", {})["variant"] = args.variant
    model_config = ModelConfig.from_dict(model_dict)
    seed = int(args.seed if args.seed is not None else train_config.get("seed", 0))
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

    data_config = train_config.get("data", {})
    training_config = train_config.get("training", {})
    optimizer_config = train_config.get("optimizer", {})
    scheduler_config = train_config.get("scheduler", {})
    augmentation = train_config.get("augmentation", {})
    loss_config = train_config.get("loss", train_config)

    batch_size = int(args.batch_size or data_config.get("batch_size", 32))
    workers = int(args.workers if args.workers is not None else data_config.get("num_workers", 8))
    epochs = int(args.epochs or training_config.get("epochs", 100))
    grad_accum = int(training_config.get("grad_accum", 1))
    clip = float(training_config.get("grad_clip", 1.0))
    window = int(data_config.get("window_frames", 64))
    stride = max(1, window // 2)
    missing_ratios = augmentation.get("missing_ratios", [0.1, 0.3, 0.5])
    exclude_scenes = [item for item in args.exclude_scenes.split(",") if item]

    train_set = SequenceWindows(
        ROOT / args.processed,
        split="train",
        window_frames=window,
        stride=stride,
        seed=seed,
        augment=bool(augmentation.get("enabled", True)),
        missing_probability=float(augmentation.get("missing_probability", 0.6)),
        missing_ratios=list(missing_ratios),
        max_sequences=args.limit_train,
        exclude_scenes=exclude_scenes or None,
    )
    val_set = SequenceWindows(
        ROOT / args.processed,
        split="val",
        window_frames=window,
        stride=window,
        seed=seed + 1,
        augment=False,
        max_sequences=args.limit_val,
        exclude_scenes=exclude_scenes or None,
    )
    logger.info("dataset: %d train windows, %d val windows", len(train_set), len(val_set))

    train_loader = iter(
        iterate_batches(
            train_set,
            batch_size,
            shuffle=True,
            generator=torch.Generator().manual_seed(seed),
            num_workers=workers,
            pin_memory=args.device.startswith("cuda"),
            persistent_workers=workers > 0,
        )
    )
    val_loader = None
    steps_per_epoch = max(1, math.ceil(len(train_set) / batch_size))
    total_steps = steps_per_epoch * epochs
    warmup_steps = int(total_steps * float(scheduler_config.get("warmup_ratio", 0.05)))

    model = TioFormer(model_config).to(args.device)
    logger.info("model: %d parameters on %s", model.count_parameters(), args.device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(optimizer_config.get("lr", 1e-3)),
        weight_decay=float(optimizer_config.get("weight_decay", 1e-4)),
        betas=tuple(optimizer_config.get("betas", [0.9, 0.999])),
    )
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer,
        lambda step: _lr_lambda(
            step,
            warmup_steps=warmup_steps,
            total_steps=total_steps,
            min_ratio=float(scheduler_config.get("min_lr_ratio", 0.01)),
        ),
    )
    amp_dtype = None
    if not args.no_amp and str(training_config.get("amp", "bf16")).lower() in {"bf16", "bfloat16"}:
        amp_dtype = torch.bfloat16
    device_type = "cuda" if str(args.device).startswith("cuda") else "cpu"
    if device_type != "cuda":
        amp_dtype = None

    out_dir = (ROOT / args.out / args.run_name).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "config.json").write_text(
        json.dumps({"train": train_config, "model": model_config.to_dict()}, indent=2),
        encoding="utf-8",
    )
    metrics_path = out_dir / "metrics.jsonl"
    best_val = float("inf")
    best_epoch = -1
    stale_epochs = 0
    early_stop = int(training_config.get("early_stop_patience", 15))
    global_step = 0
    started = time.time()

    for epoch in range(1, epochs + 1):
        model.train()
        epoch_loss = 0.0
        optimizer.zero_grad(set_to_none=True)
        for step_in_epoch in range(1, steps_per_epoch + 1):
            try:
                batch = next(train_loader)
            except StopIteration:
                train_loader = iter(
                    iterate_batches(
                        train_set,
                        batch_size,
                        shuffle=True,
                        generator=torch.Generator().manual_seed(seed + epoch),
                        num_workers=workers,
                        pin_memory=args.device.startswith("cuda"),
                        persistent_workers=workers > 0,
                    )
                )
                batch = next(train_loader)
            batch = {key: value.to(args.device, non_blocking=True) for key, value in batch.items()}
            with torch.autocast(device_type="cuda", dtype=amp_dtype, enabled=amp_dtype is not None):
                output = model(
                    batch["tof"],
                    batch["imu"],
                    weight=batch["weight"],
                    previous_tof=batch["previous_tof"],
                    previous_weight=batch["previous_weight"],
                )
            loss, _ = compute_losses(
                {key: value.float() for key, value in output.items()},
                batch["delta_p"].float(),
                batch["delta_phi"].float(),
                loss_config,
            )
            scaled_loss = loss / grad_accum
            if not torch.isfinite(scaled_loss):
                logger.error("non-finite loss at epoch %d step %d", epoch, step_in_epoch)
                return 2
            scaled_loss.backward()
            epoch_loss += float(loss.detach())
            if step_in_epoch % grad_accum == 0 or step_in_epoch == steps_per_epoch:
                if clip > 0:
                    torch.nn.utils.clip_grad_norm_(model.parameters(), clip)
                optimizer.step()
                scheduler.step()
                optimizer.zero_grad(set_to_none=True)
                global_step += 1
                if global_step % int(training_config.get("log_every", 50)) == 0:
                    logger.info(
                        "epoch %d step %d/%d | loss %.6f | lr %.3e | %.1f min",
                        epoch,
                        step_in_epoch,
                        steps_per_epoch,
                        epoch_loss / step_in_epoch,
                        optimizer.param_groups[0]["lr"],
                        (time.time() - started) / 60,
                    )
                if args.max_steps is not None and global_step >= args.max_steps:
                    break

        val_loss = _validate(
            model,
            iterate_batches(
                val_set,
                batch_size,
                shuffle=False,
                num_workers=workers,
                pin_memory=args.device.startswith("cuda"),
                persistent_workers=workers > 0,
            ),
            loss_config,
            args.device,
            amp_dtype,
        )
        record = {
            "epoch": epoch,
            "global_step": global_step,
            "train_loss": epoch_loss / steps_per_epoch,
            "val_loss": val_loss,
            "lr": optimizer.param_groups[0]["lr"],
            "elapsed_s": time.time() - started,
        }
        with metrics_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
        logger.info(
            "epoch %d | train %.6f | val %.6f | lr %.3e",
            epoch,
            record["train_loss"],
            val_loss,
            record["lr"],
        )
        state = {
            "model": model.state_dict(),
            "model_config": model_config.to_dict(),
            "train_config": train_config,
            "epoch": epoch,
            "global_step": global_step,
            "val_loss": val_loss,
            "seed": seed,
        }
        torch.save(state, out_dir / "last.pt")
        if val_loss < best_val:
            best_val = val_loss
            best_epoch = epoch
            stale_epochs = 0
            torch.save(state, out_dir / "best.pt")
        else:
            stale_epochs += 1
        if args.max_steps is not None and global_step >= args.max_steps:
            logger.info("reached max steps %d", global_step)
            break
        if early_stop > 0 and stale_epochs >= early_stop:
            logger.info("early stopping after %d stale epochs", stale_epochs)
            break

    logger.info("done: best epoch %d val %.6f -> %s", best_epoch, best_val, out_dir / "best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
