"""Losses from the TIO-Former paper (clean-room implementation)."""

from __future__ import annotations

from typing import Any

import torch
import torch.nn.functional as F

from tio_former.models.geometry import cumulative_rotation, integrate_increments, so3_log


def smooth_l1(prediction: torch.Tensor, target: torch.Tensor, beta: float) -> torch.Tensor:
    return F.smooth_l1_loss(prediction, target, beta=beta, reduction="mean")


def step_loss(
    delta_p: torch.Tensor,
    delta_phi: torch.Tensor,
    target_p: torch.Tensor,
    target_phi: torch.Tensor,
    config: dict[str, Any],
) -> torch.Tensor:
    beta_p = float(config.get("beta_translation", 0.05))
    beta_phi = float(config.get("beta_rotation", 0.02))
    return smooth_l1(delta_p, target_p, beta_p) + smooth_l1(delta_phi, target_phi, beta_phi)


def multi_horizon_position_loss(
    delta_p: torch.Tensor,
    target_p: torch.Tensor,
    horizons: list[int],
    beta: float = 0.05,
    delta_phi: torch.Tensor | None = None,
    target_phi: torch.Tensor | None = None,
) -> torch.Tensor:
    if not horizons:
        return delta_p.new_zeros(())
    if delta_phi is None:
        delta_phi = torch.zeros_like(delta_p)
    if target_phi is None:
        target_phi = torch.zeros_like(target_p)
    predicted = integrate_increments(delta_p, delta_phi)
    target = integrate_increments(target_p, target_phi)
    values = []
    for horizon in horizons:
        if predicted.shape[1] <= horizon:
            continue
        values.append(
            smooth_l1(
                predicted[:, horizon:] - predicted[:, :-horizon],
                target[:, horizon:] - target[:, :-horizon],
                beta,
            )
        )
    return torch.stack(values).mean() if values else delta_p.new_zeros(())


def multi_horizon_rotation_loss(
    delta_phi: torch.Tensor,
    target_phi: torch.Tensor,
    horizons: list[int],
) -> torch.Tensor:
    if not horizons:
        return delta_phi.new_zeros(())
    predicted = cumulative_rotation(delta_phi)
    target = cumulative_rotation(target_phi)
    values = []
    for horizon in horizons:
        if predicted.shape[1] <= horizon:
            continue
        predicted_relative = torch.matmul(
            predicted[:, horizon:], predicted[:, :-horizon].transpose(-2, -1)
        )
        target_relative = torch.matmul(
            target[:, horizon:], target[:, :-horizon].transpose(-2, -1)
        )
        error = torch.matmul(predicted_relative.transpose(-2, -1), target_relative)
        values.append(torch.linalg.norm(so3_log(error), dim=-1).mean())
    return torch.stack(values).mean() if values else delta_phi.new_zeros(())


def scale_loss(
    delta_p: torch.Tensor,
    target_p: torch.Tensor,
    beta: float = 0.10,
) -> torch.Tensor:
    predicted_norm = torch.linalg.norm(delta_p, dim=-1).sum(dim=1)
    target_norm = torch.linalg.norm(target_p, dim=-1).sum(dim=1)
    ratio = predicted_norm / target_norm.clamp_min(1e-6)
    return smooth_l1(ratio, torch.ones_like(ratio), beta)


def residual_loss(
    delta_p: torch.Tensor,
    target_p: torch.Tensor,
    beta: float = 0.02,
) -> torch.Tensor:
    if delta_p.shape[1] < 2:
        return delta_p.new_zeros(())
    return smooth_l1(delta_p[:, 1:] - delta_p[:, :-1], target_p[:, 1:] - target_p[:, :-1], beta)


def compute_losses(
    output: dict[str, torch.Tensor],
    target_p: torch.Tensor,
    target_phi: torch.Tensor,
    config: dict[str, Any],
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    """Return total loss and a detached breakdown."""
    delta_p = output["delta_p"]
    delta_phi = output["delta_phi"]
    root = config.get("loss", config)
    step_config = root.get("step", {})
    horizon_config = root.get("multi_horizon", {})
    scale_config = root.get("scale", {})
    residual_config = root.get("residual", {})

    step = step_loss(delta_p, delta_phi, target_p, target_phi, step_config)
    horizon_p = multi_horizon_position_loss(
        delta_p,
        target_p,
        [int(value) for value in horizon_config.get("position_horizons", [4, 8, 16, 32])],
        float(step_config.get("beta_translation", 0.05)),
        delta_phi=delta_phi,
        target_phi=target_phi,
    )
    horizon_phi = multi_horizon_rotation_loss(
        delta_phi,
        target_phi,
        [int(value) for value in horizon_config.get("rotation_horizons", [4, 8, 16])],
    )
    scale = scale_loss(
        delta_p,
        target_p,
        float(scale_config.get("beta", 0.10)),
    )
    residual = residual_loss(
        delta_p,
        target_p,
        float(residual_config.get("beta", 0.02)),
    )
    total = (
        step
        + float(horizon_config.get("position_weight", 0.02)) * horizon_p
        + float(horizon_config.get("rotation_weight", 0.005)) * horizon_phi
        + float(scale_config.get("weight", 0.005)) * scale
        + float(residual_config.get("weight", 0.005)) * residual
    )
    breakdown = {
        "loss": total.detach(),
        "step": step.detach(),
        "horizon_position": horizon_p.detach(),
        "horizon_rotation": horizon_phi.detach(),
        "scale": scale.detach(),
        "residual": residual.detach(),
    }
    return total, breakdown
