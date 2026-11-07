"""Differentiable SO(3) helpers used by losses and evaluation."""

from __future__ import annotations

import torch


def skew(vectors: torch.Tensor) -> torch.Tensor:
    """[..., 3] -> [..., 3, 3] skew-symmetric matrices."""
    x, y, z = vectors.unbind(dim=-1)
    zero = torch.zeros_like(x)
    return torch.stack(
        [
            torch.stack([zero, -z, y], dim=-1),
            torch.stack([z, zero, -x], dim=-1),
            torch.stack([-y, x, zero], dim=-1),
        ],
        dim=-2,
    )


def so3_exp(omega: torch.Tensor, eps: float = 1e-8) -> torch.Tensor:
    """Rodrigues exponential map for batched axis-angle vectors."""
    theta = torch.linalg.norm(omega, dim=-1, keepdim=True).clamp_min(eps)
    axis = omega / theta
    skew_axis = skew(axis)
    sin_theta = torch.sin(theta).unsqueeze(-1)
    cos_theta = torch.cos(theta).unsqueeze(-1)
    identity = torch.eye(3, device=omega.device, dtype=omega.dtype).expand(
        omega.shape[:-1] + (3, 3)
    )
    return (
        identity
        + sin_theta * skew_axis
        + (1.0 - cos_theta) * torch.matmul(skew_axis, skew_axis)
    )


def so3_log(rotation: torch.Tensor, eps: float = 1e-7) -> torch.Tensor:
    """Batched matrix logarithm; returns axis-angle vectors."""
    trace = rotation.diagonal(dim1=-2, dim2=-1).sum(dim=-1)
    cosine = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0)
    theta = torch.acos(cosine)
    vector = torch.stack(
        [
            rotation[..., 2, 1] - rotation[..., 1, 2],
            rotation[..., 0, 2] - rotation[..., 2, 0],
            rotation[..., 1, 0] - rotation[..., 0, 1],
        ],
        dim=-1,
    )
    scale = torch.where(
        theta.abs() < eps,
        torch.full_like(theta, 0.5),
        theta / (2.0 * torch.sin(theta).clamp_min(eps)),
    )
    return vector * scale.unsqueeze(-1)


def cumulative_rotation(delta_phi: torch.Tensor) -> torch.Tensor:
    """Integrate body-frame increments into world rotations.

    Args:
        delta_phi: ``[B, T, 3]`` axis-angle increments.
    """
    rotations = so3_exp(delta_phi)
    result = []
    current = torch.eye(3, device=delta_phi.device, dtype=delta_phi.dtype).expand(
        delta_phi.shape[0], 3, 3
    )
    for index in range(delta_phi.shape[1]):
        current = torch.matmul(current, rotations[:, index])
        result.append(current)
    return torch.stack(result, dim=1)


def integrate_increments(
    delta_p: torch.Tensor,
    delta_phi: torch.Tensor,
    initial_position: torch.Tensor | None = None,
) -> torch.Tensor:
    """Integrate body-frame increments to world positions ``[B, T, 3]``."""
    batch, length, _ = delta_p.shape
    position = (
        torch.zeros(batch, 3, device=delta_p.device, dtype=delta_p.dtype)
        if initial_position is None
        else initial_position
    )
    rotation = torch.eye(3, device=delta_p.device, dtype=delta_p.dtype).expand(batch, 3, 3)
    positions = []
    for index in range(length):
        position = position + torch.matmul(rotation, delta_p[:, index].unsqueeze(-1)).squeeze(-1)
        positions.append(position)
        rotation = torch.matmul(rotation, so3_exp(delta_phi[:, index]))
    return torch.stack(positions, dim=1)
