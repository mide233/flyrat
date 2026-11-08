"""Clean-room TIO-Former model.

The paper does not release code, so this module follows the text: a shared 2D
range CNN, a view transformer, an IMU encoder, IMU-guided directional
cross-attention, a bounded streaming causal Transformer and an incremental
pose head.  All choices that the paper leaves open are exposed through the
configuration file and flagged in ``configs/model/base.yaml``.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import torch
from torch import nn
import torch.nn.functional as F


def _as_int(value: Any, default: int) -> int:
    return int(default if value is None else value)


@dataclass
class ModelConfig:
    views: int = 4
    variant: str = "full"
    tof_channels: int = 3
    tof_conv_channels: tuple[int, ...] = (32, 64, 128)
    feature_dim: int = 128
    view_layers: int = 2
    view_heads: int = 4
    view_ffn_dim: int = 256
    view_dropout: float = 0.1
    view_embedding: bool = True
    imu_channels: int = 6
    imu_conv_channels: tuple[int, ...] = (64, 128, 128)
    imu_kernel_size: int = 5
    fusion_heads: int = 4
    fusion_dropout: float = 0.1
    streaming_layers: int = 2
    streaming_heads: int = 4
    streaming_ffn_dim: int = 256
    streaming_dropout: float = 0.1
    local_capacity: int = 8
    chunk_size: int = 4
    fifo_capacity: int = 4
    gated_summary: bool = False
    summary_hidden_dim: int = 256
    pose_hidden_dim: int = 128
    pose_dropout: float = 0.0

    @classmethod
    def from_dict(cls, config: dict[str, Any]) -> "ModelConfig":
        model = config.get("model", config)
        tof = model.get("tof", {})
        imu = model.get("imu", {})
        fusion = model.get("fusion", {})
        streaming = model.get("streaming", {})
        pose = model.get("pose_head", {})
        return cls(
            views=_as_int(model.get("views"), 4),
            variant=str(model.get("variant", "full")),
            tof_channels=_as_int(tof.get("input_channels"), 3),
            tof_conv_channels=tuple(tof.get("conv_channels", (32, 64, 128))),
            feature_dim=_as_int(tof.get("feature_dim"), 128),
            view_layers=_as_int(tof.get("view_layers"), 2),
            view_heads=_as_int(tof.get("view_heads"), 4),
            view_ffn_dim=_as_int(tof.get("view_ffn_dim"), 256),
            view_dropout=float(tof.get("view_dropout", 0.1)),
            view_embedding=bool(tof.get("view_embedding", True)),
            imu_channels=_as_int(imu.get("input_channels"), 6),
            imu_conv_channels=tuple(imu.get("conv_channels", (64, 128, 128))),
            imu_kernel_size=_as_int(imu.get("kernel_size"), 5),
            fusion_heads=_as_int(fusion.get("heads"), 4),
            fusion_dropout=float(fusion.get("dropout", 0.1)),
            streaming_layers=_as_int(streaming.get("layers"), 2),
            streaming_heads=_as_int(streaming.get("heads"), 4),
            streaming_ffn_dim=_as_int(streaming.get("ffn_dim"), 256),
            streaming_dropout=float(streaming.get("dropout", 0.1)),
            local_capacity=_as_int(streaming.get("local_capacity"), 8),
            chunk_size=_as_int(streaming.get("chunk_size"), 4),
            fifo_capacity=_as_int(streaming.get("fifo_capacity"), 4),
            gated_summary=bool(streaming.get("gated_summary", False)),
            summary_hidden_dim=_as_int(streaming.get("summary_hidden_dim"), 256),
            pose_hidden_dim=_as_int(pose.get("hidden_dim"), 128),
            pose_dropout=float(pose.get("dropout", 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        """Serializable nested config matching ``configs/model/base.yaml``."""
        return {
            "model": {
                "variant": self.variant,
                "tof": {
                    "input_channels": self.tof_channels,
                    "conv_channels": list(self.tof_conv_channels),
                    "feature_dim": self.feature_dim,
                    "view_layers": self.view_layers,
                    "view_heads": self.view_heads,
                    "view_ffn_dim": self.view_ffn_dim,
                    "view_dropout": self.view_dropout,
                    "view_embedding": self.view_embedding,
                },
                "imu": {
                    "input_channels": self.imu_channels,
                    "conv_channels": list(self.imu_conv_channels),
                    "kernel_size": self.imu_kernel_size,
                },
                "fusion": {
                    "heads": self.fusion_heads,
                    "dropout": self.fusion_dropout,
                },
                "streaming": {
                    "layers": self.streaming_layers,
                    "heads": self.streaming_heads,
                    "ffn_dim": self.streaming_ffn_dim,
                    "dropout": self.streaming_dropout,
                    "local_capacity": self.local_capacity,
                    "chunk_size": self.chunk_size,
                    "fifo_capacity": self.fifo_capacity,
                    "gated_summary": self.gated_summary,
                    "summary_hidden_dim": self.summary_hidden_dim,
                },
                "pose_head": {
                    "hidden_dim": self.pose_hidden_dim,
                    "dropout": self.pose_dropout,
                },
            }
        }


class RangeEncoder(nn.Module):
    """Shared 2D CNN applied to every directional range image."""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        in_channels = config.tof_channels
        for out_channels in config.tof_conv_channels:
            layers.extend(
                [
                    nn.Conv2d(in_channels, out_channels, 3, padding=1),
                    nn.GroupNorm(min(8, out_channels), out_channels),
                    nn.GELU(),
                ]
            )
            in_channels = out_channels
        self.cnn = nn.Sequential(*layers)
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.out = nn.Linear(in_channels, config.feature_dim)

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        # images: [N, C, H, W]
        features = self.cnn(images)
        return self.out(self.pool(features).flatten(1))


class ImuEncoder(nn.Module):
    """Three-layer 1D CNN over a fixed IMU window."""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        in_channels = config.imu_channels
        kernel = config.imu_kernel_size
        for out_channels in config.imu_conv_channels:
            layers.extend(
                [
                    nn.Conv1d(in_channels, out_channels, kernel, padding=kernel // 2),
                    nn.GELU(),
                ]
            )
            in_channels = out_channels
        self.cnn = nn.Sequential(*layers)
        self.out = nn.Linear(in_channels, config.feature_dim)

    def forward(self, imu: torch.Tensor) -> torch.Tensor:
        # imu: [N, 6, K]
        features = self.cnn(imu)
        return self.out(features.mean(dim=-1))


class ViewTransformer(nn.Module):
    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        layer = nn.TransformerEncoderLayer(
            d_model=config.feature_dim,
            nhead=config.view_heads,
            dim_feedforward=config.view_ffn_dim,
            dropout=config.view_dropout,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(layer, num_layers=config.view_layers)
        self.norm = nn.LayerNorm(config.feature_dim)

    def forward(self, tokens: torch.Tensor) -> torch.Tensor:
        return self.norm(self.encoder(tokens))


class CrossViewFusion(nn.Module):
    """IMU query attends to the directional range tokens.

    With ``hybrid=True`` the attention branch is added to a direct
    concatenation branch through a learnable gate.  Initialising the gate near
    zero makes the hybrid start as the stronger naive-concat baseline while
    still allowing the model to learn directional attention.
    """

    def __init__(self, config: ModelConfig, hybrid: bool = False) -> None:
        super().__init__()
        self.hybrid = hybrid
        self.attention = nn.MultiheadAttention(
            embed_dim=config.feature_dim,
            num_heads=config.fusion_heads,
            dropout=config.fusion_dropout,
            batch_first=True,
        )
        self.norm = nn.LayerNorm(config.feature_dim)
        if hybrid:
            self.concat_proj = nn.Linear(2 * config.feature_dim, config.feature_dim)
            self.out_norm = nn.LayerNorm(config.feature_dim)
            self.attention_gate = nn.Parameter(torch.tensor(-6.0))

    def forward(self, imu_token: torch.Tensor, view_tokens: torch.Tensor) -> torch.Tensor:
        query = imu_token.unsqueeze(1)
        fused, _ = self.attention(query, view_tokens, view_tokens, need_weights=False)
        attention_out = self.norm(query + fused).squeeze(1)
        if not self.hybrid:
            return attention_out
        concat = self.concat_proj(
            torch.cat([view_tokens.mean(dim=1), imu_token], dim=-1)
        )
        fused_out = concat + torch.sigmoid(self.attention_gate) * attention_out
        return self.out_norm(fused_out)


@dataclass
class LayerState:
    """Inference cache for one streaming layer."""

    local_k: torch.Tensor
    local_v: torch.Tensor
    summary_k: torch.Tensor
    summary_v: torch.Tensor

    @classmethod
    def empty(
        cls, batch: int, dim: int, device: torch.device, dtype: torch.dtype, heads: int = 1
    ) -> "LayerState":
        if dim % heads != 0:
            raise ValueError("dim must be divisible by heads")
        empty = torch.empty(batch, 0, heads, dim // heads, device=device, dtype=dtype)
        return cls(empty, empty.clone(), empty.clone(), empty.clone())


class BoundedCausalLayer(nn.Module):
    """Causal self-attention with a local KV cache and a chunk FIFO.

    ``forward`` processes a whole sequence with the same eviction schedule as
    the streaming path.  ``forward_step`` updates an explicit cache one frame
    at a time.  Both must produce identical outputs for identical weights.
    """

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.dim = config.feature_dim
        self.heads = config.streaming_heads
        if self.dim % self.heads != 0:
            raise ValueError("feature_dim must be divisible by streaming_heads")
        self.head_dim = self.dim // self.heads
        self.local_capacity = config.local_capacity
        self.chunk_size = config.chunk_size
        self.fifo_capacity = config.fifo_capacity
        self.gated_summary = bool(config.gated_summary)
        self.summary_gate = (
            nn.Parameter(torch.tensor(-6.0)) if self.gated_summary else None
        )
        self.qkv = nn.Linear(self.dim, 3 * self.dim)
        self.out = nn.Linear(self.dim, self.dim)
        self.phi = nn.Sequential(
            nn.Linear(2 * self.dim, config.summary_hidden_dim),
            nn.GELU(),
            nn.Linear(config.summary_hidden_dim, 2 * self.dim),
        )
        self.norm1 = nn.LayerNorm(self.dim)
        self.norm2 = nn.LayerNorm(self.dim)
        self.mlp = nn.Sequential(
            nn.Linear(self.dim, config.streaming_ffn_dim),
            nn.GELU(),
            nn.Dropout(config.streaming_dropout),
            nn.Linear(config.streaming_ffn_dim, self.dim),
        )
        self.dropout = nn.Dropout(config.streaming_dropout)

    def _split_qkv(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        # x: [B, N, D] -> [B, N, H, Dh]
        batch, length, _ = x.shape
        qkv = self.qkv(x).reshape(batch, length, 3, self.heads, self.head_dim)
        q, k, v = qkv.unbind(dim=2)
        return q, k, v

    def _attend(
        self, q: torch.Tensor, k: torch.Tensor, v: torch.Tensor
    ) -> torch.Tensor:
        # q: [B, 1, H, Dh]; k/v: [B, L, H, Dh]
        q = q.transpose(1, 2)  # [B, H, 1, Dh]
        k = k.transpose(1, 2)  # [B, H, L, Dh]
        v = v.transpose(1, 2)
        scores = torch.matmul(q, k.transpose(-2, -1)) / (self.head_dim**0.5)
        attention = torch.softmax(scores, dim=-1)
        return torch.matmul(attention, v).transpose(1, 2).reshape(q.shape[0], 1, self.dim)

    def _summary(self, k: torch.Tensor, v: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        # k/v: [B, G, H, Dh] -> pooled over the chunk in the full feature space.
        batch = k.shape[0]
        pooled = torch.cat(
            [k.reshape(batch, k.shape[1], -1).mean(dim=1), v.reshape(batch, v.shape[1], -1).mean(dim=1)],
            dim=-1,
        )
        summary_k, summary_v = self.phi(pooled).chunk(2, dim=-1)
        return (
            summary_k.reshape(batch, 1, self.heads, self.head_dim),
            summary_v.reshape(batch, 1, self.heads, self.head_dim),
        )

    def _finish(self, x: torch.Tensor, attention: torch.Tensor) -> torch.Tensor:
        x = x + self.dropout(self.out(attention))
        x = x + self.dropout(self.mlp(self.norm2(x)))
        return x

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Process a full sequence with the streaming eviction schedule."""
        batch, length, _ = x.shape
        q, k, v = self._split_qkv(self.norm1(x))
        outputs = []
        c = self.local_capacity
        g = self.chunk_size
        # Chunk summaries only depend on the chunk itself, so compute each
        # full chunk exactly once instead of at every later timestep.
        chunk_summaries: list[tuple[torch.Tensor, torch.Tensor]] = []
        for chunk_start in range(0, length // g * g, g):
            chunk_summaries.append(
                self._summary(
                    k[:, chunk_start : chunk_start + g],
                    v[:, chunk_start : chunk_start + g],
                )
            )
        for t in range(length):
            if c <= 0:
                start = t
                num_chunks = 0
            elif t < c:
                start = 0
                num_chunks = 0
            else:
                start = g * (1 + (t - c) // g)
                num_chunks = start // g
            first_chunk = max(0, num_chunks - self.fifo_capacity)
            summary_keys = [item[0] for item in chunk_summaries[first_chunk:num_chunks]]
            summary_values = [item[1] for item in chunk_summaries[first_chunk:num_chunks]]
            local_k = k[:, start : t + 1]
            local_v = v[:, start : t + 1]
            query = q[:, t : t + 1]
            if self.gated_summary:
                context = self._attend(query, local_k, local_v)
                if summary_keys:
                    summary_context = self._attend(
                        query,
                        torch.cat(summary_keys, dim=1),
                        torch.cat(summary_values, dim=1),
                    )
                    context = context + torch.sigmoid(self.summary_gate) * summary_context
                outputs.append(context)
            else:
                context_k = torch.cat(summary_keys + [local_k], dim=1)
                context_v = torch.cat(summary_values + [local_v], dim=1)
                outputs.append(self._attend(query, context_k, context_v))
        return self._finish(x, torch.cat(outputs, dim=1))

    def forward_step(
        self, x: torch.Tensor, state: LayerState
    ) -> tuple[torch.Tensor, LayerState]:
        """One causal step; updates and returns the cache."""
        if x.shape[1] != 1:
            raise ValueError("forward_step expects exactly one frame")
        q, k, v = self._split_qkv(self.norm1(x))
        if self.local_capacity <= 0:
            local_k, local_v = k, v
            summary_k = state.summary_k[:, :0]
            summary_v = state.summary_v[:, :0]
        else:
            local_k = torch.cat([state.local_k, k], dim=1)
            local_v = torch.cat([state.local_v, v], dim=1)
            summary_k = state.summary_k
            summary_v = state.summary_v
            if local_k.shape[1] > self.local_capacity:
                sk, sv = self._summary(
                    local_k[:, : self.chunk_size], local_v[:, : self.chunk_size]
                )
                local_k = local_k[:, self.chunk_size :]
                local_v = local_v[:, self.chunk_size :]
                summary_k = torch.cat([summary_k, sk], dim=1)
                summary_v = torch.cat([summary_v, sv], dim=1)
                if self.fifo_capacity <= 0:
                    summary_k = summary_k[:, :0]
                    summary_v = summary_v[:, :0]
                elif summary_k.shape[1] > self.fifo_capacity:
                    summary_k = summary_k[:, -self.fifo_capacity :]
                    summary_v = summary_v[:, -self.fifo_capacity :]
        if self.gated_summary:
            attention = self._attend(q, local_k, local_v)
            if summary_k.shape[1] > 0:
                summary_attention = self._attend(q, summary_k, summary_v)
                attention = attention + torch.sigmoid(self.summary_gate) * summary_attention
        else:
            context_k = torch.cat([summary_k, local_k], dim=1)
            context_v = torch.cat([summary_v, local_v], dim=1)
            attention = self._attend(q, context_k, context_v)
        out = self._finish(x, attention)
        return out, LayerState(local_k, local_v, summary_k, summary_v)


class StreamingTransformer(nn.Module):
    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.heads = config.streaming_heads
        self.layers = nn.ModuleList([BoundedCausalLayer(config) for _ in range(config.streaming_layers)])

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        for layer in self.layers:
            x = layer(x)
        return x

    def init_state(
        self, batch: int, dim: int, device: torch.device, dtype: torch.dtype
    ) -> list[LayerState]:
        return [LayerState.empty(batch, dim, device, dtype, heads=self.heads) for _ in self.layers]

    def forward_step(
        self, x: torch.Tensor, states: list[LayerState]
    ) -> tuple[torch.Tensor, list[LayerState]]:
        outputs = []
        for layer, state in zip(self.layers, states):
            x, state = layer.forward_step(x, state)
            outputs.append(state)
        return x, outputs


class PoseHead(nn.Module):
    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.LayerNorm(config.feature_dim),
            nn.Linear(config.feature_dim, config.pose_hidden_dim),
            nn.GELU(),
            nn.Dropout(config.pose_dropout),
            nn.Linear(config.pose_hidden_dim, 6),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class TioFormer(nn.Module):
    """End-to-end model producing body-frame incremental motion."""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        valid_variants = {
            "full",
            "imu_only",
            "tof_only",
            "naive_concat",
            "memoryless",
            "kv_only_no_fifo",
            "hybrid",
            "gated_fifo",
            "hybrid_gated",
            "hybrid_split",
        }
        if config.variant not in valid_variants:
            raise ValueError(f"unknown model variant {config.variant!r}")
        if config.variant == "memoryless":
            config = replace(config, local_capacity=0, fifo_capacity=0)
        elif config.variant == "kv_only_no_fifo":
            config = replace(config, fifo_capacity=0)
        elif config.variant in {"gated_fifo", "hybrid_gated"}:
            config = replace(config, gated_summary=True)
        self.config = config
        self.variant = config.variant
        self.range_encoder = (
            None if config.variant == "imu_only" else RangeEncoder(config)
        )
        self.view_embedding = (
            nn.Parameter(torch.zeros(1, config.views, config.feature_dim))
            if config.view_embedding and config.variant != "imu_only"
            else None
        )
        if self.view_embedding is not None:
            nn.init.normal_(self.view_embedding, std=0.02)
        self.view_transformer = (
            None if config.variant == "imu_only" else ViewTransformer(config)
        )
        self.imu_encoder = (
            None if config.variant == "tof_only" else ImuEncoder(config)
        )
        self.fusion = (
            CrossViewFusion(
                config,
                hybrid=config.variant in {"hybrid", "hybrid_gated"},
            )
            if config.variant
            in {
                "full",
                "memoryless",
                "kv_only_no_fifo",
                "hybrid",
                "gated_fifo",
                "hybrid_gated",
                "hybrid_split",
            }
            else None
        )
        self.concat_proj = (
            nn.Linear(2 * config.feature_dim, config.feature_dim)
            if config.variant in {"naive_concat", "hybrid_split"}
            else None
        )
        self.streaming = StreamingTransformer(config)
        self.pose_head = PoseHead(config)
        self.streaming_pos = (
            StreamingTransformer(config) if config.variant == "hybrid_split" else None
        )
        self.pose_head_pos = (
            PoseHead(config) if config.variant == "hybrid_split" else None
        )

    @classmethod
    def from_dict(cls, config: dict[str, Any]) -> "TioFormer":
        return cls(ModelConfig.from_dict(config))

    def init_state(
        self, batch_size: int, device: torch.device, dtype: torch.dtype
    ) -> list[LayerState] | tuple[list[LayerState], list[LayerState]]:
        if self.variant == "hybrid_split":
            return (
                self.streaming_pos.init_state(
                    batch_size, self.config.feature_dim, device, dtype
                ),
                self.streaming.init_state(
                    batch_size, self.config.feature_dim, device, dtype
                ),
            )
        return self.streaming.init_state(batch_size, self.config.feature_dim, device, dtype)

    def count_parameters(self) -> int:
        return sum(parameter.numel() for parameter in self.parameters() if parameter.requires_grad)

    def _range_channels(
        self,
        tof: torch.Tensor,
        weight: torch.Tensor | None,
        previous: torch.Tensor | None = None,
        previous_weight: torch.Tensor | None = None,
    ) -> torch.Tensor:
        # tof: [B, T, V, H, W]; returns [B, T, V, 3, H, W].
        batch, length, views, height, width = tof.shape
        if previous is None:
            previous = torch.cat([tof[:, :1], tof[:, :-1]], dim=1)
        if weight is None:
            current_mask = torch.ones_like(tof)
            if previous_weight is None:
                previous_mask = torch.ones_like(tof)
            else:
                previous_mask = (previous_weight > 0).to(tof.dtype)
        else:
            current_mask = (weight > 0).to(tof.dtype)
            if previous_weight is None:
                previous_mask = torch.cat([current_mask[:, :1], current_mask[:, :-1]], dim=1)
            else:
                previous_mask = (previous_weight > 0).to(tof.dtype)
        current = tof * current_mask
        previous = previous * previous_mask
        delta = (current - previous) * current_mask * previous_mask
        return torch.stack([current, previous, delta], dim=3)

    def _view_tokens(
        self,
        tof: torch.Tensor,
        weight: torch.Tensor | None,
        previous_tof: torch.Tensor | None,
        previous_weight: torch.Tensor | None,
    ) -> torch.Tensor:
        batch, length, views, height, width = tof.shape
        channels = self._range_channels(tof, weight, previous_tof, previous_weight)
        flat = channels.reshape(batch * length * views, 3, height, width)
        tokens = self.range_encoder(flat).reshape(batch * length, views, -1)
        if self.view_embedding is not None:
            tokens = tokens + self.view_embedding[:, :views]
        return self.view_transformer(tokens)

    def _imu_tokens(self, imu: torch.Tensor) -> torch.Tensor:
        batch, length, samples, channels = imu.shape
        return self.imu_encoder(
            imu.reshape(batch * length, samples, channels).transpose(1, 2)
        )

    def _fused_features(self, tof, imu, weight, previous_tof, previous_weight):
        if self.variant == "imu_only":
            return self._imu_tokens(imu)
        view_tokens = self._view_tokens(tof, weight, previous_tof, previous_weight)
        if self.variant == "tof_only":
            return view_tokens.mean(dim=1)
        imu_tokens = self._imu_tokens(imu)
        if self.variant == "naive_concat":
            return self.concat_proj(torch.cat([view_tokens.mean(dim=1), imu_tokens], dim=-1))
        return self.fusion(imu_tokens, view_tokens)

    def _fused_features_pair(self, tof, imu, weight, previous_tof, previous_weight):
        """Position branch = concat/local; rotation branch = directional attention."""
        view_tokens = self._view_tokens(tof, weight, previous_tof, previous_weight)
        imu_tokens = self._imu_tokens(imu)
        position = self.concat_proj(
            torch.cat([view_tokens.mean(dim=1), imu_tokens], dim=-1)
        )
        rotation = self.fusion(imu_tokens, view_tokens)
        return position, rotation

    def forward(
        self,
        tof: torch.Tensor,
        imu: torch.Tensor,
        weight: torch.Tensor | None = None,
        previous_tof: torch.Tensor | None = None,
        previous_weight: torch.Tensor | None = None,
    ) -> dict[str, torch.Tensor]:
        """Run a full sequence.

        Args:
            tof: ``[B, T, V, H, W]`` ranges in metres.
            imu: ``[B, T, K, 6]`` IMU windows.
            weight: optional ``[B, T, V, H, W]`` reliability weights.
            previous_tof: optional previous ranges aligned with ``tof``.
            previous_weight: optional reliability weights for ``previous_tof``.
        """
        batch, length = tof.shape[0], tof.shape[1]
        if self.variant == "hybrid_split":
            position, rotation = self._fused_features_pair(
                tof, imu, weight, previous_tof, previous_weight
            )
            position_hidden = self.streaming_pos(position.reshape(batch, length, -1))
            rotation_hidden = self.streaming(rotation.reshape(batch, length, -1))
            position_motion = self.pose_head_pos(position_hidden)
            rotation_motion = self.pose_head(rotation_hidden)
            return {
                "delta_p": position_motion[..., :3],
                "delta_phi": rotation_motion[..., 3:],
            }
        features = self._fused_features(
            tof, imu, weight, previous_tof, previous_weight
        )
        hidden = self.streaming(features.reshape(batch, length, -1))
        motion = self.pose_head(hidden)
        return {"delta_p": motion[..., :3], "delta_phi": motion[..., 3:]}

    def forward_stream(
        self,
        tof: torch.Tensor,
        imu: torch.Tensor,
        states: list[LayerState] | None = None,
        weight: torch.Tensor | None = None,
        previous_tof: torch.Tensor | None = None,
        previous_weight: torch.Tensor | None = None,
    ) -> tuple[dict[str, torch.Tensor], list[LayerState]]:
        """One-frame streaming forward.  ``tof``/``imu`` have T=1."""
        if tof.shape[1] != 1:
            raise ValueError("forward_stream expects T=1")
        batch = tof.shape[0]
        if self.variant == "hybrid_split":
            position, rotation = self._fused_features_pair(
                tof, imu, weight, previous_tof, previous_weight
            )
            if states is None:
                states = self.init_state(batch, position.device, position.dtype)
            position_states, rotation_states = states
            position_hidden, position_states = self.streaming_pos.forward_step(
                position.unsqueeze(1), position_states
            )
            rotation_hidden, rotation_states = self.streaming.forward_step(
                rotation.unsqueeze(1), rotation_states
            )
            position_motion = self.pose_head_pos(position_hidden)
            rotation_motion = self.pose_head(rotation_hidden)
            return (
                {
                    "delta_p": position_motion[..., :3],
                    "delta_phi": rotation_motion[..., 3:],
                },
                (position_states, rotation_states),
            )
        fused = self._fused_features(
            tof, imu, weight, previous_tof, previous_weight
        ).unsqueeze(1)
        if states is None:
            states = self.streaming.init_state(batch, fused.shape[-1], fused.device, fused.dtype)
        hidden, states = self.streaming.forward_step(fused, states)
        motion = self.pose_head(hidden)
        return {"delta_p": motion[..., :3], "delta_phi": motion[..., 3:]}, states


def count_parameters(model: nn.Module) -> int:
    return sum(parameter.numel() for parameter in model.parameters() if parameter.requires_grad)
