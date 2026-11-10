"""Model, streaming-cache, loss and dataset smoke tests."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from tio_former.data.torch_dataset import SequenceWindows  # noqa: E402
from tio_former.models.losses import compute_losses  # noqa: E402
from tio_former.models.tio_former import ModelConfig, TioFormer  # noqa: E402
from tio_former.utils.config import load_yaml  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/model/base.yaml"
TEST_DATA = ROOT / "data/processed/test_office_0000.npz"


def _model() -> TioFormer:
    model = TioFormer(ModelConfig.from_dict(load_yaml(CONFIG)))
    model.eval()
    return model


def test_model_forward_shapes() -> None:
    model = _model()
    torch.manual_seed(0)
    tof = torch.rand(2, 6, 4, 8, 8)
    imu = torch.randn(2, 6, 16, 6)
    weight = torch.ones(2, 6, 4, 8, 8)
    with torch.no_grad():
        out = model(tof, imu, weight)
    assert out["delta_p"].shape == (2, 6, 3)
    assert out["delta_phi"].shape == (2, 6, 3)
    assert torch.isfinite(out["delta_p"]).all()
    assert torch.isfinite(out["delta_phi"]).all()


def test_streaming_matches_full_short() -> None:
    model = _model()
    torch.manual_seed(1)
    tof = torch.rand(1, 5, 4, 8, 8)
    imu = torch.randn(1, 5, 16, 6)
    weight = torch.ones(1, 5, 4, 8, 8)
    with torch.no_grad():
        full = model(tof, imu, weight)
        states = model.init_state(batch_size=1, device=tof.device, dtype=tof.dtype)
        steps = []
        for t in range(tof.shape[1]):
            prev_slice = tof[:, t - 1 : t] if t > 0 else tof[:, 0:1]
            prev_weight = weight[:, t - 1 : t] if t > 0 else weight[:, 0:1]
            out, states = model.forward_stream(
                tof[:, t : t + 1],
                imu[:, t : t + 1],
                states=states,
                weight=weight[:, t : t + 1],
                previous_tof=prev_slice,
                previous_weight=prev_weight,
            )
            steps.append(out)
        stream_p = torch.cat([s["delta_p"] for s in steps], dim=1)
        stream_phi = torch.cat([s["delta_phi"] for s in steps], dim=1)
    torch.testing.assert_close(stream_p, full["delta_p"], atol=1e-5, rtol=1e-4)
    torch.testing.assert_close(stream_phi, full["delta_phi"], atol=1e-5, rtol=1e-4)


def test_streaming_matches_full_with_eviction() -> None:
    model = _model()
    torch.manual_seed(2)
    tof = torch.rand(1, 14, 4, 8, 8)
    imu = torch.randn(1, 14, 16, 6)
    weight = torch.ones(1, 14, 4, 8, 8)
    with torch.no_grad():
        full = model(tof, imu, weight)
        states = model.init_state(batch_size=1, device=tof.device, dtype=tof.dtype)
        steps = []
        for t in range(tof.shape[1]):
            prev_slice = tof[:, t - 1 : t] if t > 0 else tof[:, 0:1]
            prev_weight = weight[:, t - 1 : t] if t > 0 else weight[:, 0:1]
            out, states = model.forward_stream(
                tof[:, t : t + 1],
                imu[:, t : t + 1],
                states=states,
                weight=weight[:, t : t + 1],
                previous_tof=prev_slice,
                previous_weight=prev_weight,
            )
            steps.append(out)
        stream_p = torch.cat([s["delta_p"] for s in steps], dim=1)
        stream_phi = torch.cat([s["delta_phi"] for s in steps], dim=1)
    torch.testing.assert_close(stream_p, full["delta_p"], atol=1e-5, rtol=1e-4)
    torch.testing.assert_close(stream_phi, full["delta_phi"], atol=1e-5, rtol=1e-4)


def test_losses_backward() -> None:
    model = _model()
    torch.manual_seed(3)
    tof = torch.rand(1, 8, 4, 8, 8)
    imu = torch.randn(1, 8, 16, 6)
    weight = torch.ones(1, 8, 4, 8, 8)
    out = model(tof, imu, weight)
    target_p = torch.randn(1, 8, 3) * 0.02
    target_phi = torch.randn(1, 8, 3) * 0.01
    total, breakdown = compute_losses(
        out,
        target_p,
        target_phi,
        load_yaml(ROOT / "configs/train/default.yaml")["loss"],
    )
    assert torch.isfinite(total)
    assert total.requires_grad
    total.backward()
    assert any(p.grad is not None and torch.isfinite(p.grad).all() for p in model.parameters())
    assert "step" in breakdown


@pytest.mark.parametrize(
    "variant",
    [
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
    ],
)
def test_variants_forward_shapes(variant: str) -> None:
    config = load_yaml(CONFIG)
    config.setdefault("model", {})["variant"] = variant
    model = TioFormer(ModelConfig.from_dict(config))
    model.eval()
    torch.manual_seed(4)
    tof = torch.rand(1, 3, 4, 8, 8)
    imu = torch.randn(1, 3, 16, 6)
    weight = torch.ones(1, 3, 4, 8, 8)
    with torch.no_grad():
        out = model(tof, imu, weight)
    assert out["delta_p"].shape == (1, 3, 3)
    assert out["delta_phi"].shape == (1, 3, 3)
    assert torch.isfinite(out["delta_p"]).all()
    assert torch.isfinite(out["delta_phi"]).all()


@pytest.mark.parametrize(
    "variant", ["memoryless", "kv_only_no_fifo", "gated_fifo", "hybrid_gated", "hybrid_split"]
)
def test_variant_streaming_matches_full(variant: str) -> None:
    config = load_yaml(CONFIG)
    config.setdefault("model", {})["variant"] = variant
    model = TioFormer(ModelConfig.from_dict(config))
    model.eval()
    torch.manual_seed(5)
    tof = torch.rand(1, 14, 4, 8, 8)
    imu = torch.randn(1, 14, 16, 6)
    weight = torch.ones(1, 14, 4, 8, 8)
    with torch.no_grad():
        full = model(tof, imu, weight)
        states = model.init_state(batch_size=1, device=tof.device, dtype=tof.dtype)
        steps = []
        for t in range(tof.shape[1]):
            prev_slice = tof[:, t - 1 : t] if t > 0 else tof[:, 0:1]
            prev_weight = weight[:, t - 1 : t] if t > 0 else weight[:, 0:1]
            out, states = model.forward_stream(
                tof[:, t : t + 1],
                imu[:, t : t + 1],
                states=states,
                weight=weight[:, t : t + 1],
                previous_tof=prev_slice,
                previous_weight=prev_weight,
            )
            steps.append(out)
        stream_p = torch.cat([s["delta_p"] for s in steps], dim=1)
        stream_phi = torch.cat([s["delta_phi"] for s in steps], dim=1)
    torch.testing.assert_close(stream_p, full["delta_p"], atol=1e-5, rtol=1e-4)
    torch.testing.assert_close(stream_phi, full["delta_phi"], atol=1e-5, rtol=1e-4)


def test_dataset_item_shapes() -> None:
    if not TEST_DATA.exists():
        pytest.skip("processed dataset not generated")
    dataset = SequenceWindows(ROOT / "data/processed", split="test", window_frames=16, max_sequences=1)
    assert len(dataset) > 0
    item = dataset[0]
    assert item["tof"].shape == (16, 4, 8, 8)
    assert item["previous_tof"].shape == (16, 4, 8, 8)
    assert item["weight"].shape == (16, 4, 8, 8)
    assert item["previous_weight"].shape == (16, 4, 8, 8)
    assert item["imu"].shape == (16, 16, 6)
    assert item["delta_p"].shape == (16, 3)
    assert item["delta_phi"].shape == (16, 3)
    assert np.isfinite(item["delta_p"].numpy()).all()
    assert np.isfinite(item["delta_phi"].numpy()).all()


def test_dataset_scene_filter() -> None:
    if not TEST_DATA.exists():
        pytest.skip("processed dataset not generated")
    dataset = SequenceWindows(
        ROOT / "data/processed",
        split="train",
        exclude_scenes=["office"],
        cache=False,
    )
    assert len(dataset.files) > 0
    for path in dataset.files:
        metadata = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
        assert metadata["scene"] != "office"


def test_camera_fusion_forward_and_streaming() -> None:
    config = load_yaml(CONFIG)
    config["model"]["camera"] = {"enabled": True}
    model = TioFormer(ModelConfig.from_dict(config))
    model.eval()
    torch.manual_seed(3)
    tof = torch.rand(1, 4, 4, 8, 8)
    imu = torch.randn(1, 4, 16, 6)
    weight = torch.ones(1, 4, 4, 8, 8)
    camera = torch.rand(1, 4, 120, 160)
    previous_camera = torch.cat([camera[:, :1], camera[:, :-1]], dim=1)
    with torch.no_grad():
        full = model(
            tof,
            imu,
            weight,
            camera=camera,
            previous_camera=previous_camera,
        )
        states = model.init_state(1, torch.device("cpu"), torch.float32)
        streamed = []
        for index in range(4):
            out, states = model.forward_stream(
                tof[:, index : index + 1],
                imu[:, index : index + 1],
                states,
                weight=weight[:, index : index + 1],
                previous_tof=tof[:, index - 1 : index] if index else tof[:, 0:1],
                previous_weight=weight[:, index - 1 : index] if index else weight[:, 0:1],
                camera=camera[:, index : index + 1],
                previous_camera=previous_camera[:, index : index + 1],
            )
            streamed.append(out)
    stacked_p = torch.cat([item["delta_p"] for item in streamed], dim=1)
    stacked_phi = torch.cat([item["delta_phi"] for item in streamed], dim=1)
    assert full["delta_p"].shape == (1, 4, 3)
    assert "gate_weights" in full
    assert full["gate_weights"].shape == (1, 4, 3)
    torch.testing.assert_close(
        full["gate_weights"].sum(dim=-1), torch.ones(1, 4), atol=1e-5, rtol=1e-5
    )
    torch.testing.assert_close(full["delta_p"], stacked_p, atol=1e-5, rtol=1e-5)
    torch.testing.assert_close(full["delta_phi"], stacked_phi, atol=1e-5, rtol=1e-5)


def test_dataset_window_includes_camera(tmp_path) -> None:
    if not TEST_DATA.exists():
        pytest.skip("processed dataset not generated")
    import io

    from PIL import Image

    with np.load(TEST_DATA, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    frames = max(int(arrays["tof"].shape[0]), 2)
    encoded = []
    for index in range(frames):
        image = np.full((120, 160), index % 255, dtype=np.uint8)
        buffer = io.BytesIO()
        Image.fromarray(image, mode="L").save(buffer, format="JPEG", quality=85)
        encoded.append(buffer.getvalue())
    offsets = np.cumsum([0] + [len(item) for item in encoded]).astype(np.int64)
    arrays["camera_jpeg"] = np.frombuffer(b"".join(encoded), dtype=np.uint8)
    arrays["camera_offset"] = offsets
    arrays["camera_width"] = np.asarray(160, dtype=np.int32)
    arrays["camera_height"] = np.asarray(120, dtype=np.int32)
    np.savez_compressed(tmp_path / "seq.npz", **arrays)
    (tmp_path / "seq.json").write_text(json.dumps({"split": "train", "scene": "office"}), encoding="utf-8")

    dataset = SequenceWindows(tmp_path, split="train", window_frames=None, use_camera=True)
    item = dataset[0]
    assert item["camera"].shape == (frames, 120, 160)
    assert item["camera"].dtype == torch.float32
    assert float(item["camera"][0].max()) == 0.0


def test_scale_loss_is_finite_for_static_windows() -> None:
    from tio_former.models.losses import scale_loss

    torch.manual_seed(0)
    target = torch.zeros(2, 16, 3)
    target[:, 0] = 1e-4
    prediction = torch.full((2, 16, 3), 0.002)
    value = scale_loss(prediction, target)
    assert torch.isfinite(value)
    assert float(value) < 5.0
