#!/usr/bin/env python3
"""Extract a FlyRat sensor rosbag into arrays and print diagnostics.

The bag must be recorded with the real-vehicle stack:

    /flyrat/sensors/imu             sensor_msgs/msg/Imu
    /flyrat/sensors/camera/image_raw sensor_msgs/msg/Image
    /flyrat/sensors/tof_{0..3}      flyrat_sensors_interfaces/msg/TofFrame

Run it with the ROS environment sourced::

    source /opt/ros/jazzy/setup.bash
    source flyrat_play/ros2_ws/install/setup.bash
    python3 research/tio_former/scripts/inspect_rosbag.py \
        --bag issue_6_logbag_1_1 --out /tmp/opencode/issue6

Outputs ``bag.npz`` (ToF/IMU arrays plus JPEG-encoded camera), ``bag.json``
(summary statistics) and ``camera_preview.png`` (first/last/median frame).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", default="issue_6_logbag_1_1")
    parser.add_argument("--out", default="/tmp/opencode/issue6")
    parser.add_argument("--camera-width", type=int, default=160)
    parser.add_argument("--camera-height", type=int, default=120)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--sensor-order", default="1,3,2,0")
    return parser.parse_args()


def _laplacian_variance(gray: np.ndarray) -> float:
    """4-neighbour Laplacian energy; a cheap motion-blur indicator."""
    image = gray.astype(np.float32)
    lap = (
        -4.0 * image
        + np.roll(image, 1, axis=0)
        + np.roll(image, -1, axis=0)
        + np.roll(image, 1, axis=1)
        + np.roll(image, -1, axis=1)
    )
    return float(lap[1:-1, 1:-1].var())


def _gradient_energy(gray: np.ndarray) -> float:
    image = gray.astype(np.float32)
    gx = np.diff(image, axis=1)
    gy = np.diff(image, axis=0)
    return float((gx * gx).mean() + (gy * gy).mean())


def _stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean": float(values.mean()),
        "std": float(values.std()),
        "min": float(values.min()),
        "p05": float(np.percentile(values, 5)),
        "median": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def _read_bag(bag: Path) -> tuple[dict, dict[str, str]]:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}

    from flyrat_sensors_interfaces.msg import TofFrame
    from sensor_msgs.msg import Image, Imu

    imu_t: list[float] = []
    imu_accel: list[list[float]] = []
    imu_gyro: list[list[float]] = []
    camera_t: list[float] = []
    camera_jpeg: list[bytes] = []
    camera_sharpness: list[float] = []
    camera_energy: list[float] = []
    camera_brightness: list[float] = []
    tof_t: list[list[float]] = [[] for _ in range(4)]
    tof_range: list[list[np.ndarray]] = [[] for _ in range(4)]
    tof_weight: list[list[np.ndarray]] = [[] for _ in range(4)]
    tof_status: list[list[np.ndarray]] = [[] for _ in range(4)]
    tof_stamps: list[list[float]] = [[] for _ in range(4)]

    import cv2

    counts: dict[str, int] = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        stamp_s = stamp * 1e-9
        if topic == "/flyrat/sensors/imu":
            message = deserialize_message(data, Imu)
            header = message.header.stamp
            imu_t.append(header.sec + header.nanosec * 1e-9)
            imu_accel.append(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ]
            )
            imu_gyro.append(
                [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
            )
        elif topic == "/flyrat/sensors/camera/image_raw":
            message = deserialize_message(data, Image)
            header = message.header.stamp
            camera_t.append(header.sec + header.nanosec * 1e-9)
            gray = np.frombuffer(message.data, dtype=np.uint8)
            gray = gray.reshape(message.height, message.step)[:, : message.width]
            camera_sharpness.append(_laplacian_variance(gray))
            camera_energy.append(_gradient_energy(gray))
            camera_brightness.append(float(gray.mean()))
            small = np.ascontiguousarray(
                gray[:: max(1, message.height // 120), :: max(1, message.width // 160)]
            )
            if small.shape != (120, 160):
                small = cv2.resize(small, (160, 120), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(
                ".jpg", small.astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if ok:
                camera_jpeg.append(encoded.tobytes())
        elif topic.startswith("/flyrat/sensors/tof_"):
            index = int(topic.rsplit("_", 1)[1])
            message = deserialize_message(data, TofFrame)
            header = message.header.stamp
            stamp_value = header.sec + header.nanosec * 1e-9
            zones = np.zeros(64, dtype=np.float32)
            weight = np.zeros(64, dtype=np.float32)
            status = np.zeros(64, dtype=np.uint8)
            for cell, zone in enumerate(message.zones):
                distance = float(zone.distance_mm) * 1e-3
                valid = bool(zone.valid) and 0.015 <= distance <= 4.0
                zones[cell] = distance if valid else 0.0
                weight[cell] = 1.0 if valid else 0.0
                status[cell] = int(zone.target_status)
            tof_t[index].append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
            tof_stamps[index].append(stamp_s)
            tof_range[index].append(zones.reshape(8, 8))
            tof_weight[index].append(weight.reshape(8, 8))
            tof_status[index].append(status.reshape(8, 8))

    arrays = {
        "imu_t": np.asarray(imu_t, dtype=np.float64),
        "imu": np.asarray(np.column_stack([imu_accel, imu_gyro]), dtype=np.float32),
        "camera_t": np.asarray(camera_t, dtype=np.float64),
        "camera_jpeg": np.frombuffer(b"".join(camera_jpeg), dtype=np.uint8),
        "camera_offset": np.cumsum([0] + [len(item) for item in camera_jpeg]).astype(np.int64),
        "camera_sharpness": np.asarray(camera_sharpness, dtype=np.float32),
        "camera_energy": np.asarray(camera_energy, dtype=np.float32),
        "camera_brightness": np.asarray(camera_brightness, dtype=np.float32),
    }
    for index in range(4):
        arrays[f"tof_t_{index}"] = np.asarray(tof_t[index], dtype=np.float64)
        arrays[f"tof_range_{index}"] = np.asarray(tof_range[index], dtype=np.float32).reshape(
            -1, 8, 8
        )
        arrays[f"tof_weight_{index}"] = np.asarray(tof_weight[index], dtype=np.float32).reshape(
            -1, 8, 8
        )
        arrays[f"tof_status_{index}"] = np.asarray(tof_status[index], dtype=np.uint8).reshape(
            -1, 8, 8
        )
        arrays[f"tof_arrival_{index}"] = np.asarray(tof_stamps[index], dtype=np.float64)
    return arrays, counts


def _summarise(arrays: dict, counts: dict[str, str], args: argparse.Namespace) -> dict:
    summary: dict = {"message_counts": counts}
    imu_t = arrays["imu_t"]
    imu = arrays["imu"]
    if imu_t.size > 1:
        dt = np.diff(imu_t)
        gap = int(np.argmax(dt)) if dt.size else 0
        summary["imu"] = {
            "count": int(imu_t.size),
            "duration_s": float(imu_t[-1] - imu_t[0]),
            "rate_hz": float(1.0 / np.median(dt)),
            "dt": _stats(dt),
            "max_gap_s": float(dt.max()) if dt.size else 0.0,
            "max_gap_at_s": float(imu_t[gap] - imu_t[0]) if dt.size else 0.0,
            "accel_norm": _stats(np.linalg.norm(imu[:, :3], axis=1)),
            "gyro_norm": _stats(np.linalg.norm(imu[:, 3:], axis=1)),
            "accel_mean": imu[:, :3].mean(axis=0).tolist(),
            "gyro_mean": imu[:, 3:].mean(axis=0).tolist(),
            "accel_std": imu[:, :3].std(axis=0).tolist(),
            "gyro_std": imu[:, 3:].std(axis=0).tolist(),
        }
        head = imu_t - imu_t[0] < 0.5
        tail = imu_t[-1] - imu_t < 0.5
        if head.any():
            summary["imu"]["head_accel_mean"] = imu[head, :3].mean(axis=0).tolist()
            summary["imu"]["head_gyro_mean"] = imu[head, 3:].mean(axis=0).tolist()
            summary["imu"]["head_accel_norm"] = float(
                np.linalg.norm(imu[head, :3].mean(axis=0))
            )
        if tail.any():
            summary["imu"]["tail_accel_mean"] = imu[tail, :3].mean(axis=0).tolist()
            summary["imu"]["tail_gyro_mean"] = imu[tail, 3:].mean(axis=0).tolist()

    camera_t = arrays["camera_t"]
    if camera_t.size > 1:
        dt = np.diff(camera_t)
        summary["camera"] = {
            "count": int(camera_t.size),
            "rate_hz": float(1.0 / np.median(dt)),
            "dt": _stats(dt),
            "brightness": _stats(arrays["camera_brightness"]),
            "sharpness": _stats(arrays["camera_sharpness"]),
            "gradient_energy": _stats(arrays["camera_energy"]),
            "jpeg_bytes": _stats(np.diff(arrays["camera_offset"])),
        }

    order = [int(item) for item in args.sensor_order.split(",") if item.strip()]
    summary["tof"] = {}
    for index in range(4):
        weight = arrays[f"tof_weight_{index}"]
        ranges = arrays[f"tof_range_{index}"]
        t = arrays[f"tof_t_{index}"]
        arrival = arrays[f"tof_arrival_{index}"]
        valid = weight.reshape(weight.shape[0], -1)
        valid_ranges = ranges[weight > 0]
        entry = {
            "count": int(t.size),
            "rate_hz": float(1.0 / np.median(np.diff(t))) if t.size > 1 else 0.0,
            "valid_ratio": float(valid.mean()) if valid.size else 0.0,
            "valid_percent": _stats(valid.mean(axis=1) * 100.0) if valid.size else {},
            "range_m": _stats(valid_ranges),
            "all_invalid_frames": int((valid.sum(axis=1) == 0).sum()) if valid.size else 0,
            "latency_s": _stats(arrival - t) if t.size else {},
        }
        summary["tof"][f"sensor_{index}"] = entry
    if len(order) == 4:
        summary["model_view_order"] = order
        times = [arrays[f"tof_t_{index}"] for index in order]
        reference = int(np.argmin([len(item) for item in times]))
        sync_t: list[float] = []
        picked: list[list[int]] = [[] for _ in order]
        skews: list[float] = []
        for stamp in times[reference]:
            near: list[int] = []
            for position in range(4):
                if times[position].size == 0:
                    near = []
                    break
                offset = int(np.argmin(np.abs(times[position] - stamp)))
                if abs(times[position][offset] - stamp) > 0.08:
                    near = []
                    break
                near.append(offset)
            if len(near) == 4:
                sync_t.append(float(stamp))
                skews.append(max(abs(times[p][near[p]] - stamp) for p in range(4)))
                for position in range(4):
                    picked[position].append(near[position])
        tof_range = np.stack(
            [
                np.stack([arrays[f"tof_range_{order[p]}"][i] for i in picked[p]])
                for p in range(4)
            ],
            axis=1,
        )
        tof_weight = np.stack(
            [
                np.stack([arrays[f"tof_weight_{order[p]}"][i] for i in picked[p]])
                for p in range(4)
            ],
            axis=1,
        )
        summary["model_views"] = {
            "frames": int(len(sync_t)),
            "skew_s": _stats(np.asarray(skews)) if skews else {},
            "valid_ratio": float(tof_weight.mean()),
            "valid_ratio_per_view": tof_weight.mean(axis=(0, 1, 3)).tolist(),
        }
        arrays["tof"] = tof_range.astype(np.float32)
        arrays["tof_weight"] = tof_weight.astype(np.float32)
        arrays["tof_t"] = np.asarray(sync_t, dtype=np.float64)
    return summary


def main() -> int:
    args = _args()
    bag = Path(args.bag).resolve()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    arrays, counts = _read_bag(bag)
    summary = _summarise(arrays, counts, args)
    np.savez_compressed(out / "bag.npz", **arrays)
    (out / "bag.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"wrote {out / 'bag.npz'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
