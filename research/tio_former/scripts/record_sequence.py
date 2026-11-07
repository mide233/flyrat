#!/usr/bin/env python3
"""Record one Gazebo sequence (four depth images + ground-truth poses).

Gazebo is launched headless as a subprocess.  The trajectory_player system
plugin replays the CSV pose and publishes ``/tio/gt``; four depth cameras
publish 64x64 ``R_FLOAT32`` images at 15 Hz.  The raw arrays are stored as NPZ
and converted to physical 8x8 ToF frames by ``postprocess_sequence.py``.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tio_former.utils.logging import get_logger  # noqa: E402

VIEWS = ("front", "left", "right", "down")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--out", required=True, help="Output .npz path")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--gz", default="/usr/bin/gz")
    parser.add_argument("--no-realtime", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    logger = get_logger("record_sequence")

    import gz.msgs10.image_pb2 as image_pb
    import gz.msgs10.pose_v_pb2 as pose_v_pb
    import gz.transport13 as transport

    # Keep concurrent recorders isolated: Gazebo transport partitions are
    # shared by default, which would mix /tio/* topics across processes.  The
    # environment must be set before the Node is created so the subscription
    # and the Gazebo subprocess join the same partition.
    partition = f"tio_{Path(args.out).stem}"
    os.environ["GZ_PARTITION"] = partition
    node = transport.Node()
    depth: dict[str, list[np.ndarray]] = {view: [] for view in VIEWS}
    depth_t: dict[str, list[float]] = {view: [] for view in VIEWS}
    gt: list[tuple[float, np.ndarray]] = []
    first_gt = {"t": None}

    def make_image_callback(view: str):
        def callback(msg, *_unused):
            width = int(msg.width)
            height = int(msg.height)
            expected = width * height * 4
            if len(msg.data) < expected:
                return
            array = np.frombuffer(msg.data, dtype="<f4", count=width * height).reshape(height, width)
            stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nsec)
            depth[view].append(array.copy())
            depth_t[view].append(stamp)

        return callback

    def gt_callback(msg, *_unused):
        if not msg.pose:
            return
        pose = msg.pose[0]
        stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nsec)
        values = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=np.float64,
        )
        gt.append((stamp, values))
        if first_gt["t"] is None:
            first_gt["t"] = stamp

    for view in VIEWS:
        node.subscribe(image_pb.Image, f"/tio/tof_{view}", make_image_callback(view))
    node.subscribe(pose_v_pb.Pose_V, "/tio/gt", gt_callback)

    env = os.environ.copy()
    env.pop("GZ_CONFIG_PATH", None)
    env["PATH"] = "/usr/bin:/bin:" + env.get("PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        [
            str(ROOT / "assets/models"),
            str(ROOT / "sim/models"),
            str(ROOT / "sim/worlds/generated"),
        ]
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(ROOT / "sim/gz_plugins/install/lib")
    env["TIO_TRAJECTORY_FILE"] = str(Path(args.trajectory).resolve())
    env.setdefault("GZ_SIM_RENDER_ENGINE", "ogre2")

    command = [args.gz, "sim", "-s", "--headless-rendering"]
    if not args.no_realtime:
        command.append("-r")
    command.append(str(Path(args.world).resolve()))
    logger.info("launching: %s", " ".join(command))
    log_path = Path(args.out).with_suffix(".gazebo.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout:
            if process.poll() is not None:
                logger.error("gazebo exited early with code %s", process.returncode)
                break
            if first_gt["t"] is not None:
                break
            time.sleep(0.05)
        else:
            logger.error("timed out waiting for /tio/gt (see %s)", log_path)
            return 2
        if process.poll() is not None:
            return 2

        logger.info("received ground truth; recording %.1f s", args.duration)
        start_sim = float(first_gt["t"])
        last_count = 0
        while time.monotonic() - started < args.timeout:
            if process.poll() is not None:
                logger.error("gazebo exited during recording")
                break
            if gt and float(gt[-1][0]) - start_sim >= args.duration:
                break
            if len(gt) != last_count:
                last_count = len(gt)
            if len(gt) % 30 == 0 and len(gt) > 0:
                elapsed = float(gt[-1][0]) - start_sim
                if elapsed > 0.0 and int(elapsed * 15) % 30 == 0:
                    logger.info("recorded %.1f / %.1f s", elapsed, args.duration)
            time.sleep(0.05)
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log_file.close()

    if not gt:
        logger.error("no ground-truth messages; see %s", log_path)
        return 2
    counts = {view: len(depth[view]) for view in VIEWS}
    logger.info("frames: %s, gt: %d", counts, len(gt))
    if min(counts.values()) == 0:
        logger.error("at least one camera produced no frames; see %s", log_path)
        return 2

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {
        "gt_stamp": np.asarray([item[0] for item in gt], dtype=np.float64),
        "gt_pose": np.stack([item[1] for item in gt]).astype(np.float64),
    }
    for view in VIEWS:
        arrays[f"depth_{view}"] = np.stack(depth[view]).astype(np.float32)
        arrays[f"t_{view}"] = np.asarray(depth_t[view], dtype=np.float64)
    np.savez_compressed(out_path, **arrays)
    metadata = {
        "world": str(Path(args.world).resolve()),
        "trajectory": str(Path(args.trajectory).resolve()),
        "views": list(VIEWS),
        "duration_s": float(gt[-1][0] - gt[0][0]),
        "counts": counts,
        "gt_count": len(gt),
        "gazebo_log": str(log_path),
    }
    out_path.with_suffix(".json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    logger.info("wrote %s", out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
