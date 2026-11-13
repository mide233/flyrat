#!/usr/bin/env python3
"""Export a flyrat_raw_recorder bag to Kalibr-style CSV and image folders.

The raw bag contains MSP IMU counts and native MJPEG frames. This script only
converts units and optionally flips axis signs; it never applies the old IMU
intrinsics, time offset, camera intrinsics or rectification.
"""
import argparse
import csv
import json
import math
import os
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

MAGIC = b"FLYRATRW"
RECORD_HEADER = struct.Struct("<BBHIQq")
IMU = 1
IMAGE = 2
EVENT = 3
FOOTER = 4
GRAVITY = 9.80665


def read_header(path):
    with open(path, "rb") as stream:
        prefix = stream.read(16)
        if len(prefix) != 16 or prefix[:8] != MAGIC:
            raise ValueError("not a flyrat raw bag: %s" % path)
        version, _flags, header_size = struct.unpack_from("<HHI", prefix, 8)
        if version != 1:
            raise ValueError("unsupported raw bag version %d" % version)
        header = stream.read(header_size)
        if len(header) != header_size:
            raise ValueError("truncated raw bag header")
        return json.loads(header.decode("utf-8"))


def scan_clocks(path):
    """Return the end clock event, if the recorder wrote one."""
    end_clock = None
    with open(path, "rb") as stream:
        prefix = stream.read(16)
        if len(prefix) != 16 or prefix[:8] != MAGIC:
            raise ValueError("not a flyrat raw bag: %s" % path)
        header_size = struct.unpack_from("<HHI", prefix, 8)[2]
        stream.seek(16 + header_size)
        while True:
            raw = stream.read(RECORD_HEADER.size)
            if len(raw) != RECORD_HEADER.size:
                break
            record_type, _flags, _reserved, size, _seq, _t_ns = RECORD_HEADER.unpack(raw)
            if record_type == EVENT:
                payload = stream.read(size)
                try:
                    event = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if event.get("event") == "clock":
                    end_clock = event
            else:
                stream.seek(size, os.SEEK_CUR)
    return end_clock


def axis_signs(name):
    name = (name or "none").lower()
    if name in ("none", ""):
        return np.array([1.0, 1.0, 1.0])
    if name in ("frd_to_flu", "flu_to_frd"):
        # forward-right-down <-> forward-left-up
        return np.array([1.0, -1.0, -1.0])
    raise ValueError("unsupported axis mapping: %s" % name)


class TimeMap:
    def __init__(self, metadata, end_clock, use_epoch):
        clocks = metadata.get("clocks", {})
        self.start_steady = int(clocks.get("start_steady_ns", 0))
        self.start_system = int(clocks.get("start_system_ns", 0))
        self.use_epoch = use_epoch
        self.end_steady = None
        self.end_system = None
        if end_clock:
            self.end_steady = int(end_clock.get("steady_ns", 0))
            self.end_system = int(end_clock.get("system_ns", 0))
            if self.end_steady <= self.start_steady:
                self.end_steady = None

    def epoch_ns(self, t_ns):
        if self.end_steady is not None and self.end_steady > self.start_steady:
            ratio = (self.end_system - self.start_system) / (
                self.end_steady - self.start_steady)
            return int(round(self.start_system + (t_ns - self.start_steady) * ratio))
        return self.start_system + (t_ns - self.start_steady)

    def timestamp(self, t_ns):
        if self.use_epoch:
            return self.epoch_ns(t_ns) / 1e9
        return (t_ns - self.start_steady) / 1e9

    def filename(self, t_ns):
        if self.use_epoch:
            return str(self.epoch_ns(t_ns))
        return str(t_ns)

    def __str__(self):
        if self.use_epoch:
            return "steady->system (start/end clock pair)"
        return "raw steady-clock nanoseconds"


def open_writer(path):
    handle = open(path, "w", newline="")
    return handle, csv.writer(handle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="raw recorder output directory")
    parser.add_argument("--output", required=True, help="export directory")
    parser.add_argument("--no-split", action="store_true",
                        help="keep the full side-by-side image for inspection "
                             "(not a Kalibr-ready stereo folder)")
    parser.add_argument("--no-epoch", action="store_true",
                        help="use raw steady-clock nanoseconds instead of epoch")
    parser.add_argument("--no-imu", action="store_true", help="skip IMU CSV")
    parser.add_argument("--no-images", action="store_true", help="skip image export")
    parser.add_argument("--accel-axis", default=None,
                        help="override metadata axis mapping: none|frd_to_flu")
    parser.add_argument("--gyro-axis", default=None,
                        help="override metadata axis mapping: none|frd_to_flu")
    parser.add_argument("--accel-lsb-per-g", type=float, default=None)
    parser.add_argument("--gyro-lsb-per-dps", type=float, default=None)
    args = parser.parse_args()

    source = Path(args.input)
    bag_path = source / "raw_sensors.bin"
    if not bag_path.is_file():
        print("missing %s" % bag_path, file=sys.stderr)
        return 2

    metadata = read_header(bag_path)
    end_clock = scan_clocks(bag_path)
    time_map = TimeMap(metadata, end_clock, not args.no_epoch)

    imu_meta = metadata.get("imu", {})
    camera_meta = metadata.get("camera", {})
    accel_lsb = args.accel_lsb_per_g or float(imu_meta.get("acc_lsb_per_g", 2048.0))
    gyro_lsb = args.gyro_lsb_per_dps or float(imu_meta.get("gyro_lsb_per_dps", 16.4))
    accel_signs = axis_signs(args.accel_axis or imu_meta.get("accel_axis", "none"))
    gyro_signs = axis_signs(args.gyro_axis or imu_meta.get("gyro_axis", "none"))

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cam0 = output / "cam0"
    cam1 = output / "cam1"
    if not args.no_images:
        if args.no_split:
            cam0.mkdir(exist_ok=True)
        else:
            cam0.mkdir(exist_ok=True)
            cam1.mkdir(exist_ok=True)

    imu_handle = None
    imu_writer = None
    counts_handle = None
    counts_writer = None
    cam0_handle = None
    cam0_writer = None
    cam1_handle = None
    cam1_writer = None
    if not args.no_imu:
        imu_handle, imu_writer = open_writer(output / "imu.csv")
        imu_writer.writerow(["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])
        counts_handle, counts_writer = open_writer(output / "imu_counts.csv")
        counts_writer.writerow([
            "t_ns", "t_epoch_ns",
            "ax_counts", "ay_counts", "az_counts",
            "gx_counts", "gy_counts", "gz_counts",
            "mx_counts", "my_counts", "mz_counts"])
    if not args.no_images:
        cam0_handle, cam0_writer = open_writer(output / "cam0.csv")
        cam0_writer.writerow(["timestamp", "filename"])
        if not args.no_split:
            cam1_handle, cam1_writer = open_writer(output / "cam1.csv")
            cam1_writer.writerow(["timestamp", "filename"])

    counts = {"imu": 0, "images": 0, "cam0": 0, "cam1": 0, "events": 0}
    first_stamp = None
    last_stamp = None
    width = None
    height = None
    bad_images = 0

    try:
        with open(bag_path, "rb") as stream:
            prefix = stream.read(16)
            version, _flags, header_size = struct.unpack_from("<HHI", prefix, 8)
            if version != 1 or prefix[:8] != MAGIC:
                raise ValueError("invalid raw bag")
            stream.seek(16 + header_size)

            while True:
                raw = stream.read(RECORD_HEADER.size)
                if len(raw) != RECORD_HEADER.size:
                    break
                record_type, _flags, _reserved, size, _seq, t_ns = RECORD_HEADER.unpack(raw)
                payload = stream.read(size)
                if len(payload) != size:
                    break
                if first_stamp is None:
                    first_stamp = t_ns
                last_stamp = t_ns

                if record_type == IMU:
                    if args.no_imu or size != 18:
                        continue
                    values = np.array(struct.unpack("<9h", payload), dtype=np.float64)
                    acc_counts = values[0:3]
                    gyro_counts = values[3:6]
                    mag_counts = values[6:9]
                    acc = acc_counts / accel_lsb * GRAVITY * accel_signs
                    gyro = gyro_counts / gyro_lsb * math.pi / 180.0 * gyro_signs
                    stamp = time_map.timestamp(t_ns)
                    imu_writer.writerow([
                        "%.9f" % stamp,
                        "%.9f" % acc[0], "%.9f" % acc[1], "%.9f" % acc[2],
                        "%.9f" % gyro[0], "%.9f" % gyro[1], "%.9f" % gyro[2]])
                    counts_writer.writerow([
                        t_ns, time_map.epoch_ns(t_ns),
                        int(acc_counts[0]), int(acc_counts[1]), int(acc_counts[2]),
                        int(gyro_counts[0]), int(gyro_counts[1]), int(gyro_counts[2]),
                        int(mag_counts[0]), int(mag_counts[1]), int(mag_counts[2])])
                    counts["imu"] += 1
                elif record_type == IMAGE:
                    counts["images"] += 1
                    if args.no_images:
                        continue
                    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8),
                                         cv2.IMREAD_COLOR)
                    if image is None:
                        bad_images += 1
                        continue
                    height, width = image.shape[:2]
                    name = time_map.filename(t_ns) + ".png"
                    if args.no_split:
                        cv2.imwrite(str(cam0 / name), image)
                        cam0_writer.writerow(["%.9f" % time_map.timestamp(t_ns), name])
                        counts["cam0"] += 1
                    else:
                        if width % 2 != 0:
                            print("warning: odd image width %d, dropping the last column" % width,
                                  file=sys.stderr)
                        middle = width // 2
                        left = image[:, :middle]
                        right = image[:, middle:2 * middle]
                        cv2.imwrite(str(cam0 / name), left)
                        cv2.imwrite(str(cam1 / name), right)
                        cam0_writer.writerow(["%.9f" % time_map.timestamp(t_ns), name])
                        cam1_writer.writerow(["%.9f" % time_map.timestamp(t_ns), name])
                        counts["cam0"] += 1
                        counts["cam1"] += 1
                elif record_type == EVENT:
                    counts["events"] += 1
                elif record_type == FOOTER:
                    pass
    finally:
        for handle in (imu_handle, counts_handle, cam0_handle, cam1_handle):
            if handle is not None:
                handle.close()

    imu_was_enabled = bool(imu_meta.get("enabled", True))
    if counts["imu"] == 0 and not args.no_imu and imu_was_enabled:
        print("no IMU samples were exported", file=sys.stderr)
        return 1
    if counts["images"] > 0 and counts["cam0"] == 0 and not args.no_images:
        print("no images could be decoded", file=sys.stderr)
        return 1

    duration = 0.0
    if first_stamp is not None and last_stamp is not None:
        duration = (last_stamp - first_stamp) / 1e9

    exported = {
        "input": str(bag_path),
        "output": str(output),
        "time_mapping": str(time_map),
        "duration_s": duration,
        "counts": counts,
        "decoded_width": width,
        "decoded_height": height,
        "bad_images": bad_images,
        "accel_lsb_per_g": accel_lsb,
        "gyro_lsb_per_dps": gyro_lsb,
        "accel_signs": accel_signs.tolist(),
        "gyro_signs": gyro_signs.tolist(),
    }
    with open(output / "metadata.json", "w") as handle:
        json.dump({"raw": metadata, "export": exported}, handle, indent=2)
        handle.write("\n")

    if not args.no_split and not args.no_images and counts["cam0"] > 0:
        script = output / "make_kalibr_bag.sh"
        script.write_text(
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "cd \"$(dirname \"$0\")\"\n"
            "kalibr_bagcreater --folder . --output-bag calibration.bag\n")
        script.chmod(0o755)
    elif args.no_split:
        print("note: --no-split keeps the side-by-side image; "
              "no Kalibr bag script was generated", file=sys.stderr)

    print("exported %d IMU samples, %d images (%d/%d) in %.2f s"
          % (counts["imu"], counts["images"], counts["cam0"], counts["cam1"], duration))
    if bad_images:
        print("warning: %d JPEG frames could not be decoded" % bad_images, file=sys.stderr)
    print("output: %s" % output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
