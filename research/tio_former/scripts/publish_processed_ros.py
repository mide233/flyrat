#!/usr/bin/env python3
"""Publish one processed TIO-Former sequence on the FlyRat ROS 2 topics.

Used to validate the C++ TorchScript backend end-to-end without the real
sensor deck.  Source the ROS 2 workspace before running this script.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from flyrat_sensors_interfaces.msg import TofFrame, TofZone
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--no-sleep", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _args()
    data = np.load(args.sequence)
    frames = int(data["tof"].shape[0])
    if args.frames:
        frames = min(frames, args.frames)

    rclpy.init()
    node = Node("tio_sequence_publisher")
    tof_pubs = [
        node.create_publisher(TofFrame, f"/flyrat/sensors/tof_{index}", 10)
        for index in range(4)
    ]
    imu_pub = node.create_publisher(Imu, "/flyrat/sensors/imu", 200)

    period = 1.0 / args.rate
    start = time.monotonic()
    for frame in range(frames):
        frame_start = start + frame * period
        for sample in range(16):
            stamp = frame_start + sample * period / 16.0
            values = data["imu"][frame, sample]
            imu = Imu()
            imu.header.stamp = node.get_clock().now().to_msg()
            imu.header.frame_id = "imu_link"
            imu.linear_acceleration.x = float(values[0])
            imu.linear_acceleration.y = float(values[1])
            imu.linear_acceleration.z = float(values[2])
            imu.angular_velocity.x = float(values[3])
            imu.angular_velocity.y = float(values[4])
            imu.angular_velocity.z = float(values[5])
            imu_pub.publish(imu)
            if not args.no_sleep:
                delay = stamp - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

        for view, publisher in enumerate(tof_pubs):
            message = TofFrame()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = (
                "tof_front_link",
                "tof_left_link",
                "tof_right_link",
                "tof_down_link",
            )[view]
            message.sensor_id = view
            message.resolution = 8
            ranges = data["tof"][frame, view].reshape(-1)
            weights = data["tof_weight"][frame, view].reshape(-1)
            for index in range(64):
                zone = TofZone()
                zone.valid = bool(weights[index] > 0.0)
                zone.distance_mm = int(round(float(ranges[index]) * 1000.0))
                zone.target_status = 5 if zone.valid else 0
                zone.nb_target = 1 if zone.valid else 0
                message.zones[index] = zone
            publisher.publish(message)
    node.get_logger().info(f"published {frames} frames")
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
