# SPDX-License-Identifier: MIT
"""Launch the TIO-Former ToF-inertial odometry node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package = FindPackageShare("flyrat_odometry")
    model = PathJoinSubstitution([package, "models", "tio_former_window64.pt"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=model),
            DeclareLaunchArgument("world_frame", default_value="odometry_map"),
            DeclareLaunchArgument("body_frame", default_value="imu_link"),
            DeclareLaunchArgument("warmup_frames", default_value="64"),
            DeclareLaunchArgument("trajectory_path", default_value=""),
            Node(
                package="flyrat_odometry",
                executable="tio_odometry_node",
                name="tio_odometry_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [package, "config", "tio_odometry.yaml"]
                    ),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "world_frame": LaunchConfiguration("world_frame"),
                        "body_frame": LaunchConfiguration("body_frame"),
                        "warmup_frames": LaunchConfiguration("warmup_frames"),
                        "trajectory_path": LaunchConfiguration("trajectory_path"),
                    },
                ],
            ),
        ]
    )
