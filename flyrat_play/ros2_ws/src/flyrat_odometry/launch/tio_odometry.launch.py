# SPDX-License-Identifier: MIT
"""Launch the TIO-Former ToF-inertial odometry node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="false",
                description="Feed the forward mono camera to the model. Must "
                            "match the exported model metadata.",
            ),
            DeclareLaunchArgument("camera_width", default_value="160"),
            DeclareLaunchArgument("camera_height", default_value="120"),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="/flyrat/sensors/camera/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_max_skew_s",
                default_value="0.08",
                description="Maximum camera/ToF timestamp difference before "
                            "the frame is treated as missing.",
            ),
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
                        "warmup_frames": ParameterValue(
                            LaunchConfiguration("warmup_frames"), value_type=int
                        ),
                        "trajectory_path": LaunchConfiguration("trajectory_path"),
                        "camera_enabled": ParameterValue(
                            LaunchConfiguration("camera_enabled"), value_type=bool
                        ),
                        "camera_width": ParameterValue(
                            LaunchConfiguration("camera_width"), value_type=int
                        ),
                        "camera_height": ParameterValue(
                            LaunchConfiguration("camera_height"), value_type=int
                        ),
                        "camera_topic": LaunchConfiguration("camera_topic"),
                        "camera_max_skew_s": ParameterValue(
                            LaunchConfiguration("camera_max_skew_s"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
