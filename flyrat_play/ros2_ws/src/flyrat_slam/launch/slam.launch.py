# SPDX-License-Identifier: GPL-3.0-or-later
#
# Start headless VINS-Fusion stereo-inertial SLAM on the forwarded real-vehicle
# topics. Start flyrat_sensors first; the VINS configuration is installed with
# this package and can be overridden with vins_config:=/path/to/config.yaml.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vins_config = LaunchConfiguration('vins_config')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    world_frame = LaunchConfiguration('world_frame')
    body_frame = LaunchConfiguration('body_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    right_camera_frame = LaunchConfiguration('right_camera_frame')
    trajectory_path = LaunchConfiguration('trajectory_path')

    params_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_slam'), 'config', 'slam.yaml'])
    default_vins_config = PathJoinSubstitution(
        [FindPackageShare('flyrat_slam'), 'config', 'uvc_640x480_imu.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'vins_config', default_value=default_vins_config,
            description='VINS-Fusion stereo-inertial YAML configuration.'),
        DeclareLaunchArgument(
            'left_topic',
            default_value='/flyrat/sensors/stereo/left/image_raw'),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/flyrat/sensors/stereo/right/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/flyrat/sensors/imu'),
        DeclareLaunchArgument('world_frame', default_value='slam_map'),
        DeclareLaunchArgument(
            'body_frame', default_value='imu_link',
            description='Vehicle/IMU frame; receives the SLAM pose and TF.'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_left_optical_frame'),
        DeclareLaunchArgument(
            'right_camera_frame', default_value='camera_right_optical_frame'),
        DeclareLaunchArgument(
            'trajectory_path', default_value='',
            description='Optional TUM trajectory file written on shutdown.'),
        Node(
            package='flyrat_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'vins_config': vins_config,
                    'left_topic': left_topic,
                    'right_topic': right_topic,
                    'imu_topic': imu_topic,
                    'world_frame': world_frame,
                    'body_frame': body_frame,
                    'camera_frame': camera_frame,
                    'right_camera_frame': right_camera_frame,
                    'trajectory_path': trajectory_path,
                },
            ],
        ),
    ])
