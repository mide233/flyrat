# SPDX-License-Identifier: GPL-3.0-or-later
#
# Start headless ORB-SLAM3 IMU-stereo on the forwarded real-vehicle topics.
# Start flyrat_sensors first, and pass vocabulary_path explicitly because the
# 139 MB ORB vocabulary is intentionally not stored in this repository.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vocabulary_path = LaunchConfiguration('vocabulary_path')
    settings_path = LaunchConfiguration('settings_path')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    world_frame = LaunchConfiguration('world_frame')
    body_frame = LaunchConfiguration('body_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    trajectory_path = LaunchConfiguration('trajectory_path')

    params_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_slam'), 'config', 'slam.yaml'])
    default_settings = PathJoinSubstitution(
        [FindPackageShare('flyrat_slam'), 'config', 'uvc_640x480_imu.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary_path', default_value='',
            description='Path to ORBvoc.txt. Required; not stored in the repository.'),
        DeclareLaunchArgument(
            'settings_path', default_value=default_settings,
            description='ORB-SLAM3 camera/IMU YAML settings file.'),
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
                    'vocabulary_path': vocabulary_path,
                    'settings_path': settings_path,
                    'left_topic': left_topic,
                    'right_topic': right_topic,
                    'imu_topic': imu_topic,
                    'world_frame': world_frame,
                    'body_frame': body_frame,
                    'camera_frame': camera_frame,
                    'trajectory_path': trajectory_path,
                },
            ],
        ),
    ])
