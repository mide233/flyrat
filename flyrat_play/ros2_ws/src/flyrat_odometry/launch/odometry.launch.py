# SPDX-License-Identifier: MIT
#
# Start the odometry node on the forwarded real-vehicle topics. Start
# flyrat_sensors first. The default backend is SchurVINS, which tracks the
# stereo pair together with IMU measurements.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config_path = LaunchConfiguration('camera_config_path')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    mode = LaunchConfiguration('mode')
    world_frame = LaunchConfiguration('world_frame')
    body_frame = LaunchConfiguration('body_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    right_camera_frame = LaunchConfiguration('right_camera_frame')
    trajectory_path = LaunchConfiguration('trajectory_path')

    params_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_odometry'), 'config', 'odometry.yaml'])
    default_camera_config = PathJoinSubstitution(
        [FindPackageShare('flyrat_odometry'), 'config', 'uvc_640x480.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config_path', default_value=default_camera_config,
            description='Camera/IMU calibration YAML.'),
        DeclareLaunchArgument(
            'left_topic',
            default_value='/flyrat/sensors/stereo/left/image_raw'),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/flyrat/sensors/stereo/right/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/flyrat/sensors/imu'),
        DeclareLaunchArgument(
            'mode', default_value='imu_stereo',
            description="Odometry mode: 'imu_stereo' (SchurVINS); 'stereo' is unavailable in this build."),
        DeclareLaunchArgument('world_frame', default_value='odometry_map'),
        DeclareLaunchArgument(
            'body_frame', default_value='imu_link',
            description='Vehicle/IMU frame; receives the odometry pose and TF.'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_left_optical_frame'),
        DeclareLaunchArgument(
            'right_camera_frame', default_value='camera_right_optical_frame'),
        DeclareLaunchArgument(
            'trajectory_path', default_value='',
            description='Optional TUM trajectory file written on shutdown.'),
        Node(
            package='flyrat_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'camera_config_path': camera_config_path,
                    'left_topic': left_topic,
                    'right_topic': right_topic,
                    'imu_topic': imu_topic,
                    'mode': mode,
                    'world_frame': world_frame,
                    'body_frame': body_frame,
                    'camera_frame': camera_frame,
                    'right_camera_frame': right_camera_frame,
                    'trajectory_path': trajectory_path,
                },
            ],
        ),
    ])
