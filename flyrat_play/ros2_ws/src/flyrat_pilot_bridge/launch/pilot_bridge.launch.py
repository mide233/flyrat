# SPDX-License-Identifier: MIT
#
# Bridge the pose stream from the on-vehicle flyrat_pilot into ROS 2.
# Start the device process and `adb forward tcp:5001 tcp:5001` first.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    camera_config_path = LaunchConfiguration('camera_config_path')
    world_frame = LaunchConfiguration('world_frame')
    body_frame = LaunchConfiguration('body_frame')
    publish_tf = LaunchConfiguration('publish_tf')
    stamp_source = LaunchConfiguration('stamp_source')
    time_offset = LaunchConfiguration('time_offset')

    params_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_pilot_bridge'), 'config', 'pilot_bridge.yaml'])
    default_camera_config = PathJoinSubstitution(
        [FindPackageShare('flyrat_pilot_bridge'), 'config', 'uvc_640x480.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='5001'),
        DeclareLaunchArgument(
            'camera_config_path', default_value=default_camera_config,
            description='Camera/IMU calibration YAML used for static TF.'),
        DeclareLaunchArgument('world_frame', default_value='odometry_map'),
        DeclareLaunchArgument('body_frame', default_value='imu_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument(
            'stamp_source', default_value='device',
            description="Use 'device' pose timestamps or the host receive time."),
        DeclareLaunchArgument(
            'time_offset', default_value='0.0',
            description='Seconds added to device timestamps (for clock skew).'),
        Node(
            package='flyrat_pilot_bridge',
            executable='pilot_bridge_node',
            name='pilot_bridge_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'host': host,
                    'port': port,
                    'camera_config_path': camera_config_path,
                    'world_frame': world_frame,
                    'body_frame': body_frame,
                    'publish_tf': publish_tf,
                    'stamp_source': stamp_source,
                    'time_offset': time_offset,
                },
            ],
        ),
    ])
