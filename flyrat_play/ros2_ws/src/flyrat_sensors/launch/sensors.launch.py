# SPDX-License-Identifier: MIT
#
# Start the real-vehicle sensor receiver. Run adb_forward.sh first (installed
# next to this launch file) so 127.0.0.1:5000 reaches the RK3566.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    stereo_color = LaunchConfiguration('stereo_color')
    acc_lsb_per_g = LaunchConfiguration('acc_lsb_per_g')
    gyro_lsb_per_dps = LaunchConfiguration('gyro_lsb_per_dps')
    imu_accel_axis = LaunchConfiguration('imu_accel_axis')
    imu_gyro_axis = LaunchConfiguration('imu_gyro_axis')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument(
            'stereo_color', default_value='false',
            description='Publish bgr8 images instead of mono8.'),
        DeclareLaunchArgument(
            'acc_lsb_per_g', default_value='2048.0',
            description='Betaflight MSP_RAW_IMU accelerometer scale.'),
        DeclareLaunchArgument(
            'gyro_lsb_per_dps', default_value='16.4',
            description='Betaflight MSP_RAW_IMU gyroscope scale.'),
        DeclareLaunchArgument(
            'imu_accel_axis', default_value='none',
            description="Accelerometer axes: 'none' (already FLU) or 'frd_to_flu'."),
        DeclareLaunchArgument(
            'imu_gyro_axis', default_value='bf_to_flu',
            description="Gyroscope axes: 'bf_to_flu' (Betaflight, flip z), "
                        "'frd_to_flu' (flip y and z) or 'none' (already FLU)."),
        Node(
            package='flyrat_sensors',
            executable='sensor_receiver_node',
            name='sensor_receiver',
            output='screen',
            parameters=[{
                'host': host,
                'port': port,
                'stereo_color': stereo_color,
                'acc_lsb_per_g': acc_lsb_per_g,
                'gyro_lsb_per_dps': gyro_lsb_per_dps,
                'imu_accel_axis': imu_accel_axis,
                'imu_gyro_axis': imu_gyro_axis,
            }],
        ),
    ])
