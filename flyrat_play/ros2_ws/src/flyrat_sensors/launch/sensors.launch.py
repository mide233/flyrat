# SPDX-License-Identifier: MIT
#
# Start the real-vehicle sensor receiver. Run adb_forward.sh first (installed
# next to this launch file) so 127.0.0.1:5000 reaches the RK3566.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    stereo_color = LaunchConfiguration('stereo_color')
    stereo_compressed_topic = LaunchConfiguration('stereo_compressed_topic')
    stereo_frame_id = LaunchConfiguration('stereo_frame_id')
    acc_lsb_per_g = LaunchConfiguration('acc_lsb_per_g')
    gyro_lsb_per_dps = LaunchConfiguration('gyro_lsb_per_dps')
    imu_accel_axis = LaunchConfiguration('imu_accel_axis')
    imu_gyro_axis = LaunchConfiguration('imu_gyro_axis')

    intrinsics_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_sensors'), 'config', 'imu_intrinsics.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument(
            'stereo_color', default_value='false',
            description='Publish bgr8 images instead of mono8.'),
        DeclareLaunchArgument(
            'stereo_compressed_topic',
            default_value='/flyrat/sensors/stereo/compressed',
            description='Side-by-side JPEG topic used for recording.'),
        DeclareLaunchArgument(
            'stereo_frame_id', default_value='camera_stereo',
            description='Frame id of the side-by-side compressed image.'),
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
            'imu_gyro_axis', default_value='none',
            description="Gyroscope axes: 'none' (already FLU), "
                        "'bf_to_flu' (clockwise yaw, flip z) or "
                        "'frd_to_flu' (flip y and z)."),
        Node(
            package='flyrat_sensors',
            executable='sensor_receiver_node',
            name='sensor_receiver',
            output='screen',
            parameters=[intrinsics_file, {
                'host': host,
                'port': ParameterValue(port, value_type=int),
                'stereo_color': ParameterValue(stereo_color, value_type=bool),
                'stereo_compressed_topic': stereo_compressed_topic,
                'stereo_frame_id': stereo_frame_id,
                'acc_lsb_per_g': ParameterValue(acc_lsb_per_g, value_type=float),
                'gyro_lsb_per_dps': ParameterValue(
                    gyro_lsb_per_dps, value_type=float),
                'imu_accel_axis': imu_accel_axis,
                'imu_gyro_axis': imu_gyro_axis,
            }],
        ),
    ])
