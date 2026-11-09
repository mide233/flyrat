# SPDX-License-Identifier: MIT
#
# Start the real-vehicle sensor receiver. Run adb_forward.sh first (installed
# next to this launch file) so 127.0.0.1:5000 reaches the RK3566.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    acc_lsb_per_g = LaunchConfiguration('acc_lsb_per_g')
    gyro_lsb_per_dps = LaunchConfiguration('gyro_lsb_per_dps')
    imu_accel_axis = LaunchConfiguration('imu_accel_axis')
    imu_gyro_axis = LaunchConfiguration('imu_gyro_axis')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    camera_tf_enable = LaunchConfiguration('camera_tf_enable')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    intrinsics_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_sensors'), 'config', 'imu_intrinsics.yaml'])
    camera_file = PathJoinSubstitution(
        [FindPackageShare('flyrat_sensors'), 'config', 'camera_qvga.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='5000'),
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
            'camera_frame_id', default_value='camera_front_optical_frame',
            description='frame_id for /flyrat/sensors/camera/image_raw.'),
        DeclareLaunchArgument(
            'camera_tf_enable', default_value='false',
            description='Publish imu_link -> camera optical frame TF. '
                        'Enable only after the real extrinsics are measured.'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
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
            parameters=[intrinsics_file, camera_file, {
                'host': host,
                'port': ParameterValue(port, value_type=int),
                'acc_lsb_per_g': ParameterValue(acc_lsb_per_g, value_type=float),
                'gyro_lsb_per_dps': ParameterValue(
                    gyro_lsb_per_dps, value_type=float),
                'imu_accel_axis': imu_accel_axis,
                'imu_gyro_axis': imu_gyro_axis,
                'camera_frame_id': camera_frame_id,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            output='screen',
            arguments=[
                '--x', camera_x, '--y', camera_y, '--z', camera_z,
                '--roll', camera_roll, '--pitch', camera_pitch,
                '--yaw', camera_yaw,
                '--frame-id', 'imu_link',
                '--child-frame-id', camera_frame_id,
            ],
            condition=IfCondition(camera_tf_enable),
        ),
    ])
