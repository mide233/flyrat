# SPDX-License-Identifier: MIT
#
# Bring up the ROS 2 side of the Betaflight SITL stack:
#
#   flyrat_msp      - virtual RC receiver + MSP link to the SITL
#   flyrat_control  - the autopilot core, publishing RC overrides
#   camera_bridge   - Gazebo camera + /clock into ROS 2 (ros_gz_bridge)
#
# Start the simulator first (flyrat_play/gazebo/scripts/launch_sim.sh), then:
#   ros2 launch flyrat_control flyrat_sitl.launch.py
#
# The camera bridge can be turned off with camera:=false, and its topic
# namespace changed with camera_prefix:=/cam.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Scoped Gazebo topics of the sensor declared on the airframe base_link in
# flyrat_play/gazebo/models/betaflight_iris/model.sdf.
GZ_CAMERA = '/world/betaflight_sitl/model/iris/link/base_link/sensor/camera'


def generate_launch_description():
    auto_takeoff = LaunchConfiguration('auto_takeoff')
    msp_override = LaunchConfiguration('msp_override')
    controller = LaunchConfiguration('controller')
    params_file = LaunchConfiguration('params_file')
    camera = LaunchConfiguration('camera')
    camera_prefix = LaunchConfiguration('camera_prefix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_takeoff', default_value='false',
            description='Have flyrat_control arm and climb automatically.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('flyrat_control'), 'config', 'mpc.yaml']),
            description='MPC weights and limits.'),
        DeclareLaunchArgument(
            'msp_override', default_value='true',
            description='Hold the MSP OVERRIDE switch high from startup.'),
        DeclareLaunchArgument(
            'controller', default_value='attitude_hold',
            description="Autopilot: 'attitude_hold' or 'ommpc'."),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Bridge the Gazebo camera and /clock into ROS 2.'),
        DeclareLaunchArgument(
            'camera_prefix', default_value='/flyrat/camera',
            description='Namespace for the bridged camera topics.'),
        Node(
            package='flyrat_msp',
            executable='msp_node',
            name='flyrat_msp',
            output='screen',
            parameters=[{
                'msp_override': ParameterValue(msp_override, value_type=bool),
            }],
        ),
        Node(
            package='flyrat_control',
            executable='control_node',
            name='flyrat_control',
            output='screen',
            parameters=[
                params_file,
                {
                    'auto_takeoff': ParameterValue(auto_takeoff, value_type=bool),
                    'controller': ParameterValue(controller, value_type=str),
                },
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=[
                f'{GZ_CAMERA}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'{GZ_CAMERA}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                (f'{GZ_CAMERA}/image', [camera_prefix, '/image_raw']),
                (f'{GZ_CAMERA}/camera_info', [camera_prefix, '/camera_info']),
            ],
            condition=IfCondition(camera),
        ),
    ])
