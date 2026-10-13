# SPDX-License-Identifier: MIT
#
# Bring up the ROS 2 side of the Betaflight SITL stack:
#
#   flyrat_msp      - virtual RC receiver + MSP link to the SITL
#   flyrat_control  - the autopilot core, publishing RC overrides
#
# Start the simulator first (sim/scripts/launch_sim.sh), then:
#   ros2 launch flyrat_control flyrat_sitl.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    auto_takeoff = LaunchConfiguration('auto_takeoff')
    msp_override = LaunchConfiguration('msp_override')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_takeoff', default_value='false',
            description='Have flyrat_control arm and climb automatically.'),
        DeclareLaunchArgument(
            'msp_override', default_value='true',
            description='Hold the MSP OVERRIDE switch high from startup.'),
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
            parameters=[{
                'auto_takeoff': ParameterValue(auto_takeoff, value_type=bool),
            }],
        ),
    ])
