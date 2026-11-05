# SPDX-License-Identifier: GPL-3.0-or-later
#
# One-command bring-up for the real vehicle sensor + SLAM pipeline:
#
#   adb forward -> device forwarder -> sensor_receiver_node -> slam_node
#   (the push step is handled by .script/bringup_real.sh)
#
# It can also start foxglove_bridge, so the whole stack runs from one terminal.
# .script/bringup_real.sh wraps this launch and cleans up adb forward and the
# device-side forwarder when it exits.
#
# Usage:
#   ros2 launch flyrat_slam bringup.launch.py \
#     vocabulary_path:=/path/to/ORBvoc.txt
#
# All other arguments have usable defaults; see the declared arguments below.
import os
import re
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        '1', 'true', 'yes', 'on')


def _text(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _repo_file(relative_path):
    """Find a file in the repository when running from the installed tree."""
    here = Path(os.path.realpath(__file__)).parent
    for parent in (here, *here.parents):
        candidate = parent / relative_path
        if candidate.is_file():
            return str(candidate)
    return None


def _resolve_vocabulary(context):
    explicit = _text(context, 'vocabulary_path')
    if explicit:
        return explicit

    candidates = [
        os.environ.get('FLYRAT_ORB_VOCAB', '').strip(),
        _repo_file('playground/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        _repo_file('Vocabulary/ORBvoc.txt'),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return candidate

    raise RuntimeError(
        'ORB vocabulary not found. Pass vocabulary_path:=/path/to/ORBvoc.txt, '
        'set FLYRAT_ORB_VOCAB, or place it at '
        'playground/ORB_SLAM3/Vocabulary/ORBvoc.txt.')


def _resolve_settings(context):
    explicit = _text(context, 'settings_path')
    if explicit:
        return explicit

    source = _repo_file('libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml')
    if source:
        # Prefer the source tree so edits to the extrinsic take effect without
        # rebuilding the ROS package.
        return source

    return os.path.join(
        get_package_share_directory('flyrat_slam'),
        'config',
        'uvc_640x480_imu.yaml')


def _on_device_exit(event, context):
    # Ctrl-C also terminates the device process, so only treat the exit as a
    # failure when launch is not already shutting down.
    if context.is_shutdown:
        return []
    return [Shutdown(reason=(
        'device forwarder exited; check device_binary, adb and the '
        'forwarder log'))]


def _launch_setup(context, *args, **kwargs):
    serial = _text(context, 'adb_serial')
    adb = ['adb'] + (['-s', serial] if serial else [])

    start_adb_forward = _flag(context, 'start_adb_forward')
    start_device = _flag(context, 'start_device')
    start_sensors = _flag(context, 'start_sensors')
    start_slam = _flag(context, 'start_slam')
    start_foxglove = _flag(context, 'start_foxglove')

    local_port = _text(context, 'local_port')
    device_port = _text(context, 'device_port')
    device_host = _text(context, 'device_host')

    actions = [
        LogInfo(msg=(
            f'[bringup] adb serial: {serial or "(only connected device)"}, '
            f'forward tcp:{local_port} -> device tcp:{device_port}')),
    ]

    if start_adb_forward:
        actions.append(ExecuteProcess(
            cmd=adb + [
                'forward',
                f'tcp:{local_port}',
                f'tcp:{device_port}',
            ],
            name='adb_forward',
            output='screen',
        ))

    if start_device:
        device_binary = _text(context, 'device_binary')
        # Restart any forwarder left over from a previous run, otherwise the
        # new instance cannot bind the TCP port. Match the full command line,
        # anchored at the executable path: the process name is truncated to 15
        # characters, so pkill -x cannot find "flyrat_sensor_forwarder".
        forwarder_pattern = '^' + re.escape(device_binary)
        wait_old = (
            'i=0; '
            f'while pgrep -f {shlex.quote(forwarder_pattern)} >/dev/null 2>&1 '
            '&& [ $i -lt 30 ]; do sleep 0.2; i=$((i+1)); done; ')
        remote_command = (
            f'pkill -f {shlex.quote(forwarder_pattern)} 2>/dev/null; '
            f'{wait_old}'
            f'exec {shlex.quote(device_binary)} '
            f'--listen {shlex.quote(device_host)}:{device_port}')
        device_process = ExecuteProcess(
            cmd=adb + ['shell', remote_command],
            name='device_forwarder',
            output='screen',
            emulate_tty=True,
        )
        actions.append(device_process)
        # The forwarder only exits on error. Stop the whole bring-up instead of
        # leaving the receiver retrying forever.
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=device_process,
            on_exit=_on_device_exit,
        )))

    if start_sensors:
        sensors_launch = os.path.join(
            get_package_share_directory('flyrat_sensors'),
            'launch',
            'sensors.launch.py')
        actions.append(TimerAction(
            period=1.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    'host': '127.0.0.1',
                    'port': local_port,
                    'stereo_color': LaunchConfiguration('stereo_color'),
                    'acc_lsb_per_g': LaunchConfiguration('acc_lsb_per_g'),
                    'gyro_lsb_per_dps': LaunchConfiguration('gyro_lsb_per_dps'),
                    'imu_accel_axis': LaunchConfiguration('imu_accel_axis'),
                    'imu_gyro_axis': LaunchConfiguration('imu_gyro_axis'),
                }.items(),
            )],
        ))

    if start_slam:
        vocabulary = _resolve_vocabulary(context)
        settings = _resolve_settings(context)
        actions.append(LogInfo(
            msg=f'[bringup] ORB vocabulary: {vocabulary}'))
        actions.append(LogInfo(
            msg=f'[bringup] SLAM settings: {settings}'))
        slam_launch = os.path.join(
            get_package_share_directory('flyrat_slam'),
            'launch',
            'slam.launch.py')
        actions.append(TimerAction(
            period=1.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'vocabulary_path': vocabulary,
                    'settings_path': settings,
                    'mode': LaunchConfiguration('mode'),
                    'trajectory_path': LaunchConfiguration('trajectory_path'),
                }.items(),
            )],
        ))

    if start_foxglove:
        actions.append(TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml',
                ])),
                launch_arguments={
                    'port': LaunchConfiguration('foxglove_port'),
                    'address': '0.0.0.0',
                }.items(),
            )],
        ))

    # adb forward and device-process cleanup lives in .script/bringup_real.sh.
    # Its trap runs reliably after ros2 launch exits; OnShutdown actions can be
    # skipped when launch is interrupted with SIGINT.
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary_path', default_value='',
            description='ORBvoc.txt path. Defaults to FLYRAT_ORB_VOCAB or the '
                        'repository copy under playground/ORB_SLAM3.'),
        DeclareLaunchArgument(
            'settings_path', default_value='',
            description='ORB-SLAM3 settings YAML. Defaults to the source-tree '
                        'uvc_640x480_imu.yaml, then the installed copy.'),
        DeclareLaunchArgument(
            'adb_serial', default_value=os.environ.get('ADB_SERIAL', ''),
            description='adb device serial; empty uses the only connected device.'),
        DeclareLaunchArgument(
            'local_port', default_value='5000',
            description='Host TCP port used by the adb forward.'),
        DeclareLaunchArgument(
            'device_port', default_value='5000',
            description='TCP port the device forwarder listens on.'),
        DeclareLaunchArgument(
            'device_host', default_value='127.0.0.1',
            description='Interface the device forwarder binds to.'),
        DeclareLaunchArgument(
            'device_binary', default_value='/tmp/flyrat_sensor_forwarder',
            description='Path to flyrat_sensor_forwarder on the device.'),
        DeclareLaunchArgument(
            'start_adb_forward', default_value='true',
            description='Create and remove adb forward tcp:LOCAL->tcp:DEVICE.'),
        DeclareLaunchArgument(
            'start_device', default_value='true',
            description='Start flyrat_sensor_forwarder on the device.'),
        DeclareLaunchArgument(
            'start_sensors', default_value='true',
            description='Start sensor_receiver_node.'),
        DeclareLaunchArgument(
            'start_slam', default_value='true',
            description='Start slam_node.'),
        DeclareLaunchArgument(
            'mode', default_value='imu_stereo',
            description="SLAM mode: 'imu_stereo' or 'stereo'."),
        DeclareLaunchArgument(
            'start_foxglove', default_value='false',
            description='Start foxglove_bridge as well.'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='foxglove_bridge WebSocket port.'),
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
            description="Accelerometer axes: 'none' or 'frd_to_flu'."),
        DeclareLaunchArgument(
            'imu_gyro_axis', default_value='bf_to_flu',
            description="Gyroscope axes: 'bf_to_flu', 'frd_to_flu' or 'none'."),
        DeclareLaunchArgument(
            'trajectory_path', default_value='',
            description='Optional TUM trajectory file written when SLAM exits.'),
        OpaqueFunction(function=_launch_setup),
    ])
