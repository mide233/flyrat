# SPDX-License-Identifier: MIT
#
# One-command bring-up for the real vehicle sensor + odometry pipeline:
#
#   adb forward -> device forwarder -> sensor_receiver_node -> odometry_node
#   (the push step is handled by .script/bringup_real.sh)
#
# It can also start foxglove_bridge, so the whole stack runs from one terminal.
# .script/bringup_real.sh wraps this launch and cleans up adb forward and the
# device-side forwarder when it exits.
#
# Usage:
#   ros2 launch flyrat_odometry bringup.launch.py
#
# All arguments have usable defaults; see the declared arguments below.
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


def _resolve_camera_config(context):
    explicit = _text(context, 'camera_config_path')
    if explicit:
        return explicit

    source = _repo_file('libflyrat/configs/camera/uvc_640x480.yaml')
    if source:
        # Prefer the source tree so edits to the calibration take effect
        # without rebuilding the ROS package.
        return source

    return os.path.join(
        get_package_share_directory('flyrat_odometry'),
        'config',
        'uvc_640x480.yaml')


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
    start_odometry = _flag(context, 'start_odometry')
    start_foxglove = _flag(context, 'start_foxglove')
    backend = _text(context, 'odometry_backend') or 'tio'
    if backend not in ('stereo', 'tio'):
        raise RuntimeError(
            f"odometry_backend must be 'stereo' or 'tio', got {backend!r}")

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
        camera_device = _text(context, 'camera_device')
        camera_width = _text(context, 'camera_width')
        camera_height = _text(context, 'camera_height')
        camera_fps = _text(context, 'camera_fps')
        camera_exposure_us = _text(context, 'camera_exposure_us')
        camera_gain = _text(context, 'camera_gain')
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
            f'--listen {shlex.quote(device_host)}:{device_port} '
            f'--camera-device {shlex.quote(camera_device)} '
            f'--camera-width {shlex.quote(camera_width)} '
            f'--camera-height {shlex.quote(camera_height)} '
            f'--camera-fps {shlex.quote(camera_fps)} '
            f'--camera-exposure-us {shlex.quote(camera_exposure_us)} '
            f'--camera-gain {shlex.quote(camera_gain)}')
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
                    'acc_lsb_per_g': LaunchConfiguration('acc_lsb_per_g'),
                    'gyro_lsb_per_dps': LaunchConfiguration('gyro_lsb_per_dps'),
                    'imu_accel_axis': LaunchConfiguration('imu_accel_axis'),
                    'imu_gyro_axis': LaunchConfiguration('imu_gyro_axis'),
                    'camera_tf_enable': LaunchConfiguration('camera_tf_enable'),
                    'camera_x': LaunchConfiguration('camera_x'),
                    'camera_y': LaunchConfiguration('camera_y'),
                    'camera_z': LaunchConfiguration('camera_z'),
                    'camera_roll': LaunchConfiguration('camera_roll'),
                    'camera_pitch': LaunchConfiguration('camera_pitch'),
                    'camera_yaw': LaunchConfiguration('camera_yaw'),
                }.items(),
            )],
        ))

    if start_odometry:
        if backend == 'tio':
            tio_launch = os.path.join(
                get_package_share_directory('flyrat_odometry'),
                'launch',
                'tio_odometry.launch.py')
            tio_camera = _flag(context, 'tio_camera')
            model_path = _text(context, 'tio_model')
            if not model_path:
                model_path = os.path.join(
                    get_package_share_directory('flyrat_odometry'),
                    'models',
                    'tio_former_window64_camera.pt'
                    if tio_camera else 'tio_former_window64.pt')
            actions.append(LogInfo(msg=(
                f'[bringup] TIO-Former backend: {model_path} '
                f'(camera={"on" if tio_camera else "off"})')))
            actions.append(TimerAction(
                period=1.0,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(tio_launch),
                    launch_arguments={
                        'model_path': model_path,
                        'world_frame': 'odometry_map',
                        'body_frame': 'imu_link',
                        'warmup_frames': LaunchConfiguration('tio_warmup_frames'),
                        'trajectory_path': LaunchConfiguration('trajectory_path'),
                        'camera_enabled': 'true' if tio_camera else 'false',
                        'camera_width': LaunchConfiguration('tio_camera_width'),
                        'camera_height': LaunchConfiguration('tio_camera_height'),
                        'camera_topic': LaunchConfiguration('tio_camera_topic'),
                        'camera_max_skew_s': LaunchConfiguration(
                            'tio_camera_max_skew_s'),
                    }.items(),
                )],
            ))
        else:
            calibration = _resolve_camera_config(context)
            actions.append(LogInfo(
                msg=f'[bringup] odometry calibration: {calibration}'))
            odometry_launch = os.path.join(
                get_package_share_directory('flyrat_odometry'),
                'launch',
                'odometry.launch.py')
            actions.append(TimerAction(
                period=1.0,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(odometry_launch),
                    launch_arguments={
                        'camera_config_path': calibration,
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
            'camera_config_path', default_value='',
            description='Camera/IMU calibration YAML. Defaults to the '
                        'source-tree configs/camera/uvc_640x480.yaml, then the '
                        'installed copy.'),
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
            'camera_device', default_value='/dev/video10',
            description='Device monochrome camera video node.'),
        DeclareLaunchArgument(
            'camera_width', default_value='320',
            description='Camera frame width (QVGA).'),
        DeclareLaunchArgument(
            'camera_height', default_value='240',
            description='Camera frame height (QVGA).'),
        DeclareLaunchArgument(
            'camera_fps', default_value='15',
            description='Camera output FPS after capture-side throttling.'),
        DeclareLaunchArgument(
            'camera_exposure_us', default_value='5000',
            description='Manual camera exposure in microseconds (0 keeps the '
                        'camera auto exposure). Shorter exposure reduces motion blur.'),
        DeclareLaunchArgument(
            'camera_gain', default_value='-1',
            description='Camera gain; -1 leaves the current (usually auto) gain '
                        'untouched, which compensates a shorter exposure.'),
        DeclareLaunchArgument(
            'camera_tf_enable', default_value='false',
            description='Publish imu_link -> camera optical frame TF.'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
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
            'start_odometry', default_value='true',
            description='Start the selected odometry backend.'),
        DeclareLaunchArgument(
            'odometry_backend', default_value='tio',
            description="Odometry backend: 'tio' (working experimental) or 'stereo' (legacy scaffold)."),
        DeclareLaunchArgument(
            'tio_warmup_frames', default_value='64',
            description='ToF frames collected before the first TIO pose.'),
        DeclareLaunchArgument(
            'tio_camera', default_value='true',
            description='Use the forward-camera TIO-Former model and feed '
                        '/flyrat/sensors/camera/image_raw to it.'),
        DeclareLaunchArgument(
            'tio_model', default_value='',
            description='Override the TIO TorchScript path. Empty selects the '
                        'camera or ToF-only model installed by '
                        'flyrat_odometry.'),
        DeclareLaunchArgument(
            'tio_camera_topic', default_value='/flyrat/sensors/camera/image_raw',
            description='mono8 topic consumed by the camera TIO model.'),
        DeclareLaunchArgument(
            'tio_camera_width', default_value='160',
            description='Model camera input width (must match the export).'),
        DeclareLaunchArgument(
            'tio_camera_height', default_value='120',
            description='Model camera input height (must match the export).'),
        DeclareLaunchArgument(
            'tio_camera_max_skew_s', default_value='0.08',
            description='Max camera/ToF timestamp difference; the camera frame '
                        'nearest the ToF stamp is used, older frames count as missing.'),
        DeclareLaunchArgument(
            'mode', default_value='imu_stereo',
            description="Odometry mode: 'imu_stereo' or 'stereo'."),
        DeclareLaunchArgument(
            'start_foxglove', default_value='false',
            description='Start foxglove_bridge as well.'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='foxglove_bridge WebSocket port.'),
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
            'imu_gyro_axis', default_value='none',
            description="Gyroscope axes: 'none', 'bf_to_flu' or 'frd_to_flu'."),
        DeclareLaunchArgument(
            'trajectory_path', default_value='',
            description='Optional TUM trajectory file written when odometry exits.'),
        OpaqueFunction(function=_launch_setup),
    ])
