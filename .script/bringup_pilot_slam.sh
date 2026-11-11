#!/usr/bin/env bash
#
# One-command onboard SLAM bring-up:
#
#   adb push -> adb forward 5001 -> device flyrat_pilot (SchurVINS)
#            -> host flyrat_pilot_bridge -> ROS 2 topics/TF -> Foxglove
#
# The RK3566 runs the whole SchurVINS frontend locally. Only 64-byte body poses
# cross the USB link, so the camera and IMU streams stay on the device.
#
# Usage:
#   .script/bringup_pilot_slam.sh [start_foxglove:=true] [fps:=25] ...
#
# Environment:
#   ROS_DISTRO          ROS 2 distribution (default: jazzy)
#   FLYRAT_ROS_WS       ROS 2 workspace (default: <repo>/flyrat_play/ros2_ws)
#   FLYRAT_PILOT_BIN    Host aarch64 binary (default: <repo>/build-arm/flyrat_pilot)
#   FLYRAT_PUSH_PILOT   Set to 0 to skip pushing the binary
#   ADB_SERIAL          Device serial when several devices are connected
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WS="${FLYRAT_ROS_WS:-${REPO_ROOT}/flyrat_play/ros2_ws}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${ROS_WS}/install/setup.bash"

HOST_BINARY="${FLYRAT_PILOT_BIN:-${REPO_ROOT}/build-arm/flyrat_pilot}"
DEVICE_BINARY="/tmp/flyrat_pilot"
DEVICE_DIR="/tmp/flyrat/config"
LOCAL_PORT=5001
CAMERA_CONFIG="${REPO_ROOT}/libflyrat/configs/camera/uvc_640x480.yaml"
IMU_CONFIG="${REPO_ROOT}/flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml"
FPS=25
MAX_FRAMES=0
SLAM_CPU=-1
STAMP_SOURCE=device
TIME_OFFSET=0.0
TRAJECTORY="/tmp/flyrat_pilot.tum"
START_FOXGLOVE=0
LAUNCH_ARGS=()
FOXGLOVE_PID=""

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

for arg in "$@"; do
    case "${arg}" in
        start_foxglove:=true|start_foxglove:=1) START_FOXGLOVE=1 ;;
        start_foxglove:=*) START_FOXGLOVE=0 ;;
        fps:=*) FPS="${arg#fps:=}" ;;
        max_frames:=*) MAX_FRAMES="${arg#max_frames:=}" ;;
        slam_cpu:=*) SLAM_CPU="${arg#slam_cpu:=}" ;;
        stamp_source:=*) STAMP_SOURCE="${arg#stamp_source:=}" ;;
        time_offset:=*) TIME_OFFSET="${arg#time_offset:=}" ;;
        trajectory:=*) TRAJECTORY="${arg#trajectory:=}" ;;
        local_port:=*) LOCAL_PORT="${arg#local_port:=}" ;;
        camera_config:=*) CAMERA_CONFIG="${arg#camera_config:=}" ;;
        imu_config:=*) IMU_CONFIG="${arg#imu_config:=}" ;;
        world_frame:=*|body_frame:=*|publish_tf:=*) LAUNCH_ARGS+=("${arg}") ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done

if [ ! -f "${ROS_SETUP}" ]; then
    echo "error: ROS 2 ${ROS_DISTRO} not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [ ! -f "${WS_SETUP}" ]; then
    echo "error: workspace not built: ${WS_SETUP}" >&2
    echo "Build it first: .script/build_ros2.sh" >&2
    exit 1
fi
if [ ! -f "${CAMERA_CONFIG}" ]; then
    echo "error: camera calibration not found: ${CAMERA_CONFIG}" >&2
    exit 1
fi
if [ ! -f "${IMU_CONFIG}" ]; then
    echo "error: IMU calibration not found: ${IMU_CONFIG}" >&2
    exit 1
fi

ADB=(adb)
if [ -n "${ADB_SERIAL:-}" ]; then
    ADB+=(-s "${ADB_SERIAL}")
fi

echo "[bringup] adb devices:"
"${ADB[@]}" devices
if ! "${ADB[@]}" get-state >/dev/null 2>&1; then
    echo "error: no adb device. Connect the RK3566 or set ADB_SERIAL." >&2
    exit 1
fi

if [ "${FLYRAT_PUSH_PILOT:-1}" != "0" ]; then
    if [ ! -f "${HOST_BINARY}" ]; then
        echo "error: ${HOST_BINARY} not found." >&2
        echo "Build it with .script/build_pilot_arm.sh" >&2
        exit 1
    fi
    echo "[bringup] pushing ${HOST_BINARY} -> ${DEVICE_BINARY}"
    "${ADB[@]}" push "${HOST_BINARY}" "${DEVICE_BINARY}" >/dev/null
    "${ADB[@]}" shell "chmod +x ${DEVICE_BINARY}"
else
    "${ADB[@]}" shell "test -x ${DEVICE_BINARY}" || {
        echo "error: ${DEVICE_BINARY} is missing on the device." >&2
        exit 1
    }
fi

echo "[bringup] pushing calibration files to ${DEVICE_DIR}"
"${ADB[@]}" shell "mkdir -p ${DEVICE_DIR}"
"${ADB[@]}" push "${CAMERA_CONFIG}" "${DEVICE_DIR}/camera.yaml" >/dev/null
"${ADB[@]}" push "${IMU_CONFIG}" "${DEVICE_DIR}/imu.yaml" >/dev/null

cleanup() {
    if [ "${START_FOXGLOVE}" -eq 1 ] && [ -n "${FOXGLOVE_PID}" ] \
        && kill -0 "${FOXGLOVE_PID}" 2>/dev/null; then
        kill -TERM "${FOXGLOVE_PID}" 2>/dev/null || true
        wait "${FOXGLOVE_PID}" 2>/dev/null || true
    fi
    "${ADB[@]}" shell "pkill -f '^${DEVICE_BINARY}'" >/dev/null 2>&1 || true
    "${ADB[@]}" forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

"${ADB[@]}" forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
"${ADB[@]}" forward "tcp:${LOCAL_PORT}" "tcp:${LOCAL_PORT}" >/dev/null

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"
set -u

if [ "${START_FOXGLOVE}" -eq 1 ]; then
    "${SCRIPT_DIR}/foxglove.sh" >/tmp/flyrat_foxglove.log 2>&1 &
    FOXGLOVE_PID=$!
    echo "[bringup] foxglove bridge started (log: /tmp/flyrat_foxglove.log)"
fi

# Restart a stale device process. The binary path is longer than the 15-char
# comm limit, so match the anchored command line instead of using pkill -x.
echo "[bringup] starting onboard SchurVINS (${FPS} fps)"
"${ADB[@]}" shell "pkill -f '^${DEVICE_BINARY}' 2>/dev/null; \
    exec ${DEVICE_BINARY} \
    --camera-config ${DEVICE_DIR}/camera.yaml \
    --imu-config ${DEVICE_DIR}/imu.yaml \
    --msp-device /dev/ttyS3 \
    --camera-device /dev/video10 \
    --fps ${FPS} \
    --max-frames ${MAX_FRAMES} \
    --slam-cpu ${SLAM_CPU} \
    --trajectory ${TRAJECTORY}" &

echo "[bringup] host bridge on tcp:${LOCAL_PORT}"
ros2 launch flyrat_pilot_bridge pilot_bridge.launch.py \
    port:="${LOCAL_PORT}" \
    camera_config_path:="${CAMERA_CONFIG}" \
    stamp_source:="${STAMP_SOURCE}" \
    time_offset:="${TIME_OFFSET}" \
    "${LAUNCH_ARGS[@]+"${LAUNCH_ARGS[@]}"}"
