#!/usr/bin/env bash
#
# One-command bring-up for the real-vehicle sensor + odometry stack:
#
#   adb push -> adb forward -> device forwarder
#            -> sensor_receiver_node -> odometry_node
#
# Usage:
#   .script/bringup_real.sh [launch arguments...]
#
# Examples:
#   .script/bringup_real.sh
#   .script/bringup_real.sh start_foxglove:=true
#   .script/bringup_real.sh start_odometry:=false
#   .script/bringup_real.sh odometry_backend:=tio
#   .script/bringup_real.sh tio_camera:=false   # ToF+IMU model fallback
#   .script/bringup_real.sh record:=true record_imu_only:=true record_duration:=3h
#   .script/bringup_real.sh record:=true record_topics:="/flyrat/sensors/imu /flyrat/sensors/camera/image_raw /flyrat/sensors/tof_0 /flyrat/sensors/tof_1 /flyrat/sensors/tof_2 /flyrat/sensors/tof_3"
#
# Environment:
#   ROS_DISTRO                ROS 2 distribution (default: jazzy)
#   FLYRAT_ROS_WS             ROS 2 workspace (default: <repo>/flyrat_play/ros2_ws)
#   ADB_SERIAL                Device serial when several devices are connected
#   FLYRAT_SENSOR_FORWARDER   Host aarch64 binary (default: <repo>/build-arm/...)
#   FLYRAT_DEVICE_BINARY      Device path (default: /tmp/flyrat_sensor_forwarder)
#   FLYRAT_PUSH_FORWARDER     Set to 0 to skip the push step
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WS="${FLYRAT_ROS_WS:-${REPO_ROOT}/flyrat_play/ros2_ws}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${ROS_WS}/install/setup.bash"

HOST_BINARY="${FLYRAT_SENSOR_FORWARDER:-${REPO_ROOT}/build-arm/flyrat_sensor_forwarder}"
DEVICE_BINARY="${FLYRAT_DEVICE_BINARY:-/tmp/flyrat_sensor_forwarder}"

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS 2 ${ROS_DISTRO} not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [ ! -f "${WS_SETUP}" ]; then
    echo "workspace not built: ${WS_SETUP}" >&2
    echo "Build it first: .script/build_ros2.sh" >&2
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

parse_duration() {
    case "$1" in
        *h) echo $(( ${1%h} * 3600 )) ;;
        *m) echo $(( ${1%m} * 60 )) ;;
        *s) echo "${1%s}" ;;
        *) echo "$1" ;;
    esac
}

LAUNCH_ARGS=()
LOCAL_PORT=5000
START_ADB_FORWARD=1
START_DEVICE=1
RECORD=0
RECORD_PATH=""
RECORD_TOPICS="/flyrat/sensors/imu /flyrat/sensors/camera/image_raw /flyrat/sensors/tof_0 /flyrat/sensors/tof_1 /flyrat/sensors/tof_2 /flyrat/sensors/tof_3"
RECORD_IMU_ONLY=0
RECORD_STORAGE=mcap
RECORD_DURATION=0
RECORD_WARMUP=8
RECORD_PID=""
WATCHDOG_PID=""
for arg in "$@"; do
    case "${arg}" in
        device_binary:=*) DEVICE_BINARY="${arg#device_binary:=}"; LAUNCH_ARGS+=("${arg}") ;;
        local_port:=*) LOCAL_PORT="${arg#local_port:=}"; LAUNCH_ARGS+=("${arg}") ;;
        start_adb_forward:=false|start_adb_forward:=0) START_ADB_FORWARD=0; LAUNCH_ARGS+=("${arg}") ;;
        start_device:=false|start_device:=0) START_DEVICE=0; LAUNCH_ARGS+=("${arg}") ;;
        record:=true|record:=1) RECORD=1 ;;
        record:=*) RECORD=0 ;;
        record_path:=*) RECORD_PATH="${arg#record_path:=}" ;;
        record_topics:=*) RECORD_TOPICS="${arg#record_topics:=}" ;;
        record_imu_only:=true|record_imu_only:=1) RECORD_IMU_ONLY=1 ;;
        record_imu_only:=*) RECORD_IMU_ONLY=0 ;;
        record_storage:=*) RECORD_STORAGE="${arg#record_storage:=}" ;;
        record_duration:=*) RECORD_DURATION="${arg#record_duration:=}" ;;
        record_warmup:=*) RECORD_WARMUP="${arg#record_warmup:=}" ;;
        *) LAUNCH_ARGS+=("${arg}") ;;
    esac
done

RECORD_DURATION="$(parse_duration "${RECORD_DURATION}")"
if ! [[ "${RECORD_DURATION}" =~ ^[0-9]+$ ]]; then
    echo "error: record_duration must be seconds, or use a suffix such as 30m/3h" >&2
    exit 1
fi
if ! [[ "${RECORD_WARMUP}" =~ ^[0-9]+$ ]]; then
    echo "error: record_warmup must be a non-negative integer number of seconds" >&2
    exit 1
fi

# /tmp is cleared on reboot, so refresh the device binary whenever the host
# cross-compiled one exists.
if [ "${FLYRAT_PUSH_FORWARDER:-1}" != "0" ] && [ -f "${HOST_BINARY}" ]; then
    echo "[bringup] pushing ${HOST_BINARY} -> ${DEVICE_BINARY}"
    "${ADB[@]}" push "${HOST_BINARY}" "${DEVICE_BINARY}" >/dev/null
    "${ADB[@]}" shell "chmod +x ${DEVICE_BINARY}"
elif ! "${ADB[@]}" shell "test -x ${DEVICE_BINARY}" 2>/dev/null; then
    echo "error: ${DEVICE_BINARY} is missing on the device." >&2
    echo "Build and push it first:" >&2
    echo "  .script/build_sensor_forwarder_arm.sh" >&2
    exit 1
fi

# ROS setup scripts reference unset variables, so nounset must be off while
# sourcing them.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"
set -u

if [ "${RECORD}" -eq 1 ]; then
    if [ -z "${RECORD_PATH}" ]; then
        RECORD_PATH="/tmp/flyrat_bags/$(date +%Y%m%d_%H%M%S)"
    fi
    mkdir -p "$(dirname "${RECORD_PATH}")"
    if [ "${RECORD_IMU_ONLY}" -eq 1 ]; then
        RECORD_TOPICS="/flyrat/sensors/imu"
    fi
    # shellcheck disable=SC2206
    RECORD_TOPIC_ARRAY=(${RECORD_TOPICS})
    if [ "${#RECORD_TOPIC_ARRAY[@]}" -eq 0 ]; then
        echo "error: record_topics is empty" >&2
        exit 1
    fi
    echo "[bringup] recording ${RECORD_TOPIC_ARRAY[*]} -> ${RECORD_PATH}"
    # Keep the recorder outside ros2 launch: launch does not reliably forward
    # Ctrl-C to rosbag2 when stdin is not a tty, so this process is stopped
    # with SIGTERM from cleanup() and flushes the bag.
    ros2 bag record -s "${RECORD_STORAGE}" -o "${RECORD_PATH}" \
        --topics "${RECORD_TOPIC_ARRAY[@]}" \
        >"${RECORD_PATH}.log" 2>&1 &
    RECORD_PID=$!
    if [ "${RECORD_DURATION}" -gt 0 ]; then
        (
            if [ "${RECORD_WARMUP}" -gt 0 ]; then
                echo "[bringup] recording warmup: ${RECORD_WARMUP}s (device startup)"
                sleep "${RECORD_WARMUP}"
            fi
            remaining="${RECORD_DURATION}"
            while [ "${remaining}" -gt 0 ]; do
                if [ "${remaining}" -le 10 ] || [ $((remaining % 60)) -eq 0 ]; then
                    echo "[bringup] recording stops in ${remaining}s"
                fi
                sleep 1
                remaining=$((remaining - 1))
            done
            echo "[bringup] recording duration reached; finalizing ${RECORD_PATH}"
            kill -TERM "${RECORD_PID}" 2>/dev/null || true
            wait "${RECORD_PID}" 2>/dev/null || true
            echo "[bringup] bag finalized: ${RECORD_PATH}"
            echo "[bringup] bringup is still running; press Ctrl-C to stop it"
        ) &
        WATCHDOG_PID=$!
    fi
fi

has_arg() {
    local prefix="$1"
    local arg
    for arg in "${LAUNCH_ARGS[@]}"; do
        [[ "${arg}" == "${prefix}"* ]] && return 0
    done
    return 1
}

has_arg 'device_binary:=' || LAUNCH_ARGS+=("device_binary:=${DEVICE_BINARY}")
if [ -n "${ADB_SERIAL:-}" ]; then
    has_arg 'adb_serial:=' || LAUNCH_ARGS+=("adb_serial:=${ADB_SERIAL}")
fi

cleanup() {
    if [ -n "${WATCHDOG_PID}" ] && kill -0 "${WATCHDOG_PID}" 2>/dev/null; then
        kill -TERM "${WATCHDOG_PID}" 2>/dev/null || true
        wait "${WATCHDOG_PID}" 2>/dev/null || true
    fi
    if [ -n "${RECORD_PID}" ] && kill -0 "${RECORD_PID}" 2>/dev/null; then
        kill -TERM "${RECORD_PID}" 2>/dev/null || true
        wait "${RECORD_PID}" 2>/dev/null || true
    fi
    if [ "${START_ADB_FORWARD}" -eq 1 ]; then
        "${ADB[@]}" forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
    fi
    if [ "${START_DEVICE}" -eq 1 ]; then
        # pkill -x cannot match the 15-character process name; anchor the full
        # command line at the executable path instead.
        "${ADB[@]}" shell "pkill -f '^${DEVICE_BINARY}'" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

ros2 launch flyrat_odometry bringup.launch.py "${LAUNCH_ARGS[@]}"
