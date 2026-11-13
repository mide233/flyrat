#!/usr/bin/env bash
#
# Unified entry point for the operational bring-up flows. Each mode is
# implemented in this file so there is a single place to discover and maintain
# the supported modes. build/clean/foxglove still delegate to their dedicated
# scripts.
#
# Usage:
#   .script/bringup.sh sensors|real [launch arguments...]
#   .script/bringup.sh slam|pilot    [start_foxglove:=true] [fps:=25] [decode:=mpp] ...
#   .script/bringup.sh raw|calib     [output:=DIR] [duration:=10m] [fps:=20] ...
#   .script/bringup.sh build         [build_all.sh arguments...]
#   .script/bringup.sh clean         [clean_build.sh arguments...]
#   .script/bringup.sh foxglove      [foxglove.sh arguments...]
#
# sensors/real forwards every argument to ros2 launch; slam and raw also accept
# -h/--help for their mode-specific option list.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WS="${FLYRAT_ROS_WS:-${REPO_ROOT}/flyrat_play/ros2_ws}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${ROS_WS}/install/setup.bash"

usage() {
    cat <<'EOF'
Usage:
  .script/bringup.sh sensors|real [launch arguments...]
  .script/bringup.sh slam|pilot    [start_foxglove:=true] [fps:=25] [decode:=mpp] ...
  .script/bringup.sh raw|calib     [output:=DIR] [duration:=10m] [fps:=20] ...
  .script/bringup.sh build         [build_all.sh arguments...]
  .script/bringup.sh clean         [clean_build.sh arguments...]
  .script/bringup.sh foxglove      [foxglove.sh arguments...]

Run `.script/bringup.sh slam --help` or `.script/bringup.sh raw --help` for the
mode-specific options. sensors|real passes all arguments to ros2 launch.
EOF
}

slam_usage() {
    cat <<'EOF'

One-command onboard SLAM bring-up:

  adb push -> adb forward 5001 -> device flyrat_pilot (SchurVINS)
           -> host flyrat_pilot_bridge -> ROS 2 topics/TF -> Foxglove

The RK3566 runs the whole SchurVINS frontend locally. Only 64-byte body poses
cross the USB link, so the camera and IMU streams stay on the device.

Usage:
  .script/bringup.sh slam [start_foxglove:=true] [fps:=25] [decode:=mpp] ...

Environment:
  ROS_DISTRO          ROS 2 distribution (default: jazzy)
  FLYRAT_ROS_WS       ROS 2 workspace (default: <repo>/flyrat_play/ros2_ws)
  FLYRAT_PILOT_BIN    Host aarch64 binary (default: <repo>/build/arm/flyrat_pilot)
  FLYRAT_PUSH_PILOT   Set to 0 to skip pushing the binary
  ADB_SERIAL          Device serial when several devices are connected

Options:
  fps:=N              Delivered camera/SLAM rate (default: 25); capture
                      always runs at the sensor maximum (60 for 1280x480)
  decode:=mpp|cpu     MJPEG decoder (default: mpp, falls back to cpu if MPP fails)
  trace:=DIR          Enable per-stage CSV tracing on the device. On exit the
                      directory is pulled to <repo>/trace/<name>.
  record:=DIR         Record camera JPEG + IMU to <DIR>/sensor_bag.bin.
  replay:=DIR         Replay a recorded bag; skips camera and MSP.
  slam_cpu:=N         Ignored: parallel stereo reprojection needs more than one
                      core, and std::async workers inherit the SLAM affinity.

EOF
}

raw_usage() {
    cat <<'EOF'

Record raw IMU counts and native stereo MJPEG on the vehicle, then pull the
result to the host. This bypasses flyrat_sensor_forwarder and the calibrated
pilot bag on purpose: re-calibration needs data before intrinsics, axis
mapping and time offsets are applied.

Usage:
  .script/bringup.sh raw [output:=DIR] [duration:=10m] [fps:=20]
      [capture_fps:=60] [imu_hz:=200] [local:=DIR] [check:=true]
      [export:=true] [require_capture:=true] [push:=0]
      [preview:=true] [preview_port:=5000] [foxglove:=true]
      [foxglove_port:=8765] [imu_corrections:=true]

preview:=true also streams the same raw IMU and stereo frames to the host.
The sensor receiver and Foxglove bridge are started automatically; the IMU
preview uses raw scale/axis values unless imu_corrections:=true is given.

Environment:
  ADB_SERIAL       adb device serial when several devices are attached
  BUILD_DIR        host build directory (default <repo>/build/arm)
  FLYRAT_PUSH_RAW  0 skips pushing the binary
  ROS_DISTRO       ROS 2 distribution (default jazzy)
  FLYRAT_ROS_WS    ROS 2 workspace (default <repo>/flyrat_play/ros2_ws)

EOF
}

# Shared adb setup: honour ADB_SERIAL so every mode targets the same device.
init_adb() {
    ADB=(adb)
    if [ -n "${ADB_SERIAL:-}" ]; then
        ADB+=(-s "${ADB_SERIAL}")
    fi
}

require_adb_device() {
    if ! "${ADB[@]}" get-state >/dev/null 2>&1; then
        echo "error: $1" >&2
        exit 1
    fi
}

# ROS setup scripts reference unset variables, so nounset must be off while
# sourcing them.
source_ros_workspace() {
    local ros_setup="${1:-${ROS_SETUP}}"
    local ws_setup="${2:-${WS_SETUP}}"
    set +u
    # shellcheck disable=SC1090
    source "${ros_setup}"
    # shellcheck disable=SC1090
    source "${ws_setup}"
    set -u
}

parse_duration() {
    case "$1" in
        *h) echo $(( ${1%h} * 3600 )) ;;
        *m) echo $(( ${1%m} * 60 )) ;;
        *s) echo "${1%s}" ;;
        *) echo "$1" ;;
    esac
}

# Stop device processes by their anchored command line. pkill -x cannot match
# names longer than the 15-character comm limit, so match the full command line
# starting at the executable path. TERM is pkill's default signal and every
# caller wants a graceful stop.
stop_device_processes() {
    local pattern
    for pattern in "$@"; do
        "${ADB[@]}" shell "pkill -TERM -f '^${pattern}' 2>/dev/null || true" \
            >/dev/null 2>&1 || true
    done
}

# Install the per-mode cleanup function for the same signals as before.
install_cleanup_trap() {
    # shellcheck disable=SC2064
    trap "$1" EXIT INT TERM
}

# ---------------------------------------------------------------------------
# sensors|real: one-command bring-up for the real-vehicle sensor + odometry
# stack:
#
#   adb push -> adb forward -> device forwarder
#            -> sensor_receiver_node -> odometry_node
#
# Examples:
#   .script/bringup.sh sensors
#   .script/bringup.sh sensors start_foxglove:=true
#   .script/bringup.sh sensors start_odometry:=false
#   .script/bringup.sh sensors record:=true record_imu_only:=true record_duration:=3h
#   .script/bringup.sh sensors record:=true record_duration:=10m camera_fps:=30
#   .script/bringup.sh sensors camera_config_path:=/path/to/calibration.yaml
#
# Recording defaults to the combined stereo JPEG topic plus IMU and drops the
# camera to 30 fps unless camera_fps:= is given explicitly. Replay the bag and
# run stereo_splitter_node to recover the left/right raw image topics.
#
# Environment:
#   ROS_DISTRO                ROS 2 distribution (default: jazzy)
#   FLYRAT_ROS_WS             ROS 2 workspace (default: <repo>/flyrat_play/ros2_ws)
#   ADB_SERIAL                Device serial when several devices are connected
#   FLYRAT_SENSOR_FORWARDER   Host aarch64 binary (default: <repo>/build/arm/...)
#   FLYRAT_DEVICE_BINARY      Device path (default: /tmp/flyrat_sensor_forwarder)
#   FLYRAT_PUSH_FORWARDER     Set to 0 to skip the push step
#
# All arguments are forwarded to ros2 launch unchanged.
# ---------------------------------------------------------------------------
run_sensors() {
    HOST_BINARY="${FLYRAT_SENSOR_FORWARDER:-${REPO_ROOT}/build/arm/flyrat_sensor_forwarder}"
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

    init_adb

    echo "[bringup] adb devices:"
    "${ADB[@]}" devices

    require_adb_device "no adb device. Connect the RK3566 or set ADB_SERIAL."

    LAUNCH_ARGS=()
    LOCAL_PORT=5000
    START_ADB_FORWARD=1
    START_DEVICE=1
    RECORD=0
    RECORD_PATH=""
    RECORD_TOPICS="/flyrat/sensors/imu /flyrat/sensors/stereo/compressed"
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
        echo "  .script/build_pilot_arm.sh --static-forwarder" >&2
        exit 1
    fi

    source_ros_workspace

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
            --max-cache-size 268435456 \
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

    # Recording at 60 fps makes downstream consumers decode roughly 74 MB/s of
    # raw stereo images and can make rosbag2 fall behind. Default the delivered
    # camera rate to 30 fps while recording; capture still runs at the sensor
    # maximum, so the exposure limit stays short.
    if [ "${RECORD}" -eq 1 ] && [ "${RECORD_IMU_ONLY}" -eq 0 ] &&
        ! has_arg 'camera_fps:='; then
        LAUNCH_ARGS+=("camera_fps:=30")
    fi

    has_arg 'device_binary:=' || LAUNCH_ARGS+=("device_binary:=${DEVICE_BINARY}")
    if [ -n "${ADB_SERIAL:-}" ]; then
        has_arg 'adb_serial:=' || LAUNCH_ARGS+=("adb_serial:=${ADB_SERIAL}")
    fi

    cleanup_sensors() {
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
            stop_device_processes "${DEVICE_BINARY}"
        fi
    }
    install_cleanup_trap cleanup_sensors

    ros2 launch flyrat_odometry bringup.launch.py "${LAUNCH_ARGS[@]}"
}

# ---------------------------------------------------------------------------
# slam|pilot: adb push -> adb forward 5001 -> device flyrat_pilot -> host bridge
# ---------------------------------------------------------------------------
run_slam() {
    HOST_BINARY="${FLYRAT_PILOT_BIN:-${REPO_ROOT}/build/arm/flyrat_pilot}"
    DEVICE_BINARY="/tmp/flyrat_pilot"
    DEVICE_DIR="/tmp/flyrat/config"
    LOCAL_PORT=5001
    CAMERA_CONFIG="${REPO_ROOT}/libflyrat/configs/camera/uvc_640x480.yaml"
    IMU_CONFIG="${REPO_ROOT}/flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml"
    FPS=25
    MAX_FRAMES=0
    SLAM_CPU=-1
    SLAM_CPU_SET=0
    DECODE=mpp
    TRACE=""
    RECORD=""
    REPLAY=""
    STAMP_SOURCE=device
    TIME_OFFSET=0.0
    TRAJECTORY="/tmp/flyrat_pilot.tum"
    START_FOXGLOVE=0
    LAUNCH_ARGS=()
    FOXGLOVE_PID=""

    for arg in "$@"; do
        case "${arg}" in
            start_foxglove:=true|start_foxglove:=1) START_FOXGLOVE=1 ;;
            start_foxglove:=*) START_FOXGLOVE=0 ;;
            fps:=*) FPS="${arg#fps:=}" ;;
            max_frames:=*) MAX_FRAMES="${arg#max_frames:=}" ;;
            slam_cpu:=*) SLAM_CPU="${arg#slam_cpu:=}"; SLAM_CPU_SET=1 ;;
            decode:=*) DECODE="${arg#decode:=}" ;;
            trace:=off|trace:=0) TRACE="" ;;
            trace:=*) TRACE="${arg#trace:=}" ;;
            record:=*) RECORD="${arg#record:=}" ;;
            replay:=*) REPLAY="${arg#replay:=}" ;;
            stamp_source:=*) STAMP_SOURCE="${arg#stamp_source:=}" ;;
            time_offset:=*) TIME_OFFSET="${arg#time_offset:=}" ;;
            trajectory:=*) TRAJECTORY="${arg#trajectory:=}" ;;
            local_port:=*) LOCAL_PORT="${arg#local_port:=}" ;;
            camera_config:=*) CAMERA_CONFIG="${arg#camera_config:=}" ;;
            imu_config:=*) IMU_CONFIG="${arg#imu_config:=}" ;;
            world_frame:=*|body_frame:=*|publish_tf:=*) LAUNCH_ARGS+=("${arg}") ;;
            -h|--help) slam_usage; exit 0 ;;
            *) echo "error: unknown argument: ${arg}" >&2; slam_usage >&2; exit 2 ;;
        esac
    done

    if [ -n "${RECORD}" ] && [ -n "${REPLAY}" ]; then
        echo "error: record:= and replay:= are mutually exclusive" >&2
        exit 2
    fi

    case "${DECODE}" in
        cpu|mpp) ;;
        *) echo "error: decode must be 'cpu' or 'mpp'" >&2; exit 2 ;;
    esac

    if [ "${SLAM_CPU_SET}" -eq 1 ] && [ "${SLAM_CPU}" != "-1" ]; then
        echo "[bringup] warning: ignoring slam_cpu:=${SLAM_CPU}; parallel stereo" >&2
        echo "          reprojection workers inherit the affinity mask and would" >&2
        echo "          be confined to one core." >&2
        SLAM_CPU=-1
    fi

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

    init_adb

    echo "[bringup] adb devices:"
    "${ADB[@]}" devices
    require_adb_device "no adb device. Connect the RK3566 or set ADB_SERIAL."

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

    cleanup_slam() {
        if [ "${START_FOXGLOVE}" -eq 1 ] && [ -n "${FOXGLOVE_PID}" ] \
            && kill -0 "${FOXGLOVE_PID}" 2>/dev/null; then
            kill -TERM "${FOXGLOVE_PID}" 2>/dev/null || true
            wait "${FOXGLOVE_PID}" 2>/dev/null || true
        fi
        stop_device_processes "${DEVICE_BINARY}"
        if [ -n "${TRACE}" ] \
            && "${ADB[@]}" shell "test -d ${TRACE}" >/dev/null 2>&1; then
            mkdir -p "${REPO_ROOT}/trace"
            if "${ADB[@]}" pull "${TRACE}" "${REPO_ROOT}/trace/" >/dev/null 2>&1; then
                echo "[bringup] trace saved to ${REPO_ROOT}/trace/$(basename "${TRACE}")"
            fi
        fi
        "${ADB[@]}" forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
    }
    install_cleanup_trap cleanup_slam

    "${ADB[@]}" forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
    "${ADB[@]}" forward "tcp:${LOCAL_PORT}" "tcp:${LOCAL_PORT}" >/dev/null

    source_ros_workspace

    if [ "${START_FOXGLOVE}" -eq 1 ]; then
        "${SCRIPT_DIR}/foxglove.sh" >/tmp/flyrat_foxglove.log 2>&1 &
        FOXGLOVE_PID=$!
        echo "[bringup] foxglove bridge started (log: /tmp/flyrat_foxglove.log)"
    fi

    # Restart a stale device process. The binary path is longer than the 15-char
    # comm limit, so match the anchored command line instead of using pkill -x.
    SOURCE="camera"
    if [ -n "${REPLAY}" ]; then
        SOURCE="replay ${REPLAY}"
    fi
    echo "[bringup] starting onboard SchurVINS (${FPS} fps, decode ${DECODE}, ${SOURCE}${TRACE:+, trace ${TRACE}})"
    DEVICE_ARGS=(
        --camera-config "${DEVICE_DIR}/camera.yaml"
        --imu-config "${DEVICE_DIR}/imu.yaml"
        --msp-device /dev/ttyS3
        --camera-device /dev/video10
        --fps "${FPS}"
        --max-frames "${MAX_FRAMES}"
        --slam-cpu "${SLAM_CPU}"
        --decode "${DECODE}"
        --trajectory "${TRAJECTORY}"
    )
    if [ -n "${TRACE}" ]; then
        DEVICE_ARGS+=(--trace "${TRACE}")
    fi
    if [ -n "${RECORD}" ]; then
        DEVICE_ARGS+=(--record "${RECORD}")
    fi
    if [ -n "${REPLAY}" ]; then
        DEVICE_ARGS+=(--replay "${REPLAY}")
    fi
    "${ADB[@]}" shell "pkill -f '^${DEVICE_BINARY}' 2>/dev/null; \
        exec ${DEVICE_BINARY} ${DEVICE_ARGS[*]}" &

    echo "[bringup] host bridge on tcp:${LOCAL_PORT}"
    ros2 launch flyrat_pilot_bridge pilot_bridge.launch.py \
        port:="${LOCAL_PORT}" \
        camera_config_path:="${CAMERA_CONFIG}" \
        stamp_source:="${STAMP_SOURCE}" \
        time_offset:="${TIME_OFFSET}" \
        "${LAUNCH_ARGS[@]+"${LAUNCH_ARGS[@]}"}"
}

# ---------------------------------------------------------------------------
# raw|calib: record raw IMU counts + native stereo MJPEG, pull, optional export
# ---------------------------------------------------------------------------
run_raw() {
    BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build/arm}"
    DEVICE_BINARY="/tmp/flyrat_raw_recorder"
    LOCAL_ROOT="${LOCAL_ROOT:-${REPO_ROOT}/flyrat_raw}"
    REMOTE_DIR=""
    DURATION="0"
    FPS="20"
    CAPTURE_FPS="60"
    IMU_HZ="200"
    MSP_DEVICE="/dev/ttyS3"
    MSP_BAUD="921600"
    CAMERA_DEVICE="/dev/video10"
    CAMERA_WIDTH="1280"
    CAMERA_HEIGHT="480"
    CHECK=0
    EXPORT=0
    REQUIRE_CAPTURE=0
    PREVIEW=0
    FOXGLOVE=1
    PREVIEW_PORT=5000
    FOXGLOVE_PORT=8765
    IMU_CORRECTIONS="false"
    PUSH="${FLYRAT_PUSH_RAW:-1}"

    for arg in "$@"; do
        case "${arg}" in
            output:=*) REMOTE_DIR="${arg#output:=}" ;;
            local:=*) LOCAL_ROOT="${arg#local:=}" ;;
            duration:=*) DURATION="${arg#duration:=}" ;;
            fps:=*) FPS="${arg#fps:=}" ;;
            capture_fps:=*) CAPTURE_FPS="${arg#capture_fps:=}" ;;
            imu_hz:=*) IMU_HZ="${arg#imu_hz:=}" ;;
            msp_device:=*) MSP_DEVICE="${arg#msp_device:=}" ;;
            msp_baud:=*) MSP_BAUD="${arg#msp_baud:=}" ;;
            camera_device:=*) CAMERA_DEVICE="${arg#camera_device:=}" ;;
            camera_width:=*) CAMERA_WIDTH="${arg#camera_width:=}" ;;
            camera_height:=*) CAMERA_HEIGHT="${arg#camera_height:=}" ;;
            check:=true) CHECK=1 ;;
            check:=false) CHECK=0 ;;
            export:=true) EXPORT=1 ;;
            export:=false) EXPORT=0 ;;
            require_capture:=true) REQUIRE_CAPTURE=1 ;;
            require_capture:=false) REQUIRE_CAPTURE=0 ;;
            preview:=true) PREVIEW=1 ;;
            preview:=false) PREVIEW=0 ;;
            preview_port:=*) PREVIEW_PORT="${arg#preview_port:=}" ;;
            foxglove:=true) FOXGLOVE=1 ;;
            foxglove:=false) FOXGLOVE=0 ;;
            foxglove_port:=*) FOXGLOVE_PORT="${arg#foxglove_port:=}" ;;
            imu_corrections:=true) IMU_CORRECTIONS="true" ;;
            imu_corrections:=false) IMU_CORRECTIONS="false" ;;
            push:=0) PUSH=0 ;;
            push:=1) PUSH=1 ;;
            -h|--help) raw_usage; exit 0 ;;
            *) echo "error: unknown argument: ${arg}" >&2; exit 2 ;;
        esac
    done

    init_adb
    require_adb_device "no adb device; check USB and ADB_SERIAL"

    if [ "${PUSH}" -eq 1 ]; then
        HOST_BINARY="${BUILD_DIR}/flyrat_raw_recorder"
        if [ ! -x "${HOST_BINARY}" ]; then
            echo "error: ${HOST_BINARY} not found; run .script/build_pilot_arm.sh" >&2
            exit 1
        fi
        echo "[raw] pushing ${HOST_BINARY} -> ${DEVICE_BINARY}"
        "${ADB[@]}" push "${HOST_BINARY}" "${DEVICE_BINARY}" >/dev/null
        "${ADB[@]}" shell "chmod +x ${DEVICE_BINARY}"
    fi

    # The serial port and camera are exclusive. Stop the live paths before opening
    # them, otherwise the recorder gets EBUSY or mixed frames.
    stop_device_processes /tmp/flyrat_sensor_forwarder /tmp/flyrat_pilot "${DEVICE_BINARY}"

    DEVICE_ARGS=(
        --msp-device "${MSP_DEVICE}"
        --msp-baud "${MSP_BAUD}"
        --imu-hz "${IMU_HZ}"
        --camera-device "${CAMERA_DEVICE}"
        --camera-width "${CAMERA_WIDTH}"
        --camera-height "${CAMERA_HEIGHT}"
        --camera-capture-fps "${CAPTURE_FPS}"
        --camera-fps "${FPS}"
    )
    if [ "${REQUIRE_CAPTURE}" -eq 1 ]; then
        DEVICE_ARGS+=(--require-capture-fps)
    fi
    if [ "${PREVIEW}" -eq 1 ]; then
        DEVICE_ARGS+=(--preview --preview-address 127.0.0.1
            --preview-port "${PREVIEW_PORT}")
    fi

    if [ "${CHECK}" -eq 1 ]; then
        echo "[raw] checking devices"
        exec "${ADB[@]}" shell "exec ${DEVICE_BINARY} --check ${DEVICE_ARGS[*]}"
    fi

    if [ -z "${REMOTE_DIR}" ]; then
        REMOTE_DIR="/userdata/flyrat_raw/$(date +%Y%m%d_%H%M%S)"
    fi
    REMOTE_DIR="${REMOTE_DIR%/}"
    "${ADB[@]}" shell "mkdir -p '${REMOTE_DIR}'"

    PREVIEW_PID=""
    FOXGLOVE_PID=""
    RECORD_PID=""

    start_preview() {
        local ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
        local ros_ws="${FLYRAT_ROS_WS:-${REPO_ROOT}/flyrat_play/ros2_ws}"
        local ws_setup="${ros_ws}/install/setup.bash"

        if [ ! -f "${ros_setup}" ]; then
            echo "error: ROS 2 setup not found: ${ros_setup}" >&2
            exit 1
        fi
        if [ ! -f "${ws_setup}" ]; then
            echo "error: ROS workspace not built: ${ws_setup}" >&2
            echo "Build it first: .script/build_ros2.sh" >&2
            exit 1
        fi

        source_ros_workspace "${ros_setup}" "${ws_setup}"

        # The device listens on 127.0.0.1 inside its network namespace; adb forward
        # makes that port reachable from the host receiver.
        "${ADB[@]}" forward --remove "tcp:${PREVIEW_PORT}" >/dev/null 2>&1 || true
        echo "[preview] adb forward tcp:${PREVIEW_PORT} -> tcp:${PREVIEW_PORT}"
        if ! "${ADB[@]}" forward "tcp:${PREVIEW_PORT}" "tcp:${PREVIEW_PORT}" >/dev/null; then
            echo "error: adb forward failed for tcp:${PREVIEW_PORT}" >&2
            exit 1
        fi

        echo "[preview] sensor receiver: imu_corrections:=${IMU_CORRECTIONS}"
        ros2 launch flyrat_sensors sensors.launch.py \
            host:=127.0.0.1 \
            port:="${PREVIEW_PORT}" \
            imu_corrections:="${IMU_CORRECTIONS}" \
            >/tmp/flyrat_raw_preview.log 2>&1 &
        PREVIEW_PID=$!

        if [ "${FOXGLOVE}" -eq 1 ]; then
            echo "[preview] Foxglove: ws://<this-host>:${FOXGLOVE_PORT}"
            "${SCRIPT_DIR}/foxglove.sh" --port "${FOXGLOVE_PORT}" \
                >/tmp/flyrat_foxglove.log 2>&1 &
            FOXGLOVE_PID=$!
        fi

        echo "[preview] topics: /flyrat/sensors/imu"
        echo "[preview]         /flyrat/sensors/stereo/left/image_raw"
        echo "[preview]         /flyrat/sensors/stereo/right/image_raw"
        echo "[preview]         /flyrat/sensors/stereo/compressed"
    }

    cleanup_raw() {
        if [ -n "${FOXGLOVE_PID}" ] && kill -0 "${FOXGLOVE_PID}" 2>/dev/null; then
            kill -TERM "${FOXGLOVE_PID}" 2>/dev/null || true
            wait "${FOXGLOVE_PID}" 2>/dev/null || true
        fi
        if [ -n "${PREVIEW_PID}" ] && kill -0 "${PREVIEW_PID}" 2>/dev/null; then
            kill -TERM "${PREVIEW_PID}" 2>/dev/null || true
            wait "${PREVIEW_PID}" 2>/dev/null || true
        fi
        if [ "${PREVIEW}" -eq 1 ]; then
            "${ADB[@]}" forward --remove "tcp:${PREVIEW_PORT}" >/dev/null 2>&1 || true
        fi
        if [ -n "${RECORD_PID}" ] && kill -0 "${RECORD_PID}" 2>/dev/null; then
            kill "${RECORD_PID}" 2>/dev/null || true
        fi
        stop_device_processes "${DEVICE_BINARY}"
        if [ -n "${RECORD_PID}" ]; then
            # Let the recorder drain its queue and write the footer before pull.
            sleep 1
        fi
    }
    install_cleanup_trap cleanup_raw

    if [ "${PREVIEW}" -eq 1 ]; then
        start_preview
    fi

    echo "[raw] recording for ${DURATION} at ${FPS} fps (capture ${CAPTURE_FPS} fps) to ${REMOTE_DIR}"
    "${ADB[@]}" shell "exec ${DEVICE_BINARY} --output ${REMOTE_DIR} --duration ${DURATION} ${DEVICE_ARGS[*]}" &
    RECORD_PID=$!

    set +e
    wait "${RECORD_PID}"
    STATUS=$?
    set -e
    RECORD_PID=""

    LOCAL_TARGET="${LOCAL_ROOT}/$(basename "${REMOTE_DIR}")"
    mkdir -p "${LOCAL_ROOT}" "${LOCAL_TARGET}"
    echo "[raw] pulling ${REMOTE_DIR} -> ${LOCAL_TARGET}"
    "${ADB[@]}" pull "${REMOTE_DIR}/." "${LOCAL_TARGET}" >/dev/null
    echo "[raw] local bag: ${LOCAL_TARGET}"

    if [ "${EXPORT}" -eq 1 ]; then
        EXPORT_DIR="${LOCAL_TARGET}_export"
        python3 "${REPO_ROOT}/flyrat_pilot/tools/raw_export.py" \
            --input "${LOCAL_TARGET}" --output "${EXPORT_DIR}"
        echo "[raw] kalibr folder: ${EXPORT_DIR}"
    fi

    if [ "${STATUS}" -ne 0 ]; then
        echo "error: recorder exited with status ${STATUS}" >&2
        exit "${STATUS}"
    fi
}

has_arg() {
    local prefix="$1"
    local arg
    for arg in "${LAUNCH_ARGS[@]}"; do
        [[ "${arg}" == "${prefix}"* ]] && return 0
    done
    return 1
}

if [ "$#" -eq 0 ]; then
    usage
    exit 1
fi

mode="$1"
shift

case "${mode}" in
    sensors|real)
        run_sensors "$@"
        ;;
    slam|pilot)
        run_slam "$@"
        ;;
    raw|calib)
        run_raw "$@"
        ;;
    build)
        exec "${SCRIPT_DIR}/build_all.sh" "$@"
        ;;
    clean)
        exec "${SCRIPT_DIR}/clean_build.sh" "$@"
        ;;
    foxglove|fox)
        exec "${SCRIPT_DIR}/foxglove.sh" "$@"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        echo "error: unknown mode '${mode}'" >&2
        usage >&2
        exit 1
        ;;
esac
