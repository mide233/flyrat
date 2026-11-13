#!/usr/bin/env bash
#
# Record raw IMU counts and native stereo MJPEG on the vehicle, then pull the
# result to the host. This bypasses flyrat_sensor_forwarder and the calibrated
# pilot bag on purpose: re-calibration needs data before intrinsics, axis
# mapping and time offsets are applied.
#
# Usage:
#   .script/bringup_raw_record.sh [output:=DIR] [duration:=10m] [fps:=20]
#       [capture_fps:=60] [imu_hz:=200] [local:=DIR] [check:=true]
#       [export:=true] [require_capture:=true] [push:=0]
#
# Environment:
#   ADB_SERIAL       adb device serial when several devices are attached
#   BUILD_DIR        host build directory (default <repo>/build/arm)
#   FLYRAT_PUSH_RAW  0 skips pushing the binary
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
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
        push:=0) PUSH=0 ;;
        push:=1) PUSH=1 ;;
        -h|--help)
            awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *) echo "error: unknown argument: ${arg}" >&2; exit 2 ;;
    esac
done

ADB=(adb)
if [ -n "${ADB_SERIAL:-}" ]; then
    ADB+=(-s "${ADB_SERIAL}")
fi
if ! "${ADB[@]}" get-state >/dev/null 2>&1; then
    echo "error: no adb device; check USB and ADB_SERIAL" >&2
    exit 1
fi

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
"${ADB[@]}" shell "pkill -f '^/tmp/flyrat_sensor_forwarder' 2>/dev/null || true; \
    pkill -f '^/tmp/flyrat_pilot' 2>/dev/null || true; \
    pkill -f '^/tmp/flyrat_raw_recorder' 2>/dev/null || true" >/dev/null 2>&1 || true

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

if [ "${CHECK}" -eq 1 ]; then
    echo "[raw] checking devices"
    exec "${ADB[@]}" shell "exec ${DEVICE_BINARY} --check ${DEVICE_ARGS[*]}"
fi

if [ -z "${REMOTE_DIR}" ]; then
    REMOTE_DIR="/userdata/flyrat_raw/$(date +%Y%m%d_%H%M%S)"
fi
REMOTE_DIR="${REMOTE_DIR%/}"
"${ADB[@]}" shell "mkdir -p '${REMOTE_DIR}'"

RECORD_PID=""
cleanup() {
    if [ -n "${RECORD_PID}" ] && kill -0 "${RECORD_PID}" 2>/dev/null; then
        kill "${RECORD_PID}" 2>/dev/null || true
    fi
    "${ADB[@]}" shell "pkill -TERM -f '^${DEVICE_BINARY}' 2>/dev/null || true" >/dev/null 2>&1 || true
    if [ -n "${RECORD_PID}" ]; then
        # Let the recorder drain its queue and write the footer before pull.
        sleep 1
    fi
}
trap cleanup EXIT INT TERM

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
