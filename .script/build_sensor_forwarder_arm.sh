#!/usr/bin/env bash
#
# Cross-compile flyrat_sensor_forwarder for the RK3566 (aarch64) and print the
# resulting binary path. The device runs Buildroot, so link statically.
#
# Usage:
#   .script/build_sensor_forwarder_arm.sh [--clean]
#
# Environment:
#   CROSS_COMPILE   toolchain prefix (default: aarch64-linux-gnu-)
#   BUILD_DIR       output directory (default: <repo>/build-arm)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build-arm}"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-linux-gnu-}"
CC="${CROSS_COMPILE}gcc"
CXX="${CROSS_COMPILE}g++"

CLEAN=0
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        -h|--help)
            awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *) echo "error: unknown argument: ${arg}" >&2; exit 2 ;;
    esac
done

if ! command -v "${CC}" >/dev/null 2>&1; then
    echo "error: ${CC} not found; install the aarch64 cross toolchain" >&2
    exit 1
fi

if [ "${CLEAN}" -eq 1 ]; then
    rm -rf "${BUILD_DIR}"
fi
mkdir -p "${BUILD_DIR}"

cd "${REPO_ROOT}"

SYS_INC=(
    -Ilibflyrat/include
    -Ilibflyrat/third_party/vl53l5cx/include
    -Ilibflyrat/third_party/vl53l5cx/platform
)

C_SOURCES=(
    libflyrat/third_party/vl53l5cx/src/vl53l5cx_api.c
    libflyrat/third_party/vl53l5cx/src/vl53l5cx_plugin_xtalk.c
    libflyrat/third_party/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.c
    libflyrat/third_party/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.c
    libflyrat/third_party/vl53l5cx/platform/platform.c
)

CXX_SOURCES=(
    flyrat_pilot/src/sensors/sensor_forwarder.cpp
    flyrat_pilot/src/sensors/sensor_forwarder_main.cpp
    libflyrat/src/msp_frame.cpp
    libflyrat/src/msp_parse.cpp
    libflyrat/src/msp_interact.cpp
    libflyrat/src/hardware/serial_transport.cpp
    libflyrat/src/hardware/tof.cpp
    libflyrat/src/hardware/stereo_jpeg_source.cpp
)

OBJ_DIR="${BUILD_DIR}/obj"
rm -rf "${OBJ_DIR}"
mkdir -p "${OBJ_DIR}"

# Do not pass VL53L5CX_DISABLE_* here: the macros live in platform.h and must
# be identical for every translation unit.
for src in "${C_SOURCES[@]}"; do
    echo "[cross] ${src}"
    "${CC}" -O2 -static "${SYS_INC[@]}" -c "${src}" \
        -o "${OBJ_DIR}/$(basename "${src}" .c).o"
done

for src in "${CXX_SOURCES[@]}"; do
    echo "[cross] ${src}"
    "${CXX}" -O2 -static -std=c++17 -Wall -Wextra "${SYS_INC[@]}" \
        -c "${src}" -o "${OBJ_DIR}/$(basename "${src}" .cpp).o"
done

"${CXX}" -O2 -static "${OBJ_DIR}"/*.o -pthread -ldl \
    -o "${BUILD_DIR}/flyrat_sensor_forwarder"

echo "[cross] binary: ${BUILD_DIR}/flyrat_sensor_forwarder"
