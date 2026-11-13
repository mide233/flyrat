#!/usr/bin/env bash
#
# Cross-compile the RK3566 (aarch64) onboard programs.
#
# The device runs Buildroot and has no OpenCV/glog/yaml-cpp, so the CMake build
# links a minimal static OpenCV install plus static glog/yaml-cpp from the arm64
# sysroot. See docs/agent/build_and_run.md for how to build OpenCV.
#
# Usage:
#   .script/build_pilot_arm.sh [--clean] [--static-forwarder]
#
#   --clean             remove the cross build tree and binaries first
#   --static-forwarder  also build a fully static flyrat_sensor_forwarder
#                       (for rootfs images without a matching libstdc++)
#
# Outputs (under build/arm by default):
#   pilot-cmake/                 CMake build tree
#   flyrat_pilot                 stripped pilot binary
#   decode_bench                 stripped decoder benchmark
#   flyrat_sensor_forwarder      stripped dynamically linked forwarder
#   flyrat_raw_recorder          stripped raw IMU/camera calibration recorder
#   static/flyrat_sensor_forwarder  fully static forwarder
#
# Environment:
#   FLYRAT_SYSROOT  aarch64 sysroot (default: /opt/sysroots/arm64)
#   BUILD_DIR       output directory (default: <repo>/build/arm)
#   JOBS            parallel compile jobs (default: 4; the build host may be
#                   memory constrained, do not use -j12 blindly)
#   CROSS_COMPILE   toolchain prefix (default: aarch64-linux-gnu-)
#   CPU_FLAGS       target flags (default: -mcpu=cortex-a55 -mtune=cortex-a55)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build/arm}"
FLYRAT_SYSROOT="${FLYRAT_SYSROOT:-/opt/sysroots/arm64}"
JOBS="${JOBS:-4}"
CPU_FLAGS="${CPU_FLAGS:--mcpu=cortex-a55 -mtune=cortex-a55}"
CMAKE_DIR="${BUILD_DIR}/pilot-cmake"
STATIC_DIR="${BUILD_DIR}/static"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-linux-gnu-}"
CC="${CROSS_COMPILE}gcc"
CXX="${CROSS_COMPILE}g++"

CLEAN=0
STATIC_FORWARDER=0
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        --static-forwarder) STATIC_FORWARDER=1 ;;
        -h|--help)
            awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *) echo "error: unknown argument: ${arg}" >&2; exit 2 ;;
    esac
done

if ! command -v "${CXX}" >/dev/null 2>&1; then
    echo "error: ${CXX} not found; install the aarch64 cross toolchain" >&2
    exit 1
fi
if [ ! -d "${FLYRAT_SYSROOT}" ]; then
    echo "error: sysroot not found: ${FLYRAT_SYSROOT}" >&2
    exit 1
fi
if [ ! -f "${FLYRAT_SYSROOT}/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake" ]; then
    echo "error: static OpenCV not found under ${FLYRAT_SYSROOT}/usr/local" >&2
    echo "Build it first; see docs/agent/build_and_run.md (onboard SLAM)." >&2
    exit 1
fi

if [ "${CLEAN}" -eq 1 ]; then
    rm -rf "${CMAKE_DIR}" "${STATIC_DIR}"
    rm -f "${BUILD_DIR}/flyrat_pilot" "${BUILD_DIR}/decode_bench" \
        "${BUILD_DIR}/flyrat_sensor_forwarder" "${BUILD_DIR}/flyrat_raw_recorder"
fi
mkdir -p "${BUILD_DIR}"

cd "${REPO_ROOT}"
cmake -S flyrat_pilot -B "${CMAKE_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchains/aarch64-linux-gnu.cmake" \
    -DFLYRAT_SYSROOT="${FLYRAT_SYSROOT}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="${CPU_FLAGS}" \
    -DCMAKE_CXX_FLAGS="${CPU_FLAGS}" \
    -DFLYRAT_STATIC_DEPS=ON \
    -DCMAKE_EXE_LINKER_FLAGS="-static-libstdc++ -static-libgcc"
cmake --build "${CMAKE_DIR}" -j "${JOBS}"

cp "${CMAKE_DIR}/flyrat_pilot" "${BUILD_DIR}/flyrat_pilot"
"${CROSS_COMPILE}strip" "${BUILD_DIR}/flyrat_pilot" 2>/dev/null || true
cp "${CMAKE_DIR}/decode_bench" "${BUILD_DIR}/decode_bench"
"${CROSS_COMPILE}strip" "${BUILD_DIR}/decode_bench" 2>/dev/null || true
cp "${CMAKE_DIR}/flyrat_sensor_forwarder" "${BUILD_DIR}/flyrat_sensor_forwarder"
"${CROSS_COMPILE}strip" "${BUILD_DIR}/flyrat_sensor_forwarder" 2>/dev/null || true
cp "${CMAKE_DIR}/flyrat_raw_recorder" "${BUILD_DIR}/flyrat_raw_recorder"
"${CROSS_COMPILE}strip" "${BUILD_DIR}/flyrat_raw_recorder" 2>/dev/null || true
echo "[cross] binary: ${BUILD_DIR}/flyrat_pilot"
echo "[cross] helper: ${BUILD_DIR}/decode_bench"
echo "[cross] helper: ${BUILD_DIR}/flyrat_sensor_forwarder"
echo "[cross] helper: ${BUILD_DIR}/flyrat_raw_recorder"

if [ "${STATIC_FORWARDER}" -eq 1 ]; then
    # The forwarder only needs the libflyrat hardware/MSP sources and the
    # VL53L5CX C driver; linking the whole CMake target statically would pull
    # in OpenCV and the odometry stack. Compile the small source set directly.
    SYS_INC=(
        -Ilibflyrat/include
        -Ilibflyrat/third_party/vl53l5cx/include
        -Ilibflyrat/third_party/vl53l5cx/platform
        -Iflyrat_pilot/src
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
        flyrat_pilot/src/sensors/sensor_stream.cpp
        libflyrat/src/msp_frame.cpp
        libflyrat/src/msp_parse.cpp
        libflyrat/src/msp_interact.cpp
        libflyrat/src/hardware/serial_transport.cpp
        libflyrat/src/hardware/tof.cpp
        libflyrat/src/hardware/stereo_jpeg_source.cpp
    )

    OBJ_DIR="${STATIC_DIR}/obj"
    rm -rf "${STATIC_DIR}"
    mkdir -p "${OBJ_DIR}"

    # Do not pass VL53L5CX_DISABLE_* here: the macros live in platform.h and
    # must be identical for every translation unit.
    for src in "${C_SOURCES[@]}"; do
        echo "[cross] static: ${src}"
        "${CC}" -O2 -static "${SYS_INC[@]}" -c "${src}" \
            -o "${OBJ_DIR}/$(basename "${src}" .c).o"
    done

    for src in "${CXX_SOURCES[@]}"; do
        echo "[cross] static: ${src}"
        "${CXX}" -O2 -static -std=c++17 -Wall -Wextra "${SYS_INC[@]}" \
            -c "${src}" -o "${OBJ_DIR}/$(basename "${src}" .cpp).o"
    done

    "${CXX}" -O2 -static "${OBJ_DIR}"/*.o -pthread -ldl \
        -o "${STATIC_DIR}/flyrat_sensor_forwarder"
    echo "[cross] static binary: ${STATIC_DIR}/flyrat_sensor_forwarder"
fi
