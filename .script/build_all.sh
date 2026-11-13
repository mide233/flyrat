#!/usr/bin/env bash
#
# Build the host libraries, the onboard program, and optionally the cross
# toolchain, simulation, and ROS 2 artifacts.
#
# Usage:
#   .script/build_all.sh [options]
#
#   --lib               build only the host libflyrat core and tests
#   --pilot             build only the host flyrat_pilot program
#   --arm               cross-compile the RK3566 onboard binaries
#   --static-forwarder  with --arm, also build the fully static forwarder
#   --no-tests          skip the libflyrat ctest run
#   --no-sim            skip the Betaflight SITL ELF and the Gazebo plugin
#   --no-ros            skip the ROS 2 workspace
#   --debug             configure the host builds as Debug
#   --clean             remove the matching build directories first
#
# Outputs:
#   build/host/libflyrat            host core library, tests, and tools
#   build/host/pilot                host flyrat_pilot and sensor forwarder
#   build/arm/pilot-cmake           cross build tree
#   build/arm/flyrat_pilot          stripped RK3566 pilot binary
#   build/arm/decode_bench          stripped decoder benchmark
#   build/arm/flyrat_sensor_forwarder  stripped dynamic forwarder
#   build/arm/static/flyrat_sensor_forwarder  fully static forwarder
#
# Environment:
#   JOBS        parallel compile jobs (default: nproc)
#
set -euo pipefail

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_ROOT="${REPO_ROOT}/build"
HOST_BUILD="${BUILD_ROOT}/host"
JOBS="${JOBS:-$(nproc)}"

BUILD_LIB=1
BUILD_PILOT=1
HOST_ONLY=0
WITH_SIM=1
WITH_ROS=1
WITH_ARM=0
STATIC_FORWARDER=0
RUN_TESTS=1
BUILD_TYPE=Release
CLEAN=0

for arg in "$@"; do
    case "${arg}" in
        --lib) BUILD_LIB=1; BUILD_PILOT=0; HOST_ONLY=1 ;;
        --pilot) BUILD_LIB=0; BUILD_PILOT=1; HOST_ONLY=1 ;;
        --arm) WITH_ARM=1 ;;
        --static-forwarder) WITH_ARM=1; STATIC_FORWARDER=1 ;;
        --no-tests) RUN_TESTS=0 ;;
        --no-sim) WITH_SIM=0 ;;
        --no-ros) WITH_ROS=0 ;;
        --debug) BUILD_TYPE=Debug ;;
        --clean) CLEAN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done

if [ "${HOST_ONLY}" -eq 1 ]; then
    # --lib/--pilot are quick, host-only builds; simulation and ROS are opt-in
    # through the absence of --no-sim/--no-ros in the default mode only.
    WITH_SIM=0
    WITH_ROS=0
fi

CLEAN_ARG=()
[ "${CLEAN}" -eq 1 ] && CLEAN_ARG=(--clean)

if [ "${BUILD_LIB}" -eq 1 ]; then
    if [ "${CLEAN}" -eq 1 ]; then
        echo "[build_all] cleaning ${HOST_BUILD}/libflyrat"
        rm -rf "${HOST_BUILD}/libflyrat"
    fi
    cmake -S "${REPO_ROOT}/libflyrat" -B "${HOST_BUILD}/libflyrat" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    cmake --build "${HOST_BUILD}/libflyrat" -j "${JOBS}"
    if [ "${RUN_TESTS}" -eq 1 ]; then
        ctest --test-dir "${HOST_BUILD}/libflyrat" --output-on-failure
    fi
    echo "[build_all] host libflyrat: ${HOST_BUILD}/libflyrat"
fi

if [ "${BUILD_PILOT}" -eq 1 ]; then
    if [ "${CLEAN}" -eq 1 ]; then
        echo "[build_all] cleaning ${HOST_BUILD}/pilot"
        rm -rf "${HOST_BUILD}/pilot"
    fi
    cmake -S "${REPO_ROOT}/flyrat_pilot" -B "${HOST_BUILD}/pilot" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    cmake --build "${HOST_BUILD}/pilot" -j "${JOBS}"
    echo "[build_all] host flyrat_pilot: ${HOST_BUILD}/pilot/flyrat_pilot"
fi

if [ "${WITH_SIM}" -eq 1 ]; then
    # build_betaflight.sh takes CLEAN as an environment variable.
    if [ "${CLEAN}" -eq 1 ]; then
        CLEAN=1 "${SCRIPT_DIR}/build_betaflight.sh"
    else
        "${SCRIPT_DIR}/build_betaflight.sh"
    fi
    "${SCRIPT_DIR}/build_plugin.sh"
fi

if [ "${WITH_ROS}" -eq 1 ]; then
    "${SCRIPT_DIR}/build_ros2.sh" "${CLEAN_ARG[@]}"
fi

if [ "${WITH_ARM}" -eq 1 ]; then
    ARM_ARGS=("${CLEAN_ARG[@]}")
    [ "${STATIC_FORWARDER}" -eq 1 ] && ARM_ARGS+=(--static-forwarder)
    "${SCRIPT_DIR}/build_pilot_arm.sh" "${ARM_ARGS[@]}"
fi

echo "[build_all] all requested artifacts built"
