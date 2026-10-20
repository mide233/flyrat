#!/usr/bin/env bash
#
# Build the ROS-free libflyrat core (autopilot algorithms + Betaflight MSP/UDP
# transport) and run its unit tests.
#
# Usage:
#   .script/build_libflyrat.sh [--clean] [--debug] [--no-tests]
#
#   --clean     remove the build directory first
#   --debug     configure a Debug build instead of Release
#   --no-tests  skip ctest
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
LIB_DIR="${REPO_ROOT}/libflyrat"

CLEAN=0
DEBUG=0
RUN_TESTS=1
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        --debug) DEBUG=1 ;;
        --no-tests) RUN_TESTS=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done

JOBS="${JOBS:-$(nproc)}"
BUILD_TYPE=Release
[ "${DEBUG}" -eq 1 ] && BUILD_TYPE=Debug

if [ "${CLEAN}" -eq 1 ]; then
    echo "[build_libflyrat] cleaning ${LIB_DIR}/build"
    rm -rf "${LIB_DIR}/build"
fi

cmake -S "${LIB_DIR}" -B "${LIB_DIR}/build" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${LIB_DIR}/build" -j "${JOBS}"

if [ "${RUN_TESTS}" -eq 1 ]; then
    ctest --test-dir "${LIB_DIR}/build" --output-on-failure
fi

echo "[build_libflyrat] done: ${LIB_DIR}/build/libflyrat.a"
