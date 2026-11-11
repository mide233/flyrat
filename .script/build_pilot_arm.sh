#!/usr/bin/env bash
#
# Cross-compile flyrat_pilot for the RK3566 (aarch64).
#
# The device runs Buildroot and has no OpenCV/glog/yaml-cpp, so the pilot is
# linked against a minimal static OpenCV install plus static glog/yaml-cpp from
# the arm64 sysroot. See docs/agent/build_and_run.md for how to build OpenCV.
#
# Usage:
#   .script/build_pilot_arm.sh [--clean]
#
# Environment:
#   FLYRAT_SYSROOT  aarch64 sysroot (default: /opt/sysroots/arm64)
#   BUILD_DIR       output directory (default: <repo>/build-arm)
#   JOBS            parallel compile jobs (default: 4; the RK3566 build host
#                   may be memory constrained, do not use -j12 blindly)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build-arm}"
FLYRAT_SYSROOT="${FLYRAT_SYSROOT:-/opt/sysroots/arm64}"
JOBS="${JOBS:-4}"
CPU_FLAGS="${CPU_FLAGS:--mcpu=cortex-a55 -mtune=cortex-a55}"
CMAKE_DIR="${BUILD_DIR}/pilot-cmake"
CXX="${CROSS_COMPILE:-aarch64-linux-gnu-}g++"

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
    rm -rf "${CMAKE_DIR}" "${BUILD_DIR}/flyrat_pilot"
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
aarch64-linux-gnu-strip "${BUILD_DIR}/flyrat_pilot" 2>/dev/null || true
cp "${CMAKE_DIR}/decode_bench" "${BUILD_DIR}/decode_bench"
aarch64-linux-gnu-strip "${BUILD_DIR}/decode_bench" 2>/dev/null || true
echo "[cross] binary: ${BUILD_DIR}/flyrat_pilot"
echo "[cross] helper: ${BUILD_DIR}/decode_bench"
