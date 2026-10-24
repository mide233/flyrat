#!/usr/bin/env bash
#
# Build the flyrat_pilot onboard program. It is ROS-free and links the shared
# libflyrat core; the runtime logic is still a placeholder.
#
# Usage:
#   .script/build_flyrat_pilot.sh [--clean] [--debug]
#
#   --clean  remove the build directory first
#   --debug  configure a Debug build instead of Release
#
# Environment:
#   JOBS     parallel compile jobs (default: nproc)
#
set -euo pipefail

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PILOT_DIR="${REPO_ROOT}/flyrat_pilot"

CLEAN=0
DEBUG=0
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        --debug) DEBUG=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done

JOBS="${JOBS:-$(nproc)}"
BUILD_TYPE=Release
[ "${DEBUG}" -eq 1 ] && BUILD_TYPE=Debug

if [ "${CLEAN}" -eq 1 ]; then
    echo "[build_flyrat_pilot] cleaning ${PILOT_DIR}/build"
    rm -rf "${PILOT_DIR}/build"
fi

cmake -S "${PILOT_DIR}" -B "${PILOT_DIR}/build" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${PILOT_DIR}/build" -j "${JOBS}"

echo "[build_flyrat_pilot] done: ${PILOT_DIR}/build/flyrat_pilot"
