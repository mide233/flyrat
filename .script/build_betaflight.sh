#!/usr/bin/env bash
#
# Build the Betaflight SITL executable (native x86-64).
#
# The resulting ELF is consumed by betaflight SITL itself; the Gazebo bridge
# talks to it over UDP (9002/9003/9004) and MSP (TCP 5761).
#
# Usage:
#   .script/build_betaflight.sh
#
# Environment:
#   BETAFLIGHT_TARGET   target to build (default: SITL)
#   JOBS                parallel jobs (default: nproc)
#   CLEAN               set to 1 to run 'make clean' first
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_DIR="${REPO_ROOT}/flyrat_play/gazebo"
BF_DIR="${SIM_DIR}/third_party/betaflight"

TARGET="${BETAFLIGHT_TARGET:-SITL}"
JOBS="${JOBS:-$(nproc)}"

if [ ! -d "${BF_DIR}" ]; then
    echo "error: ${BF_DIR} not found." >&2
    echo "       run: git submodule update --init --recursive" >&2
    exit 1
fi

cd "${BF_DIR}"

# The upstream Makefile hydrates the firmware submodules (src/config,
# lib/modules/dronecan/libcanard, ...) via its own stamp rules. When those
# rules run inside a parallel `make` they race on the git config lock, so
# hydrate them serially here first and let make treat the stamps as fresh.
echo "[build_betaflight] hydrating submodules..."
git submodule update --init

if [ "${CLEAN:-0}" = "1" ]; then
    echo "[build_betaflight] make clean"
    make clean
fi

echo "[build_betaflight] building TARGET=${TARGET} (jobs=${JOBS})..."
make "TARGET=${TARGET}" -j"${JOBS}"

ELF="${BF_DIR}/obj/main/betaflight_${TARGET}.elf"
if [ ! -f "${ELF}" ]; then
    echo "error: expected artifact not found: ${ELF}" >&2
    exit 1
fi

echo "[build_betaflight] done: ${ELF}"
