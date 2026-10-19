#!/usr/bin/env bash
#
# Build the Gazebo System plugin (libBetaflightPlugin.so) that bridges
# Betaflight SITL and Gazebo Harmonic.
#
# The plugin is a Gazebo system plugin bound to the iris model. It reads the
# motor command packet (UDP 9002), applies rotor forces and streams simulated
# IMU/GPS/barometer state back to SITL (UDP 9003).
#
# The submodule is patched before building (see flyrat_play/gazebo/patches/) instead of being
# modified in place, so the fix is reproducible from a clean checkout.
#
# Usage:
#   flyrat_play/gazebo/scripts/build_plugin.sh
#
# Requires: gz-harmonic + dev packages (libgz-sim8-dev, libgz-plugin2-dev,
#           libgz-math7-dev) and cppzmq-dev.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
AERO_DIR="${SIM_DIR}/third_party/aeroloop_gazebo"

if [ ! -d "${AERO_DIR}" ]; then
    echo "error: ${AERO_DIR} not found." >&2
    echo "       run: git submodule update --init --recursive" >&2
    exit 1
fi

if ! pkg-config --exists gz-sim8; then
    echo "error: gz-sim8 development files not found (need Gazebo Harmonic)." >&2
    exit 1
fi

cd "${AERO_DIR}"

# Apply our patches to the pristine submodule source. The upstream plugin waits
# up to 1000 ms for a motor command packet *inside the physics update*, which
# stalls the whole simulation as soon as one packet is missed (the real-time
# factor collapses to ~0.5%). See flyrat_play/gazebo/patches/ for the details.
PATCHES_DIR="${SIM_DIR}/patches"
for patch in "${PATCHES_DIR}"/*.patch; do
    [ -e "${patch}" ] || continue
    if git apply --reverse --check "${patch}" >/dev/null 2>&1; then
        echo "[build_plugin] already applied: $(basename "${patch}")"
    elif git apply "${patch}"; then
        echo "[build_plugin] applied $(basename "${patch}")"
    else
        echo "error: cannot apply ${patch}" >&2
        echo "       (restore the submodule sources first, e.g." >&2
        echo "        git -C ${AERO_DIR} checkout -- . )" >&2
        exit 1
    fi
done

# Start from a clean build directory so a stale CMake cache from another
# Gazebo release cannot silently break the configuration.
rm -rf plugins/build

./build_plugin.sh

PLUGIN="${AERO_DIR}/plugins/build/libBetaflightPlugin.so"
if [ ! -f "${PLUGIN}" ]; then
    echo "error: expected artifact not found: ${PLUGIN}" >&2
    exit 1
fi

echo "[build_plugin] done: ${PLUGIN}"
