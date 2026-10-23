#!/usr/bin/env bash
#
# Build the Gazebo System plugin (libBetaflightPlugin.so) that bridges
# Betaflight SITL and Gazebo Harmonic.
#
# The plugin is a Gazebo system plugin bound to the iris model. It reads the
# motor command packet (UDP 9002), applies rotor forces and streams simulated
# IMU/GPS/barometer state back to SITL (UDP 9003).
#
# The plugin fixes (the physics-thread receive timeout and the IMU specific
# force) are commits on the mide233 fork of aeroloop_gazebo, branch gz, pinned
# by this repository's submodule. A clean checkout therefore needs no local
# patching: just init the submodule and build.
#
# Usage:
#   .script/build_plugin.sh
#
# Requires: gz-harmonic + dev packages (libgz-sim8-dev, libgz-plugin2-dev,
#           libgz-math7-dev) and cppzmq-dev.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_DIR="${REPO_ROOT}/flyrat_play/gazebo"
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

# The submodule already carries the fixes, so refuse to silently build a stale
# or reverted tree: the physics-thread timeout must be 2 ms and the IMU must add
# gravity before it is rotated into the body frame.
if ! grep -q "waitMs = 2;" plugins/BetaflightPlugin.cc; then
    echo "error: ${AERO_DIR} is missing the physics-thread fix" >&2
    echo "       run: git submodule update --init --recursive" >&2
    exit 1
fi

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
