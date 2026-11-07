#!/usr/bin/env bash
# Build the Gazebo trajectory-player system plugin.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/../sim/gz_plugins/trajectory_player"
BUILD_DIR="${PLUGIN_DIR}/build"
INSTALL_DIR="${SCRIPT_DIR}/../sim/gz_plugins/install"

cmake -S "${PLUGIN_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
cmake --build "${BUILD_DIR}" --parallel
cmake --install "${BUILD_DIR}"

echo "[build_gz_plugin] installed: ${INSTALL_DIR}/lib"
