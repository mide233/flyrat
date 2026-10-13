#!/usr/bin/env bash
#
# Build every native artifact required for the Betaflight SITL + Gazebo stack:
#   1. the Betaflight SITL firmware ELF
#   2. the Gazebo Harmonic bridge plugin (libBetaflightPlugin.so)
#
# Usage:
#   sim/scripts/build_all.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/build_betaflight.sh"
"${SCRIPT_DIR}/build_plugin.sh"

echo "[build_all] all artifacts built"
