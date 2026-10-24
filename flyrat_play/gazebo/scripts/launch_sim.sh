#!/usr/bin/env bash
#
# Launch the Betaflight SITL + Gazebo Harmonic stack.
#
# This drives the upstream betaloop launcher, which:
#   1. starts Gazebo (Harmonic) with our world,
#   2. starts the Betaflight SITL executable,
#   and wires up the environment (GZ_SIM_RESOURCE_PATH, plugin path, ...).
#
# Usage:
#   flyrat_play/gazebo/scripts/launch_sim.sh            # with the Gazebo GUI
#   HIDE_GAZEBO=1 flyrat_play/gazebo/scripts/launch_sim.sh   # headless Gazebo server
#
# Environment:
#   HIDE_GAZEBO   set to 1 to run the Gazebo server without the GUI (default: GUI)
#   VERBOSE       set to 1 for verbose (gz -v 4) logging
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

AERO_DIR="${SIM_DIR}/third_party/aeroloop_gazebo"
BETALOOP_DIR="${SIM_DIR}/third_party/betaloop"
ELF="${SIM_DIR}/third_party/betaflight/obj/main/betaflight_SITL.elf"
WORLD="${SIM_DIR}/worlds/betaflight_sitl.sdf"

if [ ! -f "${ELF}" ]; then
    echo "error: SITL elf not found: ${ELF}" >&2
    echo "       run: .script/build_all.sh" >&2
    exit 1
fi

if [ ! -f "${WORLD}" ]; then
    echo "error: world not found: ${WORLD}" >&2
    exit 1
fi

if [ ! -f "${AERO_DIR}/plugins/build/libBetaflightPlugin.so" ]; then
    echo "error: bridge plugin not built (${AERO_DIR}/plugins/build)" >&2
    echo "       run: .script/build_all.sh" >&2
    exit 1
fi

# Refuse to start a second stack. Two simulators publish the same scoped Gazebo
# topics (including the camera image), so a single bridge alternates between
# them and the picture visibly flickers. The SITL also cannot bind its ports.
if pgrep -f 'betaloop/start\.py' > /dev/null 2>&1; then
    echo "error: a simulation stack is already running" >&2
    echo "       stop it first, then relaunch:" >&2
    echo "         kill -TERM -\$(pgrep -f 'betaloop/start\\.py' | head -n 1)" >&2
    exit 1
fi

if awk 'NR > 1 { split($2, addr, ":"); if (addr[2] == "1681" && $4 == "0A") { found = 1 } }
        END { exit found ? 0 : 1 }' /proc/net/tcp 2> /dev/null; then
    echo "error: port 5761 (SITL MSP) is already in use by another process" >&2
    echo "       find it with: ls -l /proc/*/exe 2>/dev/null | grep betaflight" >&2
    exit 1
fi

# Betaflight SITL reads "eeprom.bin" relative to its own working directory, and
# the betaloop launcher runs it with cwd set to the ELF's directory. Install the
# version-controlled flight-controller config there so MSP override stays armed.
EEPROM_SRC="${SIM_DIR}/config/eeprom.bin"
ELF_DIR="$(dirname "${ELF}")"
if [ -f "${EEPROM_SRC}" ]; then
    cp -f "${EEPROM_SRC}" "${ELF_DIR}/eeprom.bin"
    echo "[launch_sim] installed FC config: ${EEPROM_SRC} -> ${ELF_DIR}/eeprom.bin"
else
    echo "warning: ${EEPROM_SRC} missing; SITL will boot with default settings" >&2
    echo "         (no MSP override). Run: flyrat_play/gazebo/scripts/configure_fc.sh" >&2
fi

ARGS=(
    --gazebo-assets "${AERO_DIR}"
    --world "${WORLD}"
    --elf "${ELF}"
    # No joystick: the RC link is driven by our own MSP/RC client.
    --disable-transmitter
    # No Betaflight Configurator websocket proxy by default.
    --disable-websockify
)

if [ "${HIDE_GAZEBO:-0}" = "0" ]; then
    ARGS+=(--gazebo)
fi

if [ "${VERBOSE:-0}" = "1" ]; then
    ARGS+=(--verbose)
fi

# ros-jazzy-ros-gz-bridge installs the gz_*_vendor packages, whose setup hooks
# prepend a `gz` shim and a GZ_CONFIG_PATH that only lists the vendor's own
# commands. That hides /usr/share/gz, so the `sim` command disappears and Gazebo
# prints its help text instead of starting. Use the system toolchain for the
# simulator process; the ROS bridge sources its own environment separately.
export PATH="/usr/bin:${PATH}"
unset GZ_CONFIG_PATH

echo "[launch_sim] starting Betaloop (world=${WORLD})"

# Resource resolution: our model override (flyrat_play/gazebo/models) plus the upstream
# aeroloop assets. Betaloop appends its own worlds path, so pre-seed the
# variables here to keep every lookup root on the search path.
export GZ_SIM_RESOURCE_PATH="${SIM_DIR}/models:${AERO_DIR}/models:${AERO_DIR}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export SDF_PATH="${AERO_DIR}/models:${SIM_DIR}/models${SDF_PATH:+:${SDF_PATH}}"

exec python3 "${BETALOOP_DIR}/start.py" "${ARGS[@]}"
