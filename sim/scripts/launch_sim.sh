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
#   sim/scripts/launch_sim.sh            # headless Gazebo server
#   SHOW_GAZEBO=1 sim/scripts/launch_sim.sh   # with the Gazebo GUI
#
# Environment:
#   SHOW_GAZEBO   set to 1 to open the Gazebo GUI (default: headless)
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
    echo "       run: sim/scripts/build_all.sh" >&2
    exit 1
fi

if [ ! -f "${WORLD}" ]; then
    echo "error: world not found: ${WORLD}" >&2
    exit 1
fi

if [ ! -f "${AERO_DIR}/plugins/build/libBetaflightPlugin.so" ]; then
    echo "error: bridge plugin not built (${AERO_DIR}/plugins/build)" >&2
    echo "       run: sim/scripts/build_all.sh" >&2
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
    echo "         (no MSP override). Run: sim/scripts/configure_fc.sh" >&2
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

if [ "${SHOW_GAZEBO:-0}" = "1" ]; then
    ARGS+=(--gazebo)
fi

if [ "${VERBOSE:-0}" = "1" ]; then
    ARGS+=(--verbose)
fi

echo "[launch_sim] starting Betaloop (world=${WORLD})"
exec python3 "${BETALOOP_DIR}/start.py" "${ARGS[@]}"
