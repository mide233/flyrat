#!/usr/bin/env bash
#
# (Re)generate sim/config/eeprom.bin from sim/config/betaflight_sim.txt.
#
# Betaflight SITL opens EEPROM_FILENAME ("eeprom.bin") relative to its own
# working directory. The betaloop launcher starts the ELF with cwd set to the
# ELF's directory (obj/main), so we must configure a SITL that runs from that
# same directory if we want the launcher to pick up the result.
#
# This script starts a throwaway SITL from obj/main, applies the CLI config
# (which ends with "save" and makes SITL reboot/exit) and copies the resulting
# eeprom.bin into sim/config/, where launch_sim.sh installs it next to the ELF.
#
# Usage:
#   sim/scripts/configure_fc.sh            # apply and persist to sim/config/
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ELF="${SIM_DIR}/third_party/betaflight/obj/main/betaflight_SITL.elf"
ELF_DIR="$(dirname "${ELF}")"
TARGET_EEPROM="${SIM_DIR}/config/eeprom.bin"
LOG="${TMPDIR:-/tmp}/betaflight_configure.log"

if [ ! -f "${ELF}" ]; then
    echo "error: SITL elf not found: ${ELF}" >&2
    echo "       run: sim/scripts/build_betaflight.sh" >&2
    exit 1
fi

echo "[configure_fc] starting throwaway SITL from ${ELF_DIR}"
( cd "${ELF_DIR}" && "./$(basename "${ELF}")" >"${LOG}" 2>&1 ) &
SITL_PID=$!

cleanup() {
    kill "${SITL_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# Wait for the MSP/CLI TCP port (5761 == 0x1681) to appear.
for _ in $(seq 1 150); do
    if awk 'NR > 1 && $2 ~ /:1681$/ { found = 1 } END { exit !found }' /proc/net/tcp; then
        break
    fi
    sleep 0.2
done

echo "[configure_fc] applying ${SIM_DIR}/config/betaflight_sim.txt"
python3 "${SCRIPT_DIR}/config_betaflight.py" "$@"

# "save" reboots the firmware, which makes the SITL process exit.
wait "${SITL_PID}" 2>/dev/null || true
trap - EXIT

if [ ! -f "${ELF_DIR}/eeprom.bin" ]; then
    echo "error: ${ELF_DIR}/eeprom.bin was not produced" >&2
    exit 1
fi

cp -f "${ELF_DIR}/eeprom.bin" "${TARGET_EEPROM}"
echo "[configure_fc] wrote ${TARGET_EEPROM}"
