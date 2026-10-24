#!/usr/bin/env bash
#
# Build every native artifact in the repository:
#   1. libflyrat core (algorithms + MSP/UDP transport) and its unit tests
#   2. flyrat_pilot onboard program
#   3. Betaflight SITL firmware ELF
#   4. Gazebo Harmonic bridge plugin (libBetaflightPlugin.so)
#   5. ROS 2 workspace (flyrat_msp + flyrat_control)
#
# Usage:
#   .script/build_all.sh [--clean] [--no-sim] [--no-ros]
#
#   --clean   remove every build directory first
#   --no-sim  skip the Betaflight SITL ELF and the Gazebo plugin
#   --no-ros  skip the ROS 2 workspace
#
set -euo pipefail

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CLEAN=0
WITH_SIM=1
WITH_ROS=1
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        --no-sim) WITH_SIM=0 ;;
        --no-ros) WITH_ROS=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "error: unknown argument: ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done

CLEAN_ARG=()
[ "${CLEAN}" -eq 1 ] && CLEAN_ARG=(--clean)

"${SCRIPT_DIR}/build_libflyrat.sh" "${CLEAN_ARG[@]}"
"${SCRIPT_DIR}/build_flyrat_pilot.sh" "${CLEAN_ARG[@]}"

if [ "${WITH_SIM}" -eq 1 ]; then
    # build_betaflight.sh takes CLEAN as an environment variable.
    if [ "${CLEAN}" -eq 1 ]; then
        CLEAN=1 "${SCRIPT_DIR}/build_betaflight.sh"
    else
        "${SCRIPT_DIR}/build_betaflight.sh"
    fi
    "${SCRIPT_DIR}/build_plugin.sh"
fi

if [ "${WITH_ROS}" -eq 1 ]; then
    "${SCRIPT_DIR}/build_ros2.sh" "${CLEAN_ARG[@]}"
fi

echo "[build_all] all artifacts built"
