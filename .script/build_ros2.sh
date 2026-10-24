#!/usr/bin/env bash
#
# Build the ROS 2 workspace: flyrat_msp (MSP + virtual RC bridge node) and
# flyrat_control (autopilot demo node). Both link the shared libflyrat core.
#
# Usage:
#   .script/build_ros2.sh [--clean] [-- <extra colcon arguments>]
#
#   --clean  remove build/install/log first
#
# Any other argument is forwarded to colcon, e.g.
#   .script/build_ros2.sh --packages-select flyrat_msp
#
# Environment:
#   ROS_DISTRO  ROS 2 distribution to source (default: jazzy)
#   JOBS        parallel workers (default: nproc)
#
set -euo pipefail

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/flyrat_play/ros2_ws"

CLEAN=0
EXTRA=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --clean) CLEAN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; EXTRA+=("$@"); break ;;
        *) EXTRA+=("$1"); shift ;;
    esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ ! -f "${ROS_SETUP}" ]; then
    echo "error: ${ROS_SETUP} not found (set ROS_DISTRO to your distribution)." >&2
    exit 1
fi

if [ ! -d "${WS_DIR}/src" ]; then
    echo "error: ${WS_DIR}/src not found." >&2
    exit 1
fi

if [ "${CLEAN}" -eq 1 ]; then
    echo "[build_ros2] cleaning ${WS_DIR}/{build,install,log}"
    rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
fi

JOBS="${JOBS:-$(nproc)}"

# colcon needs a bash shell with the ROS environment sourced; the caller may be
# zsh, so run it in a subshell instead of sourcing /opt/ros directly.
bash -c '
# No `set -u` here: the ROS 2 setup scripts reference unset variables.
set -eo pipefail
source "$1"
cd "$2"
shift 2
workers="$1"
shift
# colcon leaves CMAKE_BUILD_TYPE empty, which means -O0: the MPC solve takes
# 12 ms instead of 1 ms. The control loop runs at 100 Hz, so build optimised.
colcon build --symlink-install --parallel-workers "$workers" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
' _ "${ROS_SETUP}" "${WS_DIR}" "${JOBS}" "${EXTRA[@]}"

echo "[build_ros2] done: ${WS_DIR}/install"
echo "[build_ros2] source ${WS_DIR}/install/setup.bash"
