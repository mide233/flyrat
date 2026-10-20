#!/usr/bin/env bash
#
# Launch the Foxglove bridge so Foxglove Studio (normally running on the host)
# can subscribe to the ROS 2 topics published by the simulation or by
# flyrat_pilot.
#
# Usage:
#   .script/foxglove.sh                  # ws://<host>:8765
#   .script/foxglove.sh --port 9000      # different port
#   .script/foxglove.sh -- --include-hidden
#
# Environment:
#   ROS_DISTRO   ROS 2 distribution to source (default: jazzy)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PORT=8765
ADDRESS=0.0.0.0
EXTRA=()

while [ $# -gt 0 ]; do
    case "$1" in
        --port)
            PORT="${2:?--port requires a value}"
            shift 2
            ;;
        --address)
            ADDRESS="${2:?--address requires a value}"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        --)
            shift
            EXTRA+=("$@")
            break
            ;;
        *)
            echo "unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS 2 ${ROS_DISTRO} not found: ${ROS_SETUP}" >&2
    exit 1
fi

# The ROS 2 setup scripts reference unset variables, so nounset has to be off
# while sourcing them.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "foxglove_bridge is not installed." >&2
    echo "Install it with: sudo apt-get install ros-${ROS_DISTRO}-foxglove-bridge" >&2
    exit 1
fi

if [ "${ADDRESS}" = "0.0.0.0" ]; then
    echo "[foxglove] listening on ws://0.0.0.0:${PORT} (use ws://<this-host>:${PORT} from Foxglove Studio)"
else
    echo "[foxglove] listening on ws://${ADDRESS}:${PORT}"
fi
echo "[foxglove] in Foxglove Studio: Open connection -> WebSocket -> ws://<this-host>:${PORT}"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:="${PORT}" \
    address:="${ADDRESS}" \
    "${EXTRA[@]+"${EXTRA[@]}"}"
