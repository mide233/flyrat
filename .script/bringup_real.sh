#!/usr/bin/env bash
#
# One-command bring-up for the real-vehicle sensor + SLAM stack:
#
#   adb forward -> device forwarder -> sensor_receiver_node -> slam_node
#
# Usage:
#   .script/bringup_real.sh [launch arguments...]
#
# Examples:
#   .script/bringup_real.sh
#   .script/bringup_real.sh start_foxglove:=true
#   .script/bringup_real.sh start_slam:=false
#   .script/bringup_real.sh vocabulary_path:=/path/to/ORBvoc.txt
#
# Environment:
#   ROS_DISTRO       ROS 2 distribution (default: jazzy)
#   FLYRAT_ROS_WS    ROS 2 workspace (default: <repo>/flyrat_play/ros2_ws)
#   ADB_SERIAL       Device serial when several devices are connected
#   FLYRAT_ORB_VOCAB ORBvoc.txt path when it is not in playground/ORB_SLAM3
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WS="${FLYRAT_ROS_WS:-${REPO_ROOT}/flyrat_play/ros2_ws}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${ROS_WS}/install/setup.bash"

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS 2 ${ROS_DISTRO} not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [ ! -f "${WS_SETUP}" ]; then
    echo "workspace not built: ${WS_SETUP}" >&2
    echo "Build it first: .script/build_ros2.sh" >&2
    exit 1
fi

# ROS setup scripts reference unset variables, so nounset must be off while
# sourcing them.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"
set -u

echo "[bringup] adb devices:"
adb devices

exec ros2 launch flyrat_slam bringup.launch.py "$@"
