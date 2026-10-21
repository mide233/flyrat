#!/usr/bin/env bash
# Bridge the Gazebo camera on the Iris airframe into ROS 2.
#
# The camera sensor is declared on the airframe base_link in
# flyrat_play/gazebo/models/betaflight_iris/model.sdf. Gazebo publishes it on
# the scoped topics
#   /world/<world>/model/iris/link/base_link/sensor/camera/{image,camera_info}
# and this script forwards them to sensor_msgs/msg/Image and CameraInfo:
#   <prefix>/image_raw and <prefix>/camera_info   (default /flyrat/camera)
#
# Gazebo's /clock is bridged as well, so /clock carries simulation time and
# viewers can render the image stream on its own timestamps
# (`ros2 param set <node> use_sim_time true`).
#
# Run the simulator first (flyrat_play/gazebo/scripts/launch_sim.sh), then start
# this in a second terminal. Inspect the stream with .script/foxglove.sh, with
# `ros2 topic hz <prefix>/image_raw`, or with
#   ros2 run rqt_image_view rqt_image_view
#
# Usage:
#   .script/camera.sh [--prefix PREFIX]
#
#   --prefix   ROS namespace for the bridged topics (default: /flyrat/camera)
#   -h, --help show this help

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GZ_WORLD="betaflight_sitl"
GZ_SENSOR_PATH="/world/${GZ_WORLD}/model/iris/link/base_link/sensor/camera"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PREFIX="/flyrat/camera"

usage() {
    awk 'NR > 1 && /^set -euo pipefail/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix)
            [[ $# -ge 2 ]] || { echo "[camera] --prefix needs a value" >&2; exit 2; }
            PREFIX="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "[camera] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

PREFIX="${PREFIX%/}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "[camera] /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if ! ros2 pkg prefix ros_gz_bridge > /dev/null 2>&1; then
    echo "[camera] ros_gz_bridge is not installed" >&2
    echo "[camera] sudo apt-get install ros-${ROS_DISTRO}-ros-gz-bridge" >&2
    exit 1
fi

echo "[camera] ${GZ_SENSOR_PATH}/image -> ${PREFIX}/image_raw (sensor_msgs/msg/Image)"
echo "[camera] ${GZ_SENSOR_PATH}/camera_info -> ${PREFIX}/camera_info (sensor_msgs/msg/CameraInfo)"
echo "[camera] /clock -> /clock (rosgraph_msgs/msg/Clock, simulation time)"
echo "[camera] start the simulator first if the topics stay silent"

exec ros2 run ros_gz_bridge parameter_bridge \
    "${GZ_SENSOR_PATH}/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "${GZ_SENSOR_PATH}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
    --ros-args \
    -r "${GZ_SENSOR_PATH}/image:=${PREFIX}/image_raw" \
    -r "${GZ_SENSOR_PATH}/camera_info:=${PREFIX}/camera_info"
