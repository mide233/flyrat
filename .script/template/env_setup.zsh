#!/bin/zsh

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RCUTILS_COLORIZED_OUTPUT=1

export __GLX_VENDOR_LIBRARY_NAME=nvidia

source /opt/ros/jazzy/setup.zsh

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

export PATH="${PATH}:/workspaces/flyrat/.script"

fpath=("/workspaces/flyrat/.script/complete" $fpath)
autoload -Uz compinit
compinit
