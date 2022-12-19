#!/bin/bash

# Trigger an error if non-zero exit code is encountered
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$HOME/$ROS2_WORKSPACE/install/local_setup.bash"

if [ "${1}" == "launch" ]; then
    shift # consume the 1st argument
    exec ros2 launch roboprinter_moveit_config demo.launch.py
else
    # Container development
    exec "${@}"
fi
