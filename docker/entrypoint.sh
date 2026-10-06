#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f ~/ros_ws/install/setup.bash ]; then
    source ~/ros_ws/install/setup.bash
fi

exec "$@"
