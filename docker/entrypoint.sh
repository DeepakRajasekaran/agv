#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it has been built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
