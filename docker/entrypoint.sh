#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it has been built
if [ -f /agv/deepak_ws/install/setup.bash ]; then
    source /agv/deepak_ws/install/setup.bash
fi

if [ -f /agv/manasa_ws/install/setup.bash ]; then
    source /agv/manasa_ws/install/setup.bash
fi

exec "$@"
