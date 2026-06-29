#!/bin/bash
set -e

# Source environment variables if mounted
if [ -f /agv_env.bash ]; then
    source /agv_env.bash
fi

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it has been built
if [ -f /agv/robot_ws/install/local_setup.bash ]; then
    source /agv/robot_ws/install/local_setup.bash
fi

exec "$@"
