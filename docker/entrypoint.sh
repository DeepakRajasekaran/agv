#!/bin/bash
set -e

# Source environment variables if mounted
if [ -f /agv_env.bash ]; then
    source /agv_env.bash
fi

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it has been built
if [ -f /agv/deepak_ws/install/local_setup.bash ]; then
    source /agv/deepak_ws/install/local_setup.bash
fi

if [ -f /agv/manasa_ws/install/local_setup.bash ]; then
    source /agv/manasa_ws/install/local_setup.bash
fi

exec "$@"
