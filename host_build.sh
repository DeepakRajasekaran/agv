#!/bin/bash
# Interactive script for local host development with Docker

set -e

echo -e "\033[0;32m=== AGV Host Development Mode ===\033[0m"
echo "Select an option:"
echo "1) Start Host Dev Container (Mounts local src/build/install folders)"
echo "2) Build Workspaces (Compiles your code inside the container)"
echo "3) Connect to Container (Interactive Bash shell)"
echo "4) Exit"
echo -n "Enter choice [1-4]: "
read -r CHOICE

# Source the AGV environment variables to get BUILD_MODE
source ./agv_env.bash

case $CHOICE in
    1)
        echo "Starting Host Dev Container..."
        docker compose -f docker/docker-compose.yml -f docker/docker-compose.host.yml build
        docker compose -f docker/docker-compose.yml -f docker/docker-compose.host.yml up -d
        echo "Container 'agv' is now running in Host Mode!"
        ;;
    2)
        echo "Building workspaces..."
        # We run colcon build inside the running container for each workspace individually
        docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && cd /agv/deepak_ws && colcon build && source install/setup.bash && cd /agv/manasa_ws && colcon build"
        echo "Build complete!"
        ;;
    3)
        echo "Connecting to container shell..."
        docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && [ -f /agv/deepak_ws/install/setup.bash ] && source /agv/deepak_ws/install/setup.bash && [ -f /agv/manasa_ws/install/setup.bash ] && source /agv/manasa_ws/install/setup.bash; exec bash"
        ;;
    4|*)
        echo "Exiting."
        exit 0
        ;;
esac
