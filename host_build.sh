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

ensure_container_running() {
    if ! docker ps --format '{{.Names}}' | grep -Eq "^agv$"; then
        echo -e "\n\033[1;33mContainer 'agv' is not running. Building and starting it automatically...\033[0m"
        docker compose -f docker/docker-compose.yml -f docker/docker-compose.host.yml up -d --build
    fi
}

case $CHOICE in
    1)
        echo "Starting Host Dev Container..."
        docker compose -f docker/docker-compose.yml -f docker/docker-compose.host.yml build
        docker compose -f docker/docker-compose.yml -f docker/docker-compose.host.yml up -d
        echo "Container 'agv' is now running in Host Mode!"
        ;;
    2)
        ensure_container_running
        echo "Which workspace do you want to build?"
        echo "  a) deepak_ws only"
        echo "  b) manasa_ws only"
        echo "  c) Both"
        echo -n "Enter choice [a-c]: "
        read -r WS_CHOICE
        
        if [ "$WS_CHOICE" == "a" ]; then
            docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && cd /agv/deepak_ws && colcon build"
        elif [ "$WS_CHOICE" == "b" ]; then
            docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && source /agv/deepak_ws/install/setup.bash && cd /agv/manasa_ws && colcon build"
        else
            docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && cd /agv/deepak_ws && colcon build && source install/setup.bash && cd /agv/manasa_ws && colcon build"
        fi
        echo "Build complete!"
        ;;
    3)
        ensure_container_running
        echo "Connecting to container shell..."
        docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && [ -f /agv/deepak_ws/install/setup.bash ] && source /agv/deepak_ws/install/setup.bash && [ -f /agv/manasa_ws/install/setup.bash ] && source /agv/manasa_ws/install/setup.bash; exec bash"
        ;;
    4|*)
        echo "Exiting."
        exit 0
        ;;
esac
