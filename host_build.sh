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

# Source the AGV environment variables
source ./agv_env.bash
# Force host build mode for local development to avoid exec format errors
export BUILD_MODE=host

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
        echo "Building robot_ws..."
        docker exec agv bash -c "source /opt/ros/jazzy/setup.bash && cd /agv/robot_ws && colcon build"
        echo "Build complete!"
        ;;
    3)
        ensure_container_running
        echo "Connecting to container shell..."
        docker exec -it agv bash -c "source /opt/ros/jazzy/setup.bash && if [ -f /agv/robot_ws/install/setup.bash ]; then source /agv/robot_ws/install/setup.bash; fi; exec bash"
        ;;
    4|*)
        echo "Exiting."
        exit 0
        ;;
esac
