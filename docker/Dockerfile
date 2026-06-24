# Stage 1: Base image with dependencies
FROM ros:jazzy-ros-base AS base

# Install dependencies needed for both host and deployment
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    can-utils \
    iproute2 \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Setup entrypoint to source ROS 2 automatically
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Stage 2: Host Dev Image (Stops here, expects volume mounts)
FROM base AS host
ENTRYPOINT ["/entrypoint.sh"]
# Idle by default so user can exec into it and run colcon build or launch natively
CMD ["sleep", "infinity"]

# Stage 3: Builder Image (For deployment compilation)
FROM base AS builder
# Copy the source code from both workspaces
COPY deepak_ws/src/ /ros2_ws/src/
COPY manasa_ws/src/ /ros2_ws/src/

# Build the entire workspace without symlinks for a standalone install
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Stage 4: Deployment Image
FROM base AS deployment
# Copy ONLY the compiled install folder from the builder stage
COPY --from=builder /ros2_ws/install /ros2_ws/install

ENTRYPOINT ["/entrypoint.sh"]
# Run the launch file automatically
CMD ["ros2", "launch", "agv_bringup", "bringup.launch.py"]
