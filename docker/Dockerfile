# Stage 1: Build the workspace
FROM ros:humble-ros-base AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    can-utils \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy ONLY the source code (do not mount at runtime)
COPY src/ /ros2_ws/src/

# Build the workspace without symlinks for a standalone install
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Stage 2: Runtime Image
FROM ros:humble-ros-base

# Install only runtime dependencies
RUN apt-get update && apt-get install -y \
    can-utils \
    iproute2 \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy ONLY the compiled install folder from the builder stage
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Setup entrypoint to source ROS 2 automatically
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "roboteq_driver", "roboteq_driver.launch.py"]
