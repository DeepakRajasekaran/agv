# Stage 1: Base image with dependencies
FROM ros:jazzy-ros-base AS base

# Install dependencies needed for both host and deployment
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    can-utils \
    iproute2 \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-twist-stamper \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /agv

# Add ROS setup to bashrc for interactive exec sessions
RUN echo "if [ -f /agv_env.bash ]; then source /agv_env.bash; fi" >> ~/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /agv/deepak_ws/install/setup.bash ]; then source /agv/deepak_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "if [ -f /agv/manasa_ws/install/setup.bash ]; then source /agv/manasa_ws/install/setup.bash; fi" >> ~/.bashrc

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
# Build deepak_ws
WORKDIR /agv/deepak_ws
COPY deepak_ws/src/ ./src/
RUN --mount=type=cache,target=/agv/deepak_ws/build \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -G Ninja"

# Build manasa_ws independently
WORKDIR /agv/manasa_ws
COPY manasa_ws/src/ ./src/
RUN --mount=type=cache,target=/agv/manasa_ws/build \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -G Ninja"

# Stage 4: Deployment Image
FROM base AS deployment
# Copy ONLY the compiled install folders from the builder stage
COPY --from=builder /agv/deepak_ws/install /agv/deepak_ws/install
COPY --from=builder /agv/manasa_ws/install /agv/manasa_ws/install

ENTRYPOINT ["/entrypoint.sh"]
# Idle by default so user can launch specific nodes manually
CMD ["sleep", "infinity"]
