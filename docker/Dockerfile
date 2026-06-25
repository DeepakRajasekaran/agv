# Stage 1: The target sysroot (ARM64)
FROM --platform=${TARGETPLATFORM:-linux/arm64} ros:jazzy-ros-base AS target_sysroot

# Install dependencies needed for deployment on the target sysroot
# QEMU can handle apt-get fine, it's only C++ compilation that is slow/crashes
RUN apt-get update && apt-get install -y \
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
    ros-jazzy-twist-mux \
    && rm -rf /var/lib/apt/lists/*

# Stage 1.5: Host Dev Image (Stops here, expects volume mounts)
FROM target_sysroot AS host
WORKDIR /agv

# Add ROS setup to bashrc for interactive exec sessions
RUN echo "if [ -f /agv_env.bash ]; then source /agv_env.bash; fi" >> ~/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /agv/deepak_ws/install/local_setup.bash ]; then source /agv/deepak_ws/install/local_setup.bash; fi" >> ~/.bashrc && \
    echo "if [ -f /agv/manasa_ws/install/local_setup.bash ]; then source /agv/manasa_ws/install/local_setup.bash; fi" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# Idle by default so user can exec into it and run colcon build or launch natively
CMD ["sleep", "infinity"]

# Stage 2: The native builder (AMD64 host architecture)
FROM --platform=$BUILDPLATFORM ros:jazzy-ros-base AS builder
ARG BUILD_DEEPAK=true
ARG BUILD_MANASA=true

# Install cross-compiler and host build tools natively
RUN apt-get update && apt-get install -y \
    g++-aarch64-linux-gnu \
    gcc-aarch64-linux-gnu \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Copy the entire ARM64 filesystem to use as a sysroot
COPY --from=target_sysroot / /sysroot/

# SYMLINK HACK: ROS 2 ament_cmake heavily hardcodes absolute paths to /opt/ros/jazzy.
# By deleting the host's x86_64 ROS 2 install and symlinking to the ARM64 sysroot,
# we trick CMake into seamlessly finding the ARM64 .so libraries without complex toolchains!
RUN rm -rf /opt/ros && ln -s /sysroot/opt/ros /opt/ros

COPY docker/aarch64_toolchain.cmake /agv/aarch64_toolchain.cmake

RUN mkdir -p /agv/deepak_ws/install /agv/manasa_ws/install

# Build deepak_ws natively using the cross-compiler
WORKDIR /agv/deepak_ws
COPY deepak_ws/sr[c]/ ./src/
# Two-pass build: first build message interfaces, then everything else that depends on them
RUN if [ "$BUILD_DEEPAK" = "true" ]; then \
        /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
            colcon build \
                --merge-install \
                --packages-select custom_interfaces robot_description \
                --cmake-args \
                -DCMAKE_TOOLCHAIN_FILE=/agv/aarch64_toolchain.cmake \
                -DCMAKE_SYSROOT=/sysroot \
                -DCMAKE_BUILD_TYPE=Release \
                -G Ninja && \
            export CMAKE_PREFIX_PATH=/agv/deepak_ws/install:\$CMAKE_PREFIX_PATH && \
            export AMENT_PREFIX_PATH=/agv/deepak_ws/install:\$AMENT_PREFIX_PATH && \
            colcon build \
                --merge-install \
                --event-handlers console_direct+ \
                --cmake-args \
                -DCMAKE_TOOLCHAIN_FILE=/agv/aarch64_toolchain.cmake \
                -DCMAKE_SYSROOT=/sysroot \
                -DCMAKE_BUILD_TYPE=Release \
                -G Ninja"; \
    fi

# Build manasa_ws natively using the cross-compiler
WORKDIR /agv/manasa_ws
COPY manasa_ws/sr[c]/ ./src/
RUN if [ "$BUILD_MANASA" = "true" ]; then \
        /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build \
            --merge-install \
            --cmake-args \
            -DCMAKE_TOOLCHAIN_FILE=/agv/aarch64_toolchain.cmake \
            -DCMAKE_SYSROOT=/sysroot \
            -DCMAKE_BUILD_TYPE=Release \
            -G Ninja"; \
    fi

# Stage 3: Deployment Image (ARM64)
FROM target_sysroot AS deployment
WORKDIR /agv

# Add ROS setup to bashrc for interactive exec sessions
RUN echo "if [ -f /agv_env.bash ]; then source /agv_env.bash; fi" >> ~/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /agv/deepak_ws/install/local_setup.bash ]; then source /agv/deepak_ws/install/local_setup.bash; fi" >> ~/.bashrc && \
    echo "if [ -f /agv/manasa_ws/install/local_setup.bash ]; then source /agv/manasa_ws/install/local_setup.bash; fi" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Copy ONLY the compiled install folders from the builder stage
# Because we used --merge-install, the folder structure is simpler
COPY --from=builder /agv/deepak_ws/install /agv/deepak_ws/install
COPY --from=builder /agv/manasa_ws/install /agv/manasa_ws/install

ENTRYPOINT ["/entrypoint.sh"]
# Idle by default so user can launch specific nodes manually
CMD ["sleep", "infinity"]
