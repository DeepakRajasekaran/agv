#!/bin/bash
# AGV Master Configuration Environment Variables

# ==========================================
# Deployment Settings
# ==========================================

# MODE options: HARDWARE, SIM
export MODE=HARDWARE
# SIM_TOOL options: MUJOCO, GZ
export SIM_TOOL=MUJOCO

# ==========================================
# Build Settings
# ==========================================
# BUILD_MODE options: host, deployment
export BUILD_MODE=deployment

# ==========================================
# Kinematic Properties
# ==========================================
export KINEMATIC_MODEL=DIFF_DRIVE
export WHEEL_BASE=0.512
export WHEEL_RADIUS=0.08
export GEAR_RATIO=10.0
export MAX_RPM=3000
export TICKS_PER_REV=10000

# ==========================================
# Hardware & Interface Settings
# ==========================================
export CAN_INTERFACE=can0
export FEEDBACK_TOPIC=/drive/feedback
export CMD_TOPIC=/cmd_rpm

# ==========================================
# Joint Names
# ==========================================
export LEFT_JOINT_NAME=left_wheel_joint
export RIGHT_JOINT_NAME=right_wheel_joint

# Configure ROS 2 Domain ID
export ROS_DOMAIN_ID=42

# Useful Aliases
alias teleop="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop/cmd_vel"

# Print Configuration
echo "=== AGV Environment Configured ==="
echo "  MODE: $MODE"
echo "  SIM_TOOL: $SIM_TOOL"
echo "  KINEMATICS: $KINEMATIC_MODEL (Base: ${WHEEL_BASE}m, Radius: ${WHEEL_RADIUS}m)"
