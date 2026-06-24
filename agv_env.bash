#!/bin/bash
# AGV Master Configuration Environment Variables

# ==========================================
# Deployment Settings
# ==========================================

# MODE options: HARDWARE, SIM
export MODE=SIM
# SIM_TOOL options: MUJOCO, GZ
export SIM_TOOL=MUJOCO

# ==========================================
# Kinematic Properties
# ==========================================
export KINEMATIC_MODEL=DIFF_DRIVE
export WHEEL_BASE=0.512
export WHEEL_RADIUS=0.08
export GEAR_RATIO=1.0
export MAX_RPM=3000
export TICKS_PER_REV=1024

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

echo "AGV Environment Variables Loaded!"
echo "  MODE: $MODE"
echo "  SIM_TOOL: $SIM_TOOL"
echo "  KINEMATICS: $KINEMATIC_MODEL (Base: ${WHEEL_BASE}m, Radius: ${WHEEL_RADIUS}m)"
