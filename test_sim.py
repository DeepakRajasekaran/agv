import sys
sys.path.append("/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/scripts")

import mujoco
import numpy as np
import math
from TapeMap import TapeMap

# Load the model
model_path = "/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/models/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)

# Modify kv to 5.0
# gainprm[0] is the gain (kv)
# biasprm[1] is the bias gain (-kv)
model.actuator('left_wheel_motor').gainprm[0] = 5.0
model.actuator('left_wheel_motor').biasprm[1] = -5.0
model.actuator('right_wheel_motor').gainprm[0] = 5.0
model.actuator('right_wheel_motor').biasprm[1] = -5.0

data = mujoco.MjData(model)
tape_map = TapeMap()

# Set initial pose
data.qpos[0] = -2.5  # X
data.qpos[1] = 2.02 # Y
data.qpos[2] = 0.08 # Z
data.qpos[3] = 1.0  # qw
data.qpos[4] = 0.0
data.qpos[5] = 0.0
data.qpos[6] = 0.0

# PID Parameters to test
kp = 10.0
ki = 0.0
kd = 0.5
prev_error = 0.0
integral = 0.0
dt = 0.01

wheel_base = 0.512
wheel_radius = 0.08
nominal_speed = 1.5

# Step for 60 steps
for step in range(1, 61):
    # Get robot pose
    rx = data.qpos[0]
    ry = data.qpos[1]
    qw, qx, qy, qz = data.qpos[3:7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    # Compute error
    closest_pt, dist = tape_map.get_closest_point((rx, ry), tape_map.active_path)
    track_detected = dist < 0.090
    
    error = 0.0
    if track_detected:
        error = tape_map.compute_lateral_error(rx, ry, yaw)
        
    # Scale speed based on error
    error_ratio = min(1.0, max(0.0, abs(error) / 0.090))
    speed = nominal_speed * (1.0 - 0.70 * error_ratio)
    
    # PID controller
    p_term = kp * error
    integral += error * dt
    i_term = ki * integral
    d_term = kd * (error - prev_error) / dt
    prev_error = error
    
    steer = p_term + i_term + d_term
    steer = max(-3.0, min(steer, 3.0))
    
    # Differential drive kinematics
    v_l = (speed - (steer * wheel_base / 2.0)) / wheel_radius
    v_r = (speed + (steer * wheel_base / 2.0)) / wheel_radius
    
    # Apply motor commands
    data.actuator('left_wheel_motor').ctrl[0] = -v_l
    data.actuator('right_wheel_motor').ctrl[0] = -v_r
    
    mujoco.mj_step(model, data)
    
    print(f"Step {step}: pos=({data.qpos[0]:.4f}, {data.qpos[1]:.4f}), yaw={yaw:.3f} | err={error:.4f} | cmd_l={v_l:.2f}, cmd_r={v_r:.2f} | qvel_l={data.joint('left_wheel_joint').qvel[0]:.2f}, qvel_r={data.joint('right_wheel_joint').qvel[0]:.2f}")
