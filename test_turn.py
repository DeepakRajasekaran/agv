import sys
sys.path.append("/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/scripts")

import mujoco
import numpy as np
import math
from TapeMap import TapeMap

# Load the model
model_path = "/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/models/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)
tape_map = TapeMap()

# Set to follow MID_DOWN path (shortcut down)
tape_map.active_path = tape_map.path_mid_down
tape_map.active_branch = "MID_DOWN"

# Set initial pose on the top edge, prior to shortcut
data.qpos[0] = -2.5  # X
data.qpos[1] = 2.02 # Y (offset)
data.qpos[2] = 0.08 # Z
data.qpos[3] = 1.0  # qw
data.qpos[4] = 0.0
data.qpos[5] = 0.0
data.qpos[6] = 0.0

# PID Parameters to test
kp = 15.0
ki = 0.0
kd = 1.0
prev_error = 0.0
integral = 0.0
dt = 0.01

wheel_base = 0.512
wheel_radius = 0.08
nominal_speed = 1.5

max_error = 0.0
lost_track_steps = 0

# Step for 400 steps (4 seconds)
for step in range(1, 401):
    # Get robot pose
    rx = data.qpos[0]
    ry = data.qpos[1]
    qw, qx, qy, qz = data.qpos[3:7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    
    # Compute error against active path
    closest_pt, dist = tape_map.get_closest_point((rx, ry), tape_map.active_path)
    track_detected = dist < 0.090
    
    error = tape_map.compute_lateral_error(rx, ry, yaw)
    abs_error = abs(error)
    if abs_error > max_error:
        max_error = abs_error
        
    if dist >= 0.090:
        lost_track_steps += 1
        
    # Scale speed based on error (clamping speed when error is large)
    error_ratio = min(1.0, max(0.0, abs_error / 0.090))
    # During the turn, speed is 0.5x nominal
    speed_factor = 0.5 if step > 50 else 1.0
    speed = nominal_speed * speed_factor * (1.0 - 0.70 * error_ratio)
    
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
    
    # Apply motor commands (inverted for MuJoCo coordinate frame)
    data.actuator('left_wheel_motor').ctrl[0] = -v_l
    data.actuator('right_wheel_motor').ctrl[0] = -v_r
    
    mujoco.mj_step(model, data)
    
    if step % 20 == 0 or dist >= 0.090:
        print(f"Step {step:3d}: pos=({rx:6.3f}, {ry:6.3f}), yaw={yaw:6.3f} | err={error:6.3f} | dist={dist:6.3f} | speed={speed:5.3f} | steer={steer:6.3f}")

print(f"\nSimulation completed. Max error: {max_error:.4f} m. Lost track steps: {lost_track_steps}")
