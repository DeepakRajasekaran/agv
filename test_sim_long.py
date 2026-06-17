import sys
sys.path.append("/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/scripts")

import mujoco
import numpy as np
import math
from TapeMap import TapeMap

def simulate(kp, ki, kd, nominal_speed=1.5, steps=2000):
    model_path = "/home/lucifer/anscer_workspace/LineFollower/src/hardware_sim/models/scene.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    tape_map = TapeMap()
    
    # Initial pose
    data.qpos[0] = -2.5  # X
    data.qpos[1] = 2.02 # Y
    data.qpos[2] = 0.08 # Z
    data.qpos[3] = 1.0  # qw
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0

    prev_error = 0.0
    integral = 0.0
    dt = 0.01
    
    wheel_base = 0.512
    wheel_radius = 0.08
    
    max_err = 0.0
    lost_track = False
    
    for step in range(1, steps + 1):
        # Step MuJoCo physics twice per 10ms control cycle
        rx = data.qpos[0]
        ry = data.qpos[1]
        qw, qx, qy, qz = data.qpos[3:7]
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        closest_pt, dist = tape_map.get_closest_point((rx, ry), tape_map.active_path)
        track_detected = dist < 0.25
        
        if dist >= 0.25:
            lost_track = True
            break
            
        error = tape_map.compute_lateral_error(rx, ry, yaw)
        abs_err = abs(error)
        if abs_err > max_err:
            max_err = abs_err
            
        # Speed scaling based on error ratio
        error_ratio = min(1.0, max(0.0, abs_err / 0.090))
        speed = nominal_speed * (1.0 - 0.70 * error_ratio)
        
        # PID
        p_term = kp * error
        integral += error * dt
        integral = max(-0.5, min(integral, 0.5))
        i_term = ki * integral
        d_term = kd * (error - prev_error) / dt
        prev_error = error
        
        steer = p_term + i_term + d_term
        steer = max(-3.0, min(steer, 3.0))
        
        v_l = (speed - (steer * wheel_base / 2.0)) / wheel_radius
        v_r = (speed + (steer * wheel_base / 2.0)) / wheel_radius
        
        data.actuator('left_wheel_motor').ctrl[0] = -v_l
        data.actuator('right_wheel_motor').ctrl[0] = -v_r
        
        mujoco.mj_step(model, data)
        mujoco.mj_step(model, data)
        
    return max_err, lost_track, step

print("Testing default parameters: kp=10.0, kd=0.5")
max_err, lost_track, steps_run = simulate(10.0, 0.0, 0.5)
print(f"Result: max_err={max_err:.4f}, lost_track={lost_track}, steps_run={steps_run}")

# Search for better parameters
results = []
for kp in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 15.0]:
    for kd in [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]:
        max_err, lost_track, steps_run = simulate(kp, 0.0, kd)
        if not lost_track:
            results.append((max_err, kp, kd))

results.sort()
print("\nTop 5 stable parameter combinations (sorted by min max_err):")
for r in results[:5]:
    print(f"kp={r[1]:.2f}, kd={r[2]:.2f} -> max_error={r[0]:.4f} m")
