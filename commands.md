# AGV Execution and Integration Commands Reference

This document provides a comprehensive list of commands for building, running, and managing both **Deepak's** and **Manasa's** stacks inside the unified `robot_ws` workspace.

---

## 1. Building and Sourcing

### Local Host Development
To build all 11 packages natively inside the host development container, choose **Option 2** of `./host_build.sh`:
```bash
./host_build.sh
# Enter Choice: 2
```

Alternatively, run the build command directly via `docker exec`:
```bash
docker exec agv bash -c "source /opt/ros/jazzy/setup.bash && cd /agv/robot_ws && colcon build"
```

### Sourcing & Shell Connection
To open an interactive shell session inside the container with the ROS 2 and workspace parameters fully sourced, choose **Option 3**:
```bash
./host_build.sh
# Enter Choice: 3
```

Or manually source inside a shell:
```bash
source /opt/ros/jazzy/setup.bash
source /agv/robot_ws/install/setup.bash
```

---

## 2. Deploying to Jetson Hardware

To cross-compile the target `linux/arm64` container image and deploy it over the network to the Jetson, choose **Option 2** in `./deploy_to_jetson.sh`:
```bash
./deploy_to_jetson.sh
# Enter Choice: 2
```

To run the container on the Jetson robot:
```bash
ssh nvidia@192.168.1.83 "cd /home/nvidia/agv && source ./agv_env.bash && docker compose -f docker/docker-compose.yml up -d"
```

---

## 3. Running Deepak's Stack (`stack:=deepak`)

Deepak's stack uses standard `ros2_control` mapping with a custom Modbus PLC integration and a C++ Path Follower controller.

### Start Stack on Hardware (Default)
Starts the Roboteq controller, MGS1600 sensor, PLC interface, and C++ path follower:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak follower:=true
```

### Start Simulator
Launches the simulator with simulated sensor feedback:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak sim:=true
```

To launch the simulator with a custom nominal speed (e.g. `1.2` m/s) using a ROS parameter override:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak sim:=true --ros-args -p nav_simulator:nominal_speed:=1.2
```

To dynamically tune the nominal speed at runtime without restarting:
```bash
ros2 param set /nav_simulator nominal_speed 1.5
```


### Start Drivers Only (No Follower)
Starts all robot motor drivers and sensors but leaves control to an external node:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak follower:=false
```

### Start with Custom PLC Parameters
Override the default Modbus TCP target IP and Port:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak plc_ip:="192.168.1.100" plc_port:=502
```

### Start with Custom Safety Configurations
Override track detection stability times:
```bash
ros2 launch robot_bringup bringup.launch.py stack:=deepak track_detect_stable_ms:=1500 force_track_detect:=true
```

---

## 4. Running Manasa's Stack (`stack:=manasa`)

Manasa's stack uses a distinct `ros2_control` hardware interface, direct launch integrations, and a C++ line follower driven by a ROS 2 Behavior Tree.

### Start Stack on Hardware
```bash
ros2 launch robot_bringup bringup.launch.py stack:=manasa follower:=true
```

### Start Drivers Only (No Follower / Behavior Tree)
```bash
ros2 launch robot_bringup bringup.launch.py stack:=manasa follower:=false
```

---

## 5. Topic and Parameter Management (Runtime)

These commands can be executed from a container shell (Option 3 in `host_build.sh`) to inspect topics, tune parameters, and execute diagnostics on the running robot.

### Parameters
List all active node parameters:
```bash
ros2 param list
```

Retrieve specific parameter values:
```bash
ros2 param get /plc_interface plc_ip
ros2 param get /path_follower safety.track_detect_stable_ms
```

Modify parameters at runtime (without restarting nodes):
```bash
ros2 param set /plc_interface plc_ip "192.168.1.10"
```

### Monitoring Topics
Watch raw feedback from the MGS1600 tape sensor:
```bash
ros2 topic echo /sensor/track_detect
ros2 topic echo /mgs_driver/mgs_error
```

Watch PLC inputs/outputs:
```bash
ros2 topic echo /plc_interface/lidar_fb
ros2 topic echo /plc_interface/estop_fb
```

Verify motor speed commands from the controller:
```bash
ros2 topic echo /diff_drive_controller/cmd_vel
```

### Modbus PLC Service Calls
Trigger the Quickstop safety routine (sends command `7` to register 108):
```bash
ros2 service call /plc_interface/trigger_quickstop std_srvs/srv/SetBool "{data: true}"
```

Reset the Quickstop safety routine (sends command `0` to register 108):
ros2 service call /plc_interface/trigger_quickstop std_srvs/srv/SetBool "{data: false}"
```

---

## 6. Path Follower Services & State (Deepak Implementation)

### Monitoring
```bash
# See the current orchestrator state (IDLE, FOLLOW_LINE, JUNCTION_DETECTED, TURN, ERROR, etc.)
ros2 topic echo /controller_state

# Monitor cross-track divergence (distance between left/right tracks)
ros2 topic echo /path_follower/divergence

# See the final velocity output being sent to the motor driver
ros2 topic echo /path_follower/cmd_vel
```

### Services
```bash
# Start tracking
ros2 service call /path_follower/start std_srvs/srv/Trigger

# Stop tracking
ros2 service call /path_follower/stop std_srvs/srv/Trigger

# Force track selection at junction (0=avg, 1=left, 2=right)
ros2 service call /path_follower/select_track custom_interfaces/srv/SelectTrack "{track_id: 1}"

# Trigger an auto-tune sequence (resets Kp, Ki, Kd to hardcoded baseline)
ros2 service call /path_follower/autotune std_srvs/srv/Trigger

# Save current parameters to the persistent agv_config/follower_params.yaml file
ros2 service call /path_follower/save_tuning std_srvs/srv/Trigger
```

### Live Parameter Tuning
```bash
# PID Tuning
ros2 param set /path_follower_node pid.kp 1.5
ros2 param set /path_follower_node pid.ki 0.05
ros2 param set /path_follower_node pid.kd 0.2

# Velocity Clamping (Speed Limits)
ros2 param set /path_follower_node turn.clamp_straight 1.2
ros2 param set /path_follower_node turn.clamp_turn 0.4
ros2 param set /path_follower_node turn.clamp_junction 0.25

# Behavior Adjustments
ros2 param set /path_follower_node behavior_tree.enable_recovery false
ros2 param set /path_follower_node behavior_tree.exit_buffer_s 2.0
```

---

## 7. Additional Hardware Interfaces

### Magnetic Guidance Sensor (MGS)
```bash
# Force the sensor hardware to lock onto specific tape
ros2 service call /mgs_driver/follow_left std_srvs/srv/Trigger
ros2 service call /mgs_driver/follow_right std_srvs/srv/Trigger
ros2 service call /mgs_driver/clear_follow std_srvs/srv/Trigger
```

### Motor Driver Fault Management
```bash
# Reset Hardware Faults
ros2 service call /roboteq_driver/reset_faults std_srvs/srv/Trigger

# Hardware Emergency Stop / Quickstop
ros2 service call /roboteq_driver/set_estop std_srvs/srv/Trigger
ros2 service call /roboteq_driver/reset_estop std_srvs/srv/Trigger
ros2 service call /roboteq_driver/set_quickstop std_srvs/srv/Trigger
ros2 service call /roboteq_driver/reset_quickstop std_srvs/srv/Trigger
```
