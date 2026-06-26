# AGV Path Follower & Context Documentation

This document serves as a brain-dump of the current system architecture, state machines, and recent modifications to the AGV line following logic.

## 1. File Structure & Parameters
- **`params.yaml`**: The single source of truth for `path_follower_node` configuration.
- **`controller_params.yaml`**: This is an obsolete, redundant file. The launch file (`play.launch.py`) only loads `params.yaml`. We will delete `controller_params.yaml`.
- **`lost_threshold`**: This parameter is completely unused (dead code) inside `PidController.cpp` and `FaultMonitor`. It will be deleted.
- **`line_lost_grace_steps`**: Currently defines the safety timeout in terms of 50Hz control loop cycles (e.g., 10 steps = 200ms). This will be refactored to a time-based parameter (`grace_period_ms`) so it is completely agnostic of the loop rate.

## 2. Core Architecture

### **Hardware Interface (`MgsDriverNode.cpp`)**
- Communicates with the Roboteq MGS1600 magnetic sensor via CAN (SocketCAN).
- Outputs exactly three tracks:
  - `/sensor/track_position`: The currently "selected" track position (defaults to average).
  - `/sensor/left_track_position`
  - `/sensor/right_track_position`
- Provides boolean flags for `track_detect` (with a small buffer), `left_marker`, `right_marker`, and `tape_cross`.
- Exposes ROS services (`follow_left`, `follow_right`, `clear_follow`) which translate into CAN commands to force the sensor to lock onto a specific branch.

### **Path Follower (`PidController.cpp`)**
- The main 50Hz control loop. Subscribes to the sensor outputs and computes steering (`angular.z`) using a PID controller.
- **Fault Monitor**: Continuously checks if `track_detect` is false. If it remains false for longer than the grace period, it throws an `ERROR` state and halts the robot.
- **Behavior Tree (BT)**: Intercepts the nominal velocity and scales it down dynamically based on the current track error (e.g., slowing down in sharp turns to prevent derailment).
- Uses `twist_mux` to gracefully fall back to teleop or joystick if needed.

### **Junction Manager (`junction_manager.py`)**
- A 20Hz middleware script that handles track splitting (divergence).
- **Recent Refactor**: 
  - The MGS1600 side markers (`left_marker`, `right_marker`) are **no longer used**.
  - Instead, it monitors the physical divergence between `left_track_pos` and `right_track_pos`.
  - When the track splits (`divergence > 0.15m`), it looks at the robot's current physical `track_pos`.
  - If the Nav2 stack commands a turn, the robot physically drifts slightly in that direction. The `junction_manager` detects this side-drift (e.g. `track_pos > drift_tolerance`) and fires the `follow_left` or `follow_right` service to seamlessly steer the sensor onto the correct fork.

### **Nav Simulator (`nav_simulator.py`)**
- A high-level state machine acting as a stub for a fleet manager.
- If the robot drops into `ERROR` (loses the line completely), it waits 2 seconds, checks if the tape is physically detected again, and automatically calls the `/start` service to clear faults and resume tracking.
