# Magnetic AGV Architecture Plan

This document outlines the end-to-end architectural blueprint for the Magnetic Line Following AGV, integrating the strict software requirements with the hardware topology.

## 1. Hardware & Communications Topology

* **High-Level Compute (Edge PC):** An x86 industrial PC or Nvidia Jetson running Ubuntu 22.04 and ROS 2. Handles navigation algorithms, logging, and fleet communication.
* **Motor Control (Roboteq Motor Controller):** A heavy-duty motor controller managing differential drive kinematics. It receives velocity commands via CAN and executes the internal PID loop for motor speed.
* **Magnetic Line Sensor & Tag Reader:** An array of sensors detecting the magnetic tape and RFID/magnetic tags. Outputs cross-track error and tag IDs via CAN/serial.
* **Drive System:** Differential Drive with Left and Right Wheel Motors.

## 2. Software Architecture Overview

For absolute reliability and portability, the software stack relies on **Object-oriented design**, **ROS 2 Lifecycle Nodes**, and the `ros2_control` framework. 

### Task 1 – CAN Driver Development (HAL)
A dedicated, thread-safe CAN driver abstraction layer.
* **Receive & Decode:** Motor feedback, RPM, Current, Fault status, Sensor messages.
* **Transmit:** Left/right wheel speed commands, Controller enable/disable, Emergency stop.
* **Deliverables:** CAN Driver Library, Test Application, and Documentation with full logging/error handling.

### Task 2 – Differential Drive Control (Kinematics)
Implementation of differential drive kinematics bridging high-level ROS commands to low-level motor control.
* **Inputs:** Linear velocity (V) and Angular velocity (W).
* **Outputs:** Left wheel speed, Right wheel speed.
* **Capabilities:** Forward, Reverse, In-place rotation, Curved motion.

### Task 3 & 4 – Magnetic Line Following & PID Controller
* **The `/line_follower_logic_node`:** A deterministic PID node.
  * **Input:** `Error = Tape Center Position - Robot Position` (Cross-track error).
  * **Math:** $Output = K_p(error) + K_i\int(error)dt + K_d\frac{d(error)}{dt}$
  * **Output:** Steering Correction (Angular-Z velocity).
  * **Requirements:** Runtime parameter adjustment, Integral windup protection, Output limiting, and Logging.

### Task 5 – Junction Handling
Integrating the Tag Reader data to make navigation decisions at intersections. The robot will read tags to determine specific routing logic and branch onto the correct path.

### Task 6 – Navigation State Machine
Using **Behavior Trees (BT.CPP)** to manage the robot's navigation states.
* **States:** `Idle`, `Initialize`, `Follow Line`, `Junction Detected`, `Read Tag`, `Execute Turn`, `Resume Tracking`, `Stop`, `Error`.
* **Features:** State transition logging, Fault recovery, and Safe stopping.

## 3. Simulation Architecture (Optional but Recommended)
* **Engine:** Gazebo Ignition.
* **Sim-to-Real Trick:** Using `gazebo_ros2_control` in simulation, and seamlessly swapping to the custom CAN HAL for the real robot. The PID node and Behavior Tree remain completely agnostic.

## 4. UI & Fleet Management
* Web-based dashboard decoupled from the robot via MQTT.
* Live Telemetry (RPM, Battery, Currents) and Mission Dispatching.