Building an industrial-grade Magnetic Line Following AGV requires shifting from a "hobbyist" mindset to a rigid, deterministic, and compliance-driven engineering model. To provide a complete mental model, we have to design the system from the strict compliance protocols down to the exact ROS 2 lifecycle states and the user interface semantics.

Here is the exhaustive, end-to-end architectural blueprint for your AGV.

---

### 1. Safety & Compliance Layer (The Foundation)

You cannot architect an AGV without starting at safety. The architecture must comply with **ISO 3691-4** (European/International) and **ANSI/ITSDF B56.5** (North American) standards.

* **Risk Assessment & Performance Levels (PLr):** Under ISO 3691-4, your safety architecture must meet a specific Performance Level (typically PL d or PL e) for critical functions like Emergency Stops and Personnel Detection.
* **The Safety Core:** You do not handle safety in ROS or a standard PC. You route all critical safety I/O through a dedicated safety-rated controller, such as a **Schneider PLC**.
* **Zone Architecture:**
* **Warning Field:** When a person enters the outer zone of your safety LiDAR, the PLC signals the drive controller to reduce to a safe creeping speed.
* **Protective Field:** If the inner zone is breached, the PLC cuts the Safe Torque Off (STO) circuit to the motor drives directly. Zero software intervention is required; it is a hardware-enforced halt.



---

### 2. Hardware & Communications Topology

The hardware must be segmented by frequency and criticality.

* **High-Level Compute (Edge PC):** An x86 industrial PC or Nvidia Jetson running Ubuntu 22.04 and ROS 2. Handles navigation algorithms, logging, and fleet communication.
* **Safety & I/O Controller (Schneider PLC):** Polls the safety LiDARs, physical E-stop buttons, and bumper switches. It communicates with the Edge PC via Modbus TCP.
* **Motor Control (Roboteq Dual Channel):** A heavy-duty motor controller managing differential drive kinematics. It receives velocity commands via CANopen or RS232 and executes the internal PID loop for motor speed.
* **Magnetic Guide Sensor (e.g., Roboteq MGS1600):** An array of Hall-effect sensors detecting the magnetic tape. It outputs the cross-track error (distance from the tape's center) and marker detections via CAN, RS232, or MultiPWM.
* **Obstacle Avoidance (SICK TiM Lidar):** Mounted at the front, communicating zone breaches directly to the PLC via discrete I/O, while sending point cloud data to the Edge PC via Ethernet for mapping/diagnostics.
* **Power System:** A 24V/48V Li-ion pack with a Smart BMS publishing cell voltages, temperatures, and State of Charge (SOC) over CAN bus.

---

### 3. Robot Software Architecture (ROS 2 Deep Dive)

For absolute reliability and sim-to-real portability, the software stack relies heavily on **ROS 2 Lifecycle Nodes** and the `ros2_control` framework. Lifecycle nodes ensure the robot powers up predictably (Unconfigured $\rightarrow$ Inactive $\rightarrow$ Active).

**A. The `ros2_control` Hardware Abstraction Layer (HAL)**
This is the bridge between software and hardware.

* **System Interface (`robot_hardware_interface`):** A custom C++ plugin that opens the CAN/serial ports. In the `read()` method, it grabs encoder ticks from the Roboteq controller and tape position from the MGS. In the `write()` method, it sends the calculated RPM commands to the motors.
* **Standard Controllers:** You load the `diff_drive_controller` from `ros2_controllers`. It subscribes to standard `geometry_msgs/Twist` messages and outputs individual wheel velocities to your HAL.

**B. The Navigation Graph**

* **`/mgs_driver_node`:** Parses the raw CAN/Modbus frames from the magnetic sensor. It publishes a custom message (e.g., `agv_msgs/MagneticTrackData`) containing `cross_track_error` (float) and `tape_detected` (boolean).
* **`/line_follower_logic_node`:** A deterministic PID node.
* **Input:** `cross_track_error`.
* **Math:** $Output = K_p(error) + K_i\int(error)dt + K_d\frac{d(error)}{dt}$
* **Output:** The PID output maps to the angular-Z velocity in a `Twist` message. Linear-X is kept constant unless an intersection marker is detected or a warning zone is breached.


* **`/safety_monitor_node`:** Subscribes to the SICK TiM laser scan data and PLC status. If the warning field is breached, it overrides the `Twist` multiplexer (`twist_mux`) to drop the linear velocity.

**C. The State Machine**
Use **Behavior Trees (BT.CPP)** to manage the robot's state, rather than spaghetti `if/else` loops.

* **States:** `IDLE`, `FOLLOWING_LINE`, `TURNING_AT_MARKER`, `OBSTACLE_PAUSE`, `HARD_E_STOP`, `TELEOP`.
* **Transitions:** If the MGS node publishes `tape_detected = false` for more than 500ms, the BT forces a transition to `HARD_E_STOP` and alerts the fleet manager.

---

### 4. Simulation Architecture

A rigorous simulation environment means you write the code once.

* **Engine:** Gazebo Ignition.
* **URDF/Xacro:** The robot model includes exact mass properties, friction coefficients for the drive and castor wheels, and collision meshes.
* **The Sim-to-Real Trick:** Inside your URDF, you specify the `gazebo_ros2_control` plugin. When you launch in simulation, the ROS stack talks to Gazebo. When you launch on the real robot, it talks to your custom HAL. The PID node and Behavior Tree **do not know the difference**.
* **Simulating the MGS:** Since Gazebo doesn't natively simulate magnetic fields, you mount a downward-facing camera in the URDF looking at a drawn black line. An OpenCV node processes the image to calculate the cross-track error, perfectly mimicking the data structure of the real magnetic sensor.

---

### 5. UI & Fleet Management Architecture

The user interface should be a web-based dashboard strictly decoupled from the robot via an MQTT broker (like Eclipse Mosquitto).

**A. Backend Telemetry & Command**

* A ROS 2 node (`mqtt_bridge`) serializes robot health data into JSON and publishes it to the broker over Wi-Fi.
* The backend records historical data (like motor current spikes) into a time-series database (InfluxDB) for predictive maintenance.

**B. UI Functional Requirements (The Operator's View)**
The frontend must be functional, high-contrast, and focused on operational awareness.

* **Status Header:** Immediate, undeniable visibility of the AGV's state (e.g., green for `RUNNING`, flashing amber for `WARNING_ZONE`, flashing red for `E_STOP`).
* **Telemetry Gauges:**
* Live battery SOC and discharging rate.
* Drive motor temperatures.
* Current speed vs. Target speed.


* **Diagnostic Matrix (Crucial for Maintenance):**
* A visual indicator of the SICK LiDAR zones. Operators need to see exactly *which* zone (warning or protective) is currently triggered to understand why the robot stopped.
* Live visualization of the MGS array—showing which specific hall sensors currently detect the tape to diagnose dead zones.


* **Mission Dispatcher:** A semantic routing map where an operator can assign tasks (e.g., "Go to Station 4, wait 30 seconds, proceed to Charging").
* **Teleop Override:** A secure, virtual joystick for manual extraction. Engaging this must securely command the Behavior Tree to enter `TELEOP` mode, superseding the line-following PID.

---

This architecture completely isolates your safety hardware from your high-level ROS 2 logic, and decouples your robot control from your web interface. Where would you like to focus next? We can drill down into the exact structure of the `ros2_control` hardware interface plugin, or map out the specific Modbus registers for the PLC integration.