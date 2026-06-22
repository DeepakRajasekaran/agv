When configuring and troubleshooting Roboteq motor controllers for complex robotic systems, having the right documentation on hand is essential. Here is a comprehensive breakdown of the most valuable resources for driver setup, scripting, and networking.

### 1. Roboteq Driver Configuration & Setup

The core of setting up any Roboteq driver involves their PC utility and the primary user manuals.

* **Roboteq Controllers User Manual (v2.1):** This is the definitive guide for setting up both brushed and brushless motor controllers. It covers I/O configurations, motor tuning (PID loops), and functional safety features like Safe Torque Off (STO).
* **Roborun+ PC Utility:** The official desktop software used for real-time configuration, monitoring, and tuning. It provides a visual interface for adjusting parameters and plotting motor response before hardcoding any values.
* **Controller-Specific Datasheets:** Always reference the exact datasheet for the model in use (e.g., FBLG2360T or SDC2160). These documents detail the specific pinouts, hall sensor/SSI encoder wiring diagrams, and absolute maximum ratings critical for the hardware build.

### 2. MicroBasic Scripting

MicroBasic allows the controller to run autonomously, chain motion sequences, or adapt parameters at runtime without relying heavily on a master PLC or external compute unit.

* **MicroBasic Scripting Manual:** The primary syntax and function reference. It explains how to write scripts using a BASIC-like syntax, implement control loops (`While`, `Do Until`), handle conditional logic, and read/write configuration parameters directly from the script.
* **Roboteq MicroBasic Simulator:** Built into the Roborun+ utility, this allows you to test script logic, verify variable execution, and simulate timer behaviors before flashing the script to the controller's permanent memory.
* **Version Control Integration:** When managing complex scripts for various hardware iterations, keep your `.mbs` (MicroBasic) files tracked using GitHub to maintain a clear history of parameter changes and script optimizations across the team.

### 3. CAN Communication Implementations

For multi-node architectures, networking the controllers via CAN bus is highly efficient, especially when integrating with ROS or ROS 2 frameworks.

* **Roboteq CAN Networking Manual (v2.1a):** This document breaks down the four supported CAN protocols: RawCAN, MiniCAN, RoboCAN, and CANopen. It details the setup required for baud rates, termination resistors, and node IDs.
* **RoboCAN Documentation:** Ideal for simple, low-cost twisted pair networks where multiple Roboteq controllers need to share sensor data or motion commands with each other seamlessly.
* **CANopen Standard Implementation:** Essential for interoperability with third-party sensors and controllers. The manual details Object Dictionary indexing (e.g., index `0x210D` for battery voltage) and Service Data Objects (SDOs).
* **Manual SDO Frame Construction:** Roboteq provides engineering guides on how to manually construct CANopen frames (defining Node ID, Command Specifier, Index, and Data) to query specific parameters or send commands directly from a microcomputer without needing to implement a full, heavy CANopen stack.


edge device credentials

ssh nvidia@192.168.1.83
password: nvidia

restrictions -> mbs script cant be larger than 8092 bytes after compilation due license issues

