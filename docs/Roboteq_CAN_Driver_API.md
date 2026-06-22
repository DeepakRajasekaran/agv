# Roboteq CAN Driver

The `roboteq_driver` package provides a standalone ROS 2 CAN Driver and an optional `ros2_control` hardware interface for Roboteq Motor Controllers running on CANOpen / RawCAN.

## Overview

1. **`roboteq_driver_node`:** A standalone ROS 2 Node that interfaces directly with SocketCAN. It polls the Roboteq controller asynchronously at 20Hz for telemetry and acts on velocity commands and system services.
2. **`HardwareInterface`:** A `ros2_control` SystemInterface plugin. It acts as an abstraction bridge, passing data between the standard `diff_drive_controller` and the `roboteq_driver_node` over ROS topics.

---

## Configuration

Global parameters for the driver and hardware interface are configured in `config/params.yaml`.

```yaml
roboteq_driver_node:
  ros__parameters:
    can_interface: "can0"
    feedback_topic: "/drive/feedback"
    diagnostics_topic: "/drive/diagonistics"
    cmd_topic: "/cmd_rpm"

hardware_interface:
  wheel_radius: 0.1
  wheel_base: 0.5
  gear_ratio: 30.0
  ticks_per_rev: 1024.0
  left_joint_name: "left_wheel_joint"
  right_joint_name: "right_wheel_joint"
  feedback_topic: "/drive/feedback"
  cmd_topic: "/cmd_rpm"
```

---

## ROS 2 API (Driver Node)

### Publishers
- `[feedback_topic]` (`custom_interfaces/msg/DriveFeedback`): Publishes motor RPMs and Encoder counts.
- `[diagnostics_topic]` (`custom_interfaces/msg/DriveDiagnostics`): Publishes controller temperature, battery voltage, motor current, and fault flags.

### Subscribers
- `[cmd_topic]` (`custom_interfaces/msg/WheelRpm`): Accepts left and right motor speed targets in RPM.

### Services
- `~/trigger_estop` (`std_srvs/srv/Trigger`): Triggers the SafeStop action on the Roboteq Controller.
- `~/clear_estop` (`std_srvs/srv/Trigger`): Releases the SafeStop action.
- `~/reset_faults` (`std_srvs/srv/Trigger`): Resets hardware faults (e.g. overcurrent, loop error) on the controller.

---

## Testing Instructions

1. Configure your SocketCAN interface:
   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ```
2. Launch the Standalone Driver Node:
   ```bash
   ros2 run roboteq_driver roboteq_driver_node --ros-args --params-file config/params.yaml
   ```
3. Test E-Stop:
   ```bash
   ros2 service call /roboteq_driver_node/trigger_estop std_srvs/srv/Trigger
   ```
4. Verify Telemetry:
   ```bash
   ros2 topic echo /drive/feedback
   ```
