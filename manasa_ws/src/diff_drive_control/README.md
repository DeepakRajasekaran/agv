# diff_drive_control (ROS 2 Jazzy)

A topic-based differential-drive controller node. It is **not** a
`ros2_control` hardware_interface plugin (that would require writing a
C++ `SystemInterface` plugin loaded by `controller_manager`); instead it's
a standalone `rclpy` node that does the same kinematic job using plain
topics, which matches `/drive_rpm` + `/cmd_rpm` style hardware bridges
(e.g. a microcontroller/serial bridge node publishing encoder RPM and
accepting RPM commands).

## Data flow

```
            /cmd_vel (geometry_msgs/Twist)
                    |  [from teleop_twist_keyboard, later: nav2]
                    v
        diff_drive_controller_node
          (inverse kinematics: wheel_base, wheel_dia, gear_ratio)
                    |
                    v
            /cmd_rpm (std_msgs/Float32MultiArray)  -> motor driver
            data = [left_motor_rpm, right_motor_rpm]


            /drive_rpm (std_msgs/Float32MultiArray) <- motor driver/encoders
            data = [left_motor_rpm, right_motor_rpm]
                    |
                    v
        diff_drive_controller_node
          (forward kinematics + dead-reckoning integration)
                    |
                    v
            /odom (nav_msgs/Odometry)  + odom->base_link TF
```

## Kinematics

- Inverse (cmd_vel -> wheel RPM): `v_left = v - w*L/2`, `v_right = v + w*L/2`,
  then wheel linear velocity -> wheel angular velocity (using `wheel_dia`) ->
  wheel RPM -> motor RPM (multiplied by `gear_ratio`, defined as motor
  revolutions per one wheel revolution).
- Forward (drive RPM -> odom): motor RPM / `gear_ratio` -> wheel RPM -> wheel
  angular velocity -> wheel linear velocity, then standard diff-drive
  forward kinematics integrated over time (Euler integration).

## Parameters (config/diff_drive_params.yaml)

| Parameter        | Meaning                                            | Default |
|-------------------|----------------------------------------------------|---------|
| wheel_base        | distance between wheel centers (m)                 | 0.40    |
| wheel_dia         | wheel diameter (m)                                 | 0.15    |
| gear_ratio        | motor shaft rev per 1 wheel rev                    | 30.0    |
| max_motor_rpm     | clamp on commanded RPM (0 = no clamp)              | 300.0   |
| odom_frame        | odometry frame id                                  | odom    |
| base_frame        | robot base frame id                                | base_link |
| publish_tf        | broadcast odom->base_link TF                       | true    |
| cmd_vel_timeout   | seconds of silence before zero-RPM safety stop      | 0.5     |

Change them by editing the YAML and relaunching, by passing a different
file with `params_file:=/path/to/file.yaml`, or live with:
```
ros2 param set /diff_drive_controller_node wheel_base 0.42
```

## Build & run — step by step

Inside your Jazzy Docker container:

```bash
# 1) System deps (only needed once)
sudo apt update
sudo apt install -y ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions

# 2) Workspace
mkdir -p ~/ros2_ws/src
cp -r /path/to/diff_drive_control ~/ros2_ws/src/
cd ~/ros2_ws

# 3) Build
colcon build --packages-select diff_drive_control
source install/setup.bash

# 4) Run the controller node
ros2 launch diff_drive_control diff_drive.launch.py

# 5) In a second terminal (docker exec into the same container, then source
#    ~/ros2_ws/install/setup.bash), drive it with the keyboard:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 6) (Optional, only if you have no real motor/encoder hardware yet)
#    In a third terminal, run the loopback simulator so /odom updates:
ros2 run diff_drive_control fake_drive_rpm_node
```

## Verifying it works

```bash
ros2 topic list                     # confirm /cmd_vel /cmd_rpm /drive_rpm /odom exist
ros2 topic echo /cmd_rpm            # watch RPM commands while you press keys
ros2 topic echo /odom               # watch pose update (needs /drive_rpm feedback)
ros2 param get /diff_drive_controller_node wheel_base
```

## Connecting real hardware later

Point your motor-driver bridge node at `/cmd_rpm` (subscribe,
`[left_rpm, right_rpm]`) and have it publish real encoder feedback to
`/drive_rpm` in the same `[left_rpm, right_rpm]` format. Stop
`fake_drive_rpm_node` once that's running — they must not run together.
