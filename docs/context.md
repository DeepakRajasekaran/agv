# AGV Line Follower - Current Context & Progress

## 1. Current Focus
Stabilizing line-following performance, eliminating unpredictable oscillations, and refining PID parameters for high-speed curved gap crossing.

## 2. Key Technical Discoveries & Fixes
* **PID `dt` Calculation Bug (Fixed)**: Discovered that the `mgs_driver` (MGS1600 magnetic sensor) actually publishes at **50 Hz** (0.02s), not 100 Hz. The `PidController.cpp` previously hardcoded `dt = 0.01s`. This caused the Derivative ($K_d$) term to be artificially doubled in aggression and the Integral ($K_i$) term to accumulate at half speed. `dt` is now calculated dynamically in real-time using `std::chrono`.
* **Curved Gap Crossing**: Instead of returning to zero angular velocity when the track is temporarily lost (`track_detect = false`), the robot now explicitly saves and commands the last known angular arc (`m_lastPidAngularVel`) for a defined `grace_period_ms`. This prevents violent jerking when losing the line on sharp curves.
* **Zero-Crossing Estimator**: A response time estimator is now built directly into the PID loop. It logs the exact half-period (seconds) and frequency of the robot's oscillation every time the error crosses exactly over zero. This provides empirical data for pushing $K_p$ to its maximum stable limit while using $K_d$ to dampen the crossing.

## 3. Simulator Integration
* The `nav_simulator` node is now fully integrated into `bringup.launch.py`.
* Added a `force_track_detect` launch argument. When set to `true`, the simulator indefinitely publishes `True` to `/sensor/track_detect`, completely bypassing physical sensor drops for pure PID tuning in software.
* Command to launch: `ros2 launch robot_bringup bringup.launch.py launch_nav_simulator:=true force_track_detect:=true`

## 4. Parameterization & Tuning
* The nominal speed (`nominal_speed`) and all Behavior Tree scaling constraints (`error_scaling_max_dist`, `min_scale`, `error_threshold`, `fallback_scale`) have been fully parameterized and exposed to `/agv_config/follower_params.yaml`. These can now be adjusted dynamically at runtime.

## 5. Pending / Unresolved User Requests
* **Networking**: "usb tethering with my mobile is not working fix it." (Status: Not Started).

## 6. Project Architecture Rules
* **No `cat` / `ls` / `grep` editing**: Rely exclusively on Antigravity's direct file modification tools (`multi_replace_file_content`).
* **Source control**: All source code strictly resides on the Jetson; do not pull builds or src binaries to external hosts unnecessarily.
* **Environment**: Always source `/home/lucifer/anscer_workspace/agv/agv_env.bash` before compiling.
