# AGV Simulation Agent — Context

## Project overview

You are an autonomous coding agent building a **PC-side simulation** of a magnetic-tape-following AGV (Automated Guided Vehicle). The goal is to develop and validate the control logic entirely in software before any real hardware (Roboteq motor controller, CAN bus, sensors) is involved.

The simulation must be **self-contained** — no CAN communication, no serial ports, no hardware dependencies. Every hardware component is replaced by a software model.

---

## Hardware being simulated (do not implement real drivers)

| Real hardware | Simulated as |
|---|---|
| Roboteq motor controller (CAN) | Kinematic plant model |
| Differential drive (left + right wheel) | `DiffDriveModel` — integrates pose from wheel speeds |
| Magnetic line sensor | `MagSensorModel` — computes lateral error from tape polyline |
| Tag reader | `TagModel` — fires event when robot crosses a tag location |
| CAN bus | Not present — direct Python function calls only |

---

## Robot model

**Drive type:** Differential drive

**State vector:** `(x, y, θ)` — 2D position and heading in world frame

**Kinematics (forward):**
```
v_left  = rpm_left  * wheel_circumference / 60
v_right = rpm_right * wheel_circumference / 60

V = (v_right + v_left) / 2          # linear velocity
ω = (v_right - v_left) / wheel_base  # angular velocity

x     += V * cos(θ) * dt
y     += V * sin(θ) * dt
θ     += ω * dt
```

**Default parameters:**
- `wheel_radius`: 0.05 m
- `wheel_base`: 0.30 m
- `dt`: 0.02 s (50 Hz simulation step)
- `max_rpm`: 150

---

## Sensor model

The magnetic tape is defined as a **polyline** (list of 2D waypoints) in world coordinates.

**Lateral error** = signed perpendicular distance from robot center to the nearest tape segment.
- Positive error → robot is to the right of the tape
- Negative error → robot is to the left of the tape
- Error = 0 → robot is perfectly centered

**Junction detection:** Certain waypoints in the tape map are tagged as junction nodes. When the robot comes within `junction_radius` (default 0.15 m) of a junction node, fire a `JUNCTION_DETECTED` event.

**Tag detection:** Tag locations are point coordinates on the tape. When the robot comes within `tag_radius` (default 0.10 m), fire a `TAG_READ` event carrying the tag ID and navigation command.

---

## PID controller

**Input:** lateral error (meters)  
**Output:** steering correction `ω_correction` (rad/s)

```
error_integral += error * dt
error_integral  = clamp(error_integral, -windup_limit, +windup_limit)

derivative = (error - error_prev) / dt

output = Kp * error + Ki * error_integral + Kd * derivative
output = clamp(output, -max_output, +max_output)

error_prev = error
```

**Tunable parameters (runtime-adjustable):**
- `Kp` — proportional gain
- `Ki` — integral gain  
- `Kd` — derivative gain
- `windup_limit` — integral anti-windup clamp (default 1.0)
- `max_output` — output saturation (default 2.0 rad/s)

---

## Differential drive velocity mapper

Converts `(V_linear, ω_steering)` to wheel RPMs:

```
v_left  = V_linear - ω * (wheel_base / 2)
v_right = V_linear + ω * (wheel_base / 2)

rpm_left  = v_left  / wheel_circumference * 60
rpm_right = v_right / wheel_circumference * 60

# Clamp to max_rpm
```

**Motion modes (all derived from same mapper):**
- Forward / reverse: `ω = 0`, vary `V`
- In-place rotation: `V = 0`, vary `ω`
- Curved motion: both `V` and `ω` non-zero

---

## Navigation state machine

```
States:
  IDLE
  INITIALIZE
  FOLLOW_LINE
  JUNCTION_DETECTED
  READ_TAG
  EXECUTE_TURN
  RESUME_TRACKING
  STOP
  ERROR

Transitions:
  IDLE            --[start cmd]-->        INITIALIZE
  INITIALIZE      --[ready]-->            FOLLOW_LINE
  FOLLOW_LINE     --[junction event]-->   JUNCTION_DETECTED
  FOLLOW_LINE     --[fault]-->            ERROR
  JUNCTION_DETECTED --[tag read ok]-->    READ_TAG
  READ_TAG        --[command decoded]-->  EXECUTE_TURN
  EXECUTE_TURN    --[turn complete]-->    RESUME_TRACKING
  RESUME_TRACKING --[line reacquired]-->  FOLLOW_LINE
  FOLLOW_LINE     --[destination reached]--> STOP
  ERROR           --[fault cleared]-->    INITIALIZE
  any             --[e-stop]-->           STOP
```

Every transition must be logged with: `timestamp | prev_state → new_state | trigger`.

---

## Tag / navigation command format

Each tag carries a navigation command used at the next junction:

| Command | Meaning |
|---|---|
| `STRAIGHT` | Continue on current tape branch |
| `TURN_LEFT` | Take left branch at junction |
| `TURN_RIGHT` | Take right branch at junction |
| `STOP` | Halt at this location |
| `GOTO <id>` | Navigate to named destination |

---

## Safety layer

Implement the following fault conditions (injectable for simulation):

| Fault | Trigger | Required behaviour |
|---|---|---|
| Sensor dropout | Error signal frozen at last value for > N steps | → `ERROR` state, stop robot |
| Motor saturation | RPM command exceeds `max_rpm` | Clamp + log warning |
| Timeout | No state update for > 500 ms | → `ERROR` state |
| Line lost | `abs(error) > lost_threshold` (default 0.25 m) for > 1 s | → `ERROR` state |
| E-stop | External flag set | Immediate → `STOP`, zero wheel commands |

Faults must generate a structured log entry: `timestamp | fault_type | state_at_fault | sensor_values`.

---

## Module structure (recommended)

```
agv_sim/
├── models/
│   ├── robot.py          # DiffDriveModel — pose integration
│   ├── sensor.py         # MagSensorModel — lateral error computation
│   └── tags.py           # TagModel — junction and tag detection
├── control/
│   ├── pid.py            # PIDController
│   └── velocity_mapper.py # (V, ω) → RPM L/R
├── navigation/
│   ├── state_machine.py  # NavigationStateMachine
│   └── tape_map.py       # Tape polyline + junction/tag metadata
├── safety/
│   └── monitor.py        # FaultMonitor — all fault checks
├── sim/
│   ├── loop.py           # Main simulation step loop
│   └── visualizer.py     # Path plot + error signal + state log
├── tests/
│   ├── test_pid.py
│   ├── test_kinematics.py
│   └── test_state_machine.py
└── context.md            # This file
```

---

## Coding conventions

- **Language:** Python 3.10+
- **Style:** Object-oriented; one class per concept
- **Dependencies:** `numpy`, `matplotlib` (visualizer only), `dataclasses`, `enum`, `logging`
- No external simulation frameworks (no ROS, no Gazebo, no PyBullet)
- All physical units in SI: meters, rad/s, seconds
- Every public method must have a docstring
- Use `logging` (not `print`) for all runtime output
- Simulation runs faster than real-time by default; `dt` and loop rate are configurable

---

## Simulation loop (pseudo-code)

```python
while state_machine.state != State.STOP:
    error   = sensor.compute_error(robot.pose, tape_map)
    ω_corr  = pid.update(error)
    V       = nominal_speed  # set by state machine
    rpm_l, rpm_r = velocity_mapper.compute(V, ω_corr)
    robot.step(rpm_l, rpm_r)
    sensor.check_tags(robot.pose, tape_map)
    fault_monitor.check(error, rpm_l, rpm_r)
    state_machine.update(sensor.events, fault_monitor.faults)
    logger.log(robot.pose, error, ω_corr, state_machine.state)
    visualizer.update(robot.pose, error)
```

---

## Performance targets

| Metric | Target |
|---|---|
| Settling time | < 2 s after disturbance |
| Steady-state lateral error | < 0.01 m |
| Max overshoot | < 0.05 m |
| Path tracking accuracy (RMS error) | < 0.02 m |

---

## What this agent must NOT do

- Do not implement CAN bus communication
- Do not import `python-can`, `canopen`, or any serial/USB library
- Do not reference Roboteq registers or CAN frame IDs
- Do not simulate 3D physics — 2D kinematics only
- Do not add ROS nodes, topics, or services
