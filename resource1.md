# Resource 1 — MGS1600 Magnetic Sensor

**Source:** MGS1600 Magnetic Sensor Datasheet v1.4, July 14, 2023  
**Role in project:** This is the physical magnetic line sensor mounted on the AGV. In simulation, `models/sensor.py` models its output. When integrating real hardware, the driver reads from this sensor via CAN, RS232, or USB.

---

## What the sensor does

The MGS1600 measures the lateral position of a magnetic tape track along its 160mm sensing axis, with 1mm resolution. It outputs a signed position value (in mm) where 0 = tape centered under the sensor, negative = tape to the left, positive = tape to the right.

It also detects:
- **Track presence** — whether any tape is in range
- **Left and right markers** — inverted-polarity tape pieces placed beside the main track to signal special zones (junctions, speed changes, stops, etc.)
- **Fork/merge** — reports separate left-track and right-track positions when the tape splits

Update rate: **100 Hz**

---

## Position output — the PID error signal

The sensor's primary output for the PID controller is the **signed track position in millimeters**:

```
Range:  -80 mm  (tape at left edge of sensor)
Center:   0 mm  (tape centered)
Range:  +80 mm  (tape at right edge of sensor)
```

This value maps directly to the PID error input:

```python
error_mm = sensor.read_track_position()   # e.g. ?T or ?MGT via serial
error_m  = error_mm / 1000.0              # convert to meters for SI PID
```

**Sign convention (default `TINV = 0`):** left is negative, right is positive. Can be inverted with `^TINV 1`.

---

## Sensor model for simulation (`models/sensor.py`)

In simulation the sensor is replaced by a pure geometric computation. No hardware is polled.

**Simulated output:**
```python
def compute_error(robot_pose, tape_map) -> float:
    """
    Returns signed lateral distance from robot center to nearest tape segment, in meters.
    Positive = robot is right of tape (tape is to robot's left).
    Negative = robot is left of tape (tape is to robot's right).
    Clamp output to ±0.080 m (physical sensor range).
    """
```

**Simulated track detect:**
```python
def track_detected(robot_pose, tape_map) -> bool:
    """Returns True if robot is within 0.080 m of any tape segment."""
```

**Simulated marker detect:**
```python
def marker_detected(robot_pose, tape_map) -> tuple[bool, bool]:
    """Returns (left_marker, right_marker) booleans."""
```

**Simulated fork detect:**
```python
def fork_detected(robot_pose, tape_map) -> tuple[float, float]:
    """
    Returns (left_track_mm, right_track_mm).
    On a straight section, both values are equal (superimposed).
    On a fork, the values diverge — difference > fork_threshold triggers JUNCTION_DETECTED.
    """
```

**Fork detection threshold (simulation):** trigger `JUNCTION_DETECTED` when `abs(left_track - right_track) > 10` mm.

---

## Physical sensor parameters (for simulation fidelity)

| Parameter | Value |
|---|---|
| Sensing width | 160 mm total (±80 mm from center) |
| Resolution | 1 mm |
| Update rate | 100 Hz → `dt = 0.01 s` minimum |
| Optimal mount height (25mm tape) | 20–30 mm above floor |
| Optimal mount height (50mm tape) | 20–40 mm above floor |
| Max mount height (25mm tape) | 50 mm |
| Max mount height (50mm tape) | 60 mm |

In simulation: model the sensor as mounted at 25 mm height with 25 mm tape (default calibration). The `±80 mm` clamp on the error output represents the physical sensing boundary.

---

## Markers

Markers are short pieces of **inverted-polarity** tape placed 15–30 mm to the left or right of the main track. The sensor distinguishes them from the main track because their magnetic field is in the opposite direction.

**Marker use in this project:** markers encode navigation commands at junctions. When `left_marker` or `right_marker` is True, the state machine transitions `FOLLOW_LINE → JUNCTION_DETECTED → READ_TAG`.

**Simulation:** model a marker as a point location on the tape map with a side (left/right) and a tag ID. Fire the marker event when the robot center comes within `marker_radius = 0.10 m`.

**2D markers (advanced):** patterns of multiple right markers while a left marker is present can encode multi-bit information. Not required for base implementation.

---

## Fork and merge management

The sensor internally maintains two track positions: left track and right track. On a straight, both are superimposed (equal). At a fork/merge they diverge.

**Serial query to read both:** `?MGT` returns both left and right track positions.  
**Serial query for selected track only:** `?T` returns the currently selected track.

**Fork selection (hardware):**
- Fork Left pin HIGH → analog/PWM outputs report left track
- Fork Right pin HIGH → analog/PWM outputs report right track
- Both HIGH → selection via serial command (`!TX` = left, `!TV` = right)

**Simulation equivalent:** `fork_detected()` returns both track positions; the state machine issues a fork selection command to the sensor model after reading the tag.

---

## Real-time queries (serial/USB interface)

Used when integrating real hardware. **Not used in simulation.**

| Command | Description | Response format |
|---|---|---|
| `?T` | Selected track position | Signed integer, mm |
| `?MGT` | Both left and right track positions | Two signed integers, mm |
| `?MGT 1` | Left track position only | Signed integer, mm |
| `?MGT 2` | Right track position only | Signed integer, mm |
| `?MGD` | Track detect (1 = present, 0 = absent) | Integer |
| `?MGM` | Both marker states | Two integers (left, right) |
| `?MGM 1` | Left marker state | Integer |
| `?MGM 2` | Right marker state | Integer |
| `?MGS` | Full sensor status word | Bitmask |
| `?MGX` | Cross-tape detection (fw v3.0+) | Integer |
| `?MZ` | All 16 raw internal sensor values | 16 integers |

**Repeated query syntax:**
```
# 10    → repeat last query every 10 ms (100 Hz, matches sensor update rate)
# C     → clear repeat queue
```

---

## Real-time commands (serial/USB interface)

| Command | Description |
|---|---|
| `!TX` | Follow left track |
| `!TV` | Follow right track |
| `!ZER` | Set zero calibration level |
| `!R` | Run MicroBasic script |
| `!R 0` | Stop MicroBasic script |

---

## CAN interface (for real hardware driver)

The sensor publishes two TPDOs automatically:

**TPDO1** (header `0x180 + NodeID`):

| Bytes 1–2 | Bytes 3–4 | Bytes 5–6 |
|---|---|---|
| Left track position (S16, mm) | Right track position (S16, mm) | Flags byte |

**Flags byte (CANOpen layout):**

| Bit 8 | Bit 7–5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 |
|---|---|---|---|---|---|
| Sensor failure | — | Right marker | Left marker | Tape detect | Tape cross |

**TPDO2** (header `0x280 + NodeID`): VAR1 and VAR2 (user variables from scripting).

**CANOpen object dictionary — key entries for the driver:**

| Index | Sub | Name | Access |
|---|---|---|---|
| `0x210F` | 00 | Dominant track position | RO S8 |
| `0x211E` | 01 | Left track (mm) | RO S16 |
| `0x211E` | 02 | Right track (mm) | RO S16 |
| `0x211E` | 03 | Selected track (mm) | RO S16 |
| `0x211F` | 01 | Left marker | RO U8 |
| `0x211F` | 02 | Right marker | RO U8 |
| `0x211D` | 01 | Track detect | RO U8 |
| `0x2120` | 01 | Status word | RO U16 |
| `0x201A` | 00 | Follow left (`!TX`) | WO U8 |
| `0x201B` | 00 | Follow right (`!TV`) | WO U8 |

---

## Configuration parameters

Set via `^COMMAND value` (write) or `~COMMAND` (read). Save to flash with `%EESAV`.

| Command | Default | Description |
|---|---|---|
| `MMOD` | 0 | Detection mode: `0` = Absolute (supports markers), `1` = Relative (no markers) |
| `TWDT` | 0 | Tape width: `0` = 25mm, `1` = 50mm |
| `TPOL` | 0 | Tape polarity: `0` = South top (track), `1` = North top |
| `TINV` | 0 | Invert position sign: `0` = left−/right+, `1` = left+/right− |
| `TXOF` | 0 | Position offset: −100 to +100 mm |
| `TMS` | 0 | Marker sensitivity: `0` = High, `1` = Med, `2` = Low |
| `BADJ` | 0 | Left/right track reading correction: ±100 |
| `ZADJ` | 0 | Zero-level offset per internal sensor channel: ±1000 |
| `PWMM` | 0 | PWM mode: `0` = Roboteq MultiPWM, `1` = 250Hz, `2` = 500Hz |
| `FCAL` | 0 | Field calibration on startup: `0` = off, `1` = on |

**Recommended configuration for this project:**
```
^MMOD 0       # Absolute mode — needed for marker detection
^TWDT 0       # 25mm tape
^TPOL 0       # South on top (default)
^TINV 0       # Standard sign convention
^TMS 0        # High marker sensitivity
%EESAV        # Save to flash
```

---

## Fault / status flags (`?MGS` status word)

| Bit | Meaning |
|---|---|
| Bit 8 (MSB) | Sensor failure — one of the 16 internal ICs has failed |
| Bit 4 | Right marker detected |
| Bit 3 | Left marker detected |
| Bit 2 | Tape detected |
| Bit 1 | Tape cross detected (fw v3.0+) |

**In simulation:** `fault_monitor.py` should set the `SENSOR_FAILURE` fault flag if the simulated error signal is frozen (unchanged) for more than `sensor_dropout_steps` consecutive steps, mirroring what bit 8 would indicate in hardware.

---

## Serial port settings (for real hardware)

```
Baud rate:   115200 bps
Data bits:   8
Parity:      None
Stop bits:   1
Flow control: None
```

Use **RS232** (not USB) for reliable operation in electrically noisy environments. USB is for configuration and debugging only.

---

## Sensor characteristics summary

| Parameter | Min | Typical | Max | Unit |
|---|---|---|---|---|
| Sensing width | — | 160 | — | mm |
| Resolution | 1 | 1 | 2 | mm |
| Update rate | — | 100 | — | Hz |
| Operating height (25mm tape) | 10 | 30 | 50 | mm |
| Operating height (50mm tape) | 20 | 30 | 60 | mm |
| Supply voltage | 4.5 | — | 30 | V DC |
| Operating temperature | −20 | — | +85 | °C |
| Dimensions | 165 × 30 × 25 mm | | | |
