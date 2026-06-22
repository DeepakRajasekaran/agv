# Roboteq CANopen Python Driver Documentation

This document contains the technical details, architecture decisions, and fixes discovered while building the `can_receiver.py` driver for the Roboteq motor controller via native CANopen SDOs.

## 1. Object Dictionary Index Mapping

The following Object Dictionary (OD) indices are used to read and write data to the Roboteq controller. Note that some indices require specific sub-indices to work properly.

| Data Type | OD Index | Sub-index | Description |
| :--- | :--- | :--- | :--- |
| **Battery Voltage** | `0x210D` | `02` | Load Voltage (Battery). Returns voltage with 10x scaling. Sub 1 and 3 are for internal/5V lines. |
| **Motor Amps** | `0x2100` | `01`, `02` | Motor 1 & 2 Amps with 10x scaling. |
| **Encoders** | `0x2104` | `01`, `02` | Absolute encoder counter values for Motor 1 & 2. |
| **RPM (Speed)** | `0x2109` | `01`, `02` | Actual measured motor RPM. |
| **Set Motor Cmd** | `0x2000` | `01`, `02` | Send target motor speeds (Used by watchdog loop). |
| **Reset Encoders** | `0x2003` | `01`, `02` | Writing `0` to this index executes the `!C` reset command. |
| **Closed Loop Err** | `0x2114` | `01`, `02` | Closed loop PID error (`E` command). Signed 32-bit. |
| **Controller Status** | `0x2111` | `00` | General controller status (`FS`). **Must be Sub-index 0.** |
| **Fault Flags** | `0x2113` | `00` | System fault flags (`FF`). **Must be Sub-index 0.** |
| **Motor Status** | `0x2122` | `01`, `02` | Runtime Motor Status flags (`FM`). |

### ⚠️ Sub-index Warnings
Requesting Sub-index `1` on singular variables like `0x2111` (FS) or `0x2113` (FF) will trigger an **SDO Abort (Error 0x06090011)** from the Roboteq controller, meaning "Sub-index does not exist". Always use Sub-index `0` for these.

---

## 2. Motor Status Flags (`FM`) Bitmask
The Motor Status (`0x2122`) returns a 16-bit integer. To determine the active status, evaluate the integer via bitwise masking:

*   **Bit 0 (Value 1):** `AmpLim` - Current limit is active
*   **Bit 1 (Value 2):** `Stall` - Stall detection is triggered
*   **Bit 2 (Value 4):** `Loop Error` - Closed loop error protection is triggered
*   **Bit 3 (Value 8):** `SafeStop` - Safety Stop action is active
*   **Bit 4 (Value 16):** `FwdLimit` - Forward limit switch is active
*   **Bit 5 (Value 32):** `RevLimit` - Reverse limit switch is active
*   **Bit 6 (Value 64):** `AmpTrig` - Amps trigger threshold reached

---

## 3. Data Integrity & Payload Parsing

The CANopen SDO Read Response (Command Specifier) dictates the length of the payload. Parsing all payloads as 32-bit (`<i` or `<I`) causes severe data corruption (e.g. Battery reading 429,490,226V) because the Python `struct` unpacker reads garbage padding bytes off the CAN frame.

**The Fix:**
The parser dynamically checks the `cmd` byte to unpack safely:
*   `0x4F`: 1-byte payload (`<b`, `<B`)
*   `0x4B`: 2-byte payload (`<h`, `<H`)
*   `0x43`: 4-byte payload (`<i`, `<I`)

---

## 4. The Roboteq Command Watchdog

Roboteq controllers have a built-in safety watchdog that zeroes motor output if a fresh command is not received within a timeout (default 1000ms).

**The Fix:**
Instead of firing a speed command once, the application stores `target_speed = [left, right]`. The background `_query_loop` thread continuously transmits this target via `0x2000` SDO writes at ~20Hz alongside the telemetry polling. This satisfies the watchdog and keeps the motors moving until explicitly stopped.

---

## 5. Usage & CLI Commands

Run the driver using:
```bash
python3 can_receiver.py
```
This spawns a background thread logging clean telemetry to `agv_driver.log` and presents an interactive CLI in the terminal:
*   `g <left> <right>`: Sets the target speed (e.g., `g 100 -100`)
*   `s`: Safely zeroes the `target_speed` to stop motors.
*   `c`: Resets the absolute encoders to 0 by writing to `0x2003`.
*   `q`: Safely stops the motors and exits.

---

## 6. Resources & References

*   **Roboteq Manuals & Documentation:** [Roboteq Website](https://www.roboteq.com/)
*   **Object Dictionary mappings for Roboteq:** [Generation Robots Article](https://generationrobots.com)
*   **CANopen Standard Stack Reference:** [CAN in Automation (CiA)](https://canopen-stack.org)
*   **Roboteq SDO Command Reference:** Found via manual lookup of Runtime Commands (`?FM`, `!C`, `?E`).
