#!/usr/bin/env python3
"""
==============================================================================
Roboteq FBL2360T  –  CANopen Query-Response Model for NVIDIA Jetson
Pure Python SocketCAN  (stdlib  socket  only — zero external dependencies)
==============================================================================
Controller  : Roboteq FBL2360T  (Dual-channel Brushless DC Motor Controller)
Protocol    : CANopen DS301 + DS402
Interface   : SocketCAN  (can0, can1, slcan0 …)
Bitrate     : 500 kbps   (set via:  sudo ip link set can0 type can bitrate 500000)
Node ID     : 1          (set in Roborun → Configuration → CAN → Node ID)
Python      : 3.8+   (no pip packages required)

─── Jetson one-time setup ───────────────────────────────────────────────────
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
  # USB-CAN adapter (slcan):
  sudo slcand -o -c -s6 /dev/ttyUSB0 slcan0 && sudo ip link set slcan0 up

─── Roborun checklist (mandatory before running) ────────────────────────────
  Configuration → CAN → Mode       =  "CANopen"   (not RawCAN / MiniCAN / Off)
  Configuration → CAN → Node ID    =  1
  Configuration → CAN → Bit Rate   =  500 kbps
  Configuration → CAN → Heartbeat  =  non-zero (e.g. 1000 ms)
  Configuration → CAN → Autostart  =  Yes
  Click "Save Config to Flash"  →  power-cycle controller

==============================================================================
OBJECT DICTIONARY VALIDATION AGAINST CAN NETWORKING MANUAL v2.1a
==============================================================================
All types below were validated against:
  • Roboteq CAN Networking Manual v2.1a  (ManualsLib pages 34-38)
  • Roboteq object_dict.h  (mbed.com/Hublex/Roboteq_CANOpen)
  • SDO Example 3 in the manual (V query = U16, n=2)

RUNTIME COMMANDS (WRITE) — corrected types:
  Index   Cmd  Entry Name                  Manual Type   Python fmt
  0x2000  CANGO Set Motor Command ch1/2    S32  WO       "<i"   ✓
  0x2001  P     Set Position               S32  WO       "<i"   ✓
  0x2002  S     Set Velocity               S16  WO       "<h"   ← was wrongly "<i"
  0x2003  C     Set Encoder Counter        S32  WO       "<i"   ✓
  0x2004  CB    Set Brushless Counter      S32  WO       "<i"   ✓
  0x2005  VAR   Set User Integer Variable  S32  WO       "<i"   ✓
  0x2006  AC    Set Acceleration           S32  WO       "<i"   ✓
  0x2007  DC    Set Deceleration           S32  WO       "<i"   ✓
  0x2008  DS    Set All Digital Out bits   U8   WO       "<B"   ✓
  0x2009  D1    Set Individual Dout bit    U8   WO       "<B"   ✓
  0x200A  D0    Reset Individual Dout bit  U8   WO       "<B"   ✓
  0x200B  H     Load Home Counter          U8   WO       "<B"   (added)
  0x200C  EX    Emergency Shutdown         U8   WO       "<B"   ✓
  0x200D  MG    Release Shutdown           U8   WO       "<B"   ✓
  0x200E  MS    Stop in all modes          U8   WO       "<B"   ✓

RUNTIME QUERIES (READ) — corrected indices and types:
  Index   Query Entry Name                         Manual Type  Python fmt  Change?
  0x2100  A     Read Motor Amps ch1/2              S16  RO      "<h"        ✓
  0x2101  M     Read Actual Motor Command          S16  RO      "<h"        ✓
  0x2102  P     Read Applied Power Level           S16  RO      "<h"        ✓
  0x2103  S     Read Encoder Motor Speed           S16  RO      "<h"  ← was "<i" (FIXED)
  0x2104  C     Read Absolute Encoder Count        S32  RO      "<i"  ← was wrongly encoder_abs (FIXED: swap)
  0x2105  CB    Read Absolute Brushless Counter    S32  RO      "<i"  ← index corrected
  0x2106  VAR   Read User Integer Variable         S32  RO      "<i"  ← was encoder_error (FIXED)
  0x2107  SR    Read Encoder Speed as 1/1000 Max  S16  RO      "<h"  ← was battery_amps (FIXED)
  0x2108  CR    Read Encoder Count Relative        S32  RO      "<i"  ← was wrong (FIXED)
  0x2109  CBR   Read Brushless Count Relative      S32  RO      "<i"  ← added
  0x210A  BS    Read BL Motor Speed RPM            S16  RO      "<h"  ← added
  0x210B  BSR   Read BL Motor Speed as 1/1000 Max S16  RO      "<h"  ← added
  0x210C  BA    Read Battery Amps ch1/2            S16  RO      "<h"  ← was position_fb (FIXED)
  0x210D  V     Read VInt/VBat/5VOut               U16  RO      "<H"  ← was "<h" (FIXED: unsigned)
  0x210E  D     Read All Digital Inputs            U32  RO      "<I"  ← was temp (FIXED completely)
  0x210F  T     Read MCU/Transistor Temperature    S8   RO      "<b"  ← was feedback_input (FIXED)
  0x2110  F     Read Feedback                      S16  RO      "<h"  ← added
  0x2111  FS/AI Read Status Flags (sub0x00 U8) + Analog Inputs (sub01-04 S16)
  0x2112  FF    Read Fault Flags                   U8   RO      "<B"  ← was digital_inputs (FIXED)
  0x2113  DO    Read Current Digital Outputs       U8   RO      "<B"  ← was digital_outputs (FIXED)
  0x2114  E     Read Closed Loop Error             S32  RO      "<i"  ← added
  0x2122  FM    Read Motor Status Flags            U16  RO      "<H"  ← was brushless_count (FIXED)
  0x2123  HS    Read Hall Sensor States            U8   RO      "<B"  ← was slip_freq (FIXED)
  0x2132  ANG   Read Rotor Angle                   S16  RO      "<h"  ← added
  0x2136  SL    Read Slip                          S16  RO      "<h"  ← added
  0x2111  AI    Read Analog Input ch1-4            S16  RO      "<h"  ← HARDWARE CONFIRMED on FBL2360T
  NOTE: 0x6400/6401/6402/6403 do NOT exist on FBL2360T (abort 0x06020000)
        Analog inputs are at 0x2111 sub 01-04 on this controller.

DS402 OBJECTS (unchanged, already correct):
  0x6040  CH1  Control Word     U16 RW
  0x6041  CH1  Status Word      U16 RO
  0x6060  CH1  Modes of Op      S8  RW
  0x6061  CH1  Modes of Op Disp S8  RO
  0x6064  CH1  Position Actual  S32 RO
  0x606C  CH1  Velocity Actual  S32 RO
  0x6077  CH1  Torque Actual    S16 RO
  CH2 = CH1 base + 0x800

KEY BUGS FIXED vs PREVIOUS VERSION:
  1. 0x2002 Set Velocity:  was "<i" (S32) → corrected to "<h" (S16)
  2. 0x2103: was encoder_abs (S32/counts) → corrected to encoder_speed S16 RPM
  3. 0x2104: was encoder_rel (S32)        → corrected to encoder_abs S32 counts
  4. 0x210C: was position_fb (S32)        → corrected to battery_amps S16
  5. 0x210D: was "<h" (S16 signed)        → corrected to "<H" (U16 unsigned)
  6. 0x210E: was temperature S16         → corrected to digital inputs U32
  7. 0x210F: was feedback_input S16      → corrected to temperature S8
  8. 0x2111: sub 0x00 = Status Flags U8 (qFS), sub 0x01-04 = Analog Inputs S16 (qAI)
     Both are valid — different subindices of the same OD index.
  9. 0x2112: was digital_inputs U8       → corrected to fault_flags U8
  10. 0x2113: was digital_outputs U8     → corrected to digital_outputs_state U8
  11. 0x2116/0x2118: were brushless/slip → corrected to motor_status/hall U16/U8
  Analog inputs confirmed at 0x2111 (qAI); 0x6401 does NOT exist on FBL2360T
  Digital inputs confirmed at 0x210E (qD) as U32 sub=0x00
==============================================================================
"""

import socket
import struct
import threading
import time
import logging
import csv
import os
import json
import select
import sys
from datetime import datetime
from typing import Optional, Dict, Any, Tuple, List


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
CAN_INTERFACE       = "can0"
NODE_ID             = 1
BITRATE             = 500_000      # informational; set via ip link before running

SDO_TIMEOUT_S       = 0.5          # seconds to wait for SDO response
POLL_INTERVAL_S     = 0.05         # 50 ms → ~20 Hz full poll round
HEARTBEAT_TIMEOUT_S = 2.0

LOG_DIR             = "./roboteq_logs"
LOG_TO_CSV          = True
LOG_TO_JSON         = True
LOG_VERBOSE         = False        # set True to print every parsed value
STREAM_TO_LOG       = True         # set True to log continuous [STREAM] line to driver.logs
CLI_STREAM_INTERVAL_S = 0.100        # 150 ms stream log interval


# ─────────────────────────────────────────────────────────────────────────────
#  RAW SOCKETCAN CONSTANTS  (linux/can.h)
# ─────────────────────────────────────────────────────────────────────────────
AF_CAN          = 29
CAN_RAW         = 1
SOL_CAN_BASE    = 100
SOL_CAN_RAW     = SOL_CAN_BASE + CAN_RAW
CAN_RAW_FILTER  = 1
CAN_SFF_MASK    = 0x000007FF       # 11-bit standard frame ID mask
CAN_FRAME_FMT   = "<IB3x8s"       # struct can_frame: id(4) dlc(1) pad(3) data(8)
CAN_FRAME_SIZE  = struct.calcsize(CAN_FRAME_FMT)   # always 16
CAN_FILTER_FMT  = "<II"           # struct can_filter: id(4) mask(4)


# ─────────────────────────────────────────────────────────────────────────────
#  CANopen COB-IDs  (derived from NODE_ID)
# ─────────────────────────────────────────────────────────────────────────────
NMT_COB       = 0x000
SDO_TX_COB    = 0x600 + NODE_ID    # Jetson → controller (0x601)
SDO_RX_COB    = 0x580 + NODE_ID    # Controller → Jetson (0x581)
TPDO1_COB     = 0x180 + NODE_ID
TPDO2_COB     = 0x280 + NODE_ID
TPDO3_COB     = 0x380 + NODE_ID
TPDO4_COB     = 0x480 + NODE_ID
RPDO1_COB     = 0x200 + NODE_ID
RPDO2_COB     = 0x300 + NODE_ID
HEARTBEAT_COB = 0x700 + NODE_ID
EMCY_COB      = 0x080 + NODE_ID

LISTEN_COBS = {
    SDO_RX_COB, TPDO1_COB, TPDO2_COB, TPDO3_COB, TPDO4_COB,
    HEARTBEAT_COB, EMCY_COB,
}

# NMT command bytes
NMT_OPERATIONAL = 0x01
NMT_STOPPED     = 0x02
NMT_PRE_OP      = 0x80
NMT_RESET_NODE  = 0x81
NMT_RESET_COMM  = 0x82

# DS402 op-modes
DS402_MODE_PP = 1   # Profile Position
DS402_MODE_VL = 2   # Velocity (open-loop)
DS402_MODE_PV = 3   # Profile Velocity
DS402_MODE_TQ = 4   # Torque
DS402_MODE_HM = 6   # Homing


# ─────────────────────────────────────────────────────────────────────────────
#  OBJECT DICTIONARY — QUERY TABLE (READ via SDO)
#
#  Validated against:
#    Roboteq CAN Networking Manual v2.1a  pages 34-36
#    Roboteq object_dict.h  (mbed.com/Hublex/Roboteq_CANOpen)
#
#  Row format:
#    (index, subindex, key, struct_fmt, scale, unit, cmd_name, description)
#
#  struct_fmt : Python struct code for the 4-byte SDO data field (LE)
#  scale      : multiply raw integer to get engineering units
# ─────────────────────────────────────────────────────────────────────────────
QUERY_TABLE: List[Tuple] = [

    # ── 0x2100  qA  Read Motor Amps  S16 ──────────────────────────────────────
    (0x2100, 0x01, "motor_amps_ch1",          "<h", 0.1,  "A",       "qA",  "Motor Amps Ch1"),
    (0x2100, 0x02, "motor_amps_ch2",          "<h", 0.1,  "A",       "qA",  "Motor Amps Ch2"),

    # ── 0x2101  qM  Read Actual Motor Command  S16 ────────────────────────────
    (0x2101, 0x01, "motor_cmd_ch1",           "<h", 1.0,  "‰",       "qM",  "Actual Motor Cmd Ch1"),
    (0x2101, 0x02, "motor_cmd_ch2",           "<h", 1.0,  "‰",       "qM",  "Actual Motor Cmd Ch2"),

    # ── 0x2102  qP  Read Applied Power Level  S16 ─────────────────────────────
    (0x2102, 0x01, "power_level_ch1",         "<h", 1.0,  "‰",       "qP",  "Applied Power Level Ch1"),
    (0x2102, 0x02, "power_level_ch2",         "<h", 1.0,  "‰",       "qP",  "Applied Power Level Ch2"),

    # ── 0x2103  qS  Read Encoder Motor Speed in RPM  S16 ─────────────────────
    # Manual: "Read Encoder Motor Speed"  Type = S16 RO
    (0x2103, 0x01, "encoder_speed_ch1",       "<h", 1.0,  "RPM",     "qS",  "Encoder Speed Ch1 (RPM)"),
    (0x2103, 0x02, "encoder_speed_ch2",       "<h", 1.0,  "RPM",     "qS",  "Encoder Speed Ch2 (RPM)"),

    # ── 0x2104  qC  Read Absolute Encoder Count  S32 ─────────────────────────
    # Manual: "Read Absolute Encoder Count"  Type = S32 RO
    (0x2104, 0x01, "encoder_abs_ch1",         "<i", 1.0,  "counts",  "qC",  "Encoder Abs Count Ch1"),
    (0x2104, 0x02, "encoder_abs_ch2",         "<i", 1.0,  "counts",  "qC",  "Encoder Abs Count Ch2"),

    # ── 0x2105  qCB  Read Absolute Brushless Counter  S32 ────────────────────
    (0x2105, 0x01, "brushless_abs_ch1",       "<i", 1.0,  "counts",  "qCB", "Brushless Abs Counter Ch1"),
    (0x2105, 0x02, "brushless_abs_ch2",       "<i", 1.0,  "counts",  "qCB", "Brushless Abs Counter Ch2"),

    # ── 0x2106  qVAR  Read User Integer Variable  S32 ────────────────────────
    # (sub 1-32; we read VAR1 and VAR2 as examples — add more if needed)
    (0x2106, 0x01, "user_var1",               "<i", 1.0,  "",        "qVAR","User Integer Variable 1"),
    (0x2106, 0x02, "user_var2",               "<i", 1.0,  "",        "qVAR","User Integer Variable 2"),

    # ── 0x2107  qSR  Read Encoder Speed as 1/1000 of Max  S16 ───────────────
    (0x2107, 0x01, "encoder_speed_rel_ch1",   "<h", 1.0,  "‰",       "qSR", "Encoder Speed 1/1000 Max Ch1"),
    (0x2107, 0x02, "encoder_speed_rel_ch2",   "<h", 1.0,  "‰",       "qSR", "Encoder Speed 1/1000 Max Ch2"),

    # ── 0x2108  qCR  Read Encoder Count Relative  S32 ────────────────────────
    (0x2108, 0x01, "encoder_rel_ch1",         "<i", 1.0,  "counts",  "qCR", "Encoder Count Relative Ch1"),
    (0x2108, 0x02, "encoder_rel_ch2",         "<i", 1.0,  "counts",  "qCR", "Encoder Count Relative Ch2"),

    # ── 0x2109  qCBR  Read Brushless Count Relative  S32 ────────────────────
    (0x2109, 0x01, "brushless_rel_ch1",       "<i", 1.0,  "counts",  "qCBR","Brushless Count Relative Ch1"),
    (0x2109, 0x02, "brushless_rel_ch2",       "<i", 1.0,  "counts",  "qCBR","Brushless Count Relative Ch2"),

    # ── 0x210A  qBS  Read BL Motor Speed in RPM  S16 ─────────────────────────
    (0x210A, 0x01, "bl_speed_rpm_ch1",        "<h", 1.0,  "RPM",     "qBS", "BL Motor Speed RPM Ch1"),
    (0x210A, 0x02, "bl_speed_rpm_ch2",        "<h", 1.0,  "RPM",     "qBS", "BL Motor Speed RPM Ch2"),

    # ── 0x210B  qBSR  Read BL Motor Speed as 1/1000 of Max  S16 ─────────────
    (0x210B, 0x01, "bl_speed_rel_ch1",        "<h", 1.0,  "‰",       "qBSR","BL Motor Speed 1/1000 Max Ch1"),
    (0x210B, 0x02, "bl_speed_rel_ch2",        "<h", 1.0,  "‰",       "qBSR","BL Motor Speed 1/1000 Max Ch2"),

    # ── 0x210C  qBA  Read Battery Amps  S16 ─────────────────────────────────
    # Manual: "Read Battery Amps"  Type = S16 RO
    (0x210C, 0x01, "battery_amps_ch1",        "<h", 0.1,  "A",       "qBA", "Battery Amps Ch1"),
    (0x210C, 0x02, "battery_amps_ch2",        "<h", 0.1,  "A",       "qBA", "Battery Amps Ch2"),

    # ── 0x210D  qV  Read VInt, VBat, 5VOut  U16 ─────────────────────────────
    # Manual: "Read VInt, VBat, 5VOut"  Type = U16 RO
    # Sub 01 = Internal voltage, 02 = Battery voltage, 03 = 5V output
    # SDO Example 3 in manual: n=2 bytes, U16, index=0x210D sub=0x02
    (0x210D, 0x01, "internal_volts",          "<H", 0.1,  "V",       "qV",  "Internal Logic Voltage"),
    (0x210D, 0x02, "battery_volts",           "<H", 0.1,  "V",       "qV",  "Battery Voltage"),
    (0x210D, 0x03, "supply_5v",               "<H", 0.1,  "V",       "qV",  "5V Supply Output"),

    # ── 0x210E  qD  Read All Digital Inputs  U32 ────────────────────────────
    # Manual: "Read All Digital Inputs"  Type = U32 RO  sub=0x00
    (0x210E, 0x00, "digital_inputs_all",      "<I", 1.0,  "bitfield","qD",  "All Digital Inputs"),

    # ── 0x210F  qT  Read MCU and Transistor Temperature  S8 ─────────────────
    # Manual: "Read MCU and Transistor temperature"  Type = S8 RO
    # Sub 01 = ch1 heatsink, 02 = ch2 heatsink, 03 = MCU (product-specific)
    (0x210F, 0x01, "temp_heatsink_ch1",       "<b", 1.0,  "°C",      "qT",  "Heatsink Temperature Ch1"),
    (0x210F, 0x02, "temp_heatsink_ch2",       "<b", 1.0,  "°C",      "qT",  "Heatsink Temperature Ch2"),
    (0x210F, 0x03, "temp_mcu",               "<b", 1.0,  "°C",      "qT",  "MCU Temperature"),

    # ── 0x2110  qF  Read Feedback  S16 ─────────────────────────────────────
    (0x2110, 0x01, "feedback_input_ch1",      "<h", 1.0,  "raw",     "qF",  "Feedback Input Ch1"),
    (0x2110, 0x02, "feedback_input_ch2",      "<h", 1.0,  "raw",     "qF",  "Feedback Input Ch2"),

    # ── 0x2111  qFS  Read Status Flags  U8 ─────────────────────────────────
    # Manual: "Read Status Flags"  Type = U8 RO  sub=0x00
    (0x2111, 0x00, "status_flags",            "<B", 1.0,  "bitfield","qFS", "Status Flags"),

    # ── 0x2112  qFF  Read Fault Flags  U8 ──────────────────────────────────
    # Manual: "Read Fault Flags"  Type = U8 RO  sub=0x00
    (0x2112, 0x00, "fault_flags",             "<B", 1.0,  "bitfield","qFF", "Fault Flags"),

    # ── 0x2113  qDO  Read Current Digital Outputs  U8 ───────────────────────
    # Manual: "Read Current Digital Outputs"  Type = U8 RO  sub=0x00
    (0x2113, 0x00, "digital_outputs",         "<B", 1.0,  "bitfield","qDO", "Current Digital Outputs"),

    # ── 0x2114  qE  Read Closed Loop Error  S32 ─────────────────────────────
    (0x2114, 0x01, "cl_error_ch1",            "<i", 1.0,  "counts",  "qE",  "Closed Loop Error Ch1"),
    (0x2114, 0x02, "cl_error_ch2",            "<i", 1.0,  "counts",  "qE",  "Closed Loop Error Ch2"),

    # ── 0x2122  qFM  Read Motor Status Flags  U16 ───────────────────────────
    (0x2122, 0x01, "motor_status_ch1",        "<H", 1.0,  "bitfield","qFM", "Motor Status Flags Ch1"),
    (0x2122, 0x02, "motor_status_ch2",        "<H", 1.0,  "bitfield","qFM", "Motor Status Flags Ch2"),

    # ── 0x2123  qHS  Read Hall Sensor States  U8 ────────────────────────────
    (0x2123, 0x01, "hall_sensor_ch1",         "<B", 1.0,  "bitfield","qHS", "Hall Sensor States Ch1"),
    (0x2123, 0x02, "hall_sensor_ch2",         "<B", 1.0,  "bitfield","qHS", "Hall Sensor States Ch2"),

    # ── 0x2132  qANG  Read Rotor Angle  S16 ─────────────────────────────────
    (0x2132, 0x01, "rotor_angle_ch1",         "<h", 1.0,  "°",       "qANG","Rotor Angle Ch1"),
    (0x2132, 0x02, "rotor_angle_ch2",         "<h", 1.0,  "°",       "qANG","Rotor Angle Ch2"),

    # ── 0x2136  qSL  Read Slip  S16 ─────────────────────────────────────────
    (0x2136, 0x01, "slip_ch1",                "<h", 1.0,  "",        "qSL", "Slip Ch1"),
    (0x2136, 0x02, "slip_ch2",                "<h", 1.0,  "",        "qSL", "Slip Ch2"),

    # ── 0x2111  qAI  Read Analog Input  S16 ─────────────────────────────────
    # HARDWARE CONFIRMED: FBL2360T uses 0x2111 sub 01-04 for analog inputs.
    # The generic CANopen manual's 0x6401 (qAI) does NOT exist on this device
    # (abort code 0x06020000 returned). Use 0x2111 instead.
    #(0x2111, 0x01, "analog_in_ch1",           "<h", 0.1,  "V",       "qAI", "Analog Input Ch1"),
    #(0x2111, 0x02, "analog_in_ch2",           "<h", 0.1,  "V",       "qAI", "Analog Input Ch2"),
    #(0x2111, 0x03, "analog_in_ch3",           "<h", 0.1,  "V",       "qAI", "Analog Input Ch3"),
    #(0x2111, 0x04, "analog_in_ch4",           "<h", 0.1,  "V",       "qAI", "Analog Input Ch4"),

    # ── DS402  Channel 1  (base indices 0x60xx) ───────────────────────────────
    (0x6041, 0x00, "ds402_status_ch1",        "<H", 1.0,  "bitfield","SW",  "DS402 Status Word Ch1"),
    (0x6061, 0x00, "ds402_opmode_ch1",        "<b", 1.0,  "",        "AOM", "DS402 Op Mode Display Ch1"),
    (0x6064, 0x00, "ds402_pos_actual_ch1",    "<i", 1.0,  "counts",  "F",   "DS402 Position Actual Ch1"),
    (0x606C, 0x00, "ds402_vel_actual_ch1",    "<i", 1.0,  "RPM",     "F",   "DS402 Velocity Actual Ch1"),
    (0x6077, 0x00, "ds402_torque_actual_ch1", "<h", 0.1,  "%",       "TRQ", "DS402 Torque Actual Ch1"),

    # ── DS402  Channel 2  (base + 0x800 = 0x68xx) ────────────────────────────
    (0x6841, 0x00, "ds402_status_ch2",        "<H", 1.0,  "bitfield","SW",  "DS402 Status Word Ch2"),
    (0x6861, 0x00, "ds402_opmode_ch2",        "<b", 1.0,  "",        "AOM", "DS402 Op Mode Display Ch2"),
    (0x6864, 0x00, "ds402_pos_actual_ch2",    "<i", 1.0,  "counts",  "F",   "DS402 Position Actual Ch2"),
    (0x686C, 0x00, "ds402_vel_actual_ch2",    "<i", 1.0,  "RPM",     "F",   "DS402 Velocity Actual Ch2"),
    (0x6877, 0x00, "ds402_torque_actual_ch2", "<h", 0.1,  "%",       "TRQ", "DS402 Torque Actual Ch2"),
]


# ─────────────────────────────────────────────────────────────────────────────
#  BIT-FIELD DECODE TABLES
# ─────────────────────────────────────────────────────────────────────────────
# Fault Flags (0x2112 / qFF) — U8, bits 0-7
FAULT_BITS: Dict[int, str] = {
    0: "Overheat",
    1: "Overvoltage",
    2: "Undervoltage",
    3: "Short Circuit",
    4: "Emergency Stop",
    5: "Brushless Sensor Fault",
    6: "MOSFET Failure",
    7: "Default Config Loaded",
}

# Status Flags (0x2111 / qFS) — U8, bits 0-7
STATUS_BITS: Dict[int, str] = {
    0: "Serial Mode Active",
    1: "Pulse Mode Active",
    2: "Analog Mode Active",
    3: "Power Stage Off",
    4: "Stall Ch1",
    5: "Stall Ch2",
    6: "At Speed Ch1",
    7: "At Speed Ch2",
}

# Motor Status Flags (0x2122 / qFM) — U16
MOTOR_STATUS_BITS: Dict[int, str] = {
    0:  "Amps Limit Active",
    1:  "Motor Stall Detected",
    2:  "Loop Error Detected",
    3:  "Safety Stop Active",
    4:  "Forward Limit Triggered",
    5:  "Reverse Limit Triggered",
    6:  "Amps Trigger Activated",
}

# DS402 Status Word bits
DS402_STATUS_BITS: Dict[int, str] = {
    0:  "Ready to Switch On",
    1:  "Switched On",
    2:  "Operation Enabled",
    3:  "Fault",
    4:  "Voltage Enabled",
    5:  "Quick Stop",
    6:  "Switch On Disabled",
    7:  "Warning",
    9:  "Remote",
    10: "Target Reached",
    11: "Internal Limit Active",
}

DS402_OP_MODES: Dict[int, str] = {
    -1: "No Mode",
     0: "No Change",
     1: "Profile Position",
     2: "Velocity (VL)",
     3: "Profile Velocity (PV)",
     4: "Profile Torque (TQ)",
     6: "Homing",
}


def _decode_bits(value: int, table: Dict[int, str]) -> List[str]:
    """Return list of active flag names for a bit-field integer."""
    return [table[b] for b in sorted(table) if (value >> b) & 1]


# ─────────────────────────────────────────────────────────────────────────────
#  INTERACTIVE CLI LOGGING UTILS & CUSTOM LOGGING
# ─────────────────────────────────────────────────────────────────────────────
console_lock = threading.Lock()
interactive_active = False
prompt_str = "roboteq> "
current_input_buffer = ""

class CLIStreamHandler(logging.StreamHandler):
    def emit(self, record):
        try:
            msg = self.format(record)
            if "[CMD_SEND]" in msg:
                # Do not write 50ms command sends to the CLI console
                return
            with console_lock:
                if interactive_active:
                    sys.stdout.write('\r\x1b[K' + msg + '\n' + prompt_str + current_input_buffer)
                else:
                    sys.stdout.write(msg + '\n')
                sys.stdout.flush()
        except Exception:
            self.handleError(record)

def read_line_raw(prompt: str = "roboteq> ") -> str:
    global current_input_buffer, prompt_str, interactive_active
    
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        with console_lock:
            prompt_str = prompt
            current_input_buffer = ""
            interactive_active = True
        try:
            line = input(prompt)
        finally:
            with console_lock:
                interactive_active = False
        return line

    import termios
    import tty

    with console_lock:
        prompt_str = prompt
        current_input_buffer = ""
        interactive_active = True
        sys.stdout.write(prompt_str)
        sys.stdout.flush()

    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            char = sys.stdin.read(1)
            if not char:
                raise EOFError()
            
            # Ctrl-C
            if char == '\x03':
                raise KeyboardInterrupt()
            # Ctrl-D
            if char == '\x04':
                raise EOFError()
            # Enter / Carriage Return
            if char in ('\r', '\n'):
                with console_lock:
                    sys.stdout.write('\r\n')
                    sys.stdout.flush()
                    line = current_input_buffer
                    current_input_buffer = ""
                    interactive_active = False
                return line
            # Backspace / Delete
            elif char in ('\x7f', '\x08'):
                with console_lock:
                    if len(current_input_buffer) > 0:
                        current_input_buffer = current_input_buffer[:-1]
                        sys.stdout.write('\r\x1b[K' + prompt_str + current_input_buffer)
                        sys.stdout.flush()
            # ANSI escape sequence (e.g. arrow keys)
            elif char == '\x1b':
                # Check if more characters are available (up to 50ms wait)
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    char2 = sys.stdin.read(1)
                    if char2 == '[':
                        ready2, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if ready2:
                            char3 = sys.stdin.read(1)
                            # Ignore arrow key escape characters
                            continue
                continue
            # Esc or other control chars
            elif ord(char) < 32:
                continue
            else:
                with console_lock:
                    current_input_buffer += char
                    sys.stdout.write(char)
                    sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        with console_lock:
            interactive_active = False

os.makedirs(LOG_DIR, exist_ok=True)
_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

cli_handler = CLIStreamHandler()
cli_handler.setLevel(logging.INFO)

file_handler = logging.FileHandler("driver.logs")
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.DEBUG if LOG_VERBOSE else logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)-5s] %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        cli_handler,
        file_handler,
    ],
)
log = logging.getLogger("roboteq")


# ─────────────────────────────────────────────────────────────────────────────
#  LOW-LEVEL SOCKETCAN PACK / UNPACK
# ─────────────────────────────────────────────────────────────────────────────

def pack_can_frame(can_id: int, data: bytes) -> bytes:
    """Pack a CAN frame into 16-byte struct can_frame."""
    dlc = len(data)
    if dlc > 8:
        raise ValueError(f"CAN data too long: {dlc}")
    return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, dlc,
                       data.ljust(8, b'\x00'))


def unpack_can_frame(raw: bytes) -> Tuple[int, int, bytes]:
    """Unpack 16 raw bytes → (can_id, dlc, payload[:dlc])."""
    if len(raw) < CAN_FRAME_SIZE:
        raise ValueError(f"Short frame: {len(raw)}")
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    return can_id & CAN_SFF_MASK, dlc, data[:dlc]


# ─────────────────────────────────────────────────────────────────────────────
#  SDO FRAME BUILDERS
# ─────────────────────────────────────────────────────────────────────────────

def build_sdo_read(index: int, subindex: int) -> bytes:
    """
    Build 8-byte SDO upload-initiate (read) request.
    Byte 0 = 0x40  (cs = 4 = query)
    Bytes 1-2 = index LE
    Byte 3 = subindex
    Bytes 4-7 = 0x00
    """
    payload = struct.pack("<BHB4x", 0x40, index, subindex)
    log.debug(f"  SDO READ  → idx=0x{index:04X} sub=0x{subindex:02X}  {payload.hex()}")
    return payload


# SDO download command byte:  0x20 | (n<<2) | 0x03  where n = 4 - data_size
_SDO_WRITE_CMD = {4: 0x23, 2: 0x2B, 1: 0x2F}

# fmt → byte size
_FMT_SIZE = {"<b": 1, "<B": 1, "<h": 2, "<H": 2, "<i": 4, "<I": 4}


def build_sdo_write(index: int, subindex: int, value: int, fmt: str) -> bytes:
    """
    Build 8-byte SDO download-initiate (write) request.
    fmt: '<b','<B','<h','<H','<i','<I'
    """
    size = _FMT_SIZE[fmt]
    cmd  = _SDO_WRITE_CMD[size]
    data_bytes = struct.pack(fmt, value).ljust(4, b'\x00')
    return struct.pack("<BHB", cmd, index, subindex) + data_bytes


# ─────────────────────────────────────────────────────────────────────────────
#  SDO RESPONSE PARSER
# ─────────────────────────────────────────────────────────────────────────────

# Abort code descriptions
SDO_ABORT_CODES: Dict[int, str] = {
    0x05030000: "Toggle bit not alternated",
    0x05040001: "SDO protocol timed out",
    0x06010000: "Unsupported access",
    0x06010001: "Attempt to read write-only object",
    0x06010002: "Attempt to write read-only object",
    0x06020000: "Object does not exist in OD",
    0x06040041: "Object cannot be mapped to PDO",
    0x06070010: "Data type / length mismatch",  # ← wrong fmt triggers this
    0x06090011: "Subindex does not exist",
    0x06090030: "Value range exceeded",
    0x08000000: "General error",
    0x08000020: "Data cannot be transferred or stored",
}


def parse_sdo_response(payload: bytes) -> Tuple[bool, bool, int, int, bytes]:
    """
    Parse 8-byte SDO response payload.
    Returns: (ok, is_write_ack, echo_index, echo_sub, data_bytes[4])
    """
    if len(payload) < 8:
        return False, False, 0, 0, b'\x00' * 4

    cs         = payload[0]
    echo_index = struct.unpack_from("<H", payload, 1)[0]
    echo_sub   = payload[3]
    data_bytes = payload[4:8]

    if cs == 0x80:
        abort_code = struct.unpack_from("<I", data_bytes)[0]
        desc = SDO_ABORT_CODES.get(abort_code, "Unknown")
        log.warning(f"  SDO ABORT  idx=0x{echo_index:04X} sub=0x{echo_sub:02X} "
                    f"code=0x{abort_code:08X}  ({desc})")
        return False, False, echo_index, echo_sub, data_bytes

    if cs == 0x60:   # write acknowledged
        return True, True, echo_index, echo_sub, data_bytes

    # Read response: cs ∈ {0x43, 0x4B, 0x4F, …}
    return True, False, echo_index, echo_sub, data_bytes


def interpret_data(data_bytes: bytes, fmt: str) -> int:
    """
    Interpret 4-byte SDO data field with the correct struct format.
    Handles signed/unsigned 1, 2, 4-byte values.
    """
    size = _FMT_SIZE[fmt]
    return struct.unpack(fmt, data_bytes[:size])[0]


# ─────────────────────────────────────────────────────────────────────────────
#  PDO PARSERS
# ─────────────────────────────────────────────────────────────────────────────

def parse_tpdo(cob_id: int, data: bytes) -> Dict[str, Any]:
    """
    Parse Roboteq TPDO frames (default MiniCAN mapping).
    TPDO1/2/3: two S32 user variables per frame.
    TPDO4: 32 boolean variables packed into a U32 bitfield.
    Adjust if you have remapped PDOs in Roborun.
    """
    result: Dict[str, Any] = {}
    n = len(data)

    if cob_id == TPDO1_COB:
        if n >= 4: result["tpdo1_var1"] = struct.unpack_from("<i", data, 0)[0]
        if n >= 8: result["tpdo1_var2"] = struct.unpack_from("<i", data, 4)[0]
    elif cob_id == TPDO2_COB:
        if n >= 4: result["tpdo2_var3"] = struct.unpack_from("<i", data, 0)[0]
        if n >= 8: result["tpdo2_var4"] = struct.unpack_from("<i", data, 4)[0]
    elif cob_id == TPDO3_COB:
        if n >= 4: result["tpdo3_var5"] = struct.unpack_from("<i", data, 0)[0]
        if n >= 8: result["tpdo3_var6"] = struct.unpack_from("<i", data, 4)[0]
    elif cob_id == TPDO4_COB:
        if n >= 4:
            bv = struct.unpack_from("<I", data, 0)[0]
            for bit in range(32):
                result[f"tpdo4_bvar{bit + 1}"] = bool((bv >> bit) & 1)
    return result


def parse_emcy(data: bytes) -> Dict[str, Any]:
    """
    Parse 8-byte CANopen EMCY frame (DS301):
      Bytes 0-1 : Error Code
      Byte  2   : Error Register
      Bytes 3-7 : Manufacturer-specific (Roboteq fault flags low byte in byte 3)
    """
    result: Dict[str, Any] = {}
    if len(data) < 8:
        return result
    error_code     = struct.unpack_from("<H", data, 0)[0]
    error_register = data[2]
    mfr_bytes      = data[3:8]
    result["emcy_error_code"]     = f"0x{error_code:04X}"
    result["emcy_error_register"] = f"0x{error_register:02X}"
    result["emcy_mfr_raw"]        = mfr_bytes.hex()
    fault_low = data[3]
    result["emcy_active_faults"]  = _decode_bits(fault_low, FAULT_BITS)
    return result


def parse_heartbeat(data: bytes) -> str:
    if not data:
        return "Unknown"
    return {0x00: "Boot-up", 0x04: "Stopped",
            0x05: "Operational", 0x7F: "Pre-operational"
            }.get(data[0], f"0x{data[0]:02X}")


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN CONTROLLER CLASS
# ─────────────────────────────────────────────────────────────────────────────

class RoboteqCANopenNode:
    """
    Full CANopen query-response model for the Roboteq FBL2360T.
    Uses stdlib socket (AF_CAN / SOCK_RAW / CAN_RAW) — zero pip dependencies.

    Thread model:
      _rx_loop   — daemon, runs continuously, dispatches all incoming frames
      _poll_loop — daemon, cycles through QUERY_TABLE via SDO, logs results

    Usage:
        node = RoboteqCANopenNode(interface="can0", node_id=1)
        node.start()
        snap = node.get_snapshot()   # thread-safe dict of all values
        node.set_velocity(ch=1, rpm=200)
        node.stop()
    """

    def __init__(self, interface: str = CAN_INTERFACE, node_id: int = NODE_ID):
        self.interface = interface
        self.node_id   = node_id

        # Recompute COB-IDs per node_id instance
        self._sdo_tx   = 0x600 + node_id
        self._sdo_rx   = 0x580 + node_id
        self._tpdo1    = 0x180 + node_id
        self._tpdo2    = 0x280 + node_id
        self._tpdo3    = 0x380 + node_id
        self._tpdo4    = 0x480 + node_id
        self._hb_cob   = 0x700 + node_id
        self._emcy_cob = 0x080 + node_id

        self._sock: Optional[socket.socket] = None

        # Shared live data store
        self.data: Dict[str, Any] = {}
        self._data_lock = threading.Lock()

        # SDO synchronisation — poll thread sends request, rx thread signals
        self._sdo_resp_payload: Optional[bytes] = None
        self._sdo_event = threading.Event()
        self._sdo_lock  = threading.Lock()   # serialise SDO transactions
        self._active_sdo_idx = 0
        self._active_sdo_sub = 0

        self._rx_thread:   Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._cmd_thread:  Optional[threading.Thread] = None
        self._running = False
        self._stop_event = threading.Event()

        # Last command cache for periodic sending (150ms interval)
        self._last_cmd_type = {1: None, 2: None}
        self._last_cmd_val  = {1: 0, 2: 0}
        self._cmd_lock      = threading.Lock()
        self._last_cli_print = 0.0

        # Session stats
        self._sdo_ok  = 0
        self._sdo_err = 0
        self._pdo_rx  = 0
        self._hb_seen = False
        self._hb_last = 0.0

        # Log state
        self._csv_file   = None
        self._csv_writer = None
        self._json_rows: List[Dict] = []

    # ═══════════════════════════════════════════════════════════════════════════
    #  SOCKET  OPEN / CLOSE
    # ═══════════════════════════════════════════════════════════════════════════

    def _open_socket(self) -> None:
        log.info(f"Opening SocketCAN on '{self.interface}' …")
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

        # Kernel-level receive filter — only wake us for COB-IDs we care about
        filters = b"".join(
            struct.pack(CAN_FILTER_FMT, cob & CAN_SFF_MASK, CAN_SFF_MASK)
            for cob in LISTEN_COBS
        )
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
        s.settimeout(0.2)
        s.bind((self.interface,))
        self._sock = s
        log.info(f"Socket bound  ({len(LISTEN_COBS)} COB-ID filters installed)")

    def _close_socket(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            log.info("Socket closed")

    # ═══════════════════════════════════════════════════════════════════════════
    #  SEND FRAME
    # ═══════════════════════════════════════════════════════════════════════════

    def _send_frame(self, can_id: int, data: bytes) -> None:
        try:
            self._sock.send(pack_can_frame(can_id, data))
        except OSError as exc:
            log.error(f"send error: {exc}")

    # ═══════════════════════════════════════════════════════════════════════════
    #  RECEIVER THREAD
    # ═══════════════════════════════════════════════════════════════════════════

    def _rx_loop(self) -> None:
        log.info("RX thread started")
        sock = self._sock
        while self._running:
            try:
                ready, _, _ = select.select([sock], [], [], 0.1)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                raw = sock.recv(CAN_FRAME_SIZE)
            except (socket.timeout, OSError) as exc:
                if self._running:
                    log.error(f"recv: {exc}")
                break
            if len(raw) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, payload = unpack_can_frame(raw)
            self._dispatch(can_id, payload)
        log.info("RX thread stopped")

    def _dispatch(self, can_id: int, data: bytes) -> None:
        """Route received frame to the correct handler."""

        # SDO response from controller
        if can_id == self._sdo_rx:
            if len(data) >= 4:
                echo_idx = struct.unpack_from("<H", data, 1)[0]
                echo_sub = data[3]
                if echo_idx == self._active_sdo_idx and echo_sub == self._active_sdo_sub:
                    self._sdo_resp_payload = bytes(data).ljust(8, b'\x00')
                    self._sdo_event.set()
                else:
                    log.debug(f"  Discarding stale SDO resp "
                              f"0x{echo_idx:04X}:0x{echo_sub:02X} "
                              f"(expected 0x{self._active_sdo_idx:04X}:0x{self._active_sdo_sub:02X})")
            return

        # TPDO
        if can_id in (self._tpdo1, self._tpdo2, self._tpdo3, self._tpdo4):
            parsed = parse_tpdo(can_id, data)
            if parsed:
                with self._data_lock:
                    self.data.update(parsed)
                self._pdo_rx += 1
                log.debug(f"TPDO 0x{can_id:03X}: {parsed}")
            return

        # Heartbeat
        if can_id == self._hb_cob:
            state = parse_heartbeat(data)
            self._hb_seen = True
            self._hb_last = time.monotonic()
            with self._data_lock:
                self.data["heartbeat_state"] = state
            log.debug(f"Heartbeat: {state}")
            return

        # EMCY
        if can_id == self._emcy_cob:
            emcy = parse_emcy(data.ljust(8, b'\x00'))
            with self._data_lock:
                self.data.update(emcy)
            log.warning(f"EMCY: {emcy}")
            return

        log.debug(f"Unhandled: ID=0x{can_id:03X}  {data.hex()}")

    # ═══════════════════════════════════════════════════════════════════════════
    #  SDO READ / WRITE
    # ═══════════════════════════════════════════════════════════════════════════

    def sdo_read(self, index: int, subindex: int,
                 fmt: str = "<i") -> Tuple[bool, Optional[Any]]:
        """
        Send SDO read, wait for response, parse and return (ok, value).
        The _sdo_lock ensures only one SDO transaction runs at a time.
        """
        if not self._running:
            return False, None
        with self._sdo_lock:
            if not self._running:
                return False, None
            self._active_sdo_idx = index
            self._active_sdo_sub = subindex
            self._sdo_event.clear()
            self._sdo_resp_payload = None

            self._send_frame(self._sdo_tx, build_sdo_read(index, subindex))

            if not self._sdo_event.wait(timeout=SDO_TIMEOUT_S):
                log.debug(f"  SDO timeout  0x{index:04X}:0x{subindex:02X}")
                self._sdo_err += 1
                return False, None

            # Guard: stop() sets event with payload=None to unblock waiting threads
            if not self._running or self._sdo_resp_payload is None:
                return False, None

            ok, is_ack, ei, es, data_bytes = parse_sdo_response(self._sdo_resp_payload)
            if not ok or ei != index or es != subindex:
                self._sdo_err += 1
                return False, None

            value = interpret_data(data_bytes, fmt)
            log.debug(f"  SDO OK  0x{index:04X}:0x{subindex:02X} = {value}  "
                      f"(raw {data_bytes.hex()})")
            self._sdo_ok += 1
            return True, value

    def sdo_write(self, index: int, subindex: int,
                  value: int, fmt: str = "<i") -> bool:
        """
        Send SDO write, wait for ACK. Returns True on success.
        Abort code 0x06070010 means data-type mismatch — check fmt!
        """
        if not self._running:
            return False
        with self._sdo_lock:
            if not self._running:
                return False
            self._active_sdo_idx = index
            self._active_sdo_sub = subindex
            self._sdo_event.clear()
            self._sdo_resp_payload = None

            log.debug(f"  SDO WRITE 0x{index:04X}:0x{subindex:02X} = {value} ({fmt})")
            self._send_frame(self._sdo_tx,
                             build_sdo_write(index, subindex, value, fmt))

            if not self._sdo_event.wait(timeout=SDO_TIMEOUT_S):
                log.debug(f"  SDO write timeout 0x{index:04X}:0x{subindex:02X}")
                self._sdo_err += 1
                return False

            # Guard: stop() sets event with payload=None to unblock waiting threads
            if not self._running or self._sdo_resp_payload is None:
                return False

            ok, is_ack, ei, es, _ = parse_sdo_response(self._sdo_resp_payload)
            if ok and is_ack and ei == index and es == subindex:
                self._sdo_ok += 1
                return True
            self._sdo_err += 1
            return False

    # ═══════════════════════════════════════════════════════════════════════════
    #  NMT CONTROL
    # ═══════════════════════════════════════════════════════════════════════════

    def _nmt(self, command: int, target: int = 0) -> None:
        self._send_frame(NMT_COB, bytes([command, target]))
        log.info(f"NMT 0x{command:02X} → node {target}")

    def nmt_operational(self)  -> None: self._nmt(NMT_OPERATIONAL, self.node_id)
    def nmt_pre_op(self)       -> None: self._nmt(NMT_PRE_OP,      self.node_id)
    def nmt_stop(self)         -> None: self._nmt(NMT_STOPPED,      self.node_id)
    def nmt_reset_node(self)   -> None: self._nmt(NMT_RESET_NODE,   self.node_id)
    def nmt_reset_comm(self)   -> None: self._nmt(NMT_RESET_COMM,   self.node_id)

    # ═══════════════════════════════════════════════════════════════════════════
    #  HIGH-LEVEL COMMAND METHODS
    # ═══════════════════════════════════════════════════════════════════════════

    def set_motor_command(self, ch: int, value: int) -> bool:
        """
        Open-loop motor command.  OD 0x2000  Type S32  WO
        value : −1000 … +1000  (‰ of max)
        """
        assert ch in (1, 2)
        with self._cmd_lock:
            self._last_cmd_type[ch] = "motor"
            self._last_cmd_val[ch]  = value
        ok = self.sdo_write(0x2000, ch, value, "<i")
        log.info(f"Motor cmd ch{ch}={value:+d}‰ → {'OK' if ok else 'FAIL'}")
        return ok

    def set_velocity(self, ch: int, rpm: int) -> bool:
        """
        Set velocity command.  OD 0x2002  Type S16  WO   ← S16, NOT S32
        Manual Table (page 34): cS = 0x2002  S16 WO
        """
        assert ch in (1, 2)
        rpm = max(-32768, min(32767, rpm))    # clamp to S16 range
        with self._cmd_lock:
            self._last_cmd_type[ch] = "velocity"
            self._last_cmd_val[ch]  = rpm
        ok  = self.sdo_write(0x2002, ch, rpm, "<h")   # "<h" = S16
        log.info(f"Velocity ch{ch}={rpm} RPM → {'OK' if ok else 'FAIL'}")
        return ok

    def set_position(self, ch: int, counts: int) -> bool:
        """Set position.  OD 0x2001  Type S32  WO"""
        assert ch in (1, 2)
        with self._cmd_lock:
            self._last_cmd_type[ch] = "position"
            self._last_cmd_val[ch]  = counts
        ok = self.sdo_write(0x2001, ch, counts, "<i")
        log.info(f"Position ch{ch}={counts} counts → {'OK' if ok else 'FAIL'}")
        return ok

    def set_position_relative(self, ch: int, counts: int) -> bool:
        """Set relative position.  OD 0x200F  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x200F, ch, counts, "<i")

    def set_next_position(self, ch: int, counts: int) -> bool:
        """Set next absolute position.  OD 0x2010  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2010, ch, counts, "<i")

    def set_next_velocity(self, ch: int, rpm: int) -> bool:
        """Set next velocity.  OD 0x2014  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2014, ch, rpm, "<i")

    def set_encoder_counter(self, ch: int, counts: int) -> bool:
        """Reset/set encoder counter.  OD 0x2003  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2003, ch, counts, "<i")

    def set_brushless_counter(self, ch: int, counts: int) -> bool:
        """Set brushless counter.  OD 0x2004  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2004, ch, counts, "<i")

    def set_acceleration(self, ch: int, rpm_s: int) -> bool:
        """Set acceleration ramp.  OD 0x2006  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2006, ch, rpm_s, "<i")

    def set_deceleration(self, ch: int, rpm_s: int) -> bool:
        """Set deceleration ramp.  OD 0x2007  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2007, ch, rpm_s, "<i")

    def set_next_acceleration(self, ch: int, rpm_s: int) -> bool:
        """Set next acceleration.  OD 0x2012  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2012, ch, rpm_s, "<i")

    def set_next_deceleration(self, ch: int, rpm_s: int) -> bool:
        """Set next deceleration.  OD 0x2013  Type S32  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x2013, ch, rpm_s, "<i")

    def set_all_digital_outputs(self, bitmask: int) -> bool:
        """Set all digital output bits.  OD 0x2008  Type U8  WO"""
        return self.sdo_write(0x2008, 0x00, bitmask & 0xFF, "<B")

    def set_digital_output(self, bit: int, state: bool) -> bool:
        """Set or reset individual digital output.  OD 0x2009/0x200A  U8 WO"""
        idx = 0x2009 if state else 0x200A
        ok  = self.sdo_write(idx, 0x00, bit, "<B")
        log.info(f"D-out bit {bit} → {'1' if state else '0'} {'OK' if ok else 'FAIL'}")
        return ok

    def load_home_counter(self, ch: int) -> bool:
        """Load home counter.  OD 0x200B  Type U8  WO"""
        assert ch in (1, 2)
        return self.sdo_write(0x200B, ch, 0, "<B")

    def emergency_stop(self) -> bool:
        """Emergency shutdown (cuts motor power).  OD 0x200C  U8  WO"""
        with self._cmd_lock:
            self._last_cmd_type = {1: None, 2: None}
        ok = self.sdo_write(0x200C, 0x00, 0, "<B")
        log.warning(f"Emergency Stop → {'OK' if ok else 'FAIL'}")
        return ok

    def release_shutdown(self) -> bool:
        """Release emergency shutdown.  OD 0x200D  U8  WO"""
        ok = self.sdo_write(0x200D, 0x00, 0, "<B")
        log.info(f"Release Shutdown → {'OK' if ok else 'FAIL'}")
        return ok

    def stop_all(self) -> bool:
        """Stop in all modes.  OD 0x200E  U8  WO"""
        with self._cmd_lock:
            self._last_cmd_type = {1: None, 2: None}
        ok = self.sdo_write(0x200E, 0x00, 0, "<B")
        log.info(f"Stop All → {'OK' if ok else 'FAIL'}")
        return ok

    def save_config_to_flash(self) -> bool:
        """Save configuration to flash.  OD 0x2017  U8  WO"""
        return self.sdo_write(0x2017, 0x00, 0, "<B")

    def set_user_var(self, var_num: int, value: int) -> bool:
        """Set user integer variable (1-32).  OD 0x2005  S32  WO"""
        assert 1 <= var_num <= 32
        return self.sdo_write(0x2005, var_num, value, "<i")

    # ── DS402 profile commands ─────────────────────────────────────────────────

    def ds402_control_word(self, ch: int, cw: int) -> bool:
        """
        Write DS402 Control Word.
        ch=1 → 0x6040, ch=2 → 0x6840  sub 0x00  U16
        Common control words:
          0x0006  Ready to switch on
          0x0007  Switch on
          0x000F  Enable operation
          0x000B  Quick stop
          0x0080  Fault reset
        """
        idx = 0x6040 if ch == 1 else 0x6840
        ok  = self.sdo_write(idx, 0x00, cw, "<H")
        log.info(f"DS402 CW ch{ch}=0x{cw:04X} → {'OK' if ok else 'FAIL'}")
        return ok

    def ds402_set_mode(self, ch: int, mode: int) -> bool:
        """
        Write DS402 Modes of Operation.  S8
        ch=1 → 0x6060, ch=2 → 0x6860  sub 0x00
        modes: 1=PP, 2=VL, 3=PV, 4=TQ, 6=Homing
        """
        idx = 0x6060 if ch == 1 else 0x6860
        ok  = self.sdo_write(idx, 0x00, mode, "<b")
        log.info(f"DS402 mode ch{ch}={DS402_OP_MODES.get(mode, mode)} → {'OK' if ok else 'FAIL'}")
        return ok

    def ds402_set_target_velocity_vl(self, ch: int, rpm: int) -> bool:
        """VL target velocity.  S16.  ch=1→0x6042, ch=2→0x6842"""
        idx = 0x6042 if ch == 1 else 0x6842
        return self.sdo_write(idx, 0x00, max(-32768, min(32767, rpm)), "<h")

    def ds402_set_target_velocity_pv(self, ch: int, rpm: int) -> bool:
        """PV target velocity.  S32.  ch=1→0x60FF, ch=2→0x68FF"""
        idx = 0x60FF if ch == 1 else 0x68FF
        return self.sdo_write(idx, 0x00, rpm, "<i")

    def ds402_set_target_position(self, ch: int, counts: int) -> bool:
        """PP target position.  S32.  ch=1→0x607A, ch=2→0x687A"""
        idx = 0x607A if ch == 1 else 0x687A
        return self.sdo_write(idx, 0x00, counts, "<i")

    def ds402_set_target_torque(self, ch: int, torque_pct_x10: int) -> bool:
        """TQ target torque (0.1 %).  S16.  ch=1→0x6071, ch=2→0x6871"""
        idx = 0x6071 if ch == 1 else 0x6871
        return self.sdo_write(idx, 0x00, torque_pct_x10, "<h")

    def ds402_profile_velocity(self, ch: int, rpm: int) -> bool:
        """Profile velocity for PP mode.  U32.  ch=1→0x6081, ch=2→0x6881"""
        idx = 0x6081 if ch == 1 else 0x6881
        return self.sdo_write(idx, 0x00, max(0, rpm), "<I")

    # ── RPDO helpers (direct PDO — bypasses SDO) ───────────────────────────────

    def rpdo1_send(self, ch1_val: int, ch2_val: int) -> None:
        """RPDO1 → default vars VAR9/VAR10  (S32 each)"""
        self._send_frame(RPDO1_COB, struct.pack("<ii", ch1_val, ch2_val))

    def rpdo2_send(self, ch1_val: int, ch2_val: int) -> None:
        """RPDO2 → default vars VAR11/VAR12  (S32 each)"""
        self._send_frame(RPDO2_COB, struct.pack("<ii", ch1_val, ch2_val))

    # ═══════════════════════════════════════════════════════════════════════════
    #  POLL LOOP
    # ═══════════════════════════════════════════════════════════════════════════

    def _poll_loop(self) -> None:
        log.info("Poll thread started")
        cycle = 0
        while self._running:
            t0    = time.monotonic()
            cycle += 1
            if LOG_VERBOSE:
                log.info(f"──── Poll cycle #{cycle:04d} ────────────────────────────")
            round_data: Dict[str, Any] = {}

            for (idx, sub, key, fmt, scale, unit, cmd, desc) in QUERY_TABLE:
                if not self._running:
                    break
                ok, raw = self.sdo_read(idx, sub, fmt)
                if not ok or raw is None:
                    log.debug(f"  MISS [{cmd}] {desc}")
                    continue
                eng = raw * scale
                round_data[key] = eng

                # Inline bitfield decoding for rich data store
                decoded = None
                if key == "fault_flags":
                    decoded = _decode_bits(int(raw), FAULT_BITS)
                    round_data["fault_flags_decoded"] = "|".join(decoded) or "NONE"
                elif key == "status_flags":
                    decoded = _decode_bits(int(raw), STATUS_BITS)
                    round_data["status_flags_decoded"] = "|".join(decoded) or "NONE"
                elif key in ("motor_status_ch1", "motor_status_ch2"):
                    decoded = _decode_bits(int(raw), MOTOR_STATUS_BITS)
                    round_data[f"{key}_decoded"] = "|".join(decoded) or "NONE"
                elif key in ("ds402_status_ch1", "ds402_status_ch2"):
                    decoded = _decode_bits(int(raw), DS402_STATUS_BITS)
                    round_data[f"{key}_decoded"] = "|".join(decoded) or "NONE"
                elif key in ("ds402_opmode_ch1", "ds402_opmode_ch2"):
                    decoded = DS402_OP_MODES.get(int(raw), f"Unknown({raw})")
                    round_data[f"{key}_decoded"] = decoded

                if LOG_VERBOSE:
                    if decoded and isinstance(decoded, list):
                        vstr = "|".join(decoded) or "NONE"
                    elif decoded and isinstance(decoded, str):
                        vstr = decoded
                    elif isinstance(eng, float):
                        vstr = f"{eng:>12.3f}"
                    else:
                        vstr = f"{int(eng):>12d}"
                    log.info(f"  [{cmd:<5}] {desc:<45} {vstr} {unit}")

            with self._data_lock:
                self.data.update(round_data)

            # Heartbeat watchdog
            if self._hb_seen and (time.monotonic() - self._hb_last) > HEARTBEAT_TIMEOUT_S:
                log.warning(f"Heartbeat LOST! Last seen {time.monotonic()-self._hb_last:.1f}s ago")

            # Continuous stream summary line
            with self._data_lock:
                ch1_spd  = self.data.get("encoder_speed_ch1", 0.0)
                ch2_spd  = self.data.get("encoder_speed_ch2", 0.0)
                ch1_cmd  = self.data.get("motor_cmd_ch1",     0.0)
                ch2_cmd  = self.data.get("motor_cmd_ch2",     0.0)
                ch1_amp  = self.data.get("motor_amps_ch1",    0.0)
                ch2_amp  = self.data.get("motor_amps_ch2",    0.0)
                bat_v    = self.data.get("battery_volts",     0.0)
                t1       = self.data.get("temp_heatsink_ch1", 0.0)
                faults   = self.data.get("fault_flags_decoded",  "?")
                status   = self.data.get("status_flags_decoded", "?")

            now = time.monotonic()
            if STREAM_TO_LOG and (now - self._last_cli_print >= CLI_STREAM_INTERVAL_S):
                self._last_cli_print = now
                log.info(
                    f"[STREAM] "
                    f"Ch1: {ch1_spd:+6.0f} RPM ({ch1_cmd:+5.0f}‰) {ch1_amp:4.1f}A | "
                    f"Ch2: {ch2_spd:+6.0f} RPM ({ch2_cmd:+5.0f}‰) {ch2_amp:4.1f}A | "
                    f"Bat: {bat_v:4.1f}V | T1: {t1:3.0f}°C | "
                    f"Flt: {faults} | St: {status}"
                )

            self._log_row()

            elapsed = time.monotonic() - t0
            sleep_t = max(0.0, POLL_INTERVAL_S - elapsed)
            if sleep_t:
                self._stop_event.wait(sleep_t)

        log.info("Poll thread stopped")

    # ═══════════════════════════════════════════════════════════════════════════
    #  CSV / JSON LOGGING
    # ═══════════════════════════════════════════════════════════════════════════

    def _init_csv(self) -> None:
        if not LOG_TO_CSV:
            return
        base_keys   = [q[2] for q in QUERY_TABLE]
        extra_keys  = [
            "fault_flags_decoded", "status_flags_decoded",
            "motor_status_ch1_decoded", "motor_status_ch2_decoded",
            "ds402_status_ch1_decoded", "ds402_status_ch2_decoded",
            "ds402_opmode_ch1_decoded", "ds402_opmode_ch2_decoded",
            "heartbeat_state", "heartbeat_watchdog_ok",
            "sdo_ok_total", "sdo_err_total", "pdo_rx_total",
        ]
        headers = ["timestamp"] + base_keys + extra_keys
        path = f"{LOG_DIR}/roboteq_{_ts}.csv"
        self._csv_file   = open(path, "w", newline="")
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=headers, extrasaction="ignore"
        )
        self._csv_writer.writeheader()
        log.info(f"CSV → {path}")

    def _log_row(self) -> None:
        with self._data_lock:
            snap = dict(self.data)

        snap["timestamp"]          = datetime.now().isoformat(timespec="milliseconds")
        snap["sdo_ok_total"]       = self._sdo_ok
        snap["sdo_err_total"]      = self._sdo_err
        snap["pdo_rx_total"]       = self._pdo_rx
        snap["heartbeat_watchdog_ok"] = (
            self._hb_seen and
            (time.monotonic() - self._hb_last) < HEARTBEAT_TIMEOUT_S
        )

        if LOG_TO_CSV and self._csv_writer:
            self._csv_writer.writerow(snap)
            self._csv_file.flush()
        if LOG_TO_JSON:
            self._json_rows.append(snap)

    def _save_json(self) -> None:
        if not LOG_TO_JSON:
            return
        path = f"{LOG_DIR}/roboteq_{_ts}.json"
        with open(path, "w") as f:
            json.dump(self._json_rows, f, indent=2, default=str)
        log.info(f"JSON → {path}")

    # ═══════════════════════════════════════════════════════════════════════════
    #  START / STOP
    # ═══════════════════════════════════════════════════════════════════════════

    def _cmd_loop(self) -> None:
        log.info("Command sender thread started")
        while self._running:
            t0 = time.monotonic()

            with self._cmd_lock:
                cmds = []
                for ch in (1, 2):
                    ctype = self._last_cmd_type[ch]
                    cval = self._last_cmd_val[ch]
                    if ctype is not None:
                        cmds.append((ch, ctype, cval))

            for ch, ctype, cval in cmds:
                if not self._running:
                    break
                ok = False
                if ctype == "motor":
                    ok = self.sdo_write(0x2000, ch, cval, "<i")
                elif ctype == "velocity":
                    ok = self.sdo_write(0x2002, ch, cval, "<h")
                elif ctype == "position":
                    ok = self.sdo_write(0x2001, ch, cval, "<i")
                
                log.info(f"[CMD_SEND] Ch {ch} {ctype}={cval:+d} → {'OK' if ok else 'FAIL'}")

            elapsed = time.monotonic() - t0
            sleep_t = max(0.0, 0.050 - elapsed)
            if sleep_t:
                self._stop_event.wait(sleep_t)
        log.info("Command sender thread stopped")

    def start(self) -> None:
        self._open_socket()
        self._init_csv()
        self._running = True

        log.info("NMT → Operational")
        self.nmt_operational()
        time.sleep(0.1)

        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="can_rx", daemon=True
        )
        self._rx_thread.start()

        self._poll_thread = threading.Thread(
            target=self._poll_loop, name="can_poll", daemon=True
        )
        self._poll_thread.start()

        self._cmd_thread = threading.Thread(
            target=self._cmd_loop, name="can_cmd", daemon=True
        )
        self._cmd_thread.start()

        log.info("RoboteqCANopenNode running")

    def stop(self) -> None:
        log.info("Stopping …")
        self._running = False
        self._sdo_event.set()      # unblock any waiting sdo_read/write
        self._stop_event.set()     # wake up loops from wait/sleep

        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._poll_thread:
            self._poll_thread.join(timeout=2.0)
        if self._cmd_thread:
            self._cmd_thread.join(timeout=2.0)

        if self._csv_file:
            self._csv_file.close()
        self._save_json()
        self._close_socket()

        log.info(f"Shutdown — SDO ok={self._sdo_ok} err={self._sdo_err} "
                 f"PDO rx={self._pdo_rx}")

    # ═══════════════════════════════════════════════════════════════════════════
    #  DATA ACCESS
    # ═══════════════════════════════════════════════════════════════════════════

    def get_snapshot(self) -> Dict[str, Any]:
        """Thread-safe copy of all live values."""
        with self._data_lock:
            return dict(self.data)

    def print_snapshot(self) -> None:
        snap = self.get_snapshot()
        bar  = "═" * 62
        print(f"\n╔{bar}╗")
        print(f"║{'Roboteq FBL2360T  —  Live Data Snapshot':^62}║")
        print(f"║{'Node ' + str(self.node_id) + '  ·  ' + datetime.now().strftime('%H:%M:%S.%f')[:-3]:^62}║")
        print(f"╠{bar}╣")
        for k, v in sorted(snap.items()):
            if isinstance(v, list):
                vstr = ", ".join(map(str, v)) if v else "—"
            elif isinstance(v, float):
                vstr = f"{v:.3f}"
            elif isinstance(v, bool):
                vstr = "YES" if v else "NO"
            else:
                vstr = str(v)
            vstr = vstr[:38]   # truncate for display
            print(f"║  {k:<38s}  {vstr:<20s}║")
        print(f"╠{bar}╣")
        print(f"║  SDO ok={self._sdo_ok:<6d} err={self._sdo_err:<6d} "
              f"PDO={self._pdo_rx:<12d}║")
        print(f"╚{bar}╝\n")


# ─────────────────────────────────────────────────────────────────────────────
#  STANDALONE DEMO
# ─────────────────────────────────────────────────────────────────────────────

def demo_run() -> None:
    """
    End-to-end demonstration — runs indefinitely until Ctrl-C.
    Polls all OD entries, streams one summary line per cycle,
    and logs everything to CSV + JSON. Supports interactive command entry.
    """
    node = RoboteqCANopenNode(interface=CAN_INTERFACE, node_id=NODE_ID)

    try:
        node.start()
        log.info("Demo running — interactive mode active")

        # Wait for first poll round
        time.sleep(1.5)
        node.print_snapshot()

        print("\n" + "="*50)
        print(" Roboteq FBL2360T Interactive CANopen Shell")
        print("="*50)
        print("Enter commands to control the motor controller.")
        print("Format: <command> [args...]")
        print("\nCommands:")
        print("  motor <ch> <val>        - Set open-loop motor command (-1000 to 1000)")
        print("  velocity <ch> <rpm>     - Set target velocity in RPM")
        print("  position <ch> <count>   - Set target position in counts")
        print("  accel <ch> <rpm_s>      - Set acceleration rate in RPM/s")
        print("  decel <ch> <rpm_s>      - Set deceleration rate in RPM/s")
        print("  mode <ch> <mode_num>    - Set DS402 mode (1:PP, 2:VL, 3:PV, 4:TQ)")
        print("  stop                    - Stop both channels")
        print("  estop                   - Trigger emergency stop (EX)")
        print("  release                 - Release emergency shutdown (MG)")
        print("  print                   - Print live data snapshot")
        print("  help                    - Show this menu")
        print("  exit / quit             - Exit the program")
        print("="*50 + "\n")

        while True:
            try:
                cmd_in = read_line_raw("roboteq> ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                break

            if not cmd_in:
                continue

            parts = cmd_in.split()
            cmd = parts[0].lower()

            if cmd in ("exit", "quit"):
                break
            elif cmd == "help":
                print("Commands: motor, velocity, position, accel, decel, mode, stop, estop, release, print, help, exit")
            elif cmd == "print":
                node.print_snapshot()
            elif cmd == "stop":
                ok = node.stop_all()
                log.info(f"Stop command sent: {'SUCCESS' if ok else 'FAILED'}")
            elif cmd == "estop":
                ok = node.emergency_stop()
                log.info(f"Emergency stop sent: {'SUCCESS' if ok else 'FAILED'}")
            elif cmd == "release":
                ok = node.release_shutdown()
                log.info(f"Emergency release sent: {'SUCCESS' if ok else 'FAILED'}")
            elif cmd in ("motor", "velocity", "position", "accel", "decel", "mode"):
                if len(parts) < 3:
                    print(f"Error: {cmd} requires channel (1 or 2) and value.")
                    continue
                try:
                    ch = int(parts[1])
                    val = int(parts[2])
                except ValueError:
                    print("Error: Channel and value must be integers.")
                    continue

                if ch not in (1, 2):
                    print("Error: Channel must be 1 or 2.")
                    continue

                if cmd == "motor":
                    ok = node.set_motor_command(ch, val)
                    log.info(f"Set Motor Command Ch {ch} to {val}: {'SUCCESS' if ok else 'FAILED'}")
                elif cmd == "velocity":
                    ok = node.set_velocity(ch, val)
                    log.info(f"Set Velocity Ch {ch} to {val} RPM: {'SUCCESS' if ok else 'FAILED'}")
                elif cmd == "position":
                    ok = node.set_position(ch, val)
                    log.info(f"Set Position Ch {ch} to {val}: {'SUCCESS' if ok else 'FAILED'}")
                elif cmd == "accel":
                    ok = node.set_acceleration(ch, val)
                    log.info(f"Set Acceleration Ch {ch} to {val} RPM/s: {'SUCCESS' if ok else 'FAILED'}")
                elif cmd == "decel":
                    ok = node.set_deceleration(ch, val)
                    log.info(f"Set Deceleration Ch {ch} to {val} RPM/s: {'SUCCESS' if ok else 'FAILED'}")
                elif cmd == "mode":
                    ok = node.ds402_set_mode(ch, val)
                    log.info(f"Set DS402 Mode Ch {ch} to {val}: {'SUCCESS' if ok else 'FAILED'}")
            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for options.")

    except KeyboardInterrupt:
        log.info("Ctrl-C — shutting down")
    finally:
        node.stop()


if __name__ == "__main__":
    demo_run()