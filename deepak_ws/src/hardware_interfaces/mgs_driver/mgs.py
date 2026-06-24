#!/usr/bin/env python3
"""
MGS1600 CANopen Driver & Test Script for NVIDIA Jetson
=======================================================
Sensor   : Roboteq MGS1600 Magnetic Guide Sensor
Interface: SocketCAN (can0)
Protocol : CANopen (firmware v3.0+)
Node ID  : 5
Bitrate  : 500 kbps

Datasheet reference: MGS1600 Magnetic Sensor Datasheet v1.4, July 14, 2023

PDO Layout (Node ID = 5):
  TPDO1  COB-ID 0x185  →  Left Track (S16) | Right Track (S16) | Flags (U16)
  TPDO2  COB-ID 0x285  →  VAR1 (S32) | VAR2 (S32)
  SDO TX COB-ID 0x605  →  We send requests here
  SDO RX COB-ID 0x585  →  Sensor replies here
  NMT    COB-ID 0x000  →  Network Management

Setup on Jetson before running:
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set up can0
  pip3 install python-can   # optional, this script uses raw sockets only
"""

import socket
import struct
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, Tuple

# ─── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
    filename="mgs.log",
    filemode="a",
)
log = logging.getLogger("MGS1600")


# ══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

CAN_INTERFACE = "can0"
NODE_ID       = 5           # As configured in MagSensor Control Utility
BITRATE       = 500_000     # 500 kbps — set via ip link before running

# ── Derived COB-IDs ───────────────────────────────────────────────────────────
NMT_COB_ID       = 0x000
SYNC_COB_ID      = 0x080
TPDO1_COB_ID     = 0x180 + NODE_ID   # 0x185  sensor → Jetson (track + flags)
TPDO2_COB_ID     = 0x280 + NODE_ID   # 0x285  sensor → Jetson (user vars)
RPDO1_COB_ID     = 0x200 + NODE_ID   # 0x205  Jetson → sensor
RPDO2_COB_ID     = 0x300 + NODE_ID   # 0x305  Jetson → sensor
SDO_TX_COB_ID    = 0x600 + NODE_ID   # 0x605  Jetson SDO requests → sensor
SDO_RX_COB_ID    = 0x580 + NODE_ID   # 0x585  sensor SDO responses → Jetson
HEARTBEAT_COB_ID = 0x700 + NODE_ID   # 0x705  heartbeat from sensor

# ── NMT Command Specifiers ────────────────────────────────────────────────────
NMT_START      = 0x01   # → Operational  (TPDOs begin streaming)
NMT_STOP       = 0x02   # → Stopped
NMT_PRE_OP     = 0x80   # → Pre-Operational
NMT_RESET_NODE = 0x81   # Full node reset
NMT_RESET_COMM = 0x82   # Reset communication only

# ── SDO Command Specifiers ────────────────────────────────────────────────────
SDO_READ_REQ    = 0x40   # Initiate Upload   (we read from sensor)
SDO_WRITE_1B    = 0x2F   # Download 1 byte
SDO_WRITE_2B    = 0x2B   # Download 2 bytes
SDO_WRITE_4B    = 0x23   # Download 4 bytes

# SDO response byte → data payload size
SDO_RESP_SIZE: Dict[int, int] = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}

# ── MGS1600 Object Dictionary (Tables 8 & 9 in datasheet) ────────────────────
# Format: "NAME": (index, subindex)   subindex=None means caller supplies it
OD: Dict[str, Tuple[int, Optional[int]]] = {
    # ── Runtime Queries ──
    "VAR_INT":        (0x2106, None),   # S32 RO  Read User Integer Variable n  (sub 1–10)
    "DOMINANT_TRACK": (0x210F, 0x00),   # S8  RO  Dominant/selected track position
    "VAR_BOOL":       (0x2115, None),   # U8  RO  Read User Bool Variable n      (sub 1–10)
    "TRACK_DETECT":   (0x211D, 0x01),   # U8  RO  1 = tape present, 0 = no tape
    "LEFT_TRACK":     (0x211E, 0x01),   # S16 RO  Left  track position (mm, signed)
    "RIGHT_TRACK":    (0x211E, 0x02),   # S16 RO  Right track position (mm, signed)
    "SELECTED_TRACK": (0x211E, 0x03),   # S16 RO  Selected track position
    "LEFT_MARKER":    (0x211F, 0x01),   # U8  RO  1 = left marker detected
    "RIGHT_MARKER":   (0x211F, 0x02),   # U8  RO  1 = right marker detected
    "STATUS":         (0x2120, 0x01),   # U16 RO  MagSensor status word
    "RAW_SENSOR":     (0x212D, None),   # U32 RO  Raw internal sensor n          (sub 1–16)
    "ZERO_ADJ":       (0x212E, None),   # S32 RO  Zero-adjusted sensor n         (sub 1–16)
    "TAPE_CROSS":     (0x2138, 0x01),   # U8  RO  1 = cross-tape detected (fw v3.0+)
    # ── Runtime Commands ──
    "FOLLOW_LEFT":    (0x201A, 0x00),   # U8  WO  Issue !TX (follow left track)
    "FOLLOW_RIGHT":   (0x201B, 0x00),   # U8  WO  Issue !TV (follow right track)
    "SAVE_CONFIG":    (0x2017, 0x00),   # U8  WO  Save config to flash (%EESAV)
    "SET_ZERO":       (0x2020, 0x00),   # U8  WO  Set zero calibration (!ZER)
}


# ══════════════════════════════════════════════════════════════════════════════
#  DATA STRUCTURES
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SensorState:
    """Decoded live state populated by TPDO1 parsing."""
    # Track positions (mm from sensor center; negative = left of center)
    left_track:     int   = 0
    right_track:    int   = 0
    selected_track: int   = 0    # last SDO-queried selected track

    # Boolean flags (from TPDO1 flags word)
    tape_detect:    bool  = False
    tape_cross:     bool  = False  # fw v3.0+ only
    left_marker:    bool  = False
    right_marker:   bool  = False
    sensor_failure: bool  = False

    # Meta
    timestamp:      float = field(default_factory=time.time)
    tpdo1_count:    int   = 0

    def __str__(self) -> str:
        age_ms = (time.time() - self.timestamp) * 1000
        return (
            f"L={self.left_track:+5d}mm  R={self.right_track:+5d}mm  |  "
            f"TD={'Y' if self.tape_detect  else 'N'}  "
            f"LM={'Y' if self.left_marker  else 'N'}  "
            f"RM={'Y' if self.right_marker else 'N'}  "
            f"TX={'Y' if self.tape_cross   else 'N'}  "
            f"ERR={'!' if self.sensor_failure else '-'}  "
            f"({age_ms:.0f}ms ago)"
        )


# ══════════════════════════════════════════════════════════════════════════════
#  RAW SOCKETCAN HELPERS
# ══════════════════════════════════════════════════════════════════════════════

# Struct: can_id (4B LE), dlc (1B), pad (3B), data (8B)
_FRAME_FMT  = "=IB3x8s"
_FRAME_SIZE = struct.calcsize(_FRAME_FMT)


def _build_frame(can_id: int, data: bytes) -> bytes:
    dlc = len(data)
    if dlc > 8:
        data = data[:8]
        dlc = 8
    padded_data = data.ljust(8, b"\x00")
    return struct.pack(_FRAME_FMT, can_id & 0x1FFFFFFF, dlc, padded_data)


def _parse_frame(raw: bytes) -> Tuple[int, int, bytes]:
    can_id, dlc, data = struct.unpack(_FRAME_FMT, raw)
    can_id &= 0x1FFFFFFF   # strip EFF / RTR / ERR flags
    return can_id, dlc, data[:dlc]


# ══════════════════════════════════════════════════════════════════════════════
#  MGS1600 CANOPEN DRIVER
# ══════════════════════════════════════════════════════════════════════════════

class MGS1600CANopen:
    """
    Query-response CANopen driver for the MGS1600 over SocketCAN.

    Supports:
      • NMT lifecycle management
      • TPDO1/TPDO2 streaming (background thread, auto-parsed)
      • SDO expedited read/write for any Object Dictionary entry
      • Named high-level query helpers
      • User-registered COB-ID callbacks
    """

    def __init__(self, interface: str = CAN_INTERFACE, node_id: int = NODE_ID):
        self.interface  = interface
        self.node_id    = node_id
        self.state      = SensorState()

        self._sock: Optional[socket.socket]   = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running   = False
        self._lock      = threading.Lock()

        # SDO synchronisation
        self._sdo_lock     = threading.Lock()
        self._sdo_event    = threading.Event()
        self._sdo_response: Optional[bytes] = None

        # User COB-ID callbacks: {can_id: fn(can_id, bytes)}
        self._callbacks: Dict[int, Callable[[int, bytes], None]] = {}

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((self.interface,))
        self._sock.settimeout(0.5)
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="CAN-RX")
        self._rx_thread.start()
        log.info(f"SocketCAN open: iface={self.interface}  node_id={self.node_id}")

    def disconnect(self) -> None:
        self._running = False
        if self._sock:
            self._sock.close()
            self._sock = None
        log.info("SocketCAN closed.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.nmt_stop()
        self.disconnect()

    # ── Raw send ──────────────────────────────────────────────────────────────
    def _send(self, can_id: int, data: bytes) -> None:
        if self._sock is None:
            raise RuntimeError("Not connected. Call connect() first.")
        self._sock.send(_build_frame(can_id, data))
        log.debug(f"TX  COB-ID=0x{can_id:03X}  data={data.hex(' ')}")

    # ══════════════════════════════════════════════════════════════════════════
    #  NMT — Network Management
    # ══════════════════════════════════════════════════════════════════════════
    def nmt_start(self) -> None:
        """Transition node to Operational → TPDO streaming starts."""
        self._send(NMT_COB_ID, bytes([NMT_START, self.node_id]))
        log.info(f"NMT → Operational  (node {self.node_id})")

    def nmt_stop(self) -> None:
        """Transition node to Stopped → no PDO traffic."""
        self._send(NMT_COB_ID, bytes([NMT_STOP, self.node_id]))
        log.info(f"NMT → Stopped  (node {self.node_id})")

    def nmt_pre_operational(self) -> None:
        """Pre-Operational state — SDOs work, PDOs do not."""
        self._send(NMT_COB_ID, bytes([NMT_PRE_OP, self.node_id]))
        log.info(f"NMT → Pre-Operational  (node {self.node_id})")

    def nmt_reset(self, wait: float = 2.0) -> None:
        """Full node reset. Waits `wait` seconds for boot."""
        self._send(NMT_COB_ID, bytes([NMT_RESET_NODE, self.node_id]))
        log.info(f"NMT → Reset  (waiting {wait}s for boot …)")
        time.sleep(wait)

    def nmt_reset_comm(self) -> None:
        """Reset communication stack only."""
        self._send(NMT_COB_ID, bytes([NMT_RESET_COMM, self.node_id]))
        log.info(f"NMT → Reset Communication  (node {self.node_id})")

    # ══════════════════════════════════════════════════════════════════════════
    #  SDO — Service Data Object (expedited, single-frame)
    # ══════════════════════════════════════════════════════════════════════════
    def sdo_read(
        self, index: int, subindex: int, timeout: float = 1.0
    ) -> Optional[bytes]:
        """
        Expedited SDO upload (read).
        Sends a read request to the sensor and blocks until a response arrives
        or the timeout expires.

        Returns raw payload bytes on success, None on timeout/error.
        """
        req = struct.pack("<BHB4x", SDO_READ_REQ, index, subindex)
        with self._sdo_lock:
            self._sdo_event.clear()
            self._sdo_response = None
            self._send(SDO_TX_COB_ID, req)
            if self._sdo_event.wait(timeout):
                return self._sdo_response
        log.warning(f"SDO read timeout  idx=0x{index:04X} sub=0x{subindex:02X}")
        return None

    def sdo_write_u8(self, index: int, subindex: int, value: int) -> None:
        """Expedited SDO download — 1-byte unsigned."""
        data = struct.pack("<BHBB3x", SDO_WRITE_1B, index, subindex, value & 0xFF)
        self._send(SDO_TX_COB_ID, data)

    def sdo_write_u16(self, index: int, subindex: int, value: int) -> None:
        """Expedited SDO download — 2-byte unsigned."""
        data = struct.pack("<BHBH2x", SDO_WRITE_2B, index, subindex, value & 0xFFFF)
        self._send(SDO_TX_COB_ID, data)

    def sdo_write_u32(self, index: int, subindex: int, value: int) -> None:
        """Expedited SDO download — 4-byte unsigned."""
        data = struct.pack("<BHBI", SDO_WRITE_4B, index, subindex, value & 0xFFFFFFFF)
        self._send(SDO_TX_COB_ID, data)

    # ── SDO helpers that decode common types ──────────────────────────────────
    def _sdo_read_s8(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return struct.unpack("<b", raw[:1])[0] if raw else None

    def _sdo_read_u8(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return raw[0] if raw else None

    def _sdo_read_s16(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return struct.unpack("<h", raw[:2])[0] if raw and len(raw) >= 2 else None

    def _sdo_read_u16(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return struct.unpack("<H", raw[:2])[0] if raw and len(raw) >= 2 else None

    def _sdo_read_s32(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return struct.unpack("<i", raw[:4])[0] if raw and len(raw) >= 4 else None

    def _sdo_read_u32(self, index: int, subindex: int) -> Optional[int]:
        raw = self.sdo_read(index, subindex)
        return struct.unpack("<I", raw[:4])[0] if raw and len(raw) >= 4 else None

    # ══════════════════════════════════════════════════════════════════════════
    #  HIGH-LEVEL QUERY API  (SDO query-response model)
    # ══════════════════════════════════════════════════════════════════════════
    def query_track_detect(self) -> Optional[bool]:
        """True if magnetic tape is within sensor range."""
        v = self._sdo_read_u8(*OD["TRACK_DETECT"])
        return bool(v) if v is not None else None

    def query_left_track(self) -> Optional[int]:
        """Left track position in mm (signed, 0 = sensor center)."""
        return self._sdo_read_s16(*OD["LEFT_TRACK"])

    def query_right_track(self) -> Optional[int]:
        """Right track position in mm (signed, 0 = sensor center)."""
        return self._sdo_read_s16(*OD["RIGHT_TRACK"])

    def query_selected_track(self) -> Optional[int]:
        """Currently selected track position in mm."""
        return self._sdo_read_s16(*OD["SELECTED_TRACK"])

    def query_dominant_track(self) -> Optional[int]:
        """Dominant track (S8). Equivalent to ?T command."""
        return self._sdo_read_s8(*OD["DOMINANT_TRACK"])

    def query_left_marker(self) -> Optional[bool]:
        """True if left marker is detected."""
        v = self._sdo_read_u8(*OD["LEFT_MARKER"])
        return bool(v) if v is not None else None

    def query_right_marker(self) -> Optional[bool]:
        """True if right marker is detected."""
        v = self._sdo_read_u8(*OD["RIGHT_MARKER"])
        return bool(v) if v is not None else None

    def query_tape_cross(self) -> Optional[bool]:
        """True if cross-tape detected (firmware v3.0+ only)."""
        v = self._sdo_read_u8(*OD["TAPE_CROSS"])
        return bool(v) if v is not None else None

    def query_status(self) -> Optional[int]:
        """Raw MGS status word (U16). Bit 8 = sensor failure."""
        return self._sdo_read_u16(*OD["STATUS"])

    def query_raw_sensor(self, n: int) -> Optional[int]:
        """Read one of the 16 raw internal Hall-effect sensor values (n = 1…16)."""
        if not 1 <= n <= 16:
            raise ValueError("n must be 1–16")
        return self._sdo_read_u32(OD["RAW_SENSOR"][0], n)

    def query_zero_adjusted_sensor(self, n: int) -> Optional[int]:
        """Zero-adjusted internal sensor value n (n = 1…16)."""
        if not 1 <= n <= 16:
            raise ValueError("n must be 1–16")
        return self._sdo_read_s32(OD["ZERO_ADJ"][0], n)

    def query_user_int(self, n: int) -> Optional[int]:
        """Read user integer variable VAR n (n = 1…10)."""
        if not 1 <= n <= 10:
            raise ValueError("n must be 1–10")
        return self._sdo_read_s32(OD["VAR_INT"][0], n)

    def query_all(self) -> dict:
        """Convenience — single call to fetch all key sensor values via SDO."""
        return {
            "left_track":     self.query_left_track(),
            "right_track":    self.query_right_track(),
            "selected_track": self.query_selected_track(),
            "track_detect":   self.query_track_detect(),
            "left_marker":    self.query_left_marker(),
            "right_marker":   self.query_right_marker(),
            "tape_cross":     self.query_tape_cross(),
            "status":         self.query_status(),
        }

    # ══════════════════════════════════════════════════════════════════════════
    #  HIGH-LEVEL COMMAND API
    # ══════════════════════════════════════════════════════════════════════════
    def cmd_follow_left(self) -> None:
        """Instruct sensor to follow the left track (!TX)."""
        idx, sub = OD["FOLLOW_LEFT"]
        self.sdo_write_u8(idx, sub, 1)
        log.info("CMD: Follow Left track")

    def cmd_follow_right(self) -> None:
        """Instruct sensor to follow the right track (!TV)."""
        idx, sub = OD["FOLLOW_RIGHT"]
        self.sdo_write_u8(idx, sub, 1)
        log.info("CMD: Follow Right track")

    def cmd_save_config(self) -> None:
        """Save current configuration to EEPROM (%EESAV)."""
        idx, sub = OD["SAVE_CONFIG"]
        self.sdo_write_u8(idx, sub, 1)
        log.info("CMD: Save config to flash")

    def cmd_set_zero_calibration(self) -> None:
        """Set the ambient zero calibration level (!ZER)."""
        idx, sub = OD["SET_ZERO"]
        self.sdo_write_u8(idx, sub, 1)
        log.info("CMD: Zero calibration applied")

    def send_rpdo_follow(self, follow_left: bool, follow_right: bool, use_var_mapping: bool = False) -> None:
        """
        Sends RPDO1 (COB-ID 0x200 + NodeID) to change track following selection.

        If use_var_mapping is True:
          Uses the default Roboteq CANopen/MiniCAN mapping.
          Writes 1 (Left), 2 (Right), or 0 (Clear) to User Integer Variable 2 (VAR2).
          Requires a MicroBasic script running on the sensor to process VAR2.
        If use_var_mapping is False:
          Assumes custom RPDO1 mapping.
          Writes directly to FOLLOW_LEFT (0x201A, 0x00) and FOLLOW_RIGHT (0x201B, 0x00).
        """
        if use_var_mapping:
            val = 1 if follow_left else (2 if follow_right else 0)
            data = struct.pack("<ii", val, 0)  # VAR2 (4B) and VAR3 (4B)
        else:
            data = struct.pack("<BB", 1 if follow_left else 0, 1 if follow_right else 0)

        self._send(0x200 + self.node_id, data)
        log.info(f"RPDO1 TX: Follow Left={follow_left}, Follow Right={follow_right} (var_mode={use_var_mapping})")

    # ══════════════════════════════════════════════════════════════════════════
    #  CALLBACK REGISTRATION
    # ══════════════════════════════════════════════════════════════════════════
    def on_message(self, can_id: int, fn: Callable[[int, bytes], None]) -> None:
        """
        Register a callback for any COB-ID.
        fn is called from the RX thread with (can_id, data_bytes).
        """
        self._callbacks[can_id] = fn

    def on_tpdo1(self, fn: Callable[["SensorState"], None]) -> None:
        """
        Convenience wrapper — callback receives a parsed SensorState
        every time TPDO1 is received.
        """
        def _wrap(cid, data):
            fn(self.state)
        self._callbacks[TPDO1_COB_ID] = _wrap

    # ══════════════════════════════════════════════════════════════════════════
    #  RX LOOP & MESSAGE DISPATCH
    # ══════════════════════════════════════════════════════════════════════════
    def _rx_loop(self) -> None:
        while self._running:
            try:
                raw = self._sock.recv(_FRAME_SIZE)
                can_id, dlc, data = _parse_frame(raw)
                log.debug(f"RX  COB-ID=0x{can_id:03X}  dlc={dlc}  data={data.hex(' ')}")
                self._dispatch(can_id, data)
            except socket.timeout:
                continue
            except OSError:
                break

    def _dispatch(self, can_id: int, data: bytes) -> None:
        if   can_id == TPDO1_COB_ID:
            self._parse_tpdo1(data)
        elif can_id == TPDO2_COB_ID:
            self._parse_tpdo2(data)
        elif can_id == SDO_RX_COB_ID:
            self._handle_sdo_response(data)
        elif can_id == HEARTBEAT_COB_ID:
            self._handle_heartbeat(data)

        cb = self._callbacks.get(can_id)
        if cb:
            try:
                cb(can_id, data)
            except Exception as exc:
                log.error(f"Callback error for 0x{can_id:03X}: {exc}")

    # ── TPDO1 parser ─────────────────────────────────────────────────────────
    def _parse_tpdo1(self, data: bytes) -> None:
        """
        TPDO1 — COB-ID 0x185  (datasheet page 15, Table under CANbus section)

        Byte layout (little-endian):
          [0:2]  Left  Track  S16  mm  (negative = left of center)
          [2:4]  Right Track  S16  mm  (negative = left of center)
          [4:6]  Flags        U16

        CANopen flag bits:
          Bit 1 (0x0001) → Tape Cross    (fw v3.0+)
          Bit 2 (0x0002) → Tape Detect
          Bit 3 (0x0004) → Left Marker
          Bit 4 (0x0008) → Right Marker
          Bit 7 (0x0040) → Sensor Failure
        """
        if len(data) < 6:
            log.warning(f"TPDO1: short frame ({len(data)} bytes) — skipping")
            return

        left_track  = struct.unpack_from("<h", data, 0)[0]
        right_track = struct.unpack_from("<h", data, 2)[0]
        flags       = struct.unpack_from("<H", data, 4)[0]

        selected_track = 0
        if len(data) >= 8:
            selected_track = struct.unpack_from("<h", data, 6)[0]

        with self._lock:
            self.state.left_track     = left_track
            self.state.right_track    = right_track
            self.state.selected_track = selected_track
            self.state.tape_cross     = bool(flags & 0x0001)
            self.state.tape_detect    = bool(flags & 0x0002)
            self.state.left_marker    = bool(flags & 0x0004)
            self.state.right_marker   = bool(flags & 0x0008)
            self.state.sensor_failure = bool(flags & 0x0040)
            self.state.timestamp      = time.time()
            self.state.tpdo1_count   += 1

    # ── TPDO2 parser ─────────────────────────────────────────────────────────
    def _parse_tpdo2(self, data: bytes) -> None:
        """
        TPDO2 — COB-ID 0x285

        Byte layout:
          [0:4]  VAR1  S32  (User Integer Variable 1)
          [4:8]  VAR2  S32  (User Integer Variable 2)
        """
        if len(data) < 8:
            return
        var1 = struct.unpack_from("<i", data, 0)[0]
        var2 = struct.unpack_from("<i", data, 4)[0]
        log.debug(f"TPDO2 | VAR1={var1}  VAR2={var2}")

    # ── SDO response handler ──────────────────────────────────────────────────
    def _handle_sdo_response(self, data: bytes) -> None:
        """
        Expedited SDO Upload Response (sensor → Jetson)

        Byte 0 (command):
          0x4F → 1-byte payload  at data[4]
          0x4B → 2-byte payload  at data[4:6]
          0x47 → 3-byte payload  at data[4:7]
          0x43 → 4-byte payload  at data[4:8]
        Byte 1-2 : Index    (little-endian)
        Byte 3   : Subindex
        Byte 4-7 : Data     (little-endian)
        """
        if len(data) < 8:
            return
        cmd_byte = data[0]
        # index    = struct.unpack_from("<H", data, 1)[0]
        # subindex = data[3]
        n = SDO_RESP_SIZE.get(cmd_byte, 4)
        self._sdo_response = data[4: 4 + n]
        self._sdo_event.set()

    # ── Heartbeat handler ─────────────────────────────────────────────────────
    def _handle_heartbeat(self, data: bytes) -> None:
        _state_map = {
            0x00: "Boot-up",
            0x04: "Stopped",
            0x05: "Operational",
            0x7F: "Pre-Operational",
        }
        nmt_state = _state_map.get(data[0], f"Unknown(0x{data[0]:02X})")
        log.info(f"Heartbeat: node {self.node_id} → {nmt_state}")


# ══════════════════════════════════════════════════════════════════════════════
#  CONTINUOUS DATA STREAM
# ══════════════════════════════════════════════════════════════════════════════

def cli_handler(sensor: MGS1600CANopen) -> None:
    print("\n--- MGS1600 Interactive CLI Control ---")
    print("Commands:")
    print("  l / left   : Send RPDO1 Follow Left")
    print("  r / right  : Send RPDO1 Follow Right")
    print("  s / stop   : Send RPDO1 Follow None (Stop)")
    print("  sdl        : Send SDO Follow Left")
    print("  sdr        : Send SDO Follow Right")
    print("  status     : Query current SDO status word")
    print("  q / exit   : Quit script")
    print("---------------------------------------\n")

    while True:
        try:
            line = input("> ").strip().lower()
            if not line:
                continue
            if line in ("exit", "q"):
                print("Exiting...")
                import os
                os._exit(0)
            elif line in ("l", "left"):
                sensor.send_rpdo_follow(follow_left=True, follow_right=False)
            elif line in ("r", "right"):
                sensor.send_rpdo_follow(follow_left=False, follow_right=True)
            elif line in ("s", "stop"):
                sensor.send_rpdo_follow(follow_left=False, follow_right=False)
            elif line in ("sdl",):
                sensor.cmd_follow_left()
            elif line in ("sdr",):
                sensor.cmd_follow_right()
            elif line in ("status",):
                stat = sensor.query_status()
                print(f"Current SDO Status: {stat}")
            else:
                print(f"Unknown command: '{line}'")
        except (KeyboardInterrupt, EOFError):
            break


def stream():
    """
    Production entry point — streams TPDO1 data and polls all SDOs from the MGS1600 indefinitely.
    """
    log.info(f"MGS1600 stream starting  iface={CAN_INTERFACE}  node={NODE_ID}  {BITRATE//1000}kbps")
    log.info("Press Ctrl+C to stop.")

    with MGS1600CANopen(CAN_INTERFACE, NODE_ID) as sensor:
        # Put node into Operational so TPDO1 frames start arriving
        sensor.nmt_start()
        time.sleep(0.3)

        # Start interactive CLI input thread
        cli_thread = threading.Thread(target=cli_handler, args=(sensor,), daemon=True)
        cli_thread.start()

        while True:
            try:
                # Poll via SDOs for all possible data
                dt = sensor.query_dominant_track() or 0
                st = sensor.query_status() or 0
                raws = [sensor.query_raw_sensor(i) or 0 for i in range(1, 17)]
                adjs = [sensor.query_zero_adjusted_sensor(i) or 0 for i in range(1, 17)]

                tape_detect = sensor.query_track_detect() or 0
                left_track = sensor.query_left_track() or 0
                right_track = sensor.query_right_track() or 0
                selected_track = sensor.query_selected_track() or 0
                left_marker = sensor.query_left_marker() or 0
                right_marker = sensor.query_right_marker() or 0
                tape_cross = sensor.query_tape_cross() or 0

                with sensor._lock:
                    s = sensor.state
                    count = s.tpdo1_count
                    # Use TPDO1 data if it's actively updating, otherwise fallback to SDO
                    val_lt = s.left_track if count > 0 else left_track
                    val_rt = s.right_track if count > 0 else right_track
                    val_st = s.selected_track if count > 0 and s.selected_track else selected_track
                    val_td = s.tape_detect if count > 0 else tape_detect
                    val_lm = s.left_marker if count > 0 else left_marker
                    val_rm = s.right_marker if count > 0 else right_marker
                    val_tx = s.tape_cross if count > 0 else tape_cross
                    val_err = s.sensor_failure

                log.info(
                    f"log : [L:{val_lt}]:[R:{val_rt}]:[ST:{val_st}]:[DT:{dt}]:"
                    f"[TD:{int(val_td)}]:[LM:{int(val_lm)}]:"
                    f"[RM:{int(val_rm)}]:[TX:{int(val_tx)}]:"
                    f"[ERR:{int(val_err)}]:[STAT:{st}]:"
                    f"[RAW:{','.join(map(str, raws))}]:"
                    f"[ADJ:{','.join(map(str, adjs))}]:"
                    f"[frames:{count}]"
                )
            except Exception as e:
                # Suppress errors to keep streaming (SDO timeouts can happen if sensor is busy/disconnected)
                log.warning(f"Error polling SDOs: {e}")

            time.sleep(0.01)


if __name__ == "__main__":
    try:
        stream()
    except KeyboardInterrupt:
        log.info("Stream stopped by user.")


