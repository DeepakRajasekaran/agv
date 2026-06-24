#!/usr/bin/env python3
"""
MGS1600 CANopen Driver for NVIDIA Jetson
=========================================
Sensor   : Roboteq MGS1600 Magnetic Guide Sensor
Interface: SocketCAN (can0)
Protocol : CANopen (firmware v3.0+)
Node ID  : 5
Bitrate  : 500 kbps
Datasheet: MGS1600 Magnetic Sensor Datasheet v1.4, July 14, 2023
"""

import socket
import struct
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple, List, Set

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("MGS1600")


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

CAN_INTERFACE    = "can0"
NODE_ID          = 5
BITRATE          = 500_000

SDO_TIMEOUT_S    = 0.15      # per-request timeout
SDO_INTER_GAP_S  = 0.003     # 3ms gap between consecutive SDO requests
LOOP_PERIOD_S    = 0.12      # ~8Hz dashboard refresh

# TPDO1 flag decoding: set True ONLY after verifying bit positions with
# Roboteq TrackSensor utility for your firmware version.
TPDO1_FLAGS_TRUSTED     = False
TPDO1_MASK_TAPE_DETECT  = 0x0001
TPDO1_MASK_LEFT_MARKER  = 0x0002
TPDO1_MASK_RIGHT_MARKER = 0x0004
TPDO1_MASK_TAPE_CROSS   = 0x0008
TPDO1_MASK_SENSOR_FAIL  = 0x0080

NUM_SENSOR_CHANNELS = 10     # OD 0x212D / 0x212E sub 01-10
NUM_USER_VARS       = 10     # OD 0x2106 / 0x2115 sub 01-10

# SDO abort codes (common ones from CiA 301)
SDO_ABORT_CODES: Dict[int, str] = {
    0x05040000: "SDO protocol timed out",
    0x05040001: "Invalid/unknown client cmd",
    0x06010000: "Unsupported access",
    0x06010001: "Read-only object",
    0x06010002: "Write-only object",
    0x06020000: "Object does not exist",
    0x06040041: "Object cannot be PDO-mapped",
    0x06040042: "PDO length exceeded",
    0x06040043: "General parameter incompatibility",
    0x06070010: "Data type mismatch (size)",
    0x06070012: "Data type mismatch (high)",
    0x06070013: "Data type mismatch (low)",
    0x06090011: "Sub-index does not exist",
    0x06090030: "Value range exceeded",
    0x06090031: "Value too high",
    0x06090032: "Value too low",
    0x08000000: "General error",
    0x08000020: "Data cannot be transferred",
    0x08000021: "Data cannot be transferred (local)",
    0x08000022: "Data cannot be transferred (state)",
}


# ══════════════════════════════════════════════════════════════════════════════
#  CANopen COB-IDs & Protocol Constants
# ══════════════════════════════════════════════════════════════════════════════

NMT_COB_ID       = 0x000
SYNC_COB_ID      = 0x080
TPDO1_COB_ID     = 0x180 + NODE_ID
TPDO2_COB_ID     = 0x280 + NODE_ID
SDO_TX_COB_ID    = 0x600 + NODE_ID   # Host → Device
SDO_RX_COB_ID    = 0x580 + NODE_ID   # Device → Host
HEARTBEAT_COB_ID = 0x700 + NODE_ID

NMT_START      = 0x01
NMT_STOP       = 0x02
NMT_PRE_OP     = 0x80
NMT_RESET_NODE = 0x81
NMT_RESET_COMM = 0x82

SDO_READ_REQ   = 0x40
SDO_WRITE_1B   = 0x2F
SDO_WRITE_2B   = 0x2B
SDO_WRITE_4B   = 0x23

SDO_CMD_ABORT        = 0x80
SDO_CMD_WRITE_CONFIRM = 0x60


# ══════════════════════════════════════════════════════════════════════════════
#  OBJECT DICTIONARY
# ══════════════════════════════════════════════════════════════════════════════

OD: Dict[str, Tuple[int, int]] = {
    # Runtime Queries (RO)
    "VAR_INT":        (0x2106, 0x00),  # S32 RO sub 01-10
    "DOMINANT_TRACK": (0x210F, 0x00),  # S8  RO
    "VAR_BOOL":       (0x2115, 0x00),  # U8  RO sub 01-10
    "TRACK_DETECT":   (0x211D, 0x01),  # U8  RO
    "LEFT_TRACK":     (0x211E, 0x01),  # S16 RO
    "RIGHT_TRACK":    (0x211E, 0x02),  # S16 RO
    "SELECTED_TRACK": (0x211E, 0x03),  # S16 RO
    "LEFT_MARKER":    (0x211F, 0x01),  # U8  RO
    "RIGHT_MARKER":   (0x211F, 0x02),  # U8  RO
    "STATUS":         (0x2120, 0x01),  # U16 RO
    "RAW_SENSOR":     (0x212D, 0x00),  # U32 RO sub 01-10
    "ZERO_ADJ":       (0x212E, 0x00),  # S32 RO sub 01-10
    "TAPE_CROSS":     (0x2138, 0x01),  # U8  RO
    # Runtime Commands (WO)
    "VAR_INT_W":      (0x2005, 0x00),  # S32 WO sub 01-10
    "VAR_BOOL_W":     (0x2015, 0x00),  # U8  WO sub 01-32
    "SAVE_CONFIG":    (0x2017, 0x00),  # U8  WO
    "MICROBASIC_RUN": (0x2018, 0x00),  # U8  WO
    "FOLLOW_LEFT":    (0x201A, 0x00),  # U8  WO
    "FOLLOW_RIGHT":   (0x201B, 0x00),  # U8  WO
    "SET_ZERO":       (0x2020, 0x00),  # U8  WO
}


# ══════════════════════════════════════════════════════════════════════════════
#  DATA STRUCTURES
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SensorState:
    left_track:      int  = 0
    right_track:     int  = 0
    selected_track:  int  = 0
    dominant_track:  int  = 0

    tape_detect:     bool = False
    tape_cross:      bool = False
    left_marker:     bool = False
    right_marker:    bool = False
    sensor_failure:  bool = False

    raw_sensors:      List[int]  = field(default_factory=lambda: [0] * NUM_SENSOR_CHANNELS)
    zero_adj_sensors: List[int]  = field(default_factory=lambda: [0] * NUM_SENSOR_CHANNELS)
    user_ints:        List[int]  = field(default_factory=lambda: [0] * NUM_USER_VARS)
    user_bools:       List[bool] = field(default_factory=lambda: [False] * NUM_USER_VARS)

    status_word:     int  = 0
    nmt_state:       str  = "Unknown"
    timestamp:       float = field(default_factory=time.time)
    tpdo1_count:     int  = 0
    tpdo2_count:     int  = 0

    def __str__(self) -> str:
        age_ms = (time.time() - self.timestamp) * 1000
        return (
            f"L={self.left_track:+5d}mm  R={self.right_track:+5d}mm  |  "
            f"TD={'Y' if self.tape_detect  else 'N'}  "
            f"LM={'Y' if self.left_marker  else 'N'}  "
            f"RM={'Y' if self.right_marker else 'N'}  "
            f"TX={'Y' if self.tape_cross   else 'N'}  "
            f"ERR={'!' if self.sensor_failure else '-'}  "
            f"NMT={self.nmt_state}  ({age_ms:.0f}ms ago)"
        )


# ══════════════════════════════════════════════════════════════════════════════
#  SocketCAN Helpers
# ══════════════════════════════════════════════════════════════════════════════

_FRAME_FMT  = "=IB3x8s"
_FRAME_SIZE = struct.calcsize(_FRAME_FMT)


def _build_frame(can_id: int, data: bytes) -> bytes:
    data = data[:8].ljust(8, b"\x00")
    return struct.pack(_FRAME_FMT, can_id & 0x1FFFFFFF, len(data), data)


def _parse_frame(raw: bytes) -> Tuple[int, int, bytes]:
    can_id, dlc, data = struct.unpack(_FRAME_FMT, raw)
    return can_id & 0x1FFFFFFF, dlc, data[:dlc]


def _sdo_payload_size(cmd_byte: int) -> int:
    """
    CANopen expedited upload response payload size from byte-0.
    Valid codes: 0x4F(1B) 0x4B(2B) 0x47(3B) 0x43(4B).
    Upper nibble must be 0x4, bit1 must be set (expedited).
    """
    if (cmd_byte & 0xF0) != 0x40:
        return 0
    if not (cmd_byte & 0x02):
        return 0
    if not (cmd_byte & 0x01):
        return 4
    return 4 - ((cmd_byte >> 2) & 0x03)


# ══════════════════════════════════════════════════════════════════════════════
#  MGS1600 CANopen DRIVER
# ══════════════════════════════════════════════════════════════════════════════

# Sentinel value: stored in _sdo_response to distinguish "device sent abort"
# from "no response at all (timeout)".
_SDO_ABORT_SENTINEL = b"__ABORT__"


class MGS1600CANopen:

    def __init__(self, interface: str = CAN_INTERFACE, node_id: int = NODE_ID):
        self.interface = interface
        self.node_id   = node_id
        self.state     = SensorState()

        self._sock:       Optional[socket.socket]    = None
        self._rx_thread:  Optional[threading.Thread]  = None
        self._running     = False
        self._lock        = threading.Lock()           # guards SensorState

        self._sdo_lock    = threading.Lock()           # serialises SDO requests
        self._sdo_event   = threading.Event()
        self._sdo_response: Optional[bytes] = None
        self._expected_idx: int = 0
        self._expected_sub: int = 0

        # Track which (index, subindex) pairs the device aborted on.
        # These are skipped in future queries to avoid wasting bus time.
        self._aborted_objects: Set[Tuple[int, int]] = set()

        # Diagnostic rotation: 0, 1, 2
        self._diag_group: int = 0

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((self.interface,))
        self._sock.settimeout(0.5)
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="CAN-RX"
        )
        self._rx_thread.start()
        log.info(f"SocketCAN open: iface={self.interface}  node={self.node_id}")

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
            raise RuntimeError("Not connected.")
        self._sock.send(_build_frame(can_id, data))

    # ── NMT ───────────────────────────────────────────────────────────────────

    def nmt_start(self) -> None:
        self._send(NMT_COB_ID, bytes([NMT_START, self.node_id]))
        log.info(f"NMT → Operational (node {self.node_id})")

    def nmt_stop(self) -> None:
        self._send(NMT_COB_ID, bytes([NMT_STOP, self.node_id]))
        log.info(f"NMT → Stopped (node {self.node_id})")

    def nmt_pre_operational(self) -> None:
        self._send(NMT_COB_ID, bytes([NMT_PRE_OP, self.node_id]))
        log.info(f"NMT → Pre-Operational (node {self.node_id})")

    def nmt_reset(self) -> None:
        self._send(NMT_COB_ID, bytes([NMT_RESET_NODE, self.node_id]))
        log.info(f"NMT → Reset (node {self.node_id})")

    # ── SDO Read ──────────────────────────────────────────────────────────────

    def sdo_read(self, index: int, subindex: int) -> Optional[bytes]:
        """
        Blocking SDO upload request with:
        - Index/subindex echo validation (rejects stale responses)
        - Abort detection (device says object doesn't exist → immediate None
          return instead of 150ms timeout)
        - Auto-blacklist: objects that abort are skipped in future queries
        """
        # Skip objects the device has previously aborted on
        key = (index, subindex)
        if key in self._aborted_objects:
            return None

        req = struct.pack("<BHBB4x", SDO_READ_REQ, index, subindex, 0)
        with self._sdo_lock:
            self._sdo_event.clear()
            self._sdo_response = None
            self._expected_idx = index
            self._expected_sub = subindex
            self._send(SDO_TX_COB_ID, req)
            ok = self._sdo_event.wait(SDO_TIMEOUT_S)

        if not ok:
            log.warning(f"SDO timeout: 0x{index:04X}/0x{subindex:02X}")
            return None

        resp = self._sdo_response
        if resp is _SDO_ABORT_SENTINEL:
            # Device explicitly rejected this object — blacklist it
            self._aborted_objects.add(key)
            return None

        return resp

    # ── SDO Write ─────────────────────────────────────────────────────────────

    def sdo_write_u8(self, index: int, subindex: int, value: int) -> None:
        self._send(
            SDO_TX_COB_ID,
            struct.pack("<BHBB4x", SDO_WRITE_1B, index, subindex, value & 0xFF),
        )

    def sdo_write_u16(self, index: int, subindex: int, value: int) -> None:
        self._send(
            SDO_TX_COB_ID,
            struct.pack("<BHBH2x", SDO_WRITE_2B, index, subindex, value & 0xFFFF),
        )

    def sdo_write_u32(self, index: int, subindex: int, value: int) -> None:
        self._send(
            SDO_TX_COB_ID,
            struct.pack("<BHBI", SDO_WRITE_4B, index, subindex, value & 0xFFFFFFFF),
        )

    # ── Typed SDO primitives ──────────────────────────────────────────────────

    def _rd_s8(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return struct.unpack("<b", r[:1])[0] if r else None

    def _rd_u8(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return r[0] if r else None

    def _rd_s16(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return struct.unpack("<h", r[:2])[0] if r and len(r) >= 2 else None

    def _rd_u16(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return struct.unpack("<H", r[:2])[0] if r and len(r) >= 2 else None

    def _rd_s32(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return struct.unpack("<i", r[:4])[0] if r and len(r) >= 4 else None

    def _rd_u32(self, idx: int, sub: int) -> Optional[int]:
        r = self.sdo_read(idx, sub)
        return struct.unpack("<I", r[:4])[0] if r and len(r) >= 4 else None

    def _gap(self) -> None:
        """Inter-SDO gap to avoid overloading device SDO server."""
        time.sleep(SDO_INTER_GAP_S)

    # ── Query methods ─────────────────────────────────────────────────────────

    def query_left_track(self) -> Optional[int]:
        v = self._rd_s16(*OD["LEFT_TRACK"])
        if v is not None:
            with self._lock:
                self.state.left_track = v
        return v

    def query_right_track(self) -> Optional[int]:
        v = self._rd_s16(*OD["RIGHT_TRACK"])
        if v is not None:
            with self._lock:
                self.state.right_track = v
        return v

    def query_selected_track(self) -> Optional[int]:
        v = self._rd_s16(*OD["SELECTED_TRACK"])
        if v is not None:
            with self._lock:
                self.state.selected_track = v
        return v

    def query_dominant_track(self) -> Optional[int]:
        v = self._rd_s8(*OD["DOMINANT_TRACK"])
        if v is not None:
            with self._lock:
                self.state.dominant_track = v
        return v

    def query_track_detect(self) -> Optional[bool]:
        v = self._rd_u8(*OD["TRACK_DETECT"])
        if v is not None:
            with self._lock:
                self.state.tape_detect = bool(v)
        return bool(v) if v is not None else None

    def query_left_marker(self) -> Optional[bool]:
        v = self._rd_u8(*OD["LEFT_MARKER"])
        if v is not None:
            with self._lock:
                self.state.left_marker = bool(v)
        return bool(v) if v is not None else None

    def query_right_marker(self) -> Optional[bool]:
        v = self._rd_u8(*OD["RIGHT_MARKER"])
        if v is not None:
            with self._lock:
                self.state.right_marker = bool(v)
        return bool(v) if v is not None else None

    def query_tape_cross(self) -> Optional[bool]:
        v = self._rd_u8(*OD["TAPE_CROSS"])
        if v is not None:
            with self._lock:
                self.state.tape_cross = bool(v)
        return bool(v) if v is not None else None

    def query_status(self) -> Optional[int]:
        v = self._rd_u16(*OD["STATUS"])
        if v is not None:
            with self._lock:
                self.state.status_word    = v
                self.state.sensor_failure = bool(v & 0x0100)
        return v

    def query_raw_sensor(self, n: int) -> Optional[int]:
        v = self._rd_u32(OD["RAW_SENSOR"][0], n)
        if v is not None:
            with self._lock:
                self.state.raw_sensors[n - 1] = v
        return v

    def query_zero_adjusted_sensor(self, n: int) -> Optional[int]:
        v = self._rd_s32(OD["ZERO_ADJ"][0], n)
        if v is not None:
            with self._lock:
                self.state.zero_adj_sensors[n - 1] = v
        return v

    def query_user_int(self, n: int) -> Optional[int]:
        v = self._rd_s32(OD["VAR_INT"][0], n)
        if v is not None:
            with self._lock:
                self.state.user_ints[n - 1] = v
        return v

    def query_user_bool(self, n: int) -> Optional[bool]:
        v = self._rd_u8(OD["VAR_BOOL"][0], n)
        if v is not None:
            with self._lock:
                self.state.user_bools[n - 1] = bool(v)
        return bool(v) if v is not None else None

    # ── Composite sweep methods ───────────────────────────────────────────────

    def _query_nav(self) -> None:
        """9 navigation-critical SDOs — polled every cycle."""
        self.query_left_track();      self._gap()
        self.query_right_track();     self._gap()
        self.query_selected_track();  self._gap()
        self.query_dominant_track();  self._gap()
        self.query_track_detect();    self._gap()
        self.query_left_marker();     self._gap()
        self.query_right_marker();    self._gap()
        self.query_tape_cross();      self._gap()
        self.query_status()

    def _query_diag_group(self, group: int) -> None:
        """
        Diagnostic SDOs split across 3 rotation groups (~13 each).
        Completes full diagnostic coverage every 3 cycles.
        Aborted objects are auto-skipped (instant None return, no bus time).
        """
        if group == 0:
            channels = range(1, 5)    # sensor 1-4
            users    = range(1, 4)    # user 1-3
        elif group == 1:
            channels = range(5, 8)    # sensor 5-7
            users    = range(4, 8)    # user 4-7
        else:
            channels = range(8, 11)   # sensor 8-10
            users    = range(8, 11)   # user 8-10

        for ch in channels:
            self.query_raw_sensor(ch);           self._gap()
            self.query_zero_adjusted_sensor(ch); self._gap()
        for u in users:
            self.query_user_int(u);  self._gap()
            self.query_user_bool(u); self._gap()

    def query_all(self) -> None:
        """Nav sweep + one diagnostic group per cycle."""
        self._query_nav()
        self._gap()
        self._query_diag_group(self._diag_group)
        self._diag_group = (self._diag_group + 1) % 3

    # ── Command helpers ───────────────────────────────────────────────────────

    def cmd_follow_left(self)  -> None: self.sdo_write_u8(*OD["FOLLOW_LEFT"],  1)
    def cmd_follow_right(self) -> None: self.sdo_write_u8(*OD["FOLLOW_RIGHT"], 1)
    def cmd_set_zero(self)     -> None: self.sdo_write_u8(*OD["SET_ZERO"],     1)
    def cmd_save_config(self)  -> None: self.sdo_write_u8(*OD["SAVE_CONFIG"],  1)

    def cmd_set_user_int(self, n: int, value: int) -> None:
        self.sdo_write_u32(OD["VAR_INT_W"][0], n, value & 0xFFFFFFFF)

    def cmd_set_user_bool(self, n: int, value: bool) -> None:
        self.sdo_write_u8(OD["VAR_BOOL_W"][0], n, int(value))

    # ── RX loop & dispatch ────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        while self._running:
            try:
                raw = self._sock.recv(_FRAME_SIZE)
                can_id, dlc, data = _parse_frame(raw)
                self._dispatch(can_id, data)
            except socket.timeout:
                continue
            except OSError:
                break

    def _dispatch(self, can_id: int, data: bytes) -> None:
        if   can_id == TPDO1_COB_ID:     self._parse_tpdo1(data)
        elif can_id == TPDO2_COB_ID:     self._parse_tpdo2(data)
        elif can_id == SDO_RX_COB_ID:    self._handle_sdo_response(data)
        elif can_id == HEARTBEAT_COB_ID: self._handle_heartbeat(data)

    # ── TPDO parsers ─────────────────────────────────────────────────────────

    def _parse_tpdo1(self, data: bytes) -> None:
        """
        Only track positions (bytes 0-3) are decoded.
        Flag bits in bytes 4-5 are NOT decoded unless TPDO1_FLAGS_TRUSTED
        is True — unverified bit positions corrupt marker/detect state.
        """
        if len(data) < 4:
            return
        left_track  = struct.unpack_from("<h", data, 0)[0]
        right_track = struct.unpack_from("<h", data, 2)[0]
        with self._lock:
            self.state.left_track  = left_track
            self.state.right_track = right_track
            self.state.timestamp   = time.time()
            self.state.tpdo1_count += 1

        if TPDO1_FLAGS_TRUSTED and len(data) >= 6:
            flags = struct.unpack_from("<H", data, 4)[0]
            with self._lock:
                self.state.tape_detect    = bool(flags & TPDO1_MASK_TAPE_DETECT)
                self.state.left_marker    = bool(flags & TPDO1_MASK_LEFT_MARKER)
                self.state.right_marker   = bool(flags & TPDO1_MASK_RIGHT_MARKER)
                self.state.tape_cross     = bool(flags & TPDO1_MASK_TAPE_CROSS)
                self.state.sensor_failure = bool(flags & TPDO1_MASK_SENSOR_FAIL)

    def _parse_tpdo2(self, data: bytes) -> None:
        if len(data) < 8:
            return
        v1 = struct.unpack_from("<i", data, 0)[0]
        v2 = struct.unpack_from("<i", data, 4)[0]
        with self._lock:
            self.state.user_ints[0] = v1
            self.state.user_ints[1] = v2
            self.state.tpdo2_count += 1

    # ── SDO response handler ──────────────────────────────────────────────────

    def _handle_sdo_response(self, data: bytes) -> None:
        """
        Processes all SDO responses from the device:
        - Upload response (0x4F/0x4B/0x47/0x43): extracts payload, validates
          echoed index+subindex, resolves pending request.
        - Abort (0x80): if index+subindex matches pending request, resolves
          with _SDO_ABORT_SENTINEL (instant failure instead of 150ms timeout).
        - Write confirm (0x60): ignored (fire-and-forget writes).
        """
        if len(data) < 8:
            return

        cmd_byte = data[0]
        resp_idx = struct.unpack_from("<H", data, 1)[0]
        resp_sub = data[3]

        # ── Write-confirm: nothing to do ──────────────────────────────────
        if cmd_byte == SDO_CMD_WRITE_CONFIRM:
            return

        # ── Abort: device rejected the request ───────────────────────────
        if cmd_byte == SDO_CMD_ABORT:
            abort_code = struct.unpack_from("<I", data, 4)[0]
            reason = SDO_ABORT_CODES.get(abort_code, f"0x{abort_code:08X}")
            if (resp_idx == self._expected_idx and
                    resp_sub == self._expected_sub):
                log.info(
                    f"SDO abort: 0x{resp_idx:04X}/0x{resp_sub:02X} → {reason}"
                )
                self._sdo_response = _SDO_ABORT_SENTINEL
                self._sdo_event.set()
            else:
                log.debug(
                    f"SDO abort (stale): 0x{resp_idx:04X}/0x{resp_sub:02X}"
                )
            return

        # ── Upload response ───────────────────────────────────────────────
        size = _sdo_payload_size(cmd_byte)
        if size == 0:
            log.debug(f"SDO: unknown cmd byte 0x{cmd_byte:02X}")
            return

        if (resp_idx == self._expected_idx and
                resp_sub == self._expected_sub):
            self._sdo_response = data[4: 4 + size]
            self._sdo_event.set()
        else:
            log.debug(
                f"SDO stale: got 0x{resp_idx:04X}/0x{resp_sub:02X} "
                f"expected 0x{self._expected_idx:04X}/0x{self._expected_sub:02X}"
            )

    def _handle_heartbeat(self, data: bytes) -> None:
        states = {
            0x00: "Boot-up",
            0x04: "Stopped",
            0x05: "Operational",
            0x7F: "Pre-Op",
        }
        with self._lock:
            self.state.nmt_state = states.get(data[0], f"0x{data[0]:02X}")


# ══════════════════════════════════════════════════════════════════════════════
#  TERMINAL DASHBOARD
# ══════════════════════════════════════════════════════════════════════════════

def render_dashboard(sensor: MGS1600CANopen, cycle: int, elapsed: float) -> None:
    with sensor._lock:
        st  = sensor.state
        raw = list(st.raw_sensors)
        adj = list(st.zero_adj_sensors)

    hz = cycle / elapsed if elapsed > 0 else 0.0
    n_skip = len(sensor._aborted_objects)

    print("\033[H\033[J", end="")
    print("=" * 72)
    print(
        f" MGS1600 TELEMETRY  [Cycle {cycle}]  [{hz:.1f} Hz]  "
        f"Aborted ODs: {n_skip}  Ctrl+C to stop"
    )
    print("=" * 72)
    print(f"\n  {st}")
    print("-" * 72)
    print(" SDO VALUES (authoritative)")
    print(
        f"  Tracks  : L={st.left_track:+5d}mm  R={st.right_track:+5d}mm  "
        f"Sel={st.selected_track:+5d}mm  Dom={st.dominant_track}"
    )
    print(
        f"  Flags   : Detect={'Y' if st.tape_detect else 'N'}  "
        f"Cross={'Y' if st.tape_cross else 'N'}  "
        f"LMarker={'Y' if st.left_marker else 'N'}  "
        f"RMarker={'Y' if st.right_marker else 'N'}  "
        f"Fault={'Y' if st.sensor_failure else 'N'}"
    )
    print(
        f"  Status  : 0x{st.status_word:04X}  "
        f"TPDO1={st.tpdo1_count}  TPDO2={st.tpdo2_count}"
    )
    print(f"  UserInt : {st.user_ints}")
    print(f"  UserBool: {[int(b) for b in st.user_bools]}")

    if n_skip > 0:
        skipped = ", ".join(
            f"0x{idx:04X}/{sub:02X}" for idx, sub in sorted(sensor._aborted_objects)
        )
        print(f"  Skipped : {skipped}")

    print("-" * 72)
    print(
        f" HALL-EFFECT ARRAY ({NUM_SENSOR_CHANNELS} ch)  "
        f"[group {sensor._diag_group} next]"
    )
    max_v = max(raw) if max(raw) > 0 else 1
    for i in range(NUM_SENSOR_CHANNELS):
        bar = "█" * int((raw[i] / max_v) * 20)
        print(f"  CH{i+1:02d}: {bar:<20} raw={raw[i]:<6} adj={adj[i]}")
    print("=" * 72)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════════════════════

def run_continuous_driver() -> None:
    print("Initialising MGS1600 CANopen driver…")
    cycle   = 0
    t_start = time.monotonic()

    with MGS1600CANopen(CAN_INTERFACE, NODE_ID) as sensor:
        sensor.nmt_start()
        time.sleep(0.4)

        try:
            while True:
                t0 = time.monotonic()
                sensor.query_all()
                cycle += 1
                render_dashboard(sensor, cycle, time.monotonic() - t_start)

                # Adaptive sleep: maintain target cadence
                sleep_s = LOOP_PERIOD_S - (time.monotonic() - t0)
                if sleep_s > 0:
                    time.sleep(sleep_s)

        except KeyboardInterrupt:
            print("\n[INFO] Shutdown requested — stopping NMT node…")


if __name__ == "__main__":
    run_continuous_driver()
