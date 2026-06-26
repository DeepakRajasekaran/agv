/*
Name: MgsCanopen.cpp
Author: ANSCER Robotics
Date: 2026-06-24
Version: 2.0
Description: SocketCAN driver for MGS1600. Mirrors updated mgs_driver.py logic:
             SDO abort blacklisting, inter-SDO gap, echo validation, diagnostic
             group rotation, TPDO1 flag trust control.
*/

#include "mgs_hardware_interface/MgsCanopen.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace mgs_hardware_interface
{

// ── SDO protocol constants ──────────────────────────────────────────────────
static constexpr uint8_t SDO_READ_REQ = 0x40;
static constexpr uint8_t SDO_WRITE_1B = 0x2F;
static constexpr uint8_t SDO_CMD_ABORT = 0x80;
static constexpr uint8_t SDO_CMD_WRITE_CONFIRM = 0x60;

// ── Timing ──────────────────────────────────────────────────────────────────
static constexpr int SDO_TIMEOUT_MS = 150;
static constexpr int SDO_INTER_GAP_MS = 3;
static constexpr int POLL_PERIOD_MS = 120;

// ── Object Dictionary ───────────────────────────────────────────────────────
static constexpr uint16_t OD_LEFT_TRACK    = 0x211E;
static constexpr uint16_t OD_RIGHT_TRACK   = 0x211E;
static constexpr uint16_t OD_SELECTED_TRACK = 0x211E;
static constexpr uint16_t OD_DOMINANT_TRACK = 0x210F;
static constexpr uint16_t OD_TRACK_DETECT  = 0x211D;
static constexpr uint16_t OD_LEFT_MARKER   = 0x211F;
static constexpr uint16_t OD_RIGHT_MARKER  = 0x211F;
static constexpr uint16_t OD_TAPE_CROSS    = 0x2138;
static constexpr uint16_t OD_STATUS        = 0x2120;
static constexpr uint16_t OD_RAW_SENSOR    = 0x212D;
static constexpr uint16_t OD_ZERO_ADJ      = 0x212E;
static constexpr uint16_t OD_VAR_INT       = 0x2106;
static constexpr uint16_t OD_VAR_BOOL      = 0x2115;
static constexpr uint16_t OD_FOLLOW_LEFT   = 0x201A;
static constexpr uint16_t OD_FOLLOW_RIGHT  = 0x201B;
static constexpr uint16_t OD_SET_ZERO      = 0x2020;
static constexpr uint16_t OD_SAVE_CONFIG   = 0x2017;

// ── Helper: SDO payload size from cmd byte ──────────────────────────────────
static int sdoPayloadSize(uint8_t cmd)
{
  if ((cmd & 0xF0) != 0x40) return 0;
  if (!(cmd & 0x02)) return 0;
  if (!(cmd & 0x01)) return 4;
  return 4 - ((cmd >> 2) & 0x03);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════════════

MgsCanopen::MgsCanopen(const std::string & interface, int nodeId)
: m_interface(interface),
  m_nodeId(nodeId),
  m_socket(-1),
  m_running(false),
  m_sdoReceived(false),
  m_expectedIdx(0),
  m_expectedSub(0),
  m_diagGroup(0)
{
}

MgsCanopen::~MgsCanopen()
{
  stop();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

bool MgsCanopen::start()
{
  if (!openSocket()) {
    std::cerr << "[MgsCanopen] Failed to open SocketCAN on " << m_interface << std::endl;
    return false;
  }

  m_running = true;
  m_tRxLoop = std::thread(&MgsCanopen::rxLoop, this);
  m_tPollLoop = std::thread(&MgsCanopen::pollLoop, this);

  nmtStart();
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  return true;
}

void MgsCanopen::stop()
{
  if (m_running) {
    m_running = false;
    {
      std::lock_guard<std::mutex> lock(m_sdoMutex);
      m_sdoReceived = true;
      m_sdoCond.notify_all();
    }
    if (m_tRxLoop.joinable()) m_tRxLoop.join();
    if (m_tPollLoop.joinable()) m_tPollLoop.join();
    nmtStop();
    closeSocket();
  }
}

bool MgsCanopen::openSocket()
{
  m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_socket < 0) return false;

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, m_interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0) {
    closeSocket();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    closeSocket();
    return false;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Install CAN filters
  struct can_filter rfilter[4];
  rfilter[0] = {static_cast<canid_t>(0x580 + m_nodeId), CAN_SFF_MASK};  // SDO RX
  rfilter[1] = {static_cast<canid_t>(0x180 + m_nodeId), CAN_SFF_MASK};  // TPDO1
  rfilter[2] = {static_cast<canid_t>(0x280 + m_nodeId), CAN_SFF_MASK};  // TPDO2
  rfilter[3] = {static_cast<canid_t>(0x700 + m_nodeId), CAN_SFF_MASK};  // Heartbeat
  setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  return true;
}

void MgsCanopen::closeSocket()
{
  if (m_socket >= 0) {
    ::close(m_socket);
    m_socket = -1;
  }
}

void MgsCanopen::sendFrame(uint32_t canId, const uint8_t * data, uint8_t dlc)
{
  if (m_socket < 0) return;
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = canId & CAN_SFF_MASK;
  frame.can_dlc = dlc > 8 ? 8 : dlc;
  if (data) std::memcpy(frame.data, data, frame.can_dlc);
  write(m_socket, &frame, sizeof(frame));
}

// ═══════════════════════════════════════════════════════════════════════════
//  NMT
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::nmtStart()
{
  uint8_t data[2] = {0x01, static_cast<uint8_t>(m_nodeId)};
  sendFrame(0x000, data, 2);
}

void MgsCanopen::nmtStop()
{
  uint8_t data[2] = {0x02, static_cast<uint8_t>(m_nodeId)};
  sendFrame(0x000, data, 2);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Commands
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::followLeft()  { sdoWriteU8(OD_FOLLOW_LEFT,  0x00, 1); }
void MgsCanopen::followRight() { sdoWriteU8(OD_FOLLOW_RIGHT, 0x00, 1); }
void MgsCanopen::setZero()     { sdoWriteU8(OD_SET_ZERO,     0x00, 1); }
void MgsCanopen::saveConfig()  { sdoWriteU8(OD_SAVE_CONFIG,  0x00, 1); }

void MgsCanopen::getSnapshot(MgsState & state)
{
  std::lock_guard<std::mutex> lock(m_stateMutex);
  state = m_state;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SDO Inter-gap (3ms between consecutive SDO requests)
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::sdoGap()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(SDO_INTER_GAP_MS));
}

// ═══════════════════════════════════════════════════════════════════════════
//  SDO Read with echo validation + abort blacklisting
// ═══════════════════════════════════════════════════════════════════════════

bool MgsCanopen::sdoRead(uint16_t index, uint8_t subindex, int32_t & value, uint8_t size)
{
  // Skip objects that the device has previously aborted on
  auto key = std::make_pair(index, subindex);
  if (m_abortedObjects.count(key)) return false;

  uint8_t data[8] = {0};
  data[0] = SDO_READ_REQ;
  data[1] = index & 0xFF;
  data[2] = (index >> 8) & 0xFF;
  data[3] = subindex;

  std::unique_lock<std::mutex> lock(m_sdoMutex);
  m_sdoReceived = false;
  m_sdoResponseData.clear();
  m_expectedIdx = index;
  m_expectedSub = subindex;
  sendFrame(0x600 + m_nodeId, data, 8);

  if (m_sdoCond.wait_for(lock, std::chrono::milliseconds(SDO_TIMEOUT_MS),
                         [this]{ return m_sdoReceived; }))
  {
    // Check for abort sentinel
    if (m_sdoResponseData.size() == 1 && m_sdoResponseData[0] == 0xFF) {
      m_abortedObjects.insert(key);
      return false;
    }

    if (m_sdoResponseData.size() >= static_cast<size_t>(size)) {
      value = 0;
      for (int i = 0; i < size; ++i) {
        value |= (static_cast<int32_t>(m_sdoResponseData[i]) << (8 * i));
      }
      if (size == 1) value = static_cast<int8_t>(value & 0xFF);
      else if (size == 2) value = static_cast<int16_t>(value & 0xFFFF);
      return true;
    }
  }
  return false;
}

void MgsCanopen::sdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value)
{
  uint8_t data[8] = {SDO_WRITE_1B,
                     static_cast<uint8_t>(index & 0xFF),
                     static_cast<uint8_t>((index >> 8) & 0xFF),
                     subindex, value, 0, 0, 0};
  sendFrame(0x600 + m_nodeId, data, 8);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Query Methods (matching Python driver structure)
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::queryNav()
{
  int32_t val = 0;
  bool use_sdo_pos = true;

  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (m_state.tpdo1Count > 10) use_sdo_pos = false;
  }

  if (use_sdo_pos) {
    if (sdoRead(OD_LEFT_TRACK,     0x01, val, 2)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.leftTrack = val;
    }
    sdoGap();
    if (sdoRead(OD_RIGHT_TRACK,    0x02, val, 2)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.rightTrack = val;
    }
    sdoGap();
  }

  if (sdoRead(OD_SELECTED_TRACK, 0x03, val, 2)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.selectedTrack = val;
  }
  sdoGap();
  if (sdoRead(OD_DOMINANT_TRACK, 0x00, val, 1)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.dominantTrack = val;
  }
  sdoGap();
  if (sdoRead(OD_TRACK_DETECT,   0x01, val, 1)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.trackDetect = (val != 0);
  }
  sdoGap();
  if (sdoRead(OD_LEFT_MARKER,    0x01, val, 1)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.leftMarker = ((val & 0x01) != 0);
    m_state.rightMarker = ((val & 0x02) != 0);
  }
  sdoGap();
  if (sdoRead(OD_TAPE_CROSS,     0x01, val, 1)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.tapeCross = (val != 0);
  }
  sdoGap();
  if (sdoRead(OD_STATUS, 0x01, val, 2)) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.status = val;
    m_state.sensorFailure = (val & 0x0100) != 0;
  }
}

void MgsCanopen::queryDiagGroup(int group)
{
  int startCh, endCh, startU, endU;
  if (group == 0) { startCh = 1; endCh = 4; startU = 1; endU = 3; }
  else if (group == 1) { startCh = 5; endCh = 7; startU = 4; endU = 7; }
  else { startCh = 8; endCh = 10; startU = 8; endU = 10; }

  int32_t val = 0;
  for (int ch = startCh; ch <= endCh; ++ch) {
    if (sdoRead(OD_RAW_SENSOR, ch, val, 4)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.rawSensor[ch - 1] = val;
    }
    sdoGap();
    if (sdoRead(OD_ZERO_ADJ, ch, val, 4)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.zeroAdjSensor[ch - 1] = val;
    }
    sdoGap();
  }
  for (int u = startU; u <= endU; ++u) {
    if (sdoRead(OD_VAR_INT, u, val, 4)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.userInts[u - 1] = val;
    }
    sdoGap();
    if (sdoRead(OD_VAR_BOOL, u, val, 1)) {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.userBools[u - 1] = (val != 0);
    }
    sdoGap();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Poll Loop (nav sweep + one diagnostic group per cycle)
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::pollLoop()
{
  while (m_running) {
    auto t0 = std::chrono::steady_clock::now();

    queryNav();
    sdoGap();
    queryDiagGroup(m_diagGroup);
    m_diagGroup = (m_diagGroup + 1) % 3;

    auto elapsed = std::chrono::steady_clock::now() - t0;
    int elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    int sleepMs = POLL_PERIOD_MS - elapsedMs;
    if (sleepMs > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  RX Loop & Dispatch
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::rxLoop()
{
  struct can_frame frame;
  while (m_running) {
    int nbytes = read(m_socket, &frame, sizeof(struct can_frame));
    if (nbytes > 0) {
      dispatch(frame.can_id & CAN_SFF_MASK, frame.data, frame.can_dlc);
    }
  }
}

void MgsCanopen::dispatch(uint32_t canId, const uint8_t * data, uint8_t dlc)
{
  uint32_t nid = static_cast<uint32_t>(m_nodeId);
  if      (canId == 0x180 + nid) parseTpdo1(data, dlc);
  else if (canId == 0x580 + nid) handleSdoResponse(data, dlc);
  else if (canId == 0x700 + nid) handleHeartbeat(data, dlc);
}

// ═══════════════════════════════════════════════════════════════════════════
//  TPDO1 — Only track positions decoded (flags NOT trusted per Python driver)
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::parseTpdo1(const uint8_t * data, uint8_t dlc)
{
  if (dlc < 4) return;
  int16_t left  = static_cast<int16_t>(data[0] | (data[1] << 8));
  int16_t right = static_cast<int16_t>(data[2] | (data[3] << 8));

  std::lock_guard<std::mutex> lock(m_stateMutex);
  m_state.leftTrack = left;
  m_state.rightTrack = right;
  m_state.tpdo1Count++;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SDO Response Handler with echo validation + abort detection
// ═══════════════════════════════════════════════════════════════════════════

void MgsCanopen::handleSdoResponse(const uint8_t * data, uint8_t dlc)
{
  if (dlc < 8) return;

  uint8_t cmdByte = data[0];
  uint16_t respIdx = data[1] | (data[2] << 8);
  uint8_t respSub = data[3];

  // Write confirm — ignore
  if (cmdByte == SDO_CMD_WRITE_CONFIRM) return;

  // Abort
  if (cmdByte == SDO_CMD_ABORT) {
    if (respIdx == m_expectedIdx && respSub == m_expectedSub) {
      std::lock_guard<std::mutex> lock(m_sdoMutex);
      m_sdoResponseData = {0xFF};  // abort sentinel
      m_sdoReceived = true;
      m_sdoCond.notify_all();
    }
    return;
  }

  // Upload response — validate echo
  int size = sdoPayloadSize(cmdByte);
  if (size == 0) return;

  if (respIdx == m_expectedIdx && respSub == m_expectedSub) {
    std::lock_guard<std::mutex> lock(m_sdoMutex);
    m_sdoResponseData.assign(data + 4, data + 4 + size);
    m_sdoReceived = true;
    m_sdoCond.notify_all();
  }
}

void MgsCanopen::handleHeartbeat(const uint8_t * data, uint8_t dlc)
{
  if (dlc < 1) return;
  const char * states[] = {"Boot-up", "", "", "", "Stopped", "Operational"};
  std::string s;
  if (data[0] <= 5) s = states[data[0]];
  else if (data[0] == 0x7F) s = "Pre-Op";
  else s = "Unknown";

  std::lock_guard<std::mutex> lock(m_stateMutex);
  m_state.nmtState = s;
}

} // namespace mgs_hardware_interface
