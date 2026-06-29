/*
 * Name:        RoboteqCanDriver.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-23
 * Version:     2.0
 * Description: Implements the RoboteqCanDriver class for CANOpen SDO Polling.
 */

#include "roboteq_driver/RoboteqCanDriver.h"

#include <cstring>
#include <cerrno>
#include <cassert>
#include <cmath>
#include <chrono>
#include <iostream>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace roboteq_driver
{

static double now_sec()
{
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration<double>(duration).count();
}

RoboteqCanDriver::RoboteqCanDriver(const std::string& can_interface, int node_id)
  : m_canInterface(can_interface)
  , m_nodeId(node_id)
  , m_socket(-1)
  , m_running(false)
  , m_targetSpeedLeft(0)
  , m_targetSpeedRight(0)
  , m_lastUpdateTime(0.0)
  , m_timeoutSeconds(0.5)
{
}

RoboteqCanDriver::~RoboteqCanDriver()
{
  stop();
  disconnect();
}

bool RoboteqCanDriver::connect()
{
  assert(m_socket == -1);

  m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_socket < 0) return false;

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, m_canInterface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0)
  {
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 50000;
  setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (bind(m_socket, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  return true;
}

void RoboteqCanDriver::disconnect()
{
  if (m_socket >= 0)
  {
    ::close(m_socket);
    m_socket = -1;
  }
}

void RoboteqCanDriver::start()
{
  assert(m_socket >= 0);
  assert(!m_running);

  m_running = true;
  m_lastUpdateTime = now_sec();

  m_rxThread = std::thread(&RoboteqCanDriver::rxThread, this);
  m_sdoThread = std::thread(&RoboteqCanDriver::sdoLoop, this);
}

void RoboteqCanDriver::stop()
{
  if (m_running)
  {
    m_running = false;
    if (m_rxThread.joinable()) m_rxThread.join();
    if (m_sdoThread.joinable()) m_sdoThread.join();
  }
}

TelemetryData RoboteqCanDriver::getTelemetry()
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  return m_telemetry;
}

bool RoboteqCanDriver::isConnected() const
{
  return (now_sec() - m_lastUpdateTime.load()) < m_timeoutSeconds;
}

void RoboteqCanDriver::sendSpeedCommand(int32_t left_rpm, int32_t right_rpm)
{
  std::lock_guard<std::mutex> lock(m_cmdMutex);
  m_targetSpeedLeft = left_rpm;
  m_targetSpeedRight = right_rpm;
}

void RoboteqCanDriver::resetEncoders()
{
  sendSdoWrite(0x2003, 1, 0, 4);
  sendSdoWrite(0x2003, 2, 0, 4);
}

void RoboteqCanDriver::triggerEstop()
{
  // 0x200C sub 0 is Emergency Stop (!EX)
  sendSdoWrite(0x200C, 0, 1, 1);
}

void RoboteqCanDriver::clearEstop()
{
  // 0x200D sub 0 is Release Emergency Stop (!MG)
  sendSdoWrite(0x200D, 0, 1, 1);
}

void RoboteqCanDriver::triggerQuickstop()
{
  // 0x200E sub 0 is Motor Stop (!MS)
  sendSdoWrite(0x200E, 0, 1, 1);
}

void RoboteqCanDriver::sendFrame(uint32_t can_id, const uint8_t* payload, uint8_t len)
{
  if (m_socket < 0) return;
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = len;
  std::memset(frame.data, 0, 8);
  if (len > 0 && len <= 8)
  {
    std::memcpy(frame.data, payload, len);
  }
  ssize_t nbytes = ::write(m_socket, &frame, sizeof(struct can_frame));
  (void)nbytes;
}

void RoboteqCanDriver::sendSdoRead(uint16_t index, uint8_t subindex)
{
  uint32_t sdo_tx_id = 0x600 + m_nodeId;
  uint8_t payload[8] = {0x40, static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>((index >> 8) & 0xFF), subindex, 0, 0, 0, 0};
  sendFrame(sdo_tx_id, payload, 8);
}

void RoboteqCanDriver::sendSdoWrite(uint16_t index, uint8_t subindex, int32_t value, uint8_t size)
{
  uint32_t sdo_tx_id = 0x600 + m_nodeId;
  uint8_t cmd = 0x22; // Unspecified size
  if (size == 4) cmd = 0x23;
  else if (size == 2) cmd = 0x2B;
  else if (size == 1) cmd = 0x2F;

  uint8_t payload[8] = {
    cmd,
    static_cast<uint8_t>(index & 0xFF),
    static_cast<uint8_t>((index >> 8) & 0xFF),
    subindex,
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>((value >> 16) & 0xFF),
    static_cast<uint8_t>((value >> 24) & 0xFF)
  };
  sendFrame(sdo_tx_id, payload, 8);
}

void RoboteqCanDriver::parseFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
{
  if (can_id == (0x580U + static_cast<uint32_t>(m_nodeId)) && dlc >= 8)
  {
    uint8_t cmd = data[0];
    
    if (cmd == 0x80 || cmd == 0x60)
    {
      // Ignore SDO Abort (0x80) and SDO Write Success (0x60)
      // but update watchdog
      m_lastUpdateTime = now_sec();
      return;
    }

    uint16_t index = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
    uint8_t subidx = data[3];

    int32_t val_s32 = 0;
    uint32_t val_u32 = 0;

    if (cmd == 0x4F) // 1-byte
    {
      int8_t v; std::memcpy(&v, &data[4], 1);
      val_s32 = v;
      val_u32 = data[4];
    }
    else if (cmd == 0x4B) // 2-byte
    {
      int16_t v; std::memcpy(&v, &data[4], 2);
      val_s32 = v;
      uint16_t u; std::memcpy(&u, &data[4], 2);
      val_u32 = u;
    }
    else // 4-byte or unspecified
    {
      std::memcpy(&val_s32, &data[4], 4);
      std::memcpy(&val_u32, &data[4], 4);
    }

    std::lock_guard<std::mutex> lock(m_dataMutex);

    uint32_t dict_key = (static_cast<uint32_t>(index) << 8) | subidx;

    switch (dict_key)
    {
      case 0x210D02:
        m_telemetry.battery_voltage = static_cast<float>(val_u32) / 10.0f;
        break;
      case 0x210001:
        m_telemetry.current_left = static_cast<float>(val_s32) / 10.0f;
        break;
      case 0x210002:
        m_telemetry.current_right = static_cast<float>(val_s32) / 10.0f;
        break;
      case 0x210401:
        m_telemetry.left_encoder = val_s32;
        break;
      case 0x210402:
        m_telemetry.right_encoder = val_s32;
        break;
      case 0x210901:
        m_telemetry.left_rpm = val_s32;
        break;
      case 0x210902:
        m_telemetry.right_rpm = val_s32;
        break;
      case 0x211100:
        m_telemetry.status_flags = val_u32;
        break;
      case 0x211300:
        m_telemetry.fault_flags = val_u32;
        break;
      case 0x211401:
        m_telemetry.closed_loop_error_left = val_s32;
        break;
      case 0x211402:
        m_telemetry.closed_loop_error_right = val_s32;
        break;
      default:
        break;
    }

    m_lastUpdateTime = now_sec();
  }
}

void RoboteqCanDriver::rxThread()
{
  struct can_frame frame;
  while (m_running)
  {
    int nbytes = static_cast<int>(recv(m_socket, &frame, sizeof(struct can_frame), 0));
    if (nbytes >= static_cast<int>(sizeof(struct can_frame)))
    {
      uint32_t can_id = frame.can_id & CAN_EFF_MASK;
      parseFrame(can_id, frame.data, frame.can_dlc);
    }
    else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
      // Sleep to prevent 100% CPU spin on persistent socket errors (e.g. ENETDOWN)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void RoboteqCanDriver::sdoLoop()
{
  // Send NMT Start command to wake up the node
  uint8_t nmt_payload[2] = {0x01, static_cast<uint8_t>(m_nodeId)};
  sendFrame(0x00, nmt_payload, 2);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  struct PollRequest { uint16_t index; uint8_t subindex; };
  const PollRequest poll_list[] = {
    {0x210D, 2}, // Volts 2 (Battery)
    {0x2100, 1}, // Amps 1
    {0x2100, 2}, // Amps 2
    {0x2104, 1}, // Enc 1
    {0x2104, 2}, // Enc 2
    {0x2109, 1}, // RPM 1
    {0x2109, 2}, // RPM 2
    {0x2111, 0}, // Status Flags
    {0x2113, 0}, // Fault Flags
    {0x2114, 1}, // Error 1
    {0x2114, 2}  // Error 2
  };

  while (m_running)
  {
    int32_t tgt_left = 0, tgt_right = 0;
    {
      std::lock_guard<std::mutex> lock(m_cmdMutex);
      tgt_left = m_targetSpeedLeft;
      tgt_right = m_targetSpeedRight;
    }

    sendSdoWrite(0x2000, 1, tgt_left, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!m_running) break;

    sendSdoWrite(0x2000, 2, tgt_right, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!m_running) break;

    for (const auto& req : poll_list)
    {
      sendSdoRead(req.index, req.subindex);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if (!m_running) break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} // namespace roboteq_driver
