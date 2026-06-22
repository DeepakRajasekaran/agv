/*
 * Name:        RoboteqCanDriver.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-22
 * Version:     1.0
 * Description: Implements the RoboteqCanDriver class for the custom Raw CAN protocol.
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

/**
 * @brief  Constructor.
 * @param  can_interface  The SocketCAN interface name (e.g. "can0").
 */
RoboteqCanDriver::RoboteqCanDriver(const std::string& can_interface)
  : m_canInterface(can_interface)
  , m_socket(-1)
  , m_running(false)
  , m_activeQuery(0)
  , m_queryMatched(false)
  , m_lastUpdateTime(0.0)
  , m_timeoutSeconds(0.5)
{
}

/**
 * @brief  Destructor. Ensures threads and socket are closed cleanly.
 */
RoboteqCanDriver::~RoboteqCanDriver()
{
  stop();
  disconnect();
}

/**
 * @brief  Helper to get the current time in seconds.
 * @return Current time.
 */
static double now_sec()
{
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration<double>(duration).count();
}

/**
 * @brief  Initializes the CAN socket.
 * @return True if successful.
 */
bool RoboteqCanDriver::connect()
{
  assert(m_socket == -1);

  m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_socket < 0)
  {
    return false;
  }

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

  // Set receive timeout to 50ms so recv does not block indefinitely when shutting down
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 50000;
  if (setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
  {
    // Ignore error
  }

  if (bind(m_socket, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  return true;
}

/**
 * @brief  Closes the CAN socket.
 */
void RoboteqCanDriver::disconnect()
{
  if (m_socket >= 0)
  {
    ::close(m_socket);
    m_socket = -1;
  }
}

/**
 * @brief  Starts the RX and Query threads.
 */
void RoboteqCanDriver::start()
{
  assert(m_socket >= 0);
  assert(!m_running);

  m_running = true;
  m_lastUpdateTime = now_sec();

  m_rxThread = std::thread(&RoboteqCanDriver::rxThread, this);
  m_queryThread = std::thread(&RoboteqCanDriver::queryThread, this);
}

/**
 * @brief  Stops the threads safely.
 */
void RoboteqCanDriver::stop()
{
  if (m_running)
  {
    m_running = false;
    m_queryCv.notify_all();

    if (m_rxThread.joinable())
    {
      m_rxThread.join();
    }
    if (m_queryThread.joinable())
    {
      m_queryThread.join();
    }
  }
}

/**
 * @brief  Returns a thread-safe copy of the latest telemetry.
 * @return Telemetry struct.
 */
TelemetryData RoboteqCanDriver::getTelemetry()
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  return m_telemetry;
}

/**
 * @brief  Checks if CAN data is fresh.
 * @return True if connection is alive.
 */
bool RoboteqCanDriver::isConnected() const
{
  return (now_sec() - m_lastUpdateTime.load()) < m_timeoutSeconds;
}

/**
 * @brief  Sends a velocity command.
 * @param  left_rpm   Left motor RPM target.
 * @param  right_rpm  Right motor RPM target.
 */
void RoboteqCanDriver::sendSpeedCommand(int32_t left_rpm, int32_t right_rpm)
{
  if (m_socket < 0) return;

  struct can_frame frame;
  frame.can_id = 0x410;
  frame.can_dlc = 8;
  std::memcpy(&frame.data[0], &left_rpm, 4);
  std::memcpy(&frame.data[4], &right_rpm, 4);

  ssize_t nbytes = ::write(m_socket, &frame, sizeof(struct can_frame));
  (void)nbytes; // Avoid unused warning
}

/**
 * @brief  Sends a system state command.
 * @param  estop         1 for E-Stop, 0 to release.
 * @param  reset_faults  1 to reset faults.
 * @param  digital_out   Digital output bitmask.
 */
void RoboteqCanDriver::sendSystemCommand(uint8_t estop, uint8_t reset_faults, uint8_t digital_out)
{
  if (m_socket < 0) return;

  struct can_frame frame;
  frame.can_id = 0x411;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = 2; // Operational Mode
  frame.data[1] = estop;
  frame.data[2] = reset_faults;
  frame.data[3] = digital_out;

  ssize_t nbytes = ::write(m_socket, &frame, sizeof(struct can_frame));
  (void)nbytes;
}

/**
 * @brief  Sends a specific query to the controller.
 * @param  query_type  1=RPM, 2=Encoders, 3=Status.
 */
void RoboteqCanDriver::sendQuery(uint8_t query_type)
{
  if (m_socket < 0) return;

  struct can_frame frame;
  frame.can_id = 0x412;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = query_type;

  ssize_t nbytes = ::write(m_socket, &frame, sizeof(struct can_frame));
  (void)nbytes;
}

/**
 * @brief  Parses an incoming CAN frame according to the protocol.
 * @param  can_id  The masked CAN ID.
 * @param  data    The payload data.
 * @param  dlc     The payload length.
 */
void RoboteqCanDriver::parseFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
{
  if (dlc < 8) return;

  uint8_t q_type = 0;
  {
    std::lock_guard<std::mutex> lock(m_queryMutex);
    q_type = m_activeQuery;
  }

  bool matched = false;

  if (can_id == 0x420)
  {
    if (q_type == 1)
    {
      int32_t left = 0, right = 0;
      std::memcpy(&left, &data[0], 4);
      std::memcpy(&right, &data[4], 4);

      std::lock_guard<std::mutex> lock(m_dataMutex);
      m_telemetry.left_rpm = left;
      m_telemetry.right_rpm = right;
      matched = true;
    }
    else if (q_type == 2)
    {
      int32_t left = 0, right = 0;
      std::memcpy(&left, &data[0], 4);
      std::memcpy(&right, &data[4], 4);

      std::lock_guard<std::mutex> lock(m_dataMutex);
      m_telemetry.left_encoder = left;
      m_telemetry.right_encoder = right;
      matched = true;
    }
  }
  else if (can_id == 0x421)
  {
    if (q_type == 3)
    {
      uint16_t v_raw = 0;
      int16_t c1_raw = 0, c2_raw = 0;
      uint8_t ff = 0, temp = 0;

      std::memcpy(&v_raw, &data[0], 2);
      std::memcpy(&c1_raw, &data[2], 2);
      std::memcpy(&c2_raw, &data[4], 2);
      ff = data[6];
      temp = data[7];

      std::lock_guard<std::mutex> lock(m_dataMutex);
      m_telemetry.battery_voltage = static_cast<float>(v_raw) / 10.0f;
      m_telemetry.current_left = static_cast<float>(c1_raw) / 10.0f;
      m_telemetry.current_right = static_cast<float>(c2_raw) / 10.0f;
      m_telemetry.fault_flags = ff;
      m_telemetry.controller_temp = temp;
      matched = true;
    }
  }

  if (matched)
  {
    std::lock_guard<std::mutex> lock(m_queryMutex);
    m_queryMatched = true;
    m_lastUpdateTime = now_sec();
    m_queryCv.notify_all();
  }
}

/**
 * @brief  RX thread loop for reading socket.
 */
void RoboteqCanDriver::rxThread()
{
  struct can_frame frame;
  while (m_running)
  {
    int nbytes = static_cast<int>(recv(m_socket, &frame, sizeof(struct can_frame), 0));
    if (nbytes < 0)
    {
      continue;
    }
    if (static_cast<size_t>(nbytes) < sizeof(struct can_frame))
    {
      continue;
    }

    uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    parseFrame(can_id, frame.data, frame.can_dlc);
  }
}

/**
 * @brief  Query thread loop mimicking the Python prototype.
 */
void RoboteqCanDriver::queryThread()
{
  const uint8_t query_sequence[] = {1, 2, 3};
  size_t idx = 0;

  while (m_running)
  {
    uint8_t q_type = query_sequence[idx];

    {
      std::lock_guard<std::mutex> lock(m_queryMutex);
      m_activeQuery = q_type;
      m_queryMatched = false;
    }

    sendQuery(q_type);

    std::unique_lock<std::mutex> cv_lock(m_queryMutex);
    bool responded = m_queryCv.wait_for(cv_lock, std::chrono::milliseconds(20), [this]() {
      return !m_running || m_queryMatched;
    });

    if (responded && m_queryMatched)
    {
      idx = (idx + 1) % 3;
      cv_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    else
    {
      cv_lock.unlock();
      // Timeout, retry same query but wait a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

} // namespace roboteq_driver
