/*
Name: RoboteqCanopen.cpp
Author: Manasa
Date: 2026-06-23
Version: 1.0
Description: Implements the RoboteqCanopen class for CANopen communications over SocketCAN.
*/

#include "roboteq_hardware_interface/RoboteqCanopen.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <chrono>

namespace roboteq_hardware_interface
{

/**
 * @brief Constructor for the RoboteqCanopen class, initializing diagnostic lookup tables.
 * @param interface - CAN interface name (e.g. "can0").
 * @param nodeId - Node ID of the Roboteq controller.
 * @return None
 */
RoboteqCanopen::RoboteqCanopen(const std::string & interface, int nodeId)
: m_interface(interface),
  m_nodeId(nodeId),
  m_socket(-1),
  m_running(false),
  m_sdoReceived(false),
  m_activeSdoIdx(0),
  m_activeSdoSub(0),
  m_hbSeen(false),
  m_hbLastTime(0.0)
{
  m_lastCmdType[1] = CMD_NONE;
  m_lastCmdType[2] = CMD_NONE;
  m_lastCmdVal[1] = 0;
  m_lastCmdVal[2] = 0;
  // Initialize fault table
  m_faultTable[0] = "Overheat";
  m_faultTable[1] = "Overvoltage";
  m_faultTable[2] = "Undervoltage";
  m_faultTable[3] = "Short Circuit";
  m_faultTable[4] = "Emergency Stop";
  m_faultTable[5] = "Brushless Sensor Fault";
  m_faultTable[6] = "MOSFET Failure";
  m_faultTable[7] = "Default Config Loaded";

  // Initialize status table
  m_statusTable[0] = "Serial Mode Active";
  m_statusTable[1] = "Pulse Mode Active";
  m_statusTable[2] = "Analog Mode Active";
  m_statusTable[3] = "Power Stage Off";
  m_statusTable[4] = "Stall Ch1";
  m_statusTable[5] = "Stall Ch2";
  m_statusTable[6] = "At Speed Ch1";
  m_statusTable[7] = "At Speed Ch2";

  // Initialize motor status table
  m_motorStatusTable[0] = "Amps Limit Active";
  m_motorStatusTable[1] = "Motor Stall Detected";
  m_motorStatusTable[2] = "Loop Error Detected";
  m_motorStatusTable[3] = "Safety Stop Active";
  m_motorStatusTable[4] = "Forward Limit Triggered";
  m_motorStatusTable[5] = "Reverse Limit Triggered";
  m_motorStatusTable[6] = "Amps Trigger Activated";

  // Initialize DS402 status table
  m_ds402StatusTable[0] = "Ready to Switch On";
  m_ds402StatusTable[1] = "Switched On";
  m_ds402StatusTable[2] = "Operation Enabled";
  m_ds402StatusTable[3] = "Fault";
  m_ds402StatusTable[4] = "Voltage Enabled";
  m_ds402StatusTable[5] = "Quick Stop";
  m_ds402StatusTable[6] = "Switch On Disabled";
  m_ds402StatusTable[7] = "Warning";
  m_ds402StatusTable[9] = "Remote";
  m_ds402StatusTable[10] = "Target Reached";
  m_ds402StatusTable[11] = "Internal Limit Active";

  // Initialize DS402 mode table
  m_ds402OpModeTable[-1] = "No Mode";
  m_ds402OpModeTable[0] = "No Change";
  m_ds402OpModeTable[1] = "Profile Position";
  m_ds402OpModeTable[2] = "Velocity (VL)";
  m_ds402OpModeTable[3] = "Profile Velocity (PV)";
  m_ds402OpModeTable[4] = "Profile Torque (TQ)";
  m_ds402OpModeTable[6] = "Homing";
}

/**
 * @brief Destructor for the RoboteqCanopen class, stopping threads and closing socket.
 * @param None
 * @return None
 */
RoboteqCanopen::~RoboteqCanopen()
{
  stop();
}

/**
 * @brief Opens SocketCAN and starts the background loops.
 * @param None
 * @return bool - True if successful, False otherwise.
 */
bool RoboteqCanopen::start()
{
  if (m_running)
  {
    return true;
  }

  if (!openSocket())
  {
    return false;
  }

  m_running = true;

  // Start in Operational mode
  nmtCmd(0x01, m_nodeId);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Launch threads
  m_tRxLoop = std::thread(&RoboteqCanopen::rxLoop, this);
  m_tPollLoop = std::thread(&RoboteqCanopen::pollLoop, this);
  m_tCmdLoop = std::thread(&RoboteqCanopen::cmdLoop, this);

  return true;
}

/**
 * @brief Stops loops, joins threads, and resets the interface.
 * @param None
 * @return None
 */
void RoboteqCanopen::stop()
{
  if (!m_running)
  {
    return;
  }

  m_running = false;

  // Unblock SDO wait if any
  {
    std::lock_guard<std::mutex> lock(m_sdoMutex);
    m_sdoReceived = true;
    m_sdoCond.notify_all();
  }

  if (m_tRxLoop.joinable())
  {
    m_tRxLoop.join();
  }
  if (m_tPollLoop.joinable())
  {
    m_tPollLoop.join();
  }
  if (m_tCmdLoop.joinable())
  {
    m_tCmdLoop.join();
  }

  closeSocket();
}

/**
 * @brief Opens and configures the SocketCAN socket with filters and timeouts.
 * @param None
 * @return bool - True if successful, False otherwise.
 */
bool RoboteqCanopen::openSocket()
{
  m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_socket < 0)
  {
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, m_interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0)
  {
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  // Set receive timeout
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 200000; // 200ms
  setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Install filters
  struct can_filter rfilter[7];
  rfilter[0].can_id   = 0x580 + m_nodeId; // SDO_RX
  rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id   = 0x180 + m_nodeId; // TPDO1
  rfilter[1].can_mask = CAN_SFF_MASK;
  rfilter[2].can_id   = 0x280 + m_nodeId; // TPDO2
  rfilter[2].can_mask = CAN_SFF_MASK;
  rfilter[3].can_id   = 0x380 + m_nodeId; // TPDO3
  rfilter[3].can_mask = CAN_SFF_MASK;
  rfilter[4].can_id   = 0x480 + m_nodeId; // TPDO4
  rfilter[4].can_mask = CAN_SFF_MASK;
  rfilter[5].can_id   = 0x700 + m_nodeId; // HEARTBEAT
  rfilter[5].can_mask = CAN_SFF_MASK;
  rfilter[6].can_id   = 0x080 + m_nodeId; // EMCY
  rfilter[6].can_mask = CAN_SFF_MASK;

  setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  return true;
}

/**
 * @brief Closes the SocketCAN socket.
 * @param None
 * @return None
 */
void RoboteqCanopen::closeSocket()
{
  if (m_socket >= 0)
  {
    ::close(m_socket);
    m_socket = -1;
  }
}

/**
 * @brief Sends a raw CAN frame.
 * @param canId - The standard CAN identifier.
 * @param data - Pointer to the frame payload.
 * @param dlc - Data length code.
 * @return None
 */
void RoboteqCanopen::sendFrame(uint32_t canId, const uint8_t * data, uint8_t dlc)
{
  if (m_socket < 0)
  {
    return;
  }

  struct can_frame frame;
  frame.can_id = canId & CAN_SFF_MASK;
  frame.can_dlc = dlc > 8 ? 8 : dlc;
  std::memset(frame.data, 0, 8);
  if (data && frame.can_dlc > 0)
  {
    std::memcpy(frame.data, data, frame.can_dlc);
  }

  if (write(m_socket, &frame, sizeof(frame)) != sizeof(frame))
  {
    // Write error
  }
}

/**
 * @brief Sends a NMT command to transition nodes.
 * @param command - The NMT command byte (e.g. 0x01 for Operational).
 * @param target - The target Node ID (0 for all).
 * @return None
 */
void RoboteqCanopen::nmtCmd(uint8_t command, uint8_t target)
{
  uint8_t payload[2] = {command, target};
  sendFrame(0x000, payload, 2);
}

/**
 * @brief Executes a synchronous SDO write transaction.
 * @param index - Object Dictionary index.
 * @param subindex - Object Dictionary subindex.
 * @param value - Value to write.
 * @param size - Data size in bytes (1, 2, or 4).
 * @return bool - True if SDO write succeeded (ACK received), False otherwise.
 */
bool RoboteqCanopen::sdoWrite(uint16_t index, uint8_t subindex, int32_t value, uint8_t size)
{
  std::lock_guard<std::mutex> clientLock(m_sdoClientMutex);
  std::unique_lock<std::mutex> sdoLock(m_sdoMutex);
  if (m_socket < 0)
  {
    return false;
  }

  m_activeSdoIdx = index;
  m_activeSdoSub = subindex;
  m_sdoReceived = false;
  m_sdoResponseData.clear();

  uint8_t payload[8];
  std::memset(payload, 0, 8);

  uint8_t cmd = 0;
  if (size == 4) cmd = 0x23;
  else if (size == 2) cmd = 0x2B;
  else if (size == 1) cmd = 0x2F;
  else return false;

  payload[0] = cmd;
  payload[1] = index & 0xFF;
  payload[2] = (index >> 8) & 0xFF;
  payload[3] = subindex;
  std::memcpy(&payload[4], &value, size);

  sendFrame(0x600 + m_nodeId, payload, 8);

  // Wait with timeout
  if (m_sdoCond.wait_for(sdoLock, std::chrono::milliseconds(500), [this]() { return m_sdoReceived; }))
  {
    if (m_sdoResponseData.size() >= 4)
    {
      uint8_t cs = m_sdoResponseData[0];
      if (cs == 0x60) // Write acknowledged
      {
        std::lock_guard<std::mutex> stateLock(m_stateMutex);
        m_state.sdoOkTotal++;
        return true;
      }
    }
  }

  std::lock_guard<std::mutex> stateLock(m_stateMutex);
  m_state.sdoErrTotal++;
  return false;
}

/**
 * @brief Executes a synchronous SDO read transaction.
 * @param index - Object Dictionary index.
 * @param subindex - Object Dictionary subindex.
 * @param value - Variable to store the read value.
 * @param size - Expected data size in bytes (1, 2, or 4).
 * @return bool - True if SDO read succeeded, False otherwise.
 */
bool RoboteqCanopen::sdoRead(uint16_t index, uint8_t subindex, int32_t & value, uint8_t size)
{
  std::lock_guard<std::mutex> clientLock(m_sdoClientMutex);
  std::unique_lock<std::mutex> sdoLock(m_sdoMutex);
  if (m_socket < 0)
  {
    return false;
  }

  m_activeSdoIdx = index;
  m_activeSdoSub = subindex;
  m_sdoReceived = false;
  m_sdoResponseData.clear();

  uint8_t payload[8];
  std::memset(payload, 0, 8);
  payload[0] = 0x40; // Read command
  payload[1] = index & 0xFF;
  payload[2] = (index >> 8) & 0xFF;
  payload[3] = subindex;

  sendFrame(0x600 + m_nodeId, payload, 8);

  // Wait with timeout
  if (m_sdoCond.wait_for(sdoLock, std::chrono::milliseconds(500), [this]() { return m_sdoReceived; }))
  {
    if (m_sdoResponseData.size() >= 8)
    {
      uint8_t cs = m_sdoResponseData[0];
      if (cs == 0x80) // Abort response
      {
        std::lock_guard<std::mutex> stateLock(m_stateMutex);
        m_state.sdoErrTotal++;
        return false;
      }
      
      // Extract data bytes
      int32_t rawVal = 0;
      std::memcpy(&rawVal, &m_sdoResponseData[4], size);
      
      // Sign extension if necessary
      if (size == 1)
      {
        value = static_cast<int8_t>(rawVal & 0xFF);
      }
      else if (size == 2)
      {
        value = static_cast<int16_t>(rawVal & 0xFFFF);
      }
      else
      {
        value = rawVal;
      }

      std::lock_guard<std::mutex> stateLock(m_stateMutex);
      m_state.sdoOkTotal++;
      return true;
    }
  }

  std::lock_guard<std::mutex> stateLock(m_stateMutex);
  m_state.sdoErrTotal++;
  return false;
}

/**
 * @brief Sets velocity command on OD 0x2002 for motor channel.
 * @param channel - Motor channel (1 or 2).
 * @param rpm - Velocity in RPM.
 * @return None
 */
void RoboteqCanopen::setVelocity(int channel, int16_t rpm)
{
  std::lock_guard<std::mutex> lock(m_cmdMutex);
  if (channel == 1 || channel == 2) {
    m_lastCmdType[channel] = CMD_VELOCITY;
    m_lastCmdVal[channel] = rpm;
  }
}

void RoboteqCanopen::setMotorCommand(int channel, int32_t value)
{
  std::lock_guard<std::mutex> lock(m_cmdMutex);
  if (channel == 1 || channel == 2) {
    m_lastCmdType[channel] = CMD_MOTOR;
    m_lastCmdVal[channel] = value;
  }
}

void RoboteqCanopen::setPosition(int channel, int32_t value)
{
  std::lock_guard<std::mutex> lock(m_cmdMutex);
  if (channel == 1 || channel == 2) {
    m_lastCmdType[channel] = CMD_POSITION;
    m_lastCmdVal[channel] = value;
  }
}

void RoboteqCanopen::emergencyStop()
{
  {
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_lastCmdType[1] = CMD_NONE;
    m_lastCmdType[2] = CMD_NONE;
  }
  sdoWrite(0x200C, 1, 1, 1);
  sdoWrite(0x200C, 2, 1, 1);
}

void RoboteqCanopen::releaseShutdown()
{
  sdoWrite(0x200D, 1, 1, 1);
  sdoWrite(0x200D, 2, 1, 1);
}

void RoboteqCanopen::stopAll()
{
  {
    std::lock_guard<std::mutex> lock(m_cmdMutex);
    m_lastCmdType[1] = CMD_NONE;
    m_lastCmdType[2] = CMD_NONE;
  }
  sdoWrite(0x2000, 1, 0, 4);
  sdoWrite(0x2000, 2, 0, 4);
}

/**
 * @brief Retrieves a thread-safe copy of the driver status.
 * @param state - Struct to copy status to.
 * @return None
 */
void RoboteqCanopen::getSnapshot(RoboteqState & state)
{
  std::lock_guard<std::mutex> lock(m_stateMutex);
  state = m_state;
}

/**
 * @brief Background receive thread loop processing incoming CAN frames.
 * @param None
 * @return None
 */
void RoboteqCanopen::rxLoop()
{
  struct can_frame frame;
  while (m_running)
  {
    int bytesRead = read(m_socket, &frame, sizeof(frame));
    if (bytesRead <= 0)
    {
      continue;
    }

    dispatch(frame.can_id, frame.data, frame.can_dlc);
  }
}

/**
 * @brief Dispatches a received CAN frame to its specific handler.
 * @param canId - CAN identifier.
 * @param data - Frame payload pointer.
 * @param dlc - Data length.
 * @return None
 */
void RoboteqCanopen::dispatch(uint32_t canId, const uint8_t * data, uint8_t dlc)
{
  uint32_t cobId = canId & CAN_SFF_MASK;

  // SDO Response
  if (cobId == (0x580 + static_cast<uint32_t>(m_nodeId)))
  {
    if (dlc >= 4)
    {
      uint16_t echoIdx = data[1] | (data[2] << 8);
      uint8_t echoSub = data[3];
      if (echoIdx == m_activeSdoIdx && echoSub == m_activeSdoSub)
      {
        std::lock_guard<std::mutex> lock(m_sdoMutex);
        m_sdoResponseData.assign(data, data + dlc);
        m_sdoReceived = true;
        m_sdoCond.notify_all();
      }
    }
    return;
  }

  // Heartbeat
  if (cobId == (0x700 + static_cast<uint32_t>(m_nodeId)))
  {
    if (dlc >= 1)
    {
      uint8_t hbState = data[0];
      std::string stateStr = "Unknown";
      if (hbState == 0x00) stateStr = "Boot-up";
      else if (hbState == 0x04) stateStr = "Stopped";
      else if (hbState == 0x05) stateStr = "Operational";
      else if (hbState == 0x7F) stateStr = "Pre-operational";

      std::lock_guard<std::mutex> lock(m_stateMutex);
      m_state.heartbeatState = stateStr;
      m_hbSeen = true;
      // Simple timestamping (can be checked in poll loop against system time)
      m_hbLastTime = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      m_state.heartbeatWatchdogOk = true;
    }
    return;
  }

  // EMCY
  if (cobId == (0x080 + static_cast<uint32_t>(m_nodeId)))
  {
    if (dlc >= 8)
    {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      uint8_t faultLow = data[3];
      decodeFlags(faultLow, m_faultTable, m_state.faultFlagsDecoded);
    }
    return;
  }

  // TPDOs
  if (cobId == (0x180 + static_cast<uint32_t>(m_nodeId)) ||
      cobId == (0x280 + static_cast<uint32_t>(m_nodeId)) ||
      cobId == (0x380 + static_cast<uint32_t>(m_nodeId)) ||
      cobId == (0x480 + static_cast<uint32_t>(m_nodeId)))
  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state.pdoRxTotal++;
    // In our case we poll everything via SDO just like the python query-response model.
    // If TPDOs are configured, we could parse them here.
    return;
  }
}

/**
 * @brief Decodes an integer bitfield into a pipe-separated human-readable string.
 * @param value - Bitfield value.
 * @param table - Map lookup table containing bit indices and strings.
 * @param output - Output decoded string.
 * @return None
 */
void RoboteqCanopen::decodeFlags(uint32_t value, const std::map<int, std::string> & table, std::string & output)
{
  std::string result = "";
  for (const auto & pair : table)
  {
    if ((value >> pair.first) & 1)
    {
      if (!result.empty())
      {
        result += "|";
      }
      result += pair.second;
    }
  }
  output = result.empty() ? "NONE" : result;
}

/**
 * @brief Polling thread loop cycling through Object Dictionary registers.
 * @param None
 * @return None
 */
void RoboteqCanopen::pollLoop()
{
  while (m_running)
  {
    auto startTime = std::chrono::steady_clock::now();
    
    // Perform all SDO Reads. Store them into a local copy first to minimize lock time
    RoboteqState tempState;
    {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      tempState = m_state;
    }

    int32_t val = 0;

    // 0x2100 - Motor Amps (S16)
    if (sdoRead(0x2100, 1, val, 2)) tempState.motorAmpsCh1 = val * 0.1f;
    if (sdoRead(0x2100, 2, val, 2)) tempState.motorAmpsCh2 = val * 0.1f;

    // 0x2101 - Motor Command (S16)
    if (sdoRead(0x2101, 1, val, 2)) tempState.motorCmdCh1 = val * 1.0f;
    if (sdoRead(0x2101, 2, val, 2)) tempState.motorCmdCh2 = val * 1.0f;

    // 0x2102 - Power Level (S16)
    if (sdoRead(0x2102, 1, val, 2)) tempState.powerLevelCh1 = val * 1.0f;
    if (sdoRead(0x2102, 2, val, 2)) tempState.powerLevelCh2 = val * 1.0f;

    // 0x2103 - Encoder Speed RPM (S16)
    if (sdoRead(0x2103, 1, val, 2)) tempState.encoderSpeedCh1 = val * 1.0f;
    if (sdoRead(0x2103, 2, val, 2)) tempState.encoderSpeedCh2 = val * 1.0f;

    // 0x2104 - Absolute Encoder Count (S32)
    if (sdoRead(0x2104, 1, val, 4)) tempState.encoderAbsCh1 = val;
    if (sdoRead(0x2104, 2, val, 4)) tempState.encoderAbsCh2 = val;

    // 0x2105 - Absolute Brushless Counter (S32)
    if (sdoRead(0x2105, 1, val, 4)) tempState.brushlessAbsCh1 = val;
    if (sdoRead(0x2105, 2, val, 4)) tempState.brushlessAbsCh2 = val;

    // 0x210C - Battery Amps (S16)
    if (sdoRead(0x210C, 1, val, 2)) tempState.batteryAmpsCh1 = val * 0.1f;
    if (sdoRead(0x210C, 2, val, 2)) tempState.batteryAmpsCh2 = val * 0.1f;

    // 0x210D - Voltages (U16)
    if (sdoRead(0x210D, 1, val, 2)) tempState.internalVolts = val * 0.1f;
    if (sdoRead(0x210D, 2, val, 2)) tempState.batteryVolts = val * 0.1f;
    if (sdoRead(0x210D, 3, val, 2)) tempState.supply5v = val * 0.01f;

    // 0x210E - All Digital Inputs (U32)
    if (sdoRead(0x210E, 0, val, 4)) tempState.digitalInputsAll = val;

    // 0x210F - Heatsink / MCU temperatures (S8)
    if (sdoRead(0x210F, 1, val, 1)) tempState.tempHeatsinkCh1 = val * 1.0f;
    if (sdoRead(0x210F, 2, val, 1)) tempState.tempHeatsinkCh2 = val * 1.0f;
    if (sdoRead(0x210F, 3, val, 1)) tempState.tempMcu = val * 1.0f;

    // 0x2111 - Status Flags (U8)
    if (sdoRead(0x2111, 0, val, 1))
    {
      tempState.statusFlags = val;
      decodeFlags(val, m_statusTable, tempState.statusFlagsDecoded);
    }

    // 0x2112 - Fault Flags (U8)
    if (sdoRead(0x2112, 0, val, 1))
    {
      tempState.faultFlags = val;
      decodeFlags(val, m_faultTable, tempState.faultFlagsDecoded);
    }

    // 0x2113 - Current Digital Outputs (U8)
    if (sdoRead(0x2113, 0, val, 1)) tempState.digitalOutputs = val;

    // 0x2122 - Motor Status Flags (U16)
    if (sdoRead(0x2122, 1, val, 2)) tempState.motorStatusCh1 = val;
    if (sdoRead(0x2122, 2, val, 2)) tempState.motorStatusCh2 = val;

    // 0x2123 - Hall Sensor States (U8)
    if (sdoRead(0x2123, 1, val, 1)) tempState.hallSensorCh1 = val;
    if (sdoRead(0x2123, 2, val, 1)) tempState.hallSensorCh2 = val;

    // Heartbeat Watchdog check
    double now = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (m_hbSeen && (now - m_hbLastTime) > 2.0)
    {
      tempState.heartbeatWatchdogOk = false;
      tempState.heartbeatState = "LOST";
    }

    // Commit snapshot
    {
      std::lock_guard<std::mutex> lock(m_stateMutex);
      // Retain cumulative statistics
      uint32_t sdoOk = m_state.sdoOkTotal;
      uint32_t sdoErr = m_state.sdoErrTotal;
      uint32_t pdoRx = m_state.pdoRxTotal;
      
      m_state = tempState;
      
      m_state.sdoOkTotal = sdoOk;
      m_state.sdoErrTotal = sdoErr;
      m_state.pdoRxTotal = pdoRx;
    }

    auto endTime = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::ratio<1, 1000>>(endTime - startTime).count();
    double sleepMs = 50.0 - elapsed;
    if (sleepMs > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepMs)));
    }
  }
}

/**
 * @brief Background thread continuously sending the last cached command.
 * @param None
 * @return None
 */
void RoboteqCanopen::cmdLoop()
{
  while (m_running)
  {
    auto startTime = std::chrono::steady_clock::now();

    struct Cmd {
      int ch;
      CommandType type;
      int32_t val;
    };
    std::vector<Cmd> cmds;

    {
      std::lock_guard<std::mutex> lock(m_cmdMutex);
      for (int ch = 1; ch <= 2; ++ch) {
        if (m_lastCmdType[ch] != CMD_NONE) {
          cmds.push_back({ch, m_lastCmdType[ch], m_lastCmdVal[ch]});
        }
      }
    }

    for (const auto & cmd : cmds) {
      if (!m_running) break;
      if (cmd.type == CMD_MOTOR) sdoWrite(0x2000, cmd.ch, cmd.val, 4);
      else if (cmd.type == CMD_VELOCITY) sdoWrite(0x2002, cmd.ch, cmd.val, 2);
      else if (cmd.type == CMD_POSITION) sdoWrite(0x2001, cmd.ch, cmd.val, 4);
    }

    auto endTime = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::ratio<1, 1000>>(endTime - startTime).count();
    double sleepMs = 50.0 - elapsed;
    if (sleepMs > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepMs)));
    }
  }
}

} // namespace roboteq_hardware_interface
