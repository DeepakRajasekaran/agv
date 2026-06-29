/*
 * Name:        RoboteqCanDriver.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-23
 * Version:     2.0
 * Description: Abstraction layer for Roboteq Motor Controller using CANOpen SDO Polling.
 */

#ifndef ROBOTEQ_CAN_DRIVER_H
#define ROBOTEQ_CAN_DRIVER_H

#include <string>
#include <thread>
#include <mutex>
#include <atomic>

namespace roboteq_driver
{

struct TelemetryData
{
  int32_t left_rpm = 0;
  int32_t right_rpm = 0;
  int32_t left_encoder = 0;
  int32_t right_encoder = 0;
  float battery_voltage = 0.0f;
  float current_left = 0.0f;
  float current_right = 0.0f;
  uint32_t status_flags = 0;
  uint32_t fault_flags = 0;
  int32_t closed_loop_error_left = 0;
  int32_t closed_loop_error_right = 0;
};

class RoboteqCanDriver
{
public:
  RoboteqCanDriver(const std::string& can_interface, int node_id = 1);
  ~RoboteqCanDriver();

  // Lifecycle
  bool connect();
  void disconnect();
  void start();
  void stop();

  // Getters
  TelemetryData getTelemetry();
  bool isConnected() const;

  void sendSpeedCommand(int32_t left_rpm, int32_t right_rpm);
  void resetEncoders();
  void triggerEstop();
  void clearEstop();
  void triggerQuickstop();

private:
  void rxThread();
  void sdoLoop();

  void sendFrame(uint32_t can_id, const uint8_t* payload, uint8_t len);
  void sendSdoRead(uint16_t index, uint8_t subindex);
  void sendSdoWrite(uint16_t index, uint8_t subindex, int32_t value, uint8_t size = 4);
  void parseFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);

  std::string m_canInterface;
  int m_nodeId;
  int m_socket;
  std::atomic<bool> m_running;

  std::thread m_rxThread;
  std::thread m_sdoThread;

  std::mutex m_dataMutex;
  TelemetryData m_telemetry;

  std::mutex m_cmdMutex;
  int32_t m_targetSpeedLeft;
  int32_t m_targetSpeedRight;

  // Connection watchdog
  std::atomic<double> m_lastUpdateTime;
  double m_timeoutSeconds;
};

} // namespace roboteq_driver

#endif // ROBOTEQ_CAN_DRIVER_H
