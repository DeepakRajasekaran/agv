/*
 * Name:        RoboteqCanDriver.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-22
 * Version:     1.0
 * Description: Abstraction layer for Roboteq Motor Controller using Raw CAN / Custom Protocol.
 */

#ifndef ROBOTEQ_CAN_DRIVER_H
#define ROBOTEQ_CAN_DRIVER_H

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

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
  uint8_t fault_flags = 0;
  uint8_t controller_temp = 0;
};

class RoboteqCanDriver
{
public:
  RoboteqCanDriver(const std::string& can_interface);
  ~RoboteqCanDriver();

  // Lifecycle
  bool connect();
  void disconnect();
  void start();
  void stop();

  // Getters
  TelemetryData getTelemetry();
  bool isConnected() const;

  // Setters & Commands
  void sendSpeedCommand(int32_t left_rpm, int32_t right_rpm);
  void sendSystemCommand(uint8_t estop, uint8_t reset_faults, uint8_t digital_out);

private:
  void rxThread();
  void queryThread();

  void sendQuery(uint8_t query_type);
  void parseFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);

  std::string m_canInterface;
  int m_socket;
  std::atomic<bool> m_running;

  std::thread m_rxThread;
  std::thread m_queryThread;

  std::mutex m_dataMutex;
  TelemetryData m_telemetry;

  std::mutex m_queryMutex;
  std::condition_variable m_queryCv;
  uint8_t m_activeQuery;
  bool m_queryMatched;

  // Connection watchdog
  std::atomic<double> m_lastUpdateTime;
  double m_timeoutSeconds;
};

} // namespace roboteq_driver

#endif // ROBOTEQ_CAN_DRIVER_H
