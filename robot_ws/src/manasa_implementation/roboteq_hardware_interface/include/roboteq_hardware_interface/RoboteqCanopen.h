/*
Name: RoboteqCanopen.h
Author: Manasa
Date: 2026-06-23
Version: 1.0
Description: Declares the RoboteqCanopen class for low-level SocketCAN communication with FBL2360T.
*/

#ifndef ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_CANOPEN_H_
#define ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_CANOPEN_H_

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <map>

namespace roboteq_hardware_interface
{

struct RoboteqState
{
  // Telemetry from QUERY_TABLE
  float motorAmpsCh1 = 0.0f;
  float motorAmpsCh2 = 0.0f;
  float motorCmdCh1 = 0.0f;
  float motorCmdCh2 = 0.0f;
  float powerLevelCh1 = 0.0f;
  float powerLevelCh2 = 0.0f;
  float encoderSpeedCh1 = 0.0f;
  float encoderSpeedCh2 = 0.0f;
  int32_t encoderAbsCh1 = 0;
  int32_t encoderAbsCh2 = 0;
  int32_t brushlessAbsCh1 = 0;
  int32_t brushlessAbsCh2 = 0;
  int32_t userVar1 = 0;
  int32_t userVar2 = 0;
  float encoderSpeedRelCh1 = 0.0f;
  float encoderSpeedRelCh2 = 0.0f;
  int32_t encoderRelCh1 = 0;
  int32_t encoderRelCh2 = 0;
  int32_t brushlessRelCh1 = 0;
  int32_t brushlessRelCh2 = 0;
  float blSpeedRpmCh1 = 0.0f;
  float blSpeedRpmCh2 = 0.0f;
  float blSpeedRelCh1 = 0.0f;
  float blSpeedRelCh2 = 0.0f;
  float batteryAmpsCh1 = 0.0f;
  float batteryAmpsCh2 = 0.0f;
  float internalVolts = 0.0f;
  float batteryVolts = 0.0f;
  float supply5v = 0.0f;
  uint32_t digitalInputsAll = 0;
  float tempHeatsinkCh1 = 0.0f;
  float tempHeatsinkCh2 = 0.0f;
  float tempMcu = 0.0f;
  float feedbackInputCh1 = 0.0f;
  float feedbackInputCh2 = 0.0f;
  uint8_t statusFlags = 0;
  uint8_t faultFlags = 0;
  uint8_t digitalOutputs = 0;
  int32_t clErrorCh1 = 0;
  int32_t clErrorCh2 = 0;
  uint16_t motorStatusCh1 = 0;
  uint16_t motorStatusCh2 = 0;
  uint8_t hallSensorCh1 = 0;
  uint8_t hallSensorCh2 = 0;
  float rotorAngleCh1 = 0.0f;
  float rotorAngleCh2 = 0.0f;
  float slipCh1 = 0.0f;
  float slipCh2 = 0.0f;
  int32_t din1 = 0;
  int32_t din2 = 0;
  int32_t din3 = 0;
  int32_t din4 = 0;
  int32_t din5 = 0;
  int32_t din6 = 0;
  float analogInCh1 = 0.0f;
  float analogInCh2 = 0.0f;
  float analogInCh3 = 0.0f;
  float analogInCh4 = 0.0f;
  float analogConvCh1 = 0.0f;
  float analogConvCh2 = 0.0f;
  float pulseInCh1 = 0.0f;
  float pulseInCh2 = 0.0f;

  // DS402 values
  uint16_t ds402StatusCh1 = 0;
  uint16_t ds402StatusCh2 = 0;
  int8_t ds402OpmodeCh1 = 0;
  int8_t ds402OpmodeCh2 = 0;
  int32_t ds402PosActualCh1 = 0;
  int32_t ds402PosActualCh2 = 0;
  int32_t ds402VelActualCh1 = 0;
  int32_t ds402VelActualCh2 = 0;
  float ds402TorqueActualCh1 = 0.0f;
  float ds402TorqueActualCh2 = 0.0f;

  // Watchdogs / metadata
  std::string heartbeatState = "Unknown";
  bool heartbeatWatchdogOk = false;
  uint32_t sdoOkTotal = 0;
  uint32_t sdoErrTotal = 0;
  uint32_t pdoRxTotal = 0;
  std::string faultFlagsDecoded = "NONE";
  std::string statusFlagsDecoded = "NONE";
};

enum CommandType {
  CMD_NONE,
  CMD_MOTOR,
  CMD_VELOCITY,
  CMD_POSITION
};

class RoboteqCanopen
{
public:
  RoboteqCanopen(const std::string & interface, int nodeId);
  virtual ~RoboteqCanopen();

  bool start();
  void stop();

  bool sdoWrite(uint16_t index, uint8_t subindex, int32_t value, uint8_t size);
  bool sdoRead(uint16_t index, uint8_t subindex, int32_t & value, uint8_t size);

  void setVelocity(int channel, int16_t rpm);
  void setMotorCommand(int channel, int32_t value);
  void setPosition(int channel, int32_t value);
  void emergencyStop();
  void releaseShutdown();
  void stopAll();

  void nmtCmd(uint8_t command, uint8_t target);
  void getSnapshot(RoboteqState & state);

private:
  bool openSocket();
  void closeSocket();
  void sendFrame(uint32_t canId, const uint8_t * data, uint8_t dlc);
  void dispatch(uint32_t canId, const uint8_t * data, uint8_t dlc);
  void rxLoop();
  void pollLoop();
  void cmdLoop();
  void decodeFlags(uint32_t value, const std::map<int, std::string> & table, std::string & output);

  std::string m_interface;
  int m_nodeId;
  int m_socket;
  std::atomic<bool> m_running;

  std::thread m_tRxLoop;
  std::thread m_tPollLoop;
  std::thread m_tCmdLoop;

  // Background command caching
  std::mutex m_cmdMutex;
  CommandType m_lastCmdType[3];
  int32_t m_lastCmdVal[3];

  // Mutex protecting m_state
  std::mutex m_stateMutex;
  RoboteqState m_state;

  // SDO Sync variables
  std::mutex m_sdoClientMutex;
  std::mutex m_sdoMutex;
  std::condition_variable m_sdoCond;
  bool m_sdoReceived;
  std::vector<uint8_t> m_sdoResponseData;
  int m_activeSdoIdx;
  int m_activeSdoSub;

  // Heartbeat watchdog variables
  bool m_hbSeen;
  double m_hbLastTime;

  // Diagnostic helper tables
  std::map<int, std::string> m_faultTable;
  std::map<int, std::string> m_statusTable;
  std::map<int, std::string> m_motorStatusTable;
  std::map<int, std::string> m_ds402StatusTable;
  std::map<int, std::string> m_ds402OpModeTable;
};

} // namespace roboteq_hardware_interface

#endif // ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_CANOPEN_H_
