/*
Name: MgsCanopen.h
Author: Manasa
Date: 2026-06-24
Version: 2.0
Description: Low-level SocketCAN driver for MGS1600. Mirrors updated mgs_driver.py.
*/

#ifndef MGS_HARDWARE_INTERFACE__MGS_CANOPEN_H_
#define MGS_HARDWARE_INTERFACE__MGS_CANOPEN_H_

#include <string>
#include <vector>
#include <set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <array>
#include <cstdint>

namespace mgs_hardware_interface
{

static constexpr int NUM_SENSOR_CHANNELS = 10;
static constexpr int NUM_USER_VARS = 10;

struct MgsState
{
  double leftTrack = 0.0;
  double rightTrack = 0.0;
  double selectedTrack = 0.0;
  bool trackDetect = false;
  bool leftMarker = false;
  bool rightMarker = false;
  bool tapeCross = false;
  uint16_t status = 0;
  int8_t dominantTrack = 0;
  bool sensorFailure = false;

  std::array<int32_t, NUM_SENSOR_CHANNELS> rawSensor = {0};
  std::array<int32_t, NUM_SENSOR_CHANNELS> zeroAdjSensor = {0};
  std::array<int32_t, NUM_USER_VARS> userInts = {0};
  std::array<bool, NUM_USER_VARS> userBools = {false};

  std::string nmtState = "Unknown";
  uint32_t tpdo1Count = 0;
};

class MgsCanopen
{
public:
  MgsCanopen(const std::string & interface, int nodeId);
  virtual ~MgsCanopen();

  bool start();
  void stop();

  void nmtStart();
  void nmtStop();

  void followLeft();
  void followRight();
  void setZero();
  void saveConfig();

  void getSnapshot(MgsState & state);

private:
  bool openSocket();
  void closeSocket();
  void sendFrame(uint32_t canId, const uint8_t * data, uint8_t dlc);
  void rxLoop();
  void pollLoop();
  void sdoGap();

  bool sdoRead(uint16_t index, uint8_t subindex, int32_t & value, uint8_t size);
  void sdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value);

  void queryNav();
  void queryDiagGroup(int group);

  void dispatch(uint32_t canId, const uint8_t * data, uint8_t dlc);
  void parseTpdo1(const uint8_t * data, uint8_t dlc);
  void handleSdoResponse(const uint8_t * data, uint8_t dlc);
  void handleHeartbeat(const uint8_t * data, uint8_t dlc);

  std::string m_interface;
  int m_nodeId;
  int m_socket;
  std::atomic<bool> m_running;

  std::thread m_tRxLoop;
  std::thread m_tPollLoop;

  std::mutex m_stateMutex;
  MgsState m_state;

  std::mutex m_sdoMutex;
  std::condition_variable m_sdoCond;
  bool m_sdoReceived;
  std::vector<uint8_t> m_sdoResponseData;
  uint16_t m_expectedIdx;
  uint8_t m_expectedSub;

  // SDO abort blacklist — objects that abort are skipped in future queries
  std::set<std::pair<uint16_t, uint8_t>> m_abortedObjects;

  // Diagnostic group rotation (0, 1, 2)
  int m_diagGroup;
};

} // namespace mgs_hardware_interface

#endif // MGS_HARDWARE_INTERFACE__MGS_CANOPEN_H_
