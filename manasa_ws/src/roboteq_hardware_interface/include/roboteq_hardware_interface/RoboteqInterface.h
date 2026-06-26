/*
Name: RoboteqInterface.h
Author: Manasa
Date: 2026-06-23
Version: 1.0
Description: Declares the mediator interface class coordinating RoboteqRos and RoboteqCanopen.
*/

#ifndef ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_INTERFACE_H_
#define ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_INTERFACE_H_

#include <memory>
#include "roboteq_hardware_interface/RoboteqCanopen.h"
#include "roboteq_hardware_interface/RoboteqRos.h"

namespace roboteq_hardware_interface
{

class RoboteqInterface
{
public:
  RoboteqInterface();
  virtual ~RoboteqInterface();

  void initialize();
  bool start();
  void stop();

  void handleCmdRpm(float leftRpm, float rightRpm);
  void emergencyStop();
  void releaseShutdown();
  void stopAll();
  void getDriverState(RoboteqState & state);
  std::shared_ptr<RoboteqRos> getRosNode() const;

private:
  std::shared_ptr<RoboteqCanopen> m_pCanopen;
  std::shared_ptr<RoboteqRos> m_pRos;
};

} // namespace roboteq_hardware_interface

#endif // ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_INTERFACE_H_
