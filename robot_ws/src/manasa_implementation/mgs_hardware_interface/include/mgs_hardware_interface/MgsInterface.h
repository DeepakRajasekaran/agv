/*
Name: MgsInterface.h
Author: Manasa
Date: 2026-06-24
Version: 1.0
Description: Declares the mediator interface class coordinating MgsRos and MgsCanopen.
*/

#ifndef MGS_HARDWARE_INTERFACE__MGS_INTERFACE_H_
#define MGS_HARDWARE_INTERFACE__MGS_INTERFACE_H_

#include <memory>
#include "mgs_hardware_interface/MgsCanopen.h"
#include "mgs_hardware_interface/MgsRos.h"

namespace mgs_hardware_interface
{

class MgsInterface
{
public:
  MgsInterface();
  virtual ~MgsInterface();

  void initialize();
  bool start();
  void stop();

  void followLeft();
  void followRight();
  void setZero();

  void getDriverState(MgsState & state);
  std::shared_ptr<MgsRos> getRosNode() const;

private:
  std::shared_ptr<MgsCanopen> m_pCanopen;
  std::shared_ptr<MgsRos> m_pRos;
};

} // namespace mgs_hardware_interface

#endif // MGS_HARDWARE_INTERFACE__MGS_INTERFACE_H_
