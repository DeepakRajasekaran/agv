/*
Name: MgsInterface.cpp
Author: Antigravity
Date: 2026-06-24
Version: 1.0
Description: Implements the mediator interface class coordinating MgsRos and MgsCanopen.
*/

#include "mgs_hardware_interface/MgsInterface.h"

namespace mgs_hardware_interface
{

MgsInterface::MgsInterface()
: m_pCanopen(nullptr),
  m_pRos(nullptr)
{
}

MgsInterface::~MgsInterface()
{
  stop();
}

void MgsInterface::initialize()
{
  m_pRos = std::make_shared<MgsRos>(this);

  std::string canInterface;
  int nodeId;
  m_pRos->get_parameter("can_interface", canInterface);
  m_pRos->get_parameter("node_id", nodeId);

  m_pCanopen = std::make_shared<MgsCanopen>(canInterface, nodeId);
  
  std::string defaultTrack;
  m_pRos->get_parameter("default_track", defaultTrack);
  if (defaultTrack == "right") {
    m_pCanopen->followRight();
  } else {
    m_pCanopen->followLeft();
  }
}

bool MgsInterface::start()
{
  if (m_pCanopen) {
    return m_pCanopen->start();
  }
  return false;
}

void MgsInterface::stop()
{
  if (m_pCanopen) {
    m_pCanopen->stop();
  }
}

void MgsInterface::followLeft()
{
  if (m_pCanopen) {
    m_pCanopen->followLeft();
  }
}

void MgsInterface::followRight()
{
  if (m_pCanopen) {
    m_pCanopen->followRight();
  }
}

void MgsInterface::setZero()
{
  if (m_pCanopen) {
    m_pCanopen->setZero();
  }
}

void MgsInterface::getDriverState(MgsState & state)
{
  if (m_pCanopen) {
    m_pCanopen->getSnapshot(state);
  }
}

std::shared_ptr<MgsRos> MgsInterface::getRosNode() const
{
  return m_pRos;
}

} // namespace mgs_hardware_interface
