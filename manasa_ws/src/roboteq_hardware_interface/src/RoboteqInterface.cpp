/*
Name: RoboteqInterface.cpp
Author: Manasa
Date: 2026-06-23
Version: 1.0
Description: Implements the mediator interface class coordinating RoboteqRos and RoboteqCanopen.
*/

#include "roboteq_hardware_interface/RoboteqInterface.h"

namespace roboteq_hardware_interface
{

/**
 * @brief Constructor for the RoboteqInterface mediator class.
 * @param None
 * @return None
 */
RoboteqInterface::RoboteqInterface()
: m_pCanopen(nullptr),
  m_pRos(nullptr)
{
}

/**
 * @brief Destructor for the RoboteqInterface mediator class, ensuring CAN open driver stops.
 * @param None
 * @return None
 */
RoboteqInterface::~RoboteqInterface()
{
  stop();
}

/**
 * @brief Initializes the ROS 2 node and low-level CANopen interface using loaded parameters.
 * @param None
 * @return None
 */
void RoboteqInterface::initialize()
{
  m_pRos = std::make_shared<RoboteqRos>(this);

  // Retrieve CAN configuration from ROS parameters
  std::string canInterface;
  int nodeId;
  m_pRos->get_parameter("can_interface", canInterface);
  m_pRos->get_parameter("node_id", nodeId);

  m_pCanopen = std::make_shared<RoboteqCanopen>(canInterface, nodeId);
}

/**
 * @brief Starts the CANopen driver socket connection and background threads.
 * @param None
 * @return bool - True if starting succeeded, False otherwise.
 */
bool RoboteqInterface::start()
{
  if (m_pCanopen)
  {
    return m_pCanopen->start();
  }
  return false;
}

/**
 * @brief Stops the CANopen driver loops.
 * @param None
 * @return None
 */
void RoboteqInterface::stop()
{
  if (m_pCanopen)
  {
    m_pCanopen->stop();
  }
}

/**
 * @brief Interface callback passing incoming velocity commands from ROS to CAN open.
 * @param leftRpm - Commanded left wheel RPM.
 * @param rightRpm - Commanded right wheel RPM.
 * @return None
 */
void RoboteqInterface::handleCmdRpm(float leftRpm, float rightRpm)
{
  if (m_pCanopen)
  {
    m_pCanopen->setMotorCommand(1, static_cast<int32_t>(leftRpm));
    m_pCanopen->setMotorCommand(2, static_cast<int32_t>(rightRpm));
  }
}

void RoboteqInterface::emergencyStop()
{
  if (m_pCanopen)
  {
    m_pCanopen->emergencyStop();
  }
}

void RoboteqInterface::releaseShutdown()
{
  if (m_pCanopen)
  {
    m_pCanopen->releaseShutdown();
  }
}

void RoboteqInterface::stopAll()
{
  if (m_pCanopen)
  {
    m_pCanopen->stopAll();
  }
}

/**
 * @brief Collects the latest snapshot of the CANopen driver telemetry.
 * @param state - The RoboteqState struct reference to populate.
 * @return None
 */
void RoboteqInterface::getDriverState(RoboteqState & state)
{
  if (m_pCanopen)
  {
    m_pCanopen->getSnapshot(state);
  }
}

/**
 * @brief Returns the ROS 2 node pointer.
 * @param None
 * @return std::shared_ptr<RoboteqRos> - Pointer to the internal RoboteqRos node.
 */
std::shared_ptr<RoboteqRos> RoboteqInterface::getRosNode() const
{
  return m_pRos;
}

} // namespace roboteq_hardware_interface
