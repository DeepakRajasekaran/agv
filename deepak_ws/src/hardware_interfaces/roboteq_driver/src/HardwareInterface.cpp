/*
 * Name:        HardwareInterface.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-18
 * Version:     1.1
 * Description: Implements the HardwareInterface class for ros2_control to bridge with the standalone driver node.
 */

#include "roboteq_driver/HardwareInterface.h"

#include <cmath>
#include <cassert>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roboteq_driver
{

/**
 * @brief  Initializes the hardware interface, reads URDF params, and sets up publishers.
 * @param  params  Component initialization parameters containing HardwareInfo.
 * @return Success or error code.
 */
hardware_interface::CallbackReturn HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Preconditions
  assert(!params.hardware_info.joints.empty());

  m_logCounter = 0;
  m_leftWheelPosition = 0.0;
  m_leftWheelVelocity = 0.0;
  m_rightWheelPosition = 0.0;
  m_rightWheelVelocity = 0.0;
  m_leftWheelVelocityCommand = 0.0;
  m_rightWheelVelocityCommand = 0.0;

  m_leftFeedbackRpm = 0.0;
  m_rightFeedbackRpm = 0.0;
  m_leftEncoderCounts = 0;
  m_rightEncoderCounts = 0;

  m_lastLeftTicks = 0;
  m_lastRightTicks = 0;
  m_firstTickRead = false;

  // Retrieve parameters from URDF configuration
  try
  {
    m_wheelRadius = std::stod(params.hardware_info.hardware_parameters.at("wheel_radius"));
    m_wheelBase = std::stod(params.hardware_info.hardware_parameters.at("wheel_base"));
    m_gearRatio = std::stod(params.hardware_info.hardware_parameters.at("gear_ratio"));
    m_ticksPerRev = std::stod(params.hardware_info.hardware_parameters.at("ticks_per_rev"));
    m_leftJointName = params.hardware_info.hardware_parameters.at("left_joint_name");
    m_rightJointName = params.hardware_info.hardware_parameters.at("right_joint_name");
    m_feedbackTopic = params.hardware_info.hardware_parameters.at("feedback_topic");
    m_cmdTopic = params.hardware_info.hardware_parameters.at("cmd_topic");
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HardwareInterface"), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Preconditions validation
  assert(m_wheelRadius > 0.0);
  assert(m_wheelBase > 0.0);
  assert(m_gearRatio > 0.0);
  assert(m_ticksPerRev > 0.0);

  // Initialize ROS 2 node and executors
  m_node = std::make_shared<rclcpp::Node>("roboteq_hw_interface_node");
  m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Publisher for commands to the driver node
  m_pubCmdRpm = m_node->create_publisher<custom_interfaces::msg::WheelRpm>(m_cmdTopic, 10);
  
  // Subscriber for feedback from the driver node
  m_subFeedback = m_node->create_subscription<custom_interfaces::msg::DriveFeedback>(
    m_feedbackTopic, 10,
    std::bind(&HardwareInterface::feedbackCallback, this, std::placeholders::_1));

  m_executor->add_node(m_node);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief  Exports state interfaces (positions, velocities) for the controllers.
 * @return Vector of state interfaces.
 */
std::vector<hardware_interface::StateInterface> HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    m_leftJointName, hardware_interface::HW_IF_POSITION, &m_leftWheelPosition));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    m_leftJointName, hardware_interface::HW_IF_VELOCITY, &m_leftWheelVelocity));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    m_rightJointName, hardware_interface::HW_IF_POSITION, &m_rightWheelPosition));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    m_rightJointName, hardware_interface::HW_IF_VELOCITY, &m_rightWheelVelocity));

  return state_interfaces;
}

/**
 * @brief  Exports velocity command interfaces for the controllers.
 * @return Vector of command interfaces.
 */
std::vector<hardware_interface::CommandInterface> HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    m_leftJointName, hardware_interface::HW_IF_VELOCITY, &m_leftWheelVelocityCommand));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    m_rightJointName, hardware_interface::HW_IF_VELOCITY, &m_rightWheelVelocityCommand));

  return command_interfaces;
}

/**
 * @brief  Configures the hardware.
 * @return Success callback.
 */
hardware_interface::CallbackReturn HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(m_node->get_logger(), "Configuring Hardware Interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief  Cleans up the hardware resources.
 * @return Success callback.
 */
hardware_interface::CallbackReturn HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(m_node->get_logger(), "Cleaning up Hardware Interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief  Activates the hardware interface.
 * @return Success callback.
 */
hardware_interface::CallbackReturn HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(m_node->get_logger(), "Activating Hardware Interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief  Deactivates the hardware interface.
 * @return Success callback.
 */
hardware_interface::CallbackReturn HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(m_node->get_logger(), "Deactivating Hardware Interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief  Reads joints feedback (integrated position & velocity) over ROS.
 * @param  time    Current time.
 * @param  period  Time period since last update.
 * @return Return type indicating success or error.
 */
hardware_interface::return_type HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double delta_seconds = period.seconds();
  assert(delta_seconds > 0.0);

  // Spin ROS executor to receive feedback from the driver node
  m_executor->spin_some();

  std::lock_guard<std::mutex> lock(m_dataMutex);

  // Preconditions
  assert(std::isfinite(m_leftWheelPosition));
  assert(std::isfinite(m_rightWheelPosition));

  // Initialize ticks on the first cycle
  if (!m_firstTickRead)
  {
    m_lastLeftTicks = m_leftEncoderCounts;
    m_lastRightTicks = m_rightEncoderCounts;
    m_firstTickRead = true;
  }

  // Calculate encoder tick difference, handling integer wrapping correctly
  int32_t delta_left_ticks = m_leftEncoderCounts - m_lastLeftTicks;
  int32_t delta_right_ticks = m_rightEncoderCounts - m_lastRightTicks;

  m_lastLeftTicks = m_leftEncoderCounts;
  m_lastRightTicks = m_rightEncoderCounts;

  // Convert delta ticks to wheel position (radians)
  double left_pos_delta = (static_cast<double>(delta_left_ticks) / (m_ticksPerRev * m_gearRatio)) * 2.0 * M_PI;
  double right_pos_delta = (static_cast<double>(delta_right_ticks) / (m_ticksPerRev * m_gearRatio)) * 2.0 * M_PI;

  m_leftWheelPosition += left_pos_delta;
  m_rightWheelPosition += right_pos_delta;

  // Compute velocities (rad/s) from feedback drive RPM (wheel RPM)
  m_leftWheelVelocity = m_leftFeedbackRpm * (2.0 * M_PI / 60.0);
  m_rightWheelVelocity = m_rightFeedbackRpm * (2.0 * M_PI / 60.0);

  // Postconditions
  assert(std::isfinite(m_leftWheelPosition));
  assert(std::isfinite(m_rightWheelPosition));

  // Throttled logging
  if (m_logCounter++ % 100 == 0)
  {
    RCLCPP_INFO(m_node->get_logger(),
      "Joints: Left Pos=%.3f Rad, Vel=%.3f Rad/s | Right Pos=%.3f Rad, Vel=%.3f Rad/s",
      m_leftWheelPosition, m_leftWheelVelocity,
      m_rightWheelPosition, m_rightWheelVelocity);
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief  Writes joint velocity commands to motor speeds (RPM) and publishes to ROS.
 * @param  time    Current time.
 * @param  period  Time period since last update.
 * @return Return type indicating success or error.
 */
hardware_interface::return_type HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Preconditions
  assert(std::isfinite(m_leftWheelVelocityCommand));
  assert(std::isfinite(m_rightWheelVelocityCommand));

  // Convert joint velocities command (rad/s) to target drive RPM (wheel RPM)
  // The standalone driver node applies the gear reduction to get motor RPM.
  double left_rpm = m_leftWheelVelocityCommand * 60.0 / (2.0 * M_PI);
  double right_rpm = m_rightWheelVelocityCommand * 60.0 / (2.0 * M_PI);

  // Publish speed command to the standalone driver node over ROS
  auto cmd_msg = custom_interfaces::msg::WheelRpm();
  cmd_msg.left = static_cast<float>(left_rpm);
  cmd_msg.right = static_cast<float>(right_rpm);
  m_pubCmdRpm->publish(cmd_msg);

  return hardware_interface::return_type::OK;
}

/**
 * @brief  Callback to receive telemetry from the driver node.
 * @param  msg  Telemetry message.
 */
void HardwareInterface::feedbackCallback(const custom_interfaces::msg::DriveFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  m_leftEncoderCounts = msg->counts_left;
  m_rightEncoderCounts = msg->counts_right;
  m_leftFeedbackRpm = msg->feedback_left;
  m_rightFeedbackRpm = msg->feedback_right;
}

} // namespace roboteq_driver

PLUGINLIB_EXPORT_CLASS(
  roboteq_driver::HardwareInterface, hardware_interface::SystemInterface)
