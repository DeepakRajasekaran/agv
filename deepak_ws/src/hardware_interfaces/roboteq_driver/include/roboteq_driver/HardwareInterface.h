/*
 * Name:        HardwareInterface.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-18
 * Version:     1.1
 * Description: Declares the HardwareInterface class for ros2_control to bridge with the standalone driver node.
 */

#ifndef ROBOTEQ_DRIVER_HARDWARE_INTERFACE_H
#define ROBOTEQ_DRIVER_HARDWARE_INTERFACE_H

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "custom_interfaces/msg/drive_feedback.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"

namespace roboteq_driver
{

class HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS 2 Callbacks
  void feedbackCallback(const custom_interfaces::msg::DriveFeedback::SharedPtr msg);

  // Configuration Parameters
  double m_wheelRadius;
  double m_wheelBase;
  double m_gearRatio;
  double m_ticksPerRev;
  std::string m_leftJointName;
  std::string m_rightJointName;
  std::string m_feedbackTopic;
  std::string m_cmdTopic;

  // Joint States & Commands exposed to ros2_control
  double m_leftWheelPosition;
  double m_leftWheelVelocity;
  double m_rightWheelPosition;
  double m_rightWheelVelocity;

  double m_leftWheelVelocityCommand;
  double m_rightWheelVelocityCommand;

  // Telemetry state
  std::mutex m_dataMutex;
  double m_leftFeedbackRpm;
  double m_rightFeedbackRpm;
  int32_t m_leftEncoderCounts;
  int32_t m_rightEncoderCounts;

  // Encoder calculation helper variables
  int32_t m_lastLeftTicks;
  int32_t m_lastRightTicks;
  bool m_firstTickRead;

  // ROS 2 Telemetry Support
  rclcpp::Node::SharedPtr m_node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr m_executor;

  // ROS 2 Bridge communication
  rclcpp::Subscription<custom_interfaces::msg::DriveFeedback>::SharedPtr m_subFeedback;
  rclcpp::Publisher<custom_interfaces::msg::WheelRpm>::SharedPtr m_pubCmdRpm;

  // Logging and telemetry control
  int m_logCounter;
};

} // namespace roboteq_driver

#endif // ROBOTEQ_DRIVER_HARDWARE_INTERFACE_H
