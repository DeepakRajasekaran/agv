/*
Name: RoboteqRos.h
Author: Antigravity
Date: 2026-06-23
Version: 1.0
Description: Declares the RoboteqRos node class for ROS 2 pub/sub interfaces.
*/

#ifndef ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_ROS_H_
#define ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_ROS_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "roboteq_hardware_interface/msg/drive_feedback.hpp"
#include "roboteq_hardware_interface/msg/drive_diagnostics.hpp"
#include "roboteq_hardware_interface/RoboteqCanopen.h"

namespace roboteq_hardware_interface
{

class RoboteqInterface; // Forward declaration

class RoboteqRos : public rclcpp::Node
{
public:
  RoboteqRos(RoboteqInterface * p_interface);
  virtual ~RoboteqRos();

  void publishTelemetry(const RoboteqState & state);

private:
  void cmdRpmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void publishTimerCallback();

  void srvEmergencyStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void srvReleaseShutdown(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void srvStopAll(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  RoboteqInterface * m_pInterface;

  // ROS 2 publishers and subscriber
  rclcpp::Publisher<roboteq_hardware_interface::msg::DriveFeedback>::SharedPtr m_driveFeedbackPub;
  rclcpp::Publisher<roboteq_hardware_interface::msg::DriveDiagnostics>::SharedPtr m_driveDiagnosticsPub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_driveRpmPub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_cmdRpmSub;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvEstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvRelease;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvStopAll;

  // Timer for telemetry publishing
  rclcpp::TimerBase::SharedPtr m_telemetryTimer;

  // Cached parameters
  std::string m_canInterface;
  int m_nodeId;
  std::string m_cmdRpmTopic;
  std::string m_driveFeedbackTopic;
  std::string m_driveDiagnosticsTopic;
  std::string m_driveRpmTopic;
  double m_publishRateHz;
  double m_maxRpm;
};

} // namespace roboteq_hardware_interface

#endif // ROBOTEQ_HARDWARE_INTERFACE__ROBOTEQ_ROS_H_
