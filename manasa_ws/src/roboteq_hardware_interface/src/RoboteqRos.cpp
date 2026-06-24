/*
Name: RoboteqRos.cpp
Author: Antigravity
Date: 2026-06-23
Version: 1.0
Description: Implements the RoboteqRos node class for ROS 2 pub/sub interfaces.
*/

#include "roboteq_hardware_interface/RoboteqRos.h"
#include "roboteq_hardware_interface/RoboteqInterface.h"

#include <chrono>

namespace roboteq_hardware_interface
{

/**
 * @brief Constructor for RoboteqRos node, declaring parameters and initializing ROS components.
 * @param p_interface - Pointer to the mediator interface class.
 * @return None
 */
RoboteqRos::RoboteqRos(RoboteqInterface * p_interface)
: Node("roboteq_interface"),
  m_pInterface(p_interface)
{
  // Declare parameters
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<int>("node_id", 1);
  this->declare_parameter<std::string>("cmd_rpm_topic", "/cmd_rpm");
  this->declare_parameter<std::string>("drive_feedback_topic", "/drive_feedback");
  this->declare_parameter<std::string>("drive_diagnostics_topic", "/drive_diagnostics");
  this->declare_parameter<std::string>("drive_rpm_topic", "/drive_rpm");
  this->declare_parameter<double>("publish_rate_hz", 20.0);

  // Get parameters
  this->get_parameter("can_interface", m_canInterface);
  this->get_parameter("node_id", m_nodeId);
  this->get_parameter("cmd_rpm_topic", m_cmdRpmTopic);
  this->get_parameter("drive_feedback_topic", m_driveFeedbackTopic);
  this->get_parameter("drive_diagnostics_topic", m_driveDiagnosticsTopic);
  this->get_parameter("drive_rpm_topic", m_driveRpmTopic);
  this->get_parameter("publish_rate_hz", m_publishRateHz);

  // Initialize publishers
  m_driveFeedbackPub = this->create_publisher<roboteq_hardware_interface::msg::DriveFeedback>(
    m_driveFeedbackTopic, rclcpp::QoS(10).reliable());
  m_driveDiagnosticsPub = this->create_publisher<roboteq_hardware_interface::msg::DriveDiagnostics>(
    m_driveDiagnosticsTopic, rclcpp::QoS(10).reliable());
  m_driveRpmPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    m_driveRpmTopic, rclcpp::QoS(10).reliable());

  // Initialize subscription
  m_cmdRpmSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    m_cmdRpmTopic,
    rclcpp::QoS(10).reliable(),
    std::bind(&RoboteqRos::cmdRpmCallback, this, std::placeholders::_1));

  // Initialize services
  m_srvEstop = this->create_service<std_srvs::srv::Trigger>(
    "emergency_stop",
    std::bind(&RoboteqRos::srvEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));

  m_srvRelease = this->create_service<std_srvs::srv::Trigger>(
    "release_shutdown",
    std::bind(&RoboteqRos::srvReleaseShutdown, this, std::placeholders::_1, std::placeholders::_2));

  m_srvStopAll = this->create_service<std_srvs::srv::Trigger>(
    "stop_all",
    std::bind(&RoboteqRos::srvStopAll, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize wall timer for polling and publishing telemetry
  double periodSec = 1.0 / m_publishRateHz;
  auto duration = std::chrono::duration<double>(periodSec);
  m_telemetryTimer = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(duration),
    std::bind(&RoboteqRos::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "RoboteqRos node ready | CAN Interface: %s | Node ID: %d | Freq: %.1f Hz",
    m_canInterface.c_str(), m_nodeId, m_publishRateHz);
}

/**
 * @brief Destructor for the RoboteqRos node class.
 * @param None
 * @return None
 */
RoboteqRos::~RoboteqRos()
{
}

/**
 * @brief Callback for cmd_rpm topic. Receives left/right wheel velocity commands.
 * @param msg - Shared pointer to standard Float32MultiArray message.
 * @return None
 */
void RoboteqRos::cmdRpmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Received command on %s with fewer than 2 elements! Ignoring.", m_cmdRpmTopic.c_str());
    return;
  }

  float leftRpm = msg->data[0];
  float rightRpm = msg->data[1];

  m_pInterface->handleCmdRpm(leftRpm, rightRpm);
}

void RoboteqRos::srvEmergencyStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  m_pInterface->emergencyStop();
  response->success = true;
  response->message = "Emergency Stop commanded.";
  RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP TRIGGERED VIA SERVICE");
}

void RoboteqRos::srvReleaseShutdown(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  m_pInterface->releaseShutdown();
  response->success = true;
  response->message = "Release Shutdown commanded.";
  RCLCPP_INFO(this->get_logger(), "SHUTDOWN RELEASED VIA SERVICE");
}

void RoboteqRos::srvStopAll(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  m_pInterface->stopAll();
  response->success = true;
  response->message = "Stop All commanded.";
  RCLCPP_INFO(this->get_logger(), "STOP ALL TRIGGERED VIA SERVICE");
}

/**
 * @brief Periodic timer callback to query and publish current driver states.
 * @param None
 * @return None
 */
void RoboteqRos::publishTimerCallback()
{
  RoboteqState state;
  m_pInterface->getDriverState(state);
  publishTelemetry(state);
}

/**
 * @brief Formats and publishes DriveFeedback and DriveDiagnostics messages.
 * @param state - The current RoboteqState snapshot containing latest polled CAN open values.
 * @return None
 */
void RoboteqRos::publishTelemetry(const RoboteqState & state)
{
  auto now = this->get_clock()->now();

  // 1. Publish DriveFeedback
  roboteq_hardware_interface::msg::DriveFeedback feedbackMsg;
  feedbackMsg.header.stamp = now;
  feedbackMsg.header.frame_id = "base_link";
  feedbackMsg.left_rpm = state.encoderSpeedCh1;
  feedbackMsg.right_rpm = state.encoderSpeedCh2;
  feedbackMsg.left_amps = state.motorAmpsCh1;
  feedbackMsg.right_amps = state.motorAmpsCh2;
  feedbackMsg.left_encoder = state.encoderAbsCh1;
  feedbackMsg.right_encoder = state.encoderAbsCh2;

  m_driveFeedbackPub->publish(feedbackMsg);

  // Publish /drive_rpm for diff_drive_hardware
  std_msgs::msg::Float32MultiArray rpmMsg;
  rpmMsg.data.push_back(state.encoderSpeedCh1);
  rpmMsg.data.push_back(state.encoderSpeedCh2);
  m_driveRpmPub->publish(rpmMsg);

  // 2. Publish DriveDiagnostics
  roboteq_hardware_interface::msg::DriveDiagnostics diagMsg;
  diagMsg.header.stamp = now;
  diagMsg.header.frame_id = "base_link";
  diagMsg.internal_volts = state.internalVolts;
  diagMsg.battery_volts = state.batteryVolts;
  diagMsg.supply_5v = state.supply5v;
  diagMsg.temp_heatsink_ch1 = state.tempHeatsinkCh1;
  diagMsg.temp_heatsink_ch2 = state.tempHeatsinkCh2;
  diagMsg.temp_mcu = state.tempMcu;
  diagMsg.fault_flags = state.faultFlags;
  diagMsg.status_flags = state.statusFlags;
  diagMsg.motor_status_ch1 = state.motorStatusCh1;
  diagMsg.motor_status_ch2 = state.motorStatusCh2;
  diagMsg.hall_sensor_ch1 = state.hallSensorCh1;
  diagMsg.hall_sensor_ch2 = state.hallSensorCh2;
  diagMsg.fault_flags_decoded = state.faultFlagsDecoded;
  diagMsg.status_flags_decoded = state.statusFlagsDecoded;

  m_driveDiagnosticsPub->publish(diagMsg);
}

} // namespace roboteq_hardware_interface
