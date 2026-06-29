/*
 * Name:        roboteq_driver_node.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-22
 * Version:     1.0
 * Description: Standalone ROS 2 Node that wraps the RoboteqCanDriver library.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "custom_interfaces/msg/drive_feedback.hpp"
#include "custom_interfaces/msg/drive_diagnostics.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"

#include "roboteq_driver/RoboteqCanDriver.h"

using namespace std::chrono_literals;

class RoboteqDriverNode : public rclcpp::Node
{
public:
  RoboteqDriverNode()
  : Node("roboteq_driver_node")
  , m_estopActive(0)
  , m_quickstopActive(0)
  , m_digitalOut(0)
  , m_cmdTimeoutActive(false)
  {
    m_lastCmdTime = this->now();

    // Declare and get parameters
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<std::string>("feedback_topic", "/drive/feedback");
    this->declare_parameter<std::string>("diagnostics_topic", "/drive/diagonistics");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_rpm");
    this->declare_parameter<double>("gear_ratio", 1.0);
    this->declare_parameter<double>("max_rpm", 3000.0);
    this->declare_parameter<int>("node_id", 1);

    std::string can_interface = this->get_parameter("can_interface").as_string();
    std::string feedback_topic = this->get_parameter("feedback_topic").as_string();
    std::string diagnostics_topic = this->get_parameter("diagnostics_topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
    m_gearRatio = this->get_parameter("gear_ratio").as_double();
    m_maxRpm = this->get_parameter("max_rpm").as_double();
    int node_id = static_cast<int>(this->get_parameter("node_id").as_int());

    // Initialize Driver
    m_driver = std::make_unique<roboteq_driver::RoboteqCanDriver>(can_interface, node_id);
    if (!m_driver->connect())
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to bind to CAN interface: %s", can_interface.c_str());
      throw std::runtime_error("CAN interface binding failed");
    }
    m_driver->start();
    RCLCPP_INFO(this->get_logger(), "Roboteq CAN Driver started on %s | Gear Ratio: %.2f", can_interface.c_str(), m_gearRatio);

    // Publishers
    m_pubFeedback = this->create_publisher<custom_interfaces::msg::DriveFeedback>(feedback_topic, 10);
    m_pubDiagnostics = this->create_publisher<custom_interfaces::msg::DriveDiagnostics>(diagnostics_topic, 10);

    // Subscriber
    m_subCmdRpm = this->create_subscription<custom_interfaces::msg::WheelRpm>(
      cmd_topic, 10, std::bind(&RoboteqDriverNode::cmdRpmCallback, this, std::placeholders::_1));

    // Services
    m_srvSetEstop = this->create_service<std_srvs::srv::Trigger>(
      "~/set_estop", std::bind(&RoboteqDriverNode::setEstopCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvResetEstop = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_estop", std::bind(&RoboteqDriverNode::resetEstopCallback, this, std::placeholders::_1, std::placeholders::_2));

    m_srvSetQuickstop = this->create_service<std_srvs::srv::Trigger>(
      "~/set_quickstop", std::bind(&RoboteqDriverNode::setQuickstopCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvResetQuickstop = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_quickstop", std::bind(&RoboteqDriverNode::resetQuickstopCallback, this, std::placeholders::_1, std::placeholders::_2));

    m_srvResetFaults = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_faults", std::bind(&RoboteqDriverNode::resetFaultsCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Timer for publishing telemetry
    m_timer = this->create_wall_timer(50ms, std::bind(&RoboteqDriverNode::timerCallback, this)); // 20Hz
  }

  ~RoboteqDriverNode()
  {
    if (m_driver)
    {
      m_driver->stop();
    }
  }

private:
  void cmdRpmCallback(const custom_interfaces::msg::WheelRpm::SharedPtr msg)
  {
    m_lastCmdTime = this->now();
    if (m_cmdTimeoutActive) {
      RCLCPP_INFO(this->get_logger(), "Command received, resuming motor operations.");
      m_cmdTimeoutActive = false;
    }

    if (!m_driver->isConnected())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cannot send command, CAN disconnected!");
      return;
    }
    if (m_estopActive || m_quickstopActive)
    {
      // Hardware E-Stop or Software Quickstop takes priority
      return;
    }
    // drive_rpm is sent from the controllers. Motor needs motor_rpm.
    // motor_rpm = drive_rpm * gear_ratio
    double left_motor_rpm = msg->left * m_gearRatio;
    double right_motor_rpm = msg->right * m_gearRatio;

    // Convert to Roboteq command units (-1000 to +1000) representing percentage of max RPM
    int32_t left_cmd = static_cast<int32_t>((left_motor_rpm / m_maxRpm) * 1000.0);
    int32_t right_cmd = static_cast<int32_t>((right_motor_rpm / m_maxRpm) * 1000.0);

    // Clamp values
    left_cmd = std::max(-1000, std::min(1000, left_cmd));
    right_cmd = std::max(-1000, std::min(1000, right_cmd));

    m_driver->sendSpeedCommand(left_cmd, right_cmd);
  }

  void timerCallback()
  {
    // Command Timeout logic
    if (!m_estopActive && !m_quickstopActive && (this->now() - m_lastCmdTime).seconds() > 0.5)
    {
      if (!m_cmdTimeoutActive) {
        RCLCPP_WARN(this->get_logger(), "Command Timeout! No RPM commands received for 0.5s. Stopping motors.");
        m_cmdTimeoutActive = true;
      }
      m_driver->sendSpeedCommand(0, 0);
    }

    if (!m_driver->isConnected())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "CAN Connection Lost!");
      return;
    }

    roboteq_driver::TelemetryData t = m_driver->getTelemetry();

    // Publish Feedback (convert motor_rpm back to drive_rpm for controllers)
    auto fb_msg = custom_interfaces::msg::DriveFeedback();
    fb_msg.counts_left = t.left_encoder;
    fb_msg.counts_right = t.right_encoder;
    fb_msg.feedback_left = static_cast<float>(t.left_rpm) / static_cast<float>(m_gearRatio);
    fb_msg.feedback_right = static_cast<float>(t.right_rpm) / static_cast<float>(m_gearRatio);
    m_pubFeedback->publish(fb_msg);

    // Publish Diagnostics
    auto diag_msg = custom_interfaces::msg::DriveDiagnostics();
    diag_msg.voltage = t.battery_voltage;
    diag_msg.current_left = t.current_left;
    diag_msg.current_right = t.current_right;
    diag_msg.drive_fault = static_cast<uint8_t>(t.fault_flags);
    diag_msg.temp_controller = 0; // Not polled via SDO
    m_pubDiagnostics->publish(diag_msg);

    if (t.fault_flags != 0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Motor Controller Fault Detected! Fault Flags: 0x%04X", t.fault_flags);
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Telemetry: %.1fV, RPM: [%d, %d], Enc: [%d, %d], Fault: 0x%04X, Status: 0x%04X", 
      t.battery_voltage, t.left_rpm, t.right_rpm, t.left_encoder, t.right_encoder, t.fault_flags, t.status_flags);
  }

  void setEstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_estopActive = 1;
    m_driver->triggerEstop();
    res->success = true;
    res->message = "Hardware E-Stop Set (!EX via CAN)";
    RCLCPP_WARN(this->get_logger(), "E-STOP Set via Service");
  }

  void resetEstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_estopActive = 0;
    m_driver->clearEstop();
    // Safety: ensure speeds are 0 before releasing
    m_driver->sendSpeedCommand(0, 0);
    res->success = true;
    res->message = "Hardware E-Stop Reset (!MG via CAN)";
    RCLCPP_INFO(this->get_logger(), "E-STOP Reset via Service");
  }

  void setQuickstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_quickstopActive = 1;
    m_driver->triggerQuickstop();
    res->success = true;
    res->message = "Software Quickstop Set (!MS via CAN)";
    RCLCPP_WARN(this->get_logger(), "Quickstop Set via Service");
  }

  void resetQuickstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_quickstopActive = 0;
    res->success = true;
    res->message = "Software Quickstop Reset";
    RCLCPP_INFO(this->get_logger(), "Quickstop Reset via Service");
  }

  void resetFaultsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_driver->resetEncoders();
    res->success = true;
    res->message = "Encoders Reset";
    RCLCPP_INFO(this->get_logger(), "Encoders Reset via Service");
  }

  std::unique_ptr<roboteq_driver::RoboteqCanDriver> m_driver;

  rclcpp::Publisher<custom_interfaces::msg::DriveFeedback>::SharedPtr m_pubFeedback;
  rclcpp::Publisher<custom_interfaces::msg::DriveDiagnostics>::SharedPtr m_pubDiagnostics;
  rclcpp::Subscription<custom_interfaces::msg::WheelRpm>::SharedPtr m_subCmdRpm;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvSetEstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvResetEstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvSetQuickstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvResetQuickstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvResetFaults;

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Time m_lastCmdTime;
  uint8_t m_estopActive;
  uint8_t m_quickstopActive;
  uint8_t m_digitalOut;
  bool m_cmdTimeoutActive;
  double m_gearRatio;
  double m_maxRpm;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<RoboteqDriverNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception & e)
  {
    std::cerr << "Exception in main: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
