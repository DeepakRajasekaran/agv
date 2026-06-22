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
  , m_logCounter(0)
  , m_estopActive(0)
  , m_digitalOut(0)
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<std::string>("feedback_topic", "/drive/feedback");
    this->declare_parameter<std::string>("diagnostics_topic", "/drive/diagonistics");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_rpm");
    this->declare_parameter<double>("gear_ratio", 1.0);

    std::string can_interface = this->get_parameter("can_interface").as_string();
    std::string feedback_topic = this->get_parameter("feedback_topic").as_string();
    std::string diagnostics_topic = this->get_parameter("diagnostics_topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
    m_gearRatio = this->get_parameter("gear_ratio").as_double();

    // Initialize Driver
    m_driver = std::make_unique<roboteq_driver::RoboteqCanDriver>(can_interface);
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
    m_srvEstop = this->create_service<std::srvs::srv::Trigger>(
      "~/trigger_estop", std::bind(&RoboteqDriverNode::triggerEstopCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvClearEstop = this->create_service<std::srvs::srv::Trigger>(
      "~/clear_estop", std::bind(&RoboteqDriverNode::clearEstopCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvResetFaults = this->create_service<std::srvs::srv::Trigger>(
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
    if (!m_driver->isConnected())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cannot send command, CAN disconnected!");
      return;
    }
    // drive_rpm is sent from the controllers. Motor needs motor_rpm.
    // motor_rpm = drive_rpm * gear_ratio
    int32_t left_motor_rpm = static_cast<int32_t>(msg->left * m_gearRatio);
    int32_t right_motor_rpm = static_cast<int32_t>(msg->right * m_gearRatio);
    m_driver->sendSpeedCommand(left_motor_rpm, right_motor_rpm);
  }

  void timerCallback()
  {
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
    diag_msg.drive_fault = t.fault_flags;
    diag_msg.temp_controller = t.controller_temp;
    m_pubDiagnostics->publish(diag_msg);

    if (m_logCounter++ % 20 == 0) // 1Hz throttle
    {
      RCLCPP_INFO(this->get_logger(), "Telemetry: %.1fV, RPM: [%d, %d], Enc: [%d, %d]", 
        t.battery_voltage, t.left_rpm, t.right_rpm, t.left_encoder, t.right_encoder);
    }
  }

  void triggerEstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_estopActive = 1;
    m_driver->sendSystemCommand(m_estopActive, 0, m_digitalOut);
    res->success = true;
    res->message = "E-Stop Triggered";
    RCLCPP_WARN(this->get_logger(), "E-STOP Triggered via Service");
  }

  void clearEstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_estopActive = 0;
    m_driver->sendSystemCommand(m_estopActive, 0, m_digitalOut);
    res->success = true;
    res->message = "E-Stop Cleared";
    RCLCPP_INFO(this->get_logger(), "E-STOP Cleared via Service");
  }

  void resetFaultsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    m_driver->sendSystemCommand(m_estopActive, 1, m_digitalOut);
    res->success = true;
    res->message = "Faults Reset";
    RCLCPP_INFO(this->get_logger(), "Faults Reset via Service");
  }

  std::unique_ptr<roboteq_driver::RoboteqCanDriver> m_driver;

  rclcpp::Publisher<custom_interfaces::msg::DriveFeedback>::SharedPtr m_pubFeedback;
  rclcpp::Publisher<custom_interfaces::msg::DriveDiagnostics>::SharedPtr m_pubDiagnostics;
  rclcpp::Subscription<custom_interfaces::msg::WheelRpm>::SharedPtr m_subCmdRpm;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvEstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvClearEstop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvResetFaults;

  rclcpp::TimerBase::SharedPtr m_timer;
  int m_logCounter;
  uint8_t m_estopActive;
  uint8_t m_digitalOut;
  double m_gearRatio;
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
