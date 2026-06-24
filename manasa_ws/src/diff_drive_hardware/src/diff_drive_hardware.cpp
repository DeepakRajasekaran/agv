/**
 * diff_drive_hardware.cpp
 * ────────────────────────
 * Implementation of the ros2_control SystemInterface plugin.
 *
 * Conversion chain (both directions)
 * ────────────────────────────────────
 *  cmd_vel (linear.x, angular.z)
 *      └─ diff_drive_controller (built-in ros2_controllers pkg)
 *              │ CommandInterface: left_wheel_joint/velocity [rad/s]
 *              │                  right_wheel_joint/velocity [rad/s]
 *              ▼
 *         write()  ← this file
 *              │  radps_to_motor_rpm():
 *              │    wheel rad/s → wheel RPM → motor RPM (× gear_ratio)
 *              ▼
 *         /cmd_rpm  [Float32MultiArray: left_rpm, right_rpm]
 *              └─ your motor driver / microcontroller
 *
 *  /drive_rpm  [Float32MultiArray: left_rpm, right_rpm]
 *      └─ your motor driver / microcontroller (encoder feedback)
 *              │
 *              ▼
 *         read()  ← this file
 *              │  motor_rpm_to_radps():
 *              │    motor RPM / gear_ratio → wheel RPM → wheel rad/s
 *              │  integrate: pos += vel × dt
 *              ▼
 *         StateInterface: left_wheel_joint/velocity  [rad/s]
 *                         left_wheel_joint/position  [rad]
 *                         right_wheel_joint/velocity [rad/s]
 *                         right_wheel_joint/position [rad]
 *              └─ diff_drive_controller reads these → publishes /odom + TF
 */

#include "diff_drive_hardware/diff_drive_hardware.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diff_drive_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init  –  called ONCE when the plugin is first loaded from the URDF
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Always call the parent first — it populates info_ (used below)
  info_ = params.hardware_info;
  
  //if (hardware_interface::SystemInterface::on_init(params.hardware_info) !=
  //  hardware_interface::CallbackReturn::SUCCESS)
  //{
  //  return hardware_interface::CallbackReturn::ERROR;
  //}

  // ── Read kinematic params from URDF <hardware> → <param> ─────────────────
  // If a param is missing we log a warning and fall back to the header default.
  auto get_param = [&](const std::string & key, double fallback) -> double {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"),
        "URDF param '%s' not found; using default %.4f", key.c_str(), fallback);
      return fallback;
    }
    return std::stod(it->second);
  };

  wheel_radius_     = get_param("wheel_radius",    0.075);
  wheel_separation_ = get_param("wheel_separation", 0.40);
  gear_ratio_       = get_param("gear_ratio",       30.0);
  max_motor_rpm_    = get_param("max_motor_rpm",   300.0);

  if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0 || gear_ratio_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
      "wheel_radius, wheel_separation, and gear_ratio must all be > 0.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Validate joint count ─────────────────────────────────────────────────
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
      "Expected exactly 2 joints in the <ros2_control> block; found %zu.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Identify left / right joints by name ─────────────────────────────────
  // Joint names in the URDF must contain "left" or "right" (case-sensitive).
  for (const auto & joint : info_.joints) {
    if (joint.name.find("left")  != std::string::npos) left_joint_name_  = joint.name;
    if (joint.name.find("right") != std::string::npos) right_joint_name_ = joint.name;
  }

  if (left_joint_name_.empty() || right_joint_name_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
      "Cannot determine left/right joints. "
      "Joint names must contain 'left' or 'right'. Found: '%s', '%s'.",
      info_.joints[0].name.c_str(), info_.joints[1].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Validate command + state interfaces per joint ─────────────────────────
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' must have exactly one command interface: 'velocity'.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' must have exactly two state interfaces: 'position' and 'velocity'.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
    "[on_init] OK — left='%s', right='%s' | "
    "wheel_radius=%.4f m, wheel_separation=%.4f m, gear_ratio=%.1f, max_rpm=%.1f",
    left_joint_name_.c_str(), right_joint_name_.c_str(),
    wheel_radius_, wheel_separation_, gear_ratio_, max_motor_rpm_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure  –  set up pub/sub and the spin thread
// Called when controller_manager transitions the hardware from Unconfigured
// to Inactive state (happens before controllers are loaded).
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Use the node provided by ros2_control (much more reliable)
  node_ = this->get_node();

  // Publisher
  cmd_rpm_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/cmd_rpm", rclcpp::QoS(10).reliable());

  // Subscriber
  drive_rpm_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/drive_rpm",
    rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      drive_rpm_callback(msg);
    });

  RCLCPP_INFO(node_->get_logger(),
    "[on_configure] Internal node ready. Publishing /cmd_rpm, subscribed to /drive_rpm.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate  –  hardware is now live; zero out everything for a clean start
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  left_vel_cmd_  = right_vel_cmd_  = 0.0;
  left_vel_state_  = right_vel_state_  = 0.0;
  left_pos_state_  = right_pos_state_  = 0.0;
  {
    std::lock_guard<std::mutex> lock(rpm_mutex_);
    latest_left_rpm_ = latest_right_rpm_ = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "[on_activate] Hardware ACTIVE.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate  –  send zero RPM immediately for a safe stop
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std_msgs::msg::Float32MultiArray zero;
  zero.data = {0.0f, 0.0f};
  cmd_rpm_pub_->publish(zero);
  RCLCPP_WARN(rclcpp::get_logger("DiffDriveHardware"),
    "[on_deactivate] Published zero RPM — hardware STOPPED.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup  –  tear down the spin thread and internal node
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DiffDriveHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drive_rpm_sub_.reset();
  cmd_rpm_pub_.reset();
  node_.reset();   // Let ros2_control manage cleanup

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), 
    "[on_cleanup] Publishers and subscribers cleaned up.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────
// Returns pointers to our internal variables.  The diff_drive_controller reads
// these every cycle to compute odometry.  Pointer ownership stays here.
std::vector<hardware_interface::StateInterface>
DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(left_joint_name_,  hardware_interface::HW_IF_POSITION, &left_pos_state_);
  si.emplace_back(left_joint_name_,  hardware_interface::HW_IF_VELOCITY, &left_vel_state_);
  si.emplace_back(right_joint_name_, hardware_interface::HW_IF_POSITION, &right_pos_state_);
  si.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &right_vel_state_);
  return si;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
// Returns pointers to our internal variables.  The diff_drive_controller writes
// to these every cycle with the desired wheel angular velocity.
std::vector<hardware_interface::CommandInterface>
DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(left_joint_name_,  hardware_interface::HW_IF_VELOCITY, &left_vel_cmd_);
  ci.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &right_vel_cmd_);
  return ci;
}

// ─────────────────────────────────────────────────────────────────────────────
// read()  –  called every control loop cycle (e.g. 50 Hz)
// ─────────────────────────────────────────────────────────────────────────────
// Grabs the latest /drive_rpm values (written by the subscriber callback on the
// spin thread), converts motor RPM → wheel rad/s, and integrates to position.
// The framework then makes these values available to the diff_drive_controller
// through the StateInterfaces we exported above.
hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double left_rpm, right_rpm;
  {
    std::lock_guard<std::mutex> lock(rpm_mutex_);
    left_rpm  = latest_left_rpm_;
    right_rpm = latest_right_rpm_;
  }

  // Motor RPM → wheel angular velocity [rad/s]
  left_vel_state_  = motor_rpm_to_radps(left_rpm);
  right_vel_state_ = motor_rpm_to_radps(right_rpm);

  // Integrate wheel angular velocity → wheel angle [rad]  (Euler)
  const double dt = period.seconds();
  left_pos_state_  += left_vel_state_  * dt;
  right_pos_state_ += right_vel_state_ * dt;

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write()  –  called every control loop cycle (e.g. 50 Hz)
// ─────────────────────────────────────────────────────────────────────────────
// The diff_drive_controller has already written the desired wheel angular
// velocities into left_vel_cmd_ / right_vel_cmd_ via the CommandInterfaces.
// Convert those to motor RPM and publish /cmd_rpm.
hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // removed slow logging from RT loop

  const double left_rpm  = clamp_rpm(radps_to_motor_rpm(left_vel_cmd_));
  const double right_rpm = clamp_rpm(radps_to_motor_rpm(right_vel_cmd_));

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(left_rpm), static_cast<float>(right_rpm)};
  cmd_rpm_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

// Wheel linear angular velocity [rad/s] → motor shaft RPM
// Formula: wheel_RPM = (rad/s × 60) / (2π)   ;   motor_RPM = wheel_RPM × gear_ratio
double DiffDriveHardware::radps_to_motor_rpm(double rad_per_sec) const
{
  const double wheel_rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
  return wheel_rpm * gear_ratio_;
}

// Motor shaft RPM → wheel angular velocity [rad/s]
double DiffDriveHardware::motor_rpm_to_radps(double rpm) const
{
  const double wheel_rpm = rpm / gear_ratio_;
  return wheel_rpm * 2.0 * M_PI / 60.0;
}

// Clamp motor RPM to ±max_motor_rpm_ (disabled when max_motor_rpm_ <= 0)
double DiffDriveHardware::clamp_rpm(double rpm) const
{
  if (max_motor_rpm_ > 0.0) {
    return std::max(-max_motor_rpm_, std::min(max_motor_rpm_, rpm));
  }
  return rpm;
}

// Subscriber callback (runs on the spin thread)
void DiffDriveHardware::drive_rpm_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("DiffDriveHardware"),
      "/drive_rpm message must have at least 2 elements: [left_rpm, right_rpm]. Ignoring.");
    return;
  }
  std::lock_guard<std::mutex> lock(rpm_mutex_);
  latest_left_rpm_  = static_cast<double>(msg->data[0]);
  latest_right_rpm_ = static_cast<double>(msg->data[1]);
}

}  // namespace diff_drive_hardware

// This macro generates the plugin export symbol that pluginlib looks for at
// runtime when loading the shared library listed in the plugin XML.
PLUGINLIB_EXPORT_CLASS(
  diff_drive_hardware::DiffDriveHardware,
  hardware_interface::SystemInterface)
