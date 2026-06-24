#ifndef DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_
#define DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_

/**
 * diff_drive_hardware.hpp
 * ────────────────────────
 * Declares the ros2_control SystemInterface plugin class.
 *
 * Lifecycle of a hardware interface (managed by controller_manager):
 *
 *   on_init()       – called once; reads URDF params, validates joint list
 *   on_configure()  – creates the internal ROS node, subscribers, publishers,
 *                     and starts the spin thread
 *   on_activate()   – zeroes out all state; hardware is live from here
 *   read()          – called every control loop cycle; converts incoming
 *                     /drive_rpm data to wheel rad/s for the framework
 *   write()         – called every control loop cycle; converts wheel rad/s
 *                     commands from the framework to motor RPM and publishes
 *                     to /cmd_rpm
 *   on_deactivate() – publishes zero RPM for a safe stop
 *   on_cleanup()    – shuts down the spin thread and internal node
 */

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace diff_drive_hardware
{

class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  DiffDriveHardware() = default;

  // ── Lifecycle ────────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ─────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop (called by controller_manager at update_rate Hz) ────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Kinematic parameters (read from URDF <hardware> block) ───────────────
  double wheel_radius_     = 0.075;   // m
  double wheel_separation_ = 0.40;    // m, distance between wheel centers
  double gear_ratio_       = 30.0;    // motor revolutions per wheel revolution
  double max_motor_rpm_    = 300.0;   // hard cap; set <=0 to disable

  // ── Joint names discovered from URDF ─────────────────────────────────────
  std::string left_joint_name_;
  std::string right_joint_name_;

  // ── Double-buffered interface values ─────────────────────────────────────
  // The ros2_control framework reads/writes these variables through the
  // StateInterface / CommandInterface pointers returned by export_*().
  double left_vel_cmd_  = 0.0;   // [rad/s] velocity commanded by diff_drive_controller
  double right_vel_cmd_ = 0.0;   // [rad/s]
  double left_vel_state_  = 0.0; // [rad/s] fed back to diff_drive_controller for odom
  double right_vel_state_ = 0.0; // [rad/s]
  double left_pos_state_  = 0.0; // [rad]   integrated wheel angle
  double right_pos_state_ = 0.0; // [rad]

  // ── Internal ROS node (lives inside the hardware interface) ───────────────
  // controller_manager does NOT give us a node to use for arbitrary pub/sub,
  // so we create our own. It spins in a dedicated thread.
  //std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  cmd_rpm_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr drive_rpm_sub_;

  // ── Thread-safe RPM scratch area ─────────────────────────────────────────
  // The subscriber callback runs on the spin thread; read() runs on the
  // control thread. Protect shared state with a mutex.
  std::mutex   rpm_mutex_;
  double latest_left_rpm_  = 0.0;
  double latest_right_rpm_ = 0.0;

  // ── Executor & spin thread ───────────────────────────────────────────────
  //std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  //std::thread spin_thread_;

  // ── Helpers ──────────────────────────────────────────────────────────────
  double radps_to_motor_rpm(double rad_per_sec) const;
  double motor_rpm_to_radps(double rpm) const;
  double clamp_rpm(double rpm) const;
  void   drive_rpm_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

}  // namespace diff_drive_hardware

#endif  // DIFF_DRIVE_HARDWARE__DIFF_DRIVE_HARDWARE_HPP_
