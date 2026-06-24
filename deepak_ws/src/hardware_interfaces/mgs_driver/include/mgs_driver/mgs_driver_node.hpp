#ifndef MGS_DRIVER_NODE_HPP_
#define MGS_DRIVER_NODE_HPP_

#include <atomic>
#include <mutex>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>

// SocketCAN includes
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace mgs_driver
{

class MgsDriverNode : public rclcpp::Node
{
public:
  explicit MgsDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MgsDriverNode() override;

private:
  // --- ROS Parameters ---
  std::string can_interface_;
  int node_id_;

  // --- SocketCAN ---
  int can_socket_{-1};
  bool init_can_socket();
  void close_can_socket();
  void send_can_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

  // --- RX Thread ---
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  void rx_loop();
  void process_tpdo1(const struct can_frame & frame);

  // --- NMT & SDO Commands ---
  void nmt_start();
  void cmd_follow_left();
  void cmd_follow_right();
  void send_rpdo_follow(bool left, bool right);
  void send_sdo_write_u8(uint16_t index, uint8_t subindex, uint8_t value);

  // --- ROS Publishers ---
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_track_pos_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_track_pos_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_track_pos_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_track_detect_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_left_marker_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_right_marker_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tape_cross_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_status_;

  // --- ROS Services ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_follow_left_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_follow_right_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_follow_;

  void on_follow_left(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_follow_right(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_clear_follow(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace mgs_driver

#endif  // MGS_DRIVER_NODE_HPP_
