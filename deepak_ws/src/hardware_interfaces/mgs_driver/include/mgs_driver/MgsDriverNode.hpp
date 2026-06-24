/*
 * Name:        MgsDriverNode.hpp
 * Author:      Deepak
 * Date:        2026-06-24
 * Version:     1.0
 * Description: Header file for the C++ SocketCAN Driver for Roboteq MGS1600 Magnetic Guide Sensor.
 */

#ifndef MGS_DRIVER_NODE_HPP_
#define MGS_DRIVER_NODE_HPP_

#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <cassert>

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
  std::string m_canInterface;
  int m_nodeId;

  // --- SocketCAN ---
  int m_canSocket{-1};
  bool initCanSocket();
  void closeCanSocket();
  void sendCanFrame(uint32_t canId, const uint8_t * data, uint8_t dlc);

  // --- RX Thread ---
  std::thread t_rxLoopThread;
  std::atomic<bool> m_rxRunning{false};
  void rxLoop();
  void processTpdo1(const struct can_frame & frame);

  // --- NMT & SDO Commands ---
  void nmtStart();
  void cmdFollowLeft();
  void cmdFollowRight();
  void sendRpdoFollow(bool left, bool right);
  void sendSdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value);

  // --- ROS Publishers ---
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_pubTrackPos;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_pubLeftTrackPos;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_pubRightTrackPos;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubTrackDetect;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubLeftMarker;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubRightMarker;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubTapeCross;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr p_pubStatus;

  // --- ROS Services ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_srvFollowLeft;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_srvFollowRight;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_srvClearFollow;

  void onFollowLeft(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onFollowRight(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onClearFollow(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace mgs_driver

#endif  // MGS_DRIVER_NODE_HPP_
