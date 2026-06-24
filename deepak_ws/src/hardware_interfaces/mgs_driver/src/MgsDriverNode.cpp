/*
 * Name:        MgsDriverNode.cpp
 * Author:      Deepak
 * Date:        2026-06-24
 * Version:     1.0
 * Description: Implementation of the C++ SocketCAN Driver for Roboteq MGS1600 Magnetic Guide Sensor.
 */

#include "mgs_driver/MgsDriverNode.hpp"

#include <iostream>
#include <chrono>

namespace mgs_driver
{

constexpr uint32_t NMT_COB_ID = 0x000;
constexpr uint8_t NMT_START = 0x01;

/**
 * @brief  Constructs the MgsDriverNode, initializes parameters, publishers, and the CAN socket.
 * @param  options  Node options for ROS 2 configuration.
 */
MgsDriverNode::MgsDriverNode(const rclcpp::NodeOptions & options)
: Node("mgs_driver", options)
{
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<int>("node_id", 5);

  m_canInterface = this->get_parameter("can_interface").as_string();
  m_nodeId = this->get_parameter("node_id").as_int();

  assert(!m_canInterface.empty());
  assert(m_nodeId > 0 && m_nodeId <= 127);

  // Initialize Publishers
  p_pubTrackPos = this->create_publisher<std_msgs::msg::Float32>("sensor/track_position", 10);
  p_pubLeftTrackPos = this->create_publisher<std_msgs::msg::Float32>("sensor/left_track_position", 10);
  p_pubRightTrackPos = this->create_publisher<std_msgs::msg::Float32>("sensor/right_track_position", 10);
  
  p_pubTrackDetect = this->create_publisher<std_msgs::msg::Bool>("sensor/track_detect", 10);
  p_pubLeftMarker = this->create_publisher<std_msgs::msg::Bool>("sensor/left_marker", 10);
  p_pubRightMarker = this->create_publisher<std_msgs::msg::Bool>("sensor/right_marker", 10);
  p_pubTapeCross = this->create_publisher<std_msgs::msg::Bool>("sensor/tape_cross", 10);
  p_pubStatus = this->create_publisher<std_msgs::msg::UInt16>("sensor/status", 10);

  // Initialize Services for Track Switching
  p_srvFollowLeft = this->create_service<std_srvs::srv::Trigger>(
    "sensor/follow_left",
    std::bind(&MgsDriverNode::onFollowLeft, this, std::placeholders::_1, std::placeholders::_2));
  
  p_srvFollowRight = this->create_service<std_srvs::srv::Trigger>(
    "sensor/follow_right",
    std::bind(&MgsDriverNode::onFollowRight, this, std::placeholders::_1, std::placeholders::_2));
    
  p_srvClearFollow = this->create_service<std_srvs::srv::Trigger>(
    "sensor/clear_follow",
    std::bind(&MgsDriverNode::onClearFollow, this, std::placeholders::_1, std::placeholders::_2));

  bool ok = initCanSocket();
  if (ok) {
    RCLCPP_INFO(this->get_logger(), "SocketCAN initialized on %s for Node ID %d", m_canInterface.c_str(), m_nodeId);
    
    // Put node into Operational so TPDO1 streaming starts
    nmtStart();
    
    // Start RX Thread
    m_rxRunning = true;
    t_rxLoopThread = std::thread(&MgsDriverNode::rxLoop, this);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize SocketCAN on %s", m_canInterface.c_str());
  }
}

/**
 * @brief  Destroys the node, joining the RX thread and closing the CAN socket.
 */
MgsDriverNode::~MgsDriverNode()
{
  m_rxRunning = false;
  if (t_rxLoopThread.joinable()) {
    t_rxLoopThread.join();
  }
  closeCanSocket();
}

/**
 * @brief  Initializes the raw CAN socket interface.
 * @return True if successfully bound to the interface, false otherwise.
 */
bool MgsDriverNode::initCanSocket()
{
  assert(!m_canInterface.empty());
  
  struct sockaddr_can addr;
  struct ifreq ifr;

  m_canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_canSocket < 0) {
    return false;
  }

  std::strncpy(ifr.ifr_name, m_canInterface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(m_canSocket, SIOCGIFINDEX, &ifr) < 0) {
    close(m_canSocket);
    m_canSocket = -1;
    return false;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(m_canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(m_canSocket);
    m_canSocket = -1;
    return false;
  }

  // Set timeout so read doesn't block forever
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms
  int ret = setsockopt(m_canSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
  if (ret < 0) {
    close(m_canSocket);
    m_canSocket = -1;
    return false;
  }

  assert(m_canSocket >= 0);
  return true;
}

/**
 * @brief  Safely closes the CAN socket file descriptor.
 */
void MgsDriverNode::closeCanSocket()
{
  if (m_canSocket >= 0) {
    close(m_canSocket);
    m_canSocket = -1;
  }
  assert(m_canSocket == -1);
}

/**
 * @brief  Sends a raw CAN frame onto the socket.
 * @param  canId  The CAN COB-ID to transmit.
 * @param  data   Pointer to the payload bytes.
 * @param  dlc    Length of the payload (max 8).
 */
void MgsDriverNode::sendCanFrame(uint32_t canId, const uint8_t * data, uint8_t dlc)
{
  assert(data != nullptr);
  assert(dlc <= 8);

  if (m_canSocket < 0) {
    return;
  }

  struct can_frame frame;
  frame.can_id = canId;
  frame.can_dlc = dlc;
  std::memset(frame.data, 0, 8);
  std::memcpy(frame.data, data, dlc);

  ssize_t bytes_written = write(m_canSocket, &frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send CAN frame");
  }
}

/**
 * @brief  Sends the CANopen NMT Start command to transition the node to Operational.
 */
void MgsDriverNode::nmtStart()
{
  assert(m_nodeId > 0);
  uint8_t data[2] = {NMT_START, static_cast<uint8_t>(m_nodeId)};
  sendCanFrame(NMT_COB_ID, data, 2);
  RCLCPP_INFO(this->get_logger(), "Sent NMT Start to Node %d", m_nodeId);
}

/**
 * @brief  Sends an Expedited SDO Download of 1 byte.
 * @param  index    The Object Dictionary index.
 * @param  subindex The subindex.
 * @param  value    The 8-bit unsigned value to write.
 */
void MgsDriverNode::sendSdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value)
{
  assert(m_nodeId > 0);
  uint32_t sdo_tx_cob_id = 0x600 + m_nodeId;
  uint8_t data[8] = {0x2F, // SDO Download 1-byte
                     static_cast<uint8_t>(index & 0xFF),
                     static_cast<uint8_t>((index >> 8) & 0xFF),
                     subindex,
                     value, 0x00, 0x00, 0x00};
  sendCanFrame(sdo_tx_cob_id, data, 8);
}

/**
 * @brief  Sends the Follow Left SDO command (!TX).
 */
void MgsDriverNode::cmdFollowLeft()
{
  sendSdoWriteU8(0x201A, 0x00, 1);
  RCLCPP_INFO(this->get_logger(), "Sent cmdFollowLeft (SDO)");
}

/**
 * @brief  Sends the Follow Right SDO command (!TV).
 */
void MgsDriverNode::cmdFollowRight()
{
  sendSdoWriteU8(0x201B, 0x00, 1);
  RCLCPP_INFO(this->get_logger(), "Sent cmdFollowRight (SDO)");
}

/**
 * @brief  Sends the RPDO1 message mapped to the track following internal logic.
 * @param  left   True to instruct the sensor to lock left.
 * @param  right  True to instruct the sensor to lock right.
 */
void MgsDriverNode::sendRpdoFollow(bool left, bool right)
{
  assert(m_nodeId > 0);
  uint32_t rpdo1_cob_id = 0x200 + m_nodeId;
  uint8_t data[2] = {static_cast<uint8_t>(left ? 1 : 0), static_cast<uint8_t>(right ? 1 : 0)};
  sendCanFrame(rpdo1_cob_id, data, 2);
  RCLCPP_INFO(this->get_logger(), "Sent sendRpdoFollow: L=%d R=%d", left, right);
}

/**
 * @brief  Service callback to instruct the sensor to follow the left track.
 * @param  request   The service trigger request.
 * @param  response  The service trigger response.
 */
void MgsDriverNode::onFollowLeft(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  assert(response != nullptr);
  RCLCPP_INFO(this->get_logger(), "Service called: follow_left");
  sendRpdoFollow(true, false);
  cmdFollowLeft(); // Redundancy
  response->success = true;
  response->message = "Following Left Track";
}

/**
 * @brief  Service callback to instruct the sensor to follow the right track.
 * @param  request   The service trigger request.
 * @param  response  The service trigger response.
 */
void MgsDriverNode::onFollowRight(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  assert(response != nullptr);
  RCLCPP_INFO(this->get_logger(), "Service called: follow_right");
  sendRpdoFollow(false, true);
  cmdFollowRight();
  response->success = true;
  response->message = "Following Right Track";
}

/**
 * @brief  Service callback to clear track following behavior.
 * @param  request   The service trigger request.
 * @param  response  The service trigger response.
 */
void MgsDriverNode::onClearFollow(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  assert(response != nullptr);
  RCLCPP_INFO(this->get_logger(), "Service called: clear_follow");
  sendRpdoFollow(false, false);
  response->success = true;
  response->message = "Cleared Track Following";
}

/**
 * @brief  Background thread loop that continually blocks on raw CAN reads.
 */
void MgsDriverNode::rxLoop()
{
  struct can_frame frame;
  uint32_t tpdo1_cob_id = 0x180 + m_nodeId;

  while (m_rxRunning && rclcpp::ok()) {
    ssize_t nbytes = read(m_canSocket, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      break; // Fatal socket error
    }

    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      continue; // Incomplete frame
    }

    uint32_t cob_id = frame.can_id & 0x1FFFFFFF;
    if (cob_id == tpdo1_cob_id) {
      processTpdo1(frame);
    }
  }
}

/**
 * @brief  Decodes TPDO1 and publishes to individual topics immediately.
 * @param  frame  The raw TPDO1 CAN frame.
 */
void MgsDriverNode::processTpdo1(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) return;
  assert(frame.can_dlc <= 8);

  int16_t left_track = (frame.data[1] << 8) | frame.data[0];
  int16_t right_track = (frame.data[3] << 8) | frame.data[2];
  uint16_t flags = (frame.data[5] << 8) | frame.data[4];

  int16_t selected_track = 0;
  if (frame.can_dlc >= 8) {
    selected_track = (frame.data[7] << 8) | frame.data[6];
  }

  bool tape_cross = (flags & 0x0001) != 0;
  bool tape_detect = (flags & 0x0002) != 0;
  bool left_marker = (flags & 0x0004) != 0;
  bool right_marker = (flags & 0x0008) != 0;
  bool sensor_failure = (flags & 0x0040) != 0;

  // Convert mm to meters for ROS standard
  std_msgs::msg::Float32 msg_track_pos;
  msg_track_pos.data = static_cast<float>(selected_track) / 1000.0f;
  p_pubTrackPos->publish(msg_track_pos);

  std_msgs::msg::Float32 msg_left;
  msg_left.data = static_cast<float>(left_track) / 1000.0f;
  p_pubLeftTrackPos->publish(msg_left);

  std_msgs::msg::Float32 msg_right;
  msg_right.data = static_cast<float>(right_track) / 1000.0f;
  p_pubRightTrackPos->publish(msg_right);

  std_msgs::msg::Bool bmsg;
  bmsg.data = tape_detect;
  p_pubTrackDetect->publish(bmsg);

  bmsg.data = left_marker;
  p_pubLeftMarker->publish(bmsg);

  bmsg.data = right_marker;
  p_pubRightMarker->publish(bmsg);

  bmsg.data = tape_cross;
  p_pubTapeCross->publish(bmsg);

  std_msgs::msg::UInt16 umsg;
  umsg.data = flags;
  p_pubStatus->publish(umsg);

  if (sensor_failure) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MGS1600 Sensor Failure Flag Set!");
  }
}

}  // namespace mgs_driver

/**
 * @brief  Main entry point for the MGS Driver Node.
 * @param  argc  Argument count.
 * @param  argv  Argument vector.
 * @return 0 on successful exit.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto p_node = std::make_shared<mgs_driver::MgsDriverNode>();
  rclcpp::spin(p_node);
  rclcpp::shutdown();
  return 0;
}
