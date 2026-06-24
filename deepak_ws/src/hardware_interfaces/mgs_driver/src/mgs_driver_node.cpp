#include "mgs_driver/mgs_driver_node.hpp"

#include <iostream>
#include <chrono>

namespace mgs_driver
{

// CANopen Constants
constexpr uint32_t NMT_COB_ID = 0x000;
constexpr uint8_t NMT_START = 0x01;

MgsDriverNode::MgsDriverNode(const rclcpp::NodeOptions & options)
: Node("mgs_driver", options)
{
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<int>("node_id", 5);

  can_interface_ = this->get_parameter("can_interface").as_string();
  node_id_ = this->get_parameter("node_id").as_int();

  // Initialize Publishers
  pub_track_pos_ = this->create_publisher<std_msgs::msg::Float32>("sensor/track_position", 10);
  pub_left_track_pos_ = this->create_publisher<std_msgs::msg::Float32>("sensor/left_track_position", 10);
  pub_right_track_pos_ = this->create_publisher<std_msgs::msg::Float32>("sensor/right_track_position", 10);
  
  pub_track_detect_ = this->create_publisher<std_msgs::msg::Bool>("sensor/track_detect", 10);
  pub_left_marker_ = this->create_publisher<std_msgs::msg::Bool>("sensor/left_marker", 10);
  pub_right_marker_ = this->create_publisher<std_msgs::msg::Bool>("sensor/right_marker", 10);
  pub_tape_cross_ = this->create_publisher<std_msgs::msg::Bool>("sensor/tape_cross", 10);
  pub_status_ = this->create_publisher<std_msgs::msg::UInt16>("sensor/status", 10);

  // Initialize Services for Track Switching
  srv_follow_left_ = this->create_service<std_srvs::srv::Trigger>(
    "sensor/follow_left",
    std::bind(&MgsDriverNode::on_follow_left, this, std::placeholders::_1, std::placeholders::_2));
  
  srv_follow_right_ = this->create_service<std_srvs::srv::Trigger>(
    "sensor/follow_right",
    std::bind(&MgsDriverNode::on_follow_right, this, std::placeholders::_1, std::placeholders::_2));
    
  srv_clear_follow_ = this->create_service<std_srvs::srv::Trigger>(
    "sensor/clear_follow",
    std::bind(&MgsDriverNode::on_clear_follow, this, std::placeholders::_1, std::placeholders::_2));

  if (init_can_socket()) {
    RCLCPP_INFO(this->get_logger(), "SocketCAN initialized on %s for Node ID %d", can_interface_.c_str(), node_id_);
    
    // Put node into Operational so TPDO1 streaming starts
    nmt_start();
    
    // Start RX Thread
    rx_running_ = true;
    rx_thread_ = std::thread(&MgsDriverNode::rx_loop, this);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize SocketCAN on %s", can_interface_.c_str());
  }
}

MgsDriverNode::~MgsDriverNode()
{
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  close_can_socket();
}

bool MgsDriverNode::init_can_socket()
{
  struct sockaddr_can addr;
  struct ifreq ifr;

  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    return false;
  }

  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    close(can_socket_);
    return false;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(can_socket_);
    return false;
  }

  // Set timeout so read doesn't block forever
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms
  setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

  return true;
}

void MgsDriverNode::close_can_socket()
{
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
  }
}

void MgsDriverNode::send_can_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (can_socket_ < 0) return;

  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = dlc;
  std::memset(frame.data, 0, 8);
  std::memcpy(frame.data, data, dlc);

  if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send CAN frame");
  }
}

void MgsDriverNode::nmt_start()
{
  uint8_t data[2] = {NMT_START, static_cast<uint8_t>(node_id_)};
  send_can_frame(NMT_COB_ID, data, 2);
  RCLCPP_INFO(this->get_logger(), "Sent NMT Start to Node %d", node_id_);
}

void MgsDriverNode::send_sdo_write_u8(uint16_t index, uint8_t subindex, uint8_t value)
{
  uint32_t sdo_tx_cob_id = 0x600 + node_id_;
  uint8_t data[8] = {0x2F, // SDO Download 1-byte
                     static_cast<uint8_t>(index & 0xFF),
                     static_cast<uint8_t>((index >> 8) & 0xFF),
                     subindex,
                     value, 0x00, 0x00, 0x00};
  send_can_frame(sdo_tx_cob_id, data, 8);
}

void MgsDriverNode::cmd_follow_left()
{
  send_sdo_write_u8(0x201A, 0x00, 1);
  RCLCPP_INFO(this->get_logger(), "Sent cmd_follow_left (SDO)");
}

void MgsDriverNode::cmd_follow_right()
{
  send_sdo_write_u8(0x201B, 0x00, 1);
  RCLCPP_INFO(this->get_logger(), "Sent cmd_follow_right (SDO)");
}

void MgsDriverNode::send_rpdo_follow(bool left, bool right)
{
  uint32_t rpdo1_cob_id = 0x200 + node_id_;
  uint8_t data[2] = {static_cast<uint8_t>(left ? 1 : 0), static_cast<uint8_t>(right ? 1 : 0)};
  send_can_frame(rpdo1_cob_id, data, 2);
  RCLCPP_INFO(this->get_logger(), "Sent send_rpdo_follow: L=%d R=%d", left, right);
}

void MgsDriverNode::on_follow_left(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service called: follow_left");
  send_rpdo_follow(true, false);
  cmd_follow_left(); // For redundancy depending on sensor setup
  response->success = true;
  response->message = "Following Left Track";
}

void MgsDriverNode::on_follow_right(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service called: follow_right");
  send_rpdo_follow(false, true);
  cmd_follow_right();
  response->success = true;
  response->message = "Following Right Track";
}

void MgsDriverNode::on_clear_follow(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Service called: clear_follow");
  send_rpdo_follow(false, false);
  response->success = true;
  response->message = "Cleared Track Following";
}

void MgsDriverNode::rx_loop()
{
  struct can_frame frame;
  uint32_t tpdo1_cob_id = 0x180 + node_id_;

  while (rx_running_ && rclcpp::ok()) {
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      break; // Socket error
    }

    if (nbytes < (int)sizeof(struct can_frame)) {
      continue; // Incomplete frame
    }

    uint32_t cob_id = frame.can_id & 0x1FFFFFFF;
    if (cob_id == tpdo1_cob_id) {
      process_tpdo1(frame);
    }
  }
}

void MgsDriverNode::process_tpdo1(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) return;

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
  pub_track_pos_->publish(msg_track_pos);

  std_msgs::msg::Float32 msg_left;
  msg_left.data = static_cast<float>(left_track) / 1000.0f;
  pub_left_track_pos_->publish(msg_left);

  std_msgs::msg::Float32 msg_right;
  msg_right.data = static_cast<float>(right_track) / 1000.0f;
  pub_right_track_pos_->publish(msg_right);

  std_msgs::msg::Bool bmsg;
  bmsg.data = tape_detect;
  pub_track_detect_->publish(bmsg);

  bmsg.data = left_marker;
  pub_left_marker_->publish(bmsg);

  bmsg.data = right_marker;
  pub_right_marker_->publish(bmsg);

  bmsg.data = tape_cross;
  pub_tape_cross_->publish(bmsg);

  std_msgs::msg::UInt16 umsg;
  umsg.data = flags;
  pub_status_->publish(umsg);

  if (sensor_failure) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MGS1600 Sensor Failure Flag Set!");
  }
}

}  // namespace mgs_driver


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mgs_driver::MgsDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
