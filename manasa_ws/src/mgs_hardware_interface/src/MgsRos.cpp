/*
Name: MgsRos.cpp
Author: Manasa
Date: 2026-06-24
Version: 2.0
Description: ROS 2 pub/sub node for MGS1600. Publishes per-topic telemetry.
*/

#include "mgs_hardware_interface/MgsRos.h"
#include "mgs_hardware_interface/MgsInterface.h"

namespace mgs_hardware_interface
{

MgsRos::MgsRos(MgsInterface * p_interface)
: Node("mgs_interface_node"), m_pInterface(p_interface)
{
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<int>("node_id", 5);
  this->declare_parameter<double>("publish_rate_hz", 50.0);
  this->declare_parameter<std::string>("default_track", "left");

  this->get_parameter("publish_rate_hz", m_publishRateHz);

  m_statusPub = this->create_publisher<mgs_hardware_interface::msg::MgsStatus>("/mgs/full_status", 10);
  m_trackDetectPub = this->create_publisher<std_msgs::msg::Bool>("/mgs/track_detect", 10);
  m_leftTrackPub = this->create_publisher<std_msgs::msg::Float64>("/mgs/left_track", 10);
  m_rightTrackPub = this->create_publisher<std_msgs::msg::Float64>("/mgs/right_track", 10);
  m_selectedTrackPub = this->create_publisher<std_msgs::msg::Float64>("/mgs/selected_track", 10);
  m_tapeCrossPub = this->create_publisher<std_msgs::msg::Bool>("/mgs/tape_cross", 10);
  m_leftMarkerPub = this->create_publisher<std_msgs::msg::Bool>("/mgs/left_marker", 10);
  m_rightMarkerPub = this->create_publisher<std_msgs::msg::Bool>("/mgs/right_marker", 10);

  m_srvSwitchTrack = this->create_service<std_srvs::srv::SetBool>(
    "/mgs/switch_track", std::bind(&MgsRos::srvSwitchTrack, this, std::placeholders::_1, std::placeholders::_2));
  m_srvSetZero = this->create_service<std_srvs::srv::Trigger>(
    "/mgs/set_zero", std::bind(&MgsRos::srvSetZero, this, std::placeholders::_1, std::placeholders::_2));

  double periodSec = 1.0 / m_publishRateHz;
  m_telemetryTimer = this->create_wall_timer(
    std::chrono::duration<double>(periodSec), std::bind(&MgsRos::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "MgsRos node ready (%.1f Hz)", m_publishRateHz);
}

MgsRos::~MgsRos() {}

void MgsRos::publishTimerCallback()
{
  MgsState state;
  m_pInterface->getDriverState(state);
  publishTelemetry(state);
}

void MgsRos::publishTelemetry(const MgsState & state)
{
  std_msgs::msg::Bool bMsg;
  std_msgs::msg::Float64 fMsg;

  bMsg.data = state.trackDetect; m_trackDetectPub->publish(bMsg);
  bMsg.data = state.tapeCross;   m_tapeCrossPub->publish(bMsg);
  bMsg.data = state.leftMarker;  m_leftMarkerPub->publish(bMsg);
  bMsg.data = state.rightMarker; m_rightMarkerPub->publish(bMsg);

  fMsg.data = state.leftTrack;     m_leftTrackPub->publish(fMsg);
  fMsg.data = state.rightTrack;    m_rightTrackPub->publish(fMsg);
  fMsg.data = state.selectedTrack; m_selectedTrackPub->publish(fMsg);

  mgs_hardware_interface::msg::MgsStatus fullMsg;
  fullMsg.header.stamp = this->now();
  fullMsg.left_track = state.leftTrack;
  fullMsg.right_track = state.rightTrack;
  fullMsg.selected_track = state.selectedTrack;
  fullMsg.track_detect = state.trackDetect;
  fullMsg.left_marker = state.leftMarker;
  fullMsg.right_marker = state.rightMarker;
  fullMsg.tape_cross = state.tapeCross;
  fullMsg.sensor_failure = state.sensorFailure;
  fullMsg.status = state.status;
  fullMsg.dominant_track = state.dominantTrack;
  std::copy(state.rawSensor.begin(), state.rawSensor.end(), fullMsg.raw_sensor.begin());
  std::copy(state.zeroAdjSensor.begin(), state.zeroAdjSensor.end(), fullMsg.zero_adj_sensor.begin());
  std::copy(state.userInts.begin(), state.userInts.end(), fullMsg.user_ints.begin());
  std::copy(state.userBools.begin(), state.userBools.end(), fullMsg.user_bools.begin());
  m_statusPub->publish(fullMsg);
}

void MgsRos::srvSwitchTrack(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) { m_pInterface->followRight(); res->message = "Switched to Right Track"; }
  else           { m_pInterface->followLeft();  res->message = "Switched to Left Track"; }
  res->success = true;
  RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
}

void MgsRos::srvSetZero(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  m_pInterface->setZero();
  res->success = true;
  res->message = "Zero command sent";
}

} // namespace mgs_hardware_interface
