/*
Name: MgsRos.h
Author: ANSCER Robotics
Date: 2026-06-24
Version: 2.0
Description: ROS 2 pub/sub node for MGS1600 sensor telemetry.
*/

#ifndef MGS_HARDWARE_INTERFACE__MGS_ROS_H_
#define MGS_HARDWARE_INTERFACE__MGS_ROS_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mgs_hardware_interface/msg/mgs_status.hpp"
#include "mgs_hardware_interface/MgsCanopen.h"

namespace mgs_hardware_interface
{

class MgsInterface;

class MgsRos : public rclcpp::Node
{
public:
  MgsRos(MgsInterface * p_interface);
  virtual ~MgsRos();
  void publishTelemetry(const MgsState & state);

private:
  void publishTimerCallback();
  void srvSwitchTrack(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void srvSetZero(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  MgsInterface * m_pInterface;
  rclcpp::Publisher<mgs_hardware_interface::msg::MgsStatus>::SharedPtr m_statusPub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_trackDetectPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_leftTrackPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_rightTrackPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_selectedTrackPub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_tapeCrossPub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_srvSwitchTrack;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvSetZero;
  rclcpp::TimerBase::SharedPtr m_telemetryTimer;
  double m_publishRateHz;
};

} // namespace mgs_hardware_interface
#endif
