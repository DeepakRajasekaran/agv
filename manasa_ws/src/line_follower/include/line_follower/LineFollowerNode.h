/*
Name: LineFollowerNode.h
Author: ANSCER Robotics
Date: 2026-06-24
Version: 2.0
Description: PID line follower with mm->rad conversion, diagnostics, performance metrics.
*/

#ifndef LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_
#define LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "line_follower/PidController.h"
#include <mutex>
#include <deque>
#include <cmath>

namespace line_follower
{

class LineFollowerNode : public rclcpp::Node
{
public:
  LineFollowerNode();
  ~LineFollowerNode();

private:
  void controlLoop();
  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void trackErrorCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void srvEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pidStatePub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_teleopSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_trackErrorSub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enableSrv;
  rclcpp::TimerBase::SharedPtr m_controlTimer;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_paramCbHandle;

  PidController m_pid;

  std::mutex m_mutex;
  double m_currentErrorMm;
  double m_teleopLinearX;
  double m_teleopAngularZ;
  bool m_pidEnabled;

  double m_maxAngular;
  double m_errorDeadband;
  double m_sensorOffsetM;
  double m_controlRateHz;

  // Performance metrics
  std::deque<double> m_errorHistory;
  double m_peakError;
  double m_settledTime;
  bool m_wasSettled;
};

} // namespace line_follower
#endif
