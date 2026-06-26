/*
Name: LineFollowerNode.h
Author: Manasa
Date: 2026-06-26
Version: 3.0
Description: PID line follower with conditional error source, auto-linear assist,
             teleop-recovery on track loss, and turn deceleration.
*/

#ifndef LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_
#define LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "line_follower/PidController.h"
#include <mutex>
#include <deque>
#include <cmath>
#include <string>

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
  void trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void leftTrackCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void rightTrackCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void srvEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pidStatePub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_teleopSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_trackErrorSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_trackDetectSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_leftTrackSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_rightTrackSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_leftMarkerSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_rightMarkerSub;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enableSrv;
  rclcpp::TimerBase::SharedPtr m_controlTimer;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_paramCbHandle;

  PidController m_pid;

  std::mutex m_mutex;
  double m_currentErrorMm;     // /mgs/selected_track
  double m_teleopLinearX;
  double m_teleopAngularZ;
  bool m_pidEnabled;

  bool m_trackDetected;
  double m_leftTrackMm;
  double m_rightTrackMm;
  bool m_leftMarker;
  bool m_rightMarker;

  bool m_prevTrackDetected;    // edge detector for track-loss all-stop

  double m_maxAngular;         // PID-correction clamp
  double m_errorDeadband;
  double m_sensorOffsetM;
  double m_controlRateHz;

  double m_autoLinearVel;      // parameterized cruise velocity
  double m_turnLinearVel;      // parameterized turn/deceleration velocity
  double m_maxLinear;          // final linear clamp
  double m_maxAngularVel;      // final angular clamp
  double m_divergenceLimit;    // mm, within/outside junction gate

  // Performance metrics
  std::deque<double> m_errorHistory;
  double m_peakError;
  double m_settledTime;
  bool m_wasSettled;
};

} // namespace line_follower
#endif
