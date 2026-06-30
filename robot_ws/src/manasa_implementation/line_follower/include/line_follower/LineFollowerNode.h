/*
Name: LineFollowerNode.h
Author: Manasa
Date: 2026-06-30
Version: 4.0
Description: PID line follower with a 3-state navigation sub-machine.
             FOLLOW_LINE : midpoint(left,right) error, auto_linear_velocity.
             JUNCTION    : entered/exited on BOTH markers; error = left/right
                           track chosen by junction counter (odd=left, even=right);
                           turn_linear_velocity.
             TURN        : entered/exited on EITHER (single) marker; error =
                           drift-dominant track; turn_linear_velocity.
             Marker handling uses per-marker debounce + a sync window that
             disambiguates a slightly-skewed marker pair (junction) from a lone
             marker (turn). States are mutually exclusive (structural interlock).
*/

#ifndef LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_
#define LINE_FOLLOWER__LINE_FOLLOWER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "line_follower/PidController.h"
#include <mutex>
#include <deque>
#include <cmath>
#include <string>

namespace line_follower
{

enum class NavSubState { FOLLOW_LINE, TURN, JUNCTION };

class LineFollowerNode : public rclcpp::Node
{
public:
  LineFollowerNode();
  ~LineFollowerNode();

private:
  void controlLoop();
  void enterJunction();            // increments counter, picks follow side
  void resetNavStateMachine();     // clears counter/state/edges (on disable)

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_navStatePub;
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
  double m_currentErrorMm;     // /mgs/selected_track (no longer used by control law)
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

  // ---- Navigation sub-state machine ----
  NavSubState m_navState;
  int m_junctionCount;             // counts JUNCTION detections (not exits)
  bool m_junctionFollowLeft;       // resolved at junction entry from counter

  // Marker debounce (per side)
  bool m_leftMarkerDeb;
  bool m_rightMarkerDeb;
  bool m_leftMarkerCand;
  bool m_rightMarkerCand;
  rclcpp::Time m_leftMarkerSince;
  rclcpp::Time m_rightMarkerSince;

  // Marker edge latches (on debounced values)
  bool m_prevBoth;
  bool m_prevAny;
  bool m_prevSingle;

  // Turn/junction sync window
  bool m_singlePending;
  rclcpp::Time m_singlePendingStart;

  // Tunables
  double m_markerDebounceMs;
  double m_markerSyncWindowMs;
};

} // namespace line_follower
#endif
