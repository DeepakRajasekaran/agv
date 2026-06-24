/*
Name: NavigationStateMachine.h
Author: ANSCER Robotics
Date: 2026-06-24
Version: 1.0
Description: Navigation state machine for AGV line following.
*/

#ifndef LINE_FOLLOWER__NAVIGATION_STATE_MACHINE_H_
#define LINE_FOLLOWER__NAVIGATION_STATE_MACHINE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <string>

namespace line_follower
{

enum class NavState
{
  IDLE,
  INITIALIZE,
  FOLLOW_LINE,
  JUNCTION_DETECTED,
  READ_TAG,
  EXECUTE_TURN,
  RESUME_TRACKING,
  STOP,
  ERROR
};

class NavigationStateMachine : public rclcpp::Node
{
public:
  NavigationStateMachine();
  ~NavigationStateMachine();

private:
  void tick();
  void setState(NavState newState);
  std::string stateToString(NavState s) const;

  void trackDetectCb(const std_msgs::msg::Bool::SharedPtr msg);
  void tapeCrossCb(const std_msgs::msg::Bool::SharedPtr msg);
  void selectedTrackCb(const std_msgs::msg::Float64::SharedPtr msg);

  void srvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void srvSwitchTrack(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void enablePid(bool enable);
  void switchTrack(bool followRight);

  NavState m_state;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statePub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_trackDetectSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_tapeCrossSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_selectedTrackSub;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvStart;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvStop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvReset;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_srvTurn;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_pidEnableClient;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_mgsTrackClient;

  rclcpp::TimerBase::SharedPtr m_tickTimer;

  bool m_trackDetected;
  bool m_tapeCrossed;
  double m_selectedTrackMm;
  bool m_turnRight;
  double m_resumeStartTime;
  int m_trackLostCount;
  static constexpr int TRACK_LOST_THRESHOLD = 25; // ~0.5s at 50Hz
  static constexpr double RESUME_SETTLE_S = 1.0;
};

} // namespace line_follower
#endif
