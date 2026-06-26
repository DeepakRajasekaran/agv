/*
Name: NavigationStateMachine.h
Author: ANSCER Robotics
Date: 2026-06-25
Version: 2.0
Description: Navigation state machine for AGV line following with junction sequencing.
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
#include <cmath>

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
  /**
   * @brief Constructor for the NavigationStateMachine class
   */
  NavigationStateMachine();

  /**
   * @brief Destructor for the NavigationStateMachine class
   */
  virtual ~NavigationStateMachine();

private:
  /**
   * @brief State machine tick function called continuously to transition states
   */
  void tick();

  /**
   * @brief Helper to transition and log state changes
   * @param newState The target NavState
   */
  void setState(NavState newState);

  /**
   * @brief Converts state enum to string for logging
   * @param s The state enum
   * @return String representation of the state
   */
  std::string stateToString(NavState s) const;

  /**
   * @brief Callback for track detection status
   * @param msg Boolean indicating if track is detected
   */
  void trackDetectCb(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback for tape cross detection
   * @param msg Boolean indicating if tape is crossed
   */
  void tapeCrossCb(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback for selected track error
   * @param msg The error value in mm
   */
  void selectedTrackCb(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback for left track error
   * @param msg The error value in mm
   */
  void leftTrackCb(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback for right track error
   * @param msg The error value in mm
   */
  void rightTrackCb(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Service handler to start navigation
   * @param req The trigger request
   * @param res The trigger response
   */
  void srvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Service handler to stop navigation
   * @param req The trigger request
   * @param res The trigger response
   */
  void srvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Service handler to reset the state machine
   * @param req The trigger request
   * @param res The trigger response
   */
  void srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Service handler to manually command a track switch
   * @param req Boolean flag (true = right, false = left)
   * @param res Response indicating success
   */
  void srvSwitchTrack(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  /**
   * @brief Calls the line follower enable service
   * @param enable True to enable PID, false for pass-through
   */
  void enablePid(bool enable);

  /**
   * @brief Calls the MGS track switch service
   * @param followRight True for right track, false for left track
   */
  void switchTrack(bool followRight);

  NavState m_state;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statePub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_trackDetectSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_tapeCrossSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_selectedTrackSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_leftTrackSub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_rightTrackSub;

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
  double m_leftTrackMm;
  double m_rightTrackMm;
  bool m_turnRight;
  double m_resumeStartTime;
  int m_trackLostCount;

  // ── Junction sequencing ──
  int m_junctionCount;     // total junctions encountered
  bool m_inJunction;       // hysteresis latch (one count per physical junction)

  int m_trackLostThreshold;
  double m_resumeSettleS;
  double m_sdoSettleGuardS;
  double m_junctionDiffThreshold;
  double m_junctionClearThreshold;
  double m_junctionRightThreshold;
  double m_junctionLeftThreshold;
};

} // namespace line_follower
#endif
