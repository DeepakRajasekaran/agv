/*
Name: NavigationStateMachine.cpp
Author: ANSCER Robotics
Date: 2026-06-25
Version: 2.0
Description: Navigation state machine with junction detection and odd/even turn sequencing.
*/

#include "line_follower/NavigationStateMachine.h"

namespace line_follower
{

NavigationStateMachine::NavigationStateMachine()
: Node("navigation_state_machine"),
  m_state(NavState::IDLE),
  m_trackDetected(false), m_tapeCrossed(false), m_selectedTrackMm(0.0),
  m_leftTrackMm(0.0), m_rightTrackMm(0.0),
  m_turnRight(false), m_resumeStartTime(0.0), m_trackLostCount(0),
  m_junctionCount(0), m_inJunction(false)
{
  this->declare_parameter<int>("track_lost_threshold", 25);
  this->declare_parameter<double>("resume_settle_s", 1.0);
  this->declare_parameter<double>("sdo_settle_guard_s", 0.3);
  this->declare_parameter<double>("junction_diff_threshold", 25.0);
  this->declare_parameter<double>("junction_clear_threshold", 20.0);
  this->declare_parameter<double>("junction_right_threshold", -40.0);
  this->declare_parameter<double>("junction_left_threshold", 40.0);

  this->get_parameter("track_lost_threshold", m_trackLostThreshold);
  this->get_parameter("resume_settle_s", m_resumeSettleS);
  this->get_parameter("sdo_settle_guard_s", m_sdoSettleGuardS);
  this->get_parameter("junction_diff_threshold", m_junctionDiffThreshold);
  this->get_parameter("junction_clear_threshold", m_junctionClearThreshold);
  this->get_parameter("junction_right_threshold", m_junctionRightThreshold);
  this->get_parameter("junction_left_threshold", m_junctionLeftThreshold);
  m_statePub = this->create_publisher<std_msgs::msg::String>("/navigation/state", 10);

  m_trackDetectSub = this->create_subscription<std_msgs::msg::Bool>(
    "/mgs/track_detect", 10, std::bind(&NavigationStateMachine::trackDetectCb, this, std::placeholders::_1));
  m_tapeCrossSub = this->create_subscription<std_msgs::msg::Bool>(
    "/mgs/tape_cross", 10, std::bind(&NavigationStateMachine::tapeCrossCb, this, std::placeholders::_1));
  m_selectedTrackSub = this->create_subscription<std_msgs::msg::Float64>(
    "/mgs/selected_track", 10, std::bind(&NavigationStateMachine::selectedTrackCb, this, std::placeholders::_1));
  m_leftTrackSub = this->create_subscription<std_msgs::msg::Float64>(
    "/mgs/left_track", 10, std::bind(&NavigationStateMachine::leftTrackCb, this, std::placeholders::_1));
  m_rightTrackSub = this->create_subscription<std_msgs::msg::Float64>(
    "/mgs/right_track", 10, std::bind(&NavigationStateMachine::rightTrackCb, this, std::placeholders::_1));

  m_srvStart = this->create_service<std_srvs::srv::Trigger>(
    "/navigation/start", std::bind(&NavigationStateMachine::srvStart, this, std::placeholders::_1, std::placeholders::_2));
  m_srvStop = this->create_service<std_srvs::srv::Trigger>(
    "/navigation/stop", std::bind(&NavigationStateMachine::srvStop, this, std::placeholders::_1, std::placeholders::_2));
  m_srvReset = this->create_service<std_srvs::srv::Trigger>(
    "/navigation/reset", std::bind(&NavigationStateMachine::srvReset, this, std::placeholders::_1, std::placeholders::_2));
  m_srvTurn = this->create_service<std_srvs::srv::SetBool>(
    "/navigation/switch_track", std::bind(&NavigationStateMachine::srvSwitchTrack, this, std::placeholders::_1, std::placeholders::_2));

  m_pidEnableClient = this->create_client<std_srvs::srv::SetBool>("/line_follower/enable");
  m_mgsTrackClient = this->create_client<std_srvs::srv::SetBool>("/mgs/switch_track");

  m_tickTimer = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&NavigationStateMachine::tick, this));

  RCLCPP_INFO(this->get_logger(), "Navigation State Machine ready [IDLE]");
}

NavigationStateMachine::~NavigationStateMachine() {}

std::string NavigationStateMachine::stateToString(NavState s) const
{
  switch (s) {
    case NavState::IDLE:              return "IDLE";
    case NavState::INITIALIZE:        return "INITIALIZE";
    case NavState::FOLLOW_LINE:       return "FOLLOW_LINE";
    case NavState::JUNCTION_DETECTED: return "JUNCTION_DETECTED";
    case NavState::READ_TAG:          return "READ_TAG";
    case NavState::EXECUTE_TURN:      return "EXECUTE_TURN";
    case NavState::RESUME_TRACKING:   return "RESUME_TRACKING";
    case NavState::STOP:              return "STOP";
    case NavState::ERROR:             return "ERROR";
    default:                          return "UNKNOWN";
  }
}

void NavigationStateMachine::setState(NavState newState)
{
  if (newState == m_state) return;
  RCLCPP_INFO(this->get_logger(), "State: %s → %s",
              stateToString(m_state).c_str(), stateToString(newState).c_str());
  m_state = newState;

  std_msgs::msg::String msg;
  msg.data = stateToString(m_state);
  m_statePub->publish(msg);
}

void NavigationStateMachine::trackDetectCb(const std_msgs::msg::Bool::SharedPtr msg) { m_trackDetected = msg->data; }
void NavigationStateMachine::tapeCrossCb(const std_msgs::msg::Bool::SharedPtr msg) { m_tapeCrossed = msg->data; }
void NavigationStateMachine::selectedTrackCb(const std_msgs::msg::Float64::SharedPtr msg) { m_selectedTrackMm = msg->data; }
void NavigationStateMachine::leftTrackCb(const std_msgs::msg::Float64::SharedPtr msg) { m_leftTrackMm = msg->data; }
void NavigationStateMachine::rightTrackCb(const std_msgs::msg::Float64::SharedPtr msg) { m_rightTrackMm = msg->data; }

void NavigationStateMachine::enablePid(bool enable)
{
  if (!m_pidEnableClient->wait_for_service(std::chrono::milliseconds(100))) return;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;
  m_pidEnableClient->async_send_request(req);
}

void NavigationStateMachine::switchTrack(bool followRight)
{
  if (!m_mgsTrackClient->wait_for_service(std::chrono::milliseconds(100))) return;
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = followRight;
  m_mgsTrackClient->async_send_request(req);
}

// ═══════════════════════════════════════════════════════════════════════════
//  State Machine Tick (50Hz)
// ═══════════════════════════════════════════════════════════════════════════

void NavigationStateMachine::tick()
{
  // Junction hysteresis: clear the latch once the two tracks re-converge
  // (robot has physically moved past the junction). Runs every state so the
  // latch is always ready when we return to FOLLOW_LINE.
  if (std::abs(m_leftTrackMm - m_rightTrackMm) < m_junctionClearThreshold)
    m_inJunction = false;

  switch (m_state) {

    case NavState::IDLE:
      // Waiting for /navigation/start service call
      break;

    case NavState::INITIALIZE:
      enablePid(true);
      m_trackLostCount = 0;
      setState(NavState::FOLLOW_LINE);
      break;

    case NavState::FOLLOW_LINE:
    {
      // Track-loss is handled by LineFollowerNode (it zeroes cmd_vel while
      // track_detect == false and resumes automatically). PID stays enabled.
      if (!m_trackDetected) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Track not detected — robot holding, PID stays enabled");
        break;  // don't run junction detection on stale left/right values
      }

      // ── Junction detection: width AND directional condition ──
      double diff = std::abs(m_leftTrackMm - m_rightTrackMm);
      bool rightJunction = (m_rightTrackMm > m_junctionRightThreshold);
      bool leftJunction  = (m_leftTrackMm  >= m_junctionLeftThreshold);

      if (diff > m_junctionDiffThreshold && (rightJunction || leftJunction) && !m_inJunction) {
        m_inJunction = true;
        m_junctionCount++;

        const char* detectedDir = leftJunction ? (rightJunction ? "BOTH" : "LEFT")
                                                : "RIGHT";
        RCLCPP_INFO(this->get_logger(),
                    "Junction #%d detected [%s] (L=%.1f R=%.1f diff=%.1fmm)",
                    m_junctionCount, detectedDir,
                    m_leftTrackMm, m_rightTrackMm, diff);
        setState(NavState::JUNCTION_DETECTED);
      }
      break;
    }

    case NavState::JUNCTION_DETECTED:
      // Sequence rule: odd junction → RIGHT, even junction → LEFT.
      m_turnRight = (m_junctionCount % 2 == 1);
      RCLCPP_INFO(this->get_logger(), "Junction #%d → %s turn",
                  m_junctionCount, m_turnRight ? "RIGHT" : "LEFT");
      // READ_TAG skipped (no RFID) — proceed straight to turn execution.
      setState(NavState::EXECUTE_TURN);
      break;

    case NavState::READ_TAG:
      // Reachable only via manual /navigation/switch_track. No RFID — auto-pass.
      break;

    case NavState::EXECUTE_TURN:
      switchTrack(m_turnRight);
      m_resumeStartTime = this->now().seconds();
      setState(NavState::RESUME_TRACKING);
      break;

    case NavState::RESUME_TRACKING:
    {
      double elapsed = this->now().seconds() - m_resumeStartTime;

      // Give the MGS CANopen SDO write time to complete and
      // selected_track to reflect the new tape before checking settle.
      if (elapsed < m_sdoSettleGuardS) break;

      bool settled = std::abs(m_selectedTrackMm) < 10.0; // within 10mm
      if (settled && elapsed > m_resumeSettleS) {
        RCLCPP_INFO(this->get_logger(), "Track locked (%.1fmm error, %.1fs)", m_selectedTrackMm, elapsed);
        setState(NavState::FOLLOW_LINE);
      }
      if (elapsed > 5.0) {
        RCLCPP_WARN(this->get_logger(), "Resume timeout — forcing FOLLOW_LINE");
        setState(NavState::FOLLOW_LINE);
      }
      break;
    }

    case NavState::STOP:
      // PID disabled, robot stopped. Stay here until start/reset.
      break;

    case NavState::ERROR:
      // Fault state. Wait for /navigation/reset or /navigation/start.
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Service Handlers
// ═══════════════════════════════════════════════════════════════════════════

void NavigationStateMachine::srvStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  if (m_state == NavState::IDLE || m_state == NavState::STOP || m_state == NavState::ERROR) {
    setState(NavState::INITIALIZE);
    res->success = true;
    res->message = "Starting navigation";
  } else {
    res->success = false;
    res->message = "Cannot start from state: " + stateToString(m_state);
  }
}

void NavigationStateMachine::srvStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  enablePid(false);
  setState(NavState::STOP);
  res->success = true;
  res->message = "Navigation stopped";
}

void NavigationStateMachine::srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  enablePid(false);
  m_trackLostCount = 0;
  m_junctionCount = 0;     // reset sequence on full reset
  m_inJunction = false;
  setState(NavState::IDLE);
  res->success = true;
  res->message = "Reset to IDLE";
  RCLCPP_INFO(this->get_logger(), "Fault recovery — reset to IDLE");
}

void NavigationStateMachine::srvSwitchTrack(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  m_turnRight = req->data;
  if (m_state == NavState::READ_TAG || m_state == NavState::JUNCTION_DETECTED ||
      m_state == NavState::FOLLOW_LINE) {
    setState(NavState::EXECUTE_TURN);
    res->success = true;
    res->message = m_turnRight ? "Executing RIGHT turn" : "Executing LEFT turn";
  } else {
    res->success = false;
    res->message = "Cannot switch track in state: " + stateToString(m_state);
  }
}

} // namespace line_follower
