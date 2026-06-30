/*
Name: LineFollowerNode.cpp
Author: Manasa
Date: 2026-06-30
Version: 4.0
Description: PID line follower with FOLLOW_LINE / TURN / JUNCTION sub-machine.

  DISABLED (pre-start / IDLE)      : robot fully stopped (0,0).
  ENABLED + track lost             : one all-stop, then teleop pass-through.
  ENABLED + on track               : 3-state navigation sub-machine ↓

    FOLLOW_LINE (default)
      error = midpoint(left_track, right_track)
      linear = auto_linear_velocity
      BOTH markers (rising, debounced)              -> JUNCTION
      ONE  marker  (rising, debounced, sync-window)  -> TURN

    JUNCTION  (interlocks out TURN)
      counter++ on entry; odd -> follow LEFT track, even -> follow RIGHT track
      error = left_track or right_track per counter
      linear = turn_linear_velocity
      BOTH markers (rising) again                    -> FOLLOW_LINE (exit)

    TURN  (interlocks out JUNCTION)
      error = drift-dominant track ( larger |left|,|right| )
      linear = turn_linear_velocity
      EITHER marker (rising) again                    -> FOLLOW_LINE (exit)

  Marker pipeline: raw -> per-side debounce -> edge detect -> sync window.
  Sub-state string published on /navigation/state
  (FOLLOW_LINE | TURN_DETECTED | TURN | TURN_EXITED |
   JUNCTION_DETECTED | JUNCTION | JUNCTION_EXITED | IDLE).
*/

#include "line_follower/LineFollowerNode.h"

namespace line_follower
{

LineFollowerNode::LineFollowerNode()
: Node("line_follower_node"),
  m_currentErrorMm(0.0), m_teleopLinearX(0.0), m_teleopAngularZ(0.0),
  m_pidEnabled(false),
  m_trackDetected(false), m_leftTrackMm(0.0), m_rightTrackMm(0.0),
  m_leftMarker(false), m_rightMarker(false),
  m_prevTrackDetected(true),
  m_peakError(0.0), m_settledTime(0.0), m_wasSettled(true),
  m_navState(NavSubState::FOLLOW_LINE),
  m_junctionCount(0), m_junctionFollowLeft(false),
  m_leftMarkerDeb(false), m_rightMarkerDeb(false),
  m_leftMarkerCand(false), m_rightMarkerCand(false),
  m_prevBoth(false), m_prevAny(false), m_prevSingle(false),
  m_singlePending(false)
{
  this->declare_parameter<std::string>("teleop_input_topic", "/teleop_cmd_vel");
  this->declare_parameter<std::string>("corrected_output_topic", "/cmd_vel");
  this->declare_parameter<std::string>("sensor_track_topic", "/mgs/selected_track");

  this->declare_parameter<double>("pid.p", 1.0);
  this->declare_parameter<double>("pid.i", 0.0);
  this->declare_parameter<double>("pid.d", 0.0);
  this->declare_parameter<double>("pid.i_clamp_max", 0.3);
  this->declare_parameter<double>("pid.i_clamp_min", -0.3);

  this->declare_parameter<double>("max_angular_correction", 0.5);
  this->declare_parameter<double>("error_deadband_mm", 2.0);
  this->declare_parameter<double>("control_rate_hz", 50.0);
  this->declare_parameter<bool>("pid_enabled", false);   // stays still until /navigation/start
  this->declare_parameter<double>("sensor_offset_m", 0.30);
  this->declare_parameter<double>("wheel_base", 0.512);

  this->declare_parameter<double>("auto_linear_velocity", 0.0);  // straight cruise
  this->declare_parameter<double>("turn_linear_velocity", 0.02); // turn/junction velocity
  this->declare_parameter<double>("max_linear_velocity", 0.5);
  this->declare_parameter<double>("max_angular_velocity", 1.0);
  this->declare_parameter<double>("divergence_limit_mm", 35.0);

  // ── marker pipeline tunables ──
  this->declare_parameter<double>("marker_debounce_ms", 40.0);
  this->declare_parameter<double>("marker_sync_window_ms", 50.0);

  std::string teleopIn, teleopOut, trackIn;
  this->get_parameter("teleop_input_topic", teleopIn);
  this->get_parameter("corrected_output_topic", teleopOut);
  this->get_parameter("sensor_track_topic", trackIn);

  double p, i, d, i_max, i_min;
  this->get_parameter("pid.p", p);
  this->get_parameter("pid.i", i);
  this->get_parameter("pid.d", d);
  this->get_parameter("pid.i_clamp_max", i_max);
  this->get_parameter("pid.i_clamp_min", i_min);

  this->get_parameter("max_angular_correction", m_maxAngular);
  this->get_parameter("error_deadband_mm", m_errorDeadband);
  this->get_parameter("control_rate_hz", m_controlRateHz);
  this->get_parameter("pid_enabled", m_pidEnabled);
  this->get_parameter("sensor_offset_m", m_sensorOffsetM);
  this->get_parameter("auto_linear_velocity", m_autoLinearVel);
  this->get_parameter("turn_linear_velocity", m_turnLinearVel);
  this->get_parameter("max_linear_velocity", m_maxLinear);
  this->get_parameter("max_angular_velocity", m_maxAngularVel);
  this->get_parameter("divergence_limit_mm", m_divergenceLimit);
  this->get_parameter("marker_debounce_ms", m_markerDebounceMs);
  this->get_parameter("marker_sync_window_ms", m_markerSyncWindowMs);

  m_pid.init(p, i, d, i_min, i_max);

  // Marker timestamps need a valid clock value
  m_leftMarkerSince    = this->now();
  m_rightMarkerSince   = this->now();
  m_singlePendingStart = this->now();

  m_cmdVelPub   = this->create_publisher<geometry_msgs::msg::Twist>(teleopOut, 10);
  m_pidStatePub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line_follower/pid_state", 10);
  m_navStatePub = this->create_publisher<std_msgs::msg::String>("/navigation/state", 10);

  m_teleopSub = this->create_subscription<geometry_msgs::msg::Twist>(
    teleopIn, 10, std::bind(&LineFollowerNode::teleopCallback, this, std::placeholders::_1));
  m_trackErrorSub = this->create_subscription<std_msgs::msg::Float64>(
    trackIn, 10, std::bind(&LineFollowerNode::trackErrorCallback, this, std::placeholders::_1));
  m_trackDetectSub = this->create_subscription<std_msgs::msg::Bool>(
    "/mgs/track_detect", 10, std::bind(&LineFollowerNode::trackDetectCallback, this, std::placeholders::_1));
  m_leftTrackSub = this->create_subscription<std_msgs::msg::Float64>(
    "/mgs/left_track", 10, std::bind(&LineFollowerNode::leftTrackCallback, this, std::placeholders::_1));
  m_rightTrackSub = this->create_subscription<std_msgs::msg::Float64>(
    "/mgs/right_track", 10, std::bind(&LineFollowerNode::rightTrackCallback, this, std::placeholders::_1));
  m_leftMarkerSub = this->create_subscription<std_msgs::msg::Bool>(
    "/mgs/left_marker", 10, std::bind(&LineFollowerNode::leftMarkerCallback, this, std::placeholders::_1));
  m_rightMarkerSub = this->create_subscription<std_msgs::msg::Bool>(
    "/mgs/right_marker", 10, std::bind(&LineFollowerNode::rightMarkerCallback, this, std::placeholders::_1));

  m_enableSrv = this->create_service<std_srvs::srv::SetBool>(
    "/line_follower/enable", std::bind(&LineFollowerNode::srvEnable, this, std::placeholders::_1, std::placeholders::_2));

  double period = 1.0 / m_controlRateHz;
  m_controlTimer = this->create_wall_timer(
    std::chrono::duration<double>(period), std::bind(&LineFollowerNode::controlLoop, this));

  m_paramCbHandle = this->add_on_set_parameters_callback(
    std::bind(&LineFollowerNode::paramCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "Line follower ready | PID:%s | offset:%.2fm | auto:%.3f turn:%.3f | "
              "debounce:%.0fms sync:%.0fms",
              m_pidEnabled ? "ON" : "OFF", m_sensorOffsetM,
              m_autoLinearVel, m_turnLinearVel,
              m_markerDebounceMs, m_markerSyncWindowMs);
}

LineFollowerNode::~LineFollowerNode() {}

void LineFollowerNode::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_teleopLinearX = msg->linear.x;
  m_teleopAngularZ = msg->angular.z;
}

void LineFollowerNode::trackErrorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_currentErrorMm = msg->data;
}

void LineFollowerNode::trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_trackDetected = msg->data;
}

void LineFollowerNode::leftTrackCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_leftTrackMm = msg->data;
}

void LineFollowerNode::rightTrackCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_rightTrackMm = msg->data;
}

void LineFollowerNode::leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_leftMarker = msg->data;
}

void LineFollowerNode::rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_rightMarker = msg->data;
}

void LineFollowerNode::enterJunction()
{
  m_navState = NavSubState::JUNCTION;
  m_junctionCount++;                                  // count detections only
  m_junctionFollowLeft = (m_junctionCount % 2 != 0);  // odd -> LEFT, even -> RIGHT
  m_singlePending = false;
  RCLCPP_INFO(this->get_logger(),
              "JUNCTION #%d DETECTED -> follow %s track",
              m_junctionCount, m_junctionFollowLeft ? "LEFT" : "RIGHT");
}

void LineFollowerNode::resetNavStateMachine()
{
  m_navState = NavSubState::FOLLOW_LINE;
  m_junctionCount = 0;                 // counter reset on disable
  m_junctionFollowLeft = false;
  m_singlePending = false;
  m_leftMarkerDeb = m_rightMarkerDeb = false;
  m_leftMarkerCand = m_rightMarkerCand = false;
  m_prevBoth = m_prevAny = m_prevSingle = false;
}

void LineFollowerNode::controlLoop()
{
  double teleopLinX, teleopAngZ, leftMm, rightMm, autoLinear, turnLinear;
  bool enabled, trackDetected, leftMarkerRaw, rightMarkerRaw;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    teleopLinX     = m_teleopLinearX;
    teleopAngZ     = m_teleopAngularZ;
    enabled        = m_pidEnabled;
    trackDetected  = m_trackDetected;
    leftMm         = m_leftTrackMm;
    rightMm        = m_rightTrackMm;
    leftMarkerRaw  = m_leftMarker;
    rightMarkerRaw = m_rightMarker;
    autoLinear     = m_autoLinearVel;
    turnLinear     = m_turnLinearVel;
  }

  geometry_msgs::msg::Twist cmd;

  // ── DISABLED: robot must not move ──
  if (!enabled) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    m_prevTrackDetected = true;
    m_cmdVelPub->publish(cmd);
    
    std_msgs::msg::String stateMsg;
    stateMsg.data = "IDLE";
    m_navStatePub->publish(stateMsg);
    return;
  }

  // ── ENABLED but TRACK LOST: one all-stop, then full manual teleop ──
  if (!trackDetected) {
    if (m_prevTrackDetected) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      m_pid.reset();
      m_prevTrackDetected = false;
      RCLCPP_WARN(this->get_logger(), "Track LOST — all-stop sent, handing control to teleop");
      m_cmdVelPub->publish(cmd);
      return;
    }
    cmd.linear.x  = teleopLinX;
    cmd.angular.z = teleopAngZ;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Track lost — MANUAL teleop control active");
    m_cmdVelPub->publish(cmd);
    return;
  }
  m_prevTrackDetected = true;

  // ── Marker pipeline: debounce → edges ──
  rclcpp::Time now = this->now();
  auto debounce = [&](bool raw, bool &deb, bool &cand, rclcpp::Time &since) {
    if (raw != cand) { cand = raw; since = now; }
    if (cand != deb && (now - since).seconds() * 1000.0 >= m_markerDebounceMs) deb = cand;
  };
  debounce(leftMarkerRaw,  m_leftMarkerDeb,  m_leftMarkerCand,  m_leftMarkerSince);
  debounce(rightMarkerRaw, m_rightMarkerDeb, m_rightMarkerCand, m_rightMarkerSince);

  bool both   = m_leftMarkerDeb && m_rightMarkerDeb;
  bool any    = m_leftMarkerDeb || m_rightMarkerDeb;
  bool single = m_leftMarkerDeb ^ m_rightMarkerDeb;

  bool bothRising   = both   && !m_prevBoth;
  bool anyRising    = any    && !m_prevAny;
  bool singleRising = single && !m_prevSingle;

  std::string navLabel;
  bool transition = false;

  // ── State machine (mutually exclusive == structural interlock) ──
  switch (m_navState)
  {
    case NavSubState::FOLLOW_LINE:
    {
      if (bothRising) {
        // markers aligned within one cycle -> junction immediately
        enterJunction();
        navLabel = "JUNCTION_DETECTED";
        transition = true;
      } else {
        // a lone marker starts the sync window; a partner within the window
        // promotes it to a junction (handled by bothRising above), otherwise
        // it commits to a turn once the window elapses.
        if (singleRising && !m_singlePending) {
          m_singlePending = true;
          m_singlePendingStart = now;
        }
        if (m_singlePending &&
            (now - m_singlePendingStart).seconds() * 1000.0 >= m_markerSyncWindowMs)
        {
          m_singlePending = false;
          if (single) {
            m_navState = NavSubState::TURN;
            navLabel = "TURN_DETECTED";
            transition = true;
            RCLCPP_INFO(this->get_logger(), "TURN DETECTED (mk L=%d R=%d)",
                        m_leftMarkerDeb ? 1 : 0, m_rightMarkerDeb ? 1 : 0);
          }
          // else: marker vanished within the window -> treat as flicker, ignore
        }
      }
      break;
    }

    case NavSubState::JUNCTION:
    {
      // interlock: single/turn edges ignored while in a junction
      if (bothRising) {
        m_navState = NavSubState::FOLLOW_LINE;
        navLabel = "JUNCTION_EXITED";
        transition = true;
        RCLCPP_INFO(this->get_logger(), "JUNCTION #%d EXITED", m_junctionCount);
      }
      break;
    }

    case NavSubState::TURN:
    {
      // interlock: both/junction edges ignored while in a turn;
      // any fresh marker edge ends the turn.
      if (anyRising) {
        m_navState = NavSubState::FOLLOW_LINE;
        navLabel = "TURN_EXITED";
        transition = true;
        RCLCPP_INFO(this->get_logger(), "TURN EXITED");
      }
      break;
    }
  }

  if (!transition) {
    switch (m_navState) {
      case NavSubState::FOLLOW_LINE: navLabel = "FOLLOW_LINE"; break;
      case NavSubState::TURN:        navLabel = "TURN";        break;
      case NavSubState::JUNCTION:    navLabel = "JUNCTION";    break;
    }
  }

  // update edge latches AFTER the machine has consumed this cycle's edges
  m_prevBoth   = both;
  m_prevAny    = any;
  m_prevSingle = single;

  std_msgs::msg::String stateMsg;
  stateMsg.data = navLabel;
  m_navStatePub->publish(stateMsg);

  // ── Error source + linear velocity selected purely by state ──
  double errorMm = 0.0;
  double currentTargetVel = autoLinear;
  std::string srcLabel;

  switch (m_navState)
  {
    case NavSubState::FOLLOW_LINE:
      errorMm = (leftMm + rightMm) / 2.0;
      currentTargetVel = autoLinear;
      srcLabel = "MIDPOINT";
      break;

    case NavSubState::JUNCTION:
      errorMm = m_junctionFollowLeft ? leftMm : rightMm;
      currentTargetVel = turnLinear;
      srcLabel = m_junctionFollowLeft ? "JCT_LEFT" : "JCT_RIGHT";
      break;

    case NavSubState::TURN:
      if (std::abs(rightMm) > std::abs(leftMm)) { errorMm = rightMm; srcLabel = "TURN_RIGHT"; }
      else                                      { errorMm = leftMm;  srcLabel = "TURN_LEFT";  }
      currentTargetVel = turnLinear;
      break;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "[%-11s] jc=%d src=%-10s err=%.1f L=%.1f R=%.1f mk[L=%d R=%d] vel=%.2f",
                       navLabel.c_str(), m_junctionCount, srcLabel.c_str(),
                       errorMm, leftMm, rightMm,
                       m_leftMarkerDeb ? 1 : 0, m_rightMarkerDeb ? 1 : 0, currentTargetVel);

  double rawErrorMm = errorMm;
  if (std::abs(errorMm) < m_errorDeadband) errorMm = 0.0;

  double errorRad = std::atan2(errorMm / 1000.0, m_sensorOffsetM);
  double dt = 1.0 / m_controlRateHz;
  double pidCorrection = m_pid.compute(errorRad, dt);
  pidCorrection = std::clamp(pidCorrection, -m_maxAngular, m_maxAngular);

  cmd.linear.x  = currentTargetVel + teleopLinX;
  cmd.angular.z = pidCorrection + teleopAngZ;

  // ── metrics ──
  m_errorHistory.push_back(rawErrorMm);
  if (m_errorHistory.size() > 500) m_errorHistory.pop_front();
  if (std::abs(rawErrorMm) > std::abs(m_peakError)) m_peakError = rawErrorMm;
  bool settled = std::abs(rawErrorMm) < m_errorDeadband;
  if (!m_wasSettled && settled) m_settledTime = this->now().seconds();
  m_wasSettled = settled;

  double rms = 0.0;
  for (auto e : m_errorHistory) rms += e * e;
  rms = m_errorHistory.empty() ? 0.0 : std::sqrt(rms / m_errorHistory.size());

  std_msgs::msg::Float64MultiArray pidMsg;
  pidMsg.data = {rawErrorMm, errorRad, m_pid.getLastPTerm(), m_pid.getLastITerm(),
                 m_pid.getLastDTerm(), pidCorrection, m_peakError, rms};
  m_pidStatePub->publish(pidMsg);

  // ── final velocity clamps ──
  cmd.linear.x  = std::clamp(cmd.linear.x,  -m_maxLinear,     m_maxLinear);
  cmd.angular.z = std::clamp(cmd.angular.z, -m_maxAngularVel, m_maxAngularVel);
  m_cmdVelPub->publish(cmd);
}

void LineFollowerNode::srvEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  bool changed = false;
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_pidEnabled != req->data) {
    m_pidEnabled = req->data;
    changed = true;
    if (!m_pidEnabled) {
      m_pid.reset();
      m_peakError = 0.0;
      m_errorHistory.clear();
      resetNavStateMachine();          // counter -> 0, state -> FOLLOW_LINE
    }
    m_prevTrackDetected = true;        // clean edge state whenever enable toggles
  }
  res->success = true;
  res->message = m_pidEnabled ? "PID ENABLED" : "PID DISABLED (robot stopped)";
  if (changed) {
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }
}

rcl_interfaces::msg::SetParametersResult LineFollowerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (p.get_name() == "pid.p" || p.get_name() == "pid.i" || p.get_name() == "pid.d") {
      double pp = this->get_parameter("pid.p").as_double();
      double pi = this->get_parameter("pid.i").as_double();
      double pd = this->get_parameter("pid.d").as_double();
      if (p.get_name() == "pid.p") pp = p.as_double();
      if (p.get_name() == "pid.i") pi = p.as_double();
      if (p.get_name() == "pid.d") pd = p.as_double();
      m_pid.setGains(pp, pi, pd);
      RCLCPP_INFO(this->get_logger(), "PID gains: P=%.6f I=%.6f D=%.6f", pp, pi, pd);
    }
    if (p.get_name() == "sensor_offset_m")        m_sensorOffsetM     = p.as_double();
    if (p.get_name() == "max_angular_correction") m_maxAngular        = p.as_double();
    if (p.get_name() == "error_deadband_mm")      m_errorDeadband     = p.as_double();
    if (p.get_name() == "auto_linear_velocity") {
      m_autoLinearVel = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Auto-linear velocity: %.3f m/s", m_autoLinearVel);
    }
    if (p.get_name() == "turn_linear_velocity") {
      m_turnLinearVel = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Turn-linear velocity: %.3f m/s", m_turnLinearVel);
    }
    if (p.get_name() == "max_linear_velocity")    m_maxLinear         = p.as_double();
    if (p.get_name() == "max_angular_velocity")   m_maxAngularVel     = p.as_double();
    if (p.get_name() == "divergence_limit_mm")    m_divergenceLimit   = p.as_double();
    if (p.get_name() == "marker_debounce_ms")     m_markerDebounceMs  = p.as_double();
    if (p.get_name() == "marker_sync_window_ms")  m_markerSyncWindowMs = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

} // namespace line_follower
