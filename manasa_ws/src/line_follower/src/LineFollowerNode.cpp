/*
Name: LineFollowerNode.cpp
Author: ANSCER Robotics
Date: 2026-06-26
Version: 3.0
Description: PID line follower.
  - DISABLED (IDLE/STOP/pre-start): robot fully stopped (0,0). No motion until /navigation/start.
  - ENABLED + track lost: one all-stop, then FULL teleop pass-through for manual recovery.
  - ENABLED + on track: conditional error source + PID yaw + auto-linear cruise.
      diff<35 & a marker  → error = marker_side_track
      diff>=35 & a marker → error = marker_side_track (junction)
      neither marker      → error = midpoint(left,right) (smooth)
  - cmd.linear.x  = (abs(error) > 15) ? turn_linear_velocity : auto_linear_velocity
  - cmd.angular.z = pid_correction + teleop.angular.z
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
  m_peakError(0.0), m_settledTime(0.0), m_wasSettled(true)
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

  // ── new params ──
  this->declare_parameter<double>("auto_linear_velocity", 0.0);  // cruise; set via param
  this->declare_parameter<double>("turn_linear_velocity", 0.02); // decelerated cruise
  this->declare_parameter<double>("max_linear_velocity", 0.5);
  this->declare_parameter<double>("max_angular_velocity", 1.0);
  this->declare_parameter<double>("divergence_limit_mm", 35.0);

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

  m_pid.init(p, i, d, i_min, i_max);

  m_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>(teleopOut, 10);
  m_pidStatePub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/line_follower/pid_state", 10);

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
              "Line follower ready | PID: %s | offset: %.2fm | auto_lin: %.3f | turn_lin: %.3f | div_limit: %.0fmm",
              m_pidEnabled ? "ON" : "OFF", m_sensorOffsetM, m_autoLinearVel, m_turnLinearVel, m_divergenceLimit);
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

void LineFollowerNode::controlLoop()
{
  double teleopLinX, teleopAngZ, leftMm, rightMm, autoLinear, turnLinear;
  bool enabled, trackDetected, leftMarker, rightMarker;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    teleopLinX    = m_teleopLinearX;
    teleopAngZ    = m_teleopAngularZ;
    enabled       = m_pidEnabled;
    trackDetected = m_trackDetected;
    leftMm        = m_leftTrackMm;
    rightMm       = m_rightTrackMm;
    leftMarker    = m_leftMarker;
    rightMarker   = m_rightMarker;
    autoLinear    = m_autoLinearVel;
    turnLinear    = m_turnLinearVel;
  }

  geometry_msgs::msg::Twist cmd;

  // ── DISABLED: IDLE / STOP / before /navigation/start → robot must not move ──
  if (!enabled) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    m_prevTrackDetected = true;   // arm edge detector for a clean re-enable
    m_cmdVelPub->publish(cmd);
    return;
  }

  // ── ENABLED but TRACK LOST: one all-stop, then full manual teleop ──
  if (!trackDetected) {
    if (m_prevTrackDetected) {
      // edge true→false: single all-stop command
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      m_pid.reset();
      m_prevTrackDetected = false;
      RCLCPP_WARN(this->get_logger(), "Track LOST — all-stop sent, handing control to teleop");
      m_cmdVelPub->publish(cmd);
      return;
    }
    // already stopped: full teleop pass-through for manual recovery
    cmd.linear.x  = teleopLinX;
    cmd.angular.z = teleopAngZ;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Track lost — MANUAL teleop control active");
    m_cmdVelPub->publish(cmd);
    return;
  }

  // ── ENABLED and ON TRACK ──
  m_prevTrackDetected = true;

  double diff = std::abs(leftMm - rightMm);
  double errorMm;
  std::string srcLabel;

  if (diff > 20.0) {
    if (rightMm > leftMm) {
      errorMm = rightMm;
      srcLabel = "DRIFT_RIGHT";
    } else {
      errorMm = leftMm;
      srcLabel = "DRIFT_LEFT";
    }
  } else {
    errorMm = (leftMm + rightMm) / 2.0;
    srcLabel = "STRAIGHT_AVG";
  }

  // Deceleration logic based on error magnitude
  double currentTargetVel = (std::abs(errorMm) > 15.0) ? turnLinear : autoLinear;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Src:%-12s err=%.1f L=%.1f R=%.1f diff=%.1f mk[L=%d R=%d] vel=%.2f",
                       srcLabel.c_str(), errorMm, leftMm, rightMm, diff,
                       leftMarker ? 1 : 0, rightMarker ? 1 : 0, currentTargetVel);

  double rawErrorMm = errorMm;
  if (std::abs(errorMm) < m_errorDeadband) errorMm = 0.0;

  double errorRad = std::atan2(errorMm / 1000.0, m_sensorOffsetM);
  double dt = 1.0 / m_controlRateHz;
  double pidCorrection = m_pid.compute(errorRad, dt);
  pidCorrection = std::clamp(pidCorrection, -m_maxAngular, m_maxAngular);

  // Command: auto cruise (or decelerated) (+teleop trim) ; PID yaw (+teleop trim)
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
    }
    m_prevTrackDetected = true;   // clean edge state whenever enable toggles
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
    if (p.get_name() == "sensor_offset_m")        m_sensorOffsetM   = p.as_double();
    if (p.get_name() == "max_angular_correction") m_maxAngular      = p.as_double();
    if (p.get_name() == "error_deadband_mm")      m_errorDeadband   = p.as_double();
    if (p.get_name() == "auto_linear_velocity") {
      m_autoLinearVel = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Auto-linear velocity: %.3f m/s", m_autoLinearVel);
    }
    if (p.get_name() == "turn_linear_velocity") {
      m_turnLinearVel = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Turn-linear velocity: %.3f m/s", m_turnLinearVel);
    }
    if (p.get_name() == "max_linear_velocity")    m_maxLinear       = p.as_double();
    if (p.get_name() == "max_angular_velocity")   m_maxAngularVel   = p.as_double();
    if (p.get_name() == "divergence_limit_mm")    m_divergenceLimit = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

} // namespace line_follower
