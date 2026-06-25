/*
Name: LineFollowerNode.cpp
Author: ANSCER Robotics
Date: 2026-06-24
Version: 2.0
Description: PID line follower.
  - Polarity: Left-positive, Right-negative (sensor convention)
  - selected_track > 0 means tape is LEFT → need positive angular.z (turn left)
  - PID(selected_track) with positive P → correct sign
  - mm→rad conversion: errorRad = atan2(errorMm / 1000.0, sensorOffsetM)
  - Operator assist: teleop angular.z is ADDED to PID output
  - Runtime parameter adjustment via on_set_parameters_callback
  - Publishes /line_follower/pid_state for diagnostics
*/

#include "line_follower/LineFollowerNode.h"

namespace line_follower
{

LineFollowerNode::LineFollowerNode()
: Node("line_follower_node"),
  m_currentErrorMm(0.0), m_teleopLinearX(0.0), m_teleopAngularZ(0.0),
  m_trackDetected(false), m_leftTrackMm(0.0), m_rightTrackMm(0.0),
  m_peakError(0.0), m_settledTime(0.0), m_wasSettled(true)
{
  this->declare_parameter<std::string>("teleop_input_topic", "/cmd_vel");
  this->declare_parameter<std::string>("corrected_output_topic", "/line_cmd_vel");
  this->declare_parameter<std::string>("sensor_track_topic", "/mgs/selected_track");

  this->declare_parameter<double>("pid.p", 0.005);
  this->declare_parameter<double>("pid.i", 0.0001);
  this->declare_parameter<double>("pid.d", 0.001);
  this->declare_parameter<double>("pid.i_clamp_max", 0.3);
  this->declare_parameter<double>("pid.i_clamp_min", -0.3);

  this->declare_parameter<double>("max_angular_correction", 0.5);
  this->declare_parameter<double>("error_deadband_mm", 2.0);
  this->declare_parameter<double>("control_rate_hz", 50.0);
  this->declare_parameter<bool>("pid_enabled", true);
  this->declare_parameter<double>("sensor_offset_m", 0.30);
  this->declare_parameter<double>("wheel_base", 0.512);

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

  m_enableSrv = this->create_service<std_srvs::srv::SetBool>(
    "/line_follower/enable", std::bind(&LineFollowerNode::srvEnable, this, std::placeholders::_1, std::placeholders::_2));

  double period = 1.0 / m_controlRateHz;
  m_controlTimer = this->create_wall_timer(
    std::chrono::duration<double>(period), std::bind(&LineFollowerNode::controlLoop, this));

  // Runtime parameter update callback
  m_paramCbHandle = this->add_on_set_parameters_callback(
    std::bind(&LineFollowerNode::paramCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Line follower ready | PID: %s | sensor_offset: %.2fm",
              m_pidEnabled ? "ON" : "OFF", m_sensorOffsetM);
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

void LineFollowerNode::controlLoop()
{
  double linearX, angularZ, errorMm, leftTrackMm, rightTrackMm;
  bool enabled, trackDetected;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    linearX = m_teleopLinearX;
    angularZ = m_teleopAngularZ;
    errorMm = m_currentErrorMm;
    enabled = m_pidEnabled;
    trackDetected = m_trackDetected;
    leftTrackMm = m_leftTrackMm;
    rightTrackMm = m_rightTrackMm;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linearX;

  if (enabled) {
    if (!trackDetected) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Track LOST! Stopping robot.");
      m_cmdVelPub->publish(cmd);
      return;
    }

    // Identify which track we are following
    std::string trackingStatus = "Unknown";
    if (std::abs(errorMm - leftTrackMm) < 1.0) trackingStatus = "LEFT";
    else if (std::abs(errorMm - rightTrackMm) < 1.0) trackingStatus = "RIGHT";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Tracking: %s (Error: %.1fmm)", trackingStatus.c_str(), errorMm);

    // Deadband
    if (std::abs(errorMm) < m_errorDeadband) errorMm = 0.0;

    // Convert mm error to yaw angle (radians)
    // errorRad = atan2(lateral_offset_m, longitudinal_offset_m)
    double errorRad = std::atan2(errorMm / 1000.0, m_sensorOffsetM);

    // PID computes angular velocity correction from yaw error
    // Polarity: Left-positive convention.
    // errorMm > 0 (tape left) → errorRad > 0 → PID output > 0 → positive angular.z → turn left ✓
    double dt = 1.0 / m_controlRateHz;
    double pidCorrection = m_pid.compute(errorRad, dt);

    pidCorrection = std::clamp(pidCorrection, -m_maxAngular, m_maxAngular);

    // Operator assist: teleop angular.z ADDED to PID correction
    cmd.angular.z = pidCorrection + angularZ;

    // ── Performance metrics ──
    m_errorHistory.push_back(errorMm);
    if (m_errorHistory.size() > 500) m_errorHistory.pop_front();

    if (std::abs(errorMm) > std::abs(m_peakError)) m_peakError = errorMm;

    bool settled = std::abs(errorMm) < m_errorDeadband;
    if (!m_wasSettled && settled) {
      m_settledTime = this->now().seconds();
    }
    m_wasSettled = settled;

    // ── Publish PID state diagnostics ──
    // [0]=errorMm [1]=errorRad [2]=p_term [3]=i_term [4]=d_term [5]=output [6]=peakError [7]=rmsError
    double rms = 0.0;
    for (auto e : m_errorHistory) rms += e * e;
    rms = m_errorHistory.empty() ? 0.0 : std::sqrt(rms / m_errorHistory.size());

    std_msgs::msg::Float64MultiArray pidMsg;
    pidMsg.data = {errorMm, errorRad, m_pid.getLastPTerm(), m_pid.getLastITerm(),
                   m_pid.getLastDTerm(), pidCorrection, m_peakError, rms};
    m_pidStatePub->publish(pidMsg);
  } else {
    cmd.angular.z = angularZ;
  }

  m_cmdVelPub->publish(cmd);
}

void LineFollowerNode::srvEnable(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_pidEnabled = req->data;
  if (!m_pidEnabled) {
    m_pid.reset();
    m_peakError = 0.0;
    m_errorHistory.clear();
  }
  res->success = true;
  res->message = m_pidEnabled ? "PID ENABLED" : "PID DISABLED (pass-through)";
  RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
}

rcl_interfaces::msg::SetParametersResult LineFollowerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (p.get_name() == "pid.p" || p.get_name() == "pid.i" || p.get_name() == "pid.d") {
      double pp = this->get_parameter("pid.p").as_double();
      double pi = this->get_parameter("pid.i").as_double();
      double pd = this->get_parameter("pid.d").as_double();
      // Override with new value
      if (p.get_name() == "pid.p") pp = p.as_double();
      if (p.get_name() == "pid.i") pi = p.as_double();
      if (p.get_name() == "pid.d") pd = p.as_double();
      m_pid.setGains(pp, pi, pd);
      RCLCPP_INFO(this->get_logger(), "PID gains updated: P=%.6f I=%.6f D=%.6f", pp, pi, pd);
    }
    if (p.get_name() == "sensor_offset_m") {
      m_sensorOffsetM = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Sensor offset updated: %.3fm", m_sensorOffsetM);
    }
    if (p.get_name() == "max_angular_correction") {
      m_maxAngular = p.as_double();
    }
    if (p.get_name() == "error_deadband_mm") {
      m_errorDeadband = p.as_double();
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

} // namespace line_follower
