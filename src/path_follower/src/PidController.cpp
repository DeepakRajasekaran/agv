/*
 * Name:        PidController.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Implements the PidController class for ROS 2.
 */

#include "PidController.h"
#include <cassert>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace path_follower {

/**
 * @brief  Constructor of the PidController ROS 2 class.
 *         Loads parameters, sets up publishers/subscribers,
 *         and initializes states.
 * @param  options   Node options.
 */
PidController::PidController(const rclcpp::NodeOptions& options)
    : Node("path_follower_node", options),
      m_kp(1.5),
      m_ki(0.02),
      m_kd(0.12),
      m_windupLimit(0.5),
      m_maxOutput(1.5),
      m_nominalSpeed(0.20),
      m_maxRpm(150.0),
      m_wheelBase(0.512),
      m_wheelRadius(0.08),
      m_lostThreshold(0.25),
      m_maxFrozenSteps(50),
      m_turnDuration(3.0),
      m_integralError(0.0),
      m_prevError(0.0),
      m_turnStartTime(this->now()),
      m_trackDetect(false),
      m_leftMarker(false),
      m_rightMarker(false),
      m_leftTrackPos(0.0),
      m_rightTrackPos(0.0),
      m_currentWaypointIndex(0),
      m_firstMessageReceived(false),
      m_navLinearVel(0.0),
      m_navAngularVel(0.0),
      m_navVelReceived(false),
      m_logCounter(0)
{
    // Declare ROS 2 Parameters with defaults
    this->declare_parameter<double>("pid.kp", m_kp);
    this->declare_parameter<double>("pid.ki", m_ki);
    this->declare_parameter<double>("pid.kd", m_kd);
    this->declare_parameter<double>("pid.windup_limit", m_windupLimit);
    this->declare_parameter<double>("pid.max_output", m_maxOutput);
    this->declare_parameter<double>("robot.nominal_speed", m_nominalSpeed);
    this->declare_parameter<double>("robot.max_rpm", m_maxRpm);
    this->declare_parameter<double>("robot.wheel_base", m_wheelBase);
    this->declare_parameter<double>("robot.wheel_radius", m_wheelRadius);
    this->declare_parameter<double>("safety.lost_threshold", m_lostThreshold);
    this->declare_parameter<int>("safety.max_frozen_steps", m_maxFrozenSteps);
    this->declare_parameter<double>("safety.turn_duration", m_turnDuration);

    // Retrieve parameter values
    this->get_parameter("pid.kp", m_kp);
    this->get_parameter("pid.ki", m_ki);
    this->get_parameter("pid.kd", m_kd);
    this->get_parameter("pid.windup_limit", m_windupLimit);
    this->get_parameter("pid.max_output", m_maxOutput);
    this->get_parameter("robot.nominal_speed", m_nominalSpeed);
    this->get_parameter("robot.max_rpm", m_maxRpm);
    this->get_parameter("robot.wheel_base", m_wheelBase);
    this->get_parameter("robot.wheel_radius", m_wheelRadius);
    this->get_parameter("safety.lost_threshold", m_lostThreshold);
    this->get_parameter("safety.max_frozen_steps", m_maxFrozenSteps);
    this->get_parameter("safety.turn_duration", m_turnDuration);

    // Initial time points
    m_lastLoopTime = std::chrono::steady_clock::now();
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    // Instantiate State Machine and Safety Monitor (Smart Pointers)
    p_stateMachine = std::make_unique<NavigationStateMachine>();
    p_faultMonitor = std::make_unique<FaultMonitor>(m_lostThreshold, m_maxFrozenSteps);

    // Initialize state transitions
    p_stateMachine->transitionTo(State::INITIALIZE, "NODE_START");

    // ROS 2 Subscribers
    m_subTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/track_position", 10, std::bind(&PidController::trackPosCallback, this, std::placeholders::_1));
    m_subTrackDetect = this->create_subscription<std_msgs::msg::Bool>(
        "/sensor/track_detect", 10, std::bind(&PidController::trackDetectCallback, this, std::placeholders::_1));
    m_subLeftMarker = this->create_subscription<std_msgs::msg::Bool>(
        "/sensor/left_marker", 10, std::bind(&PidController::leftMarkerCallback, this, std::placeholders::_1));
    m_subRightMarker = this->create_subscription<std_msgs::msg::Bool>(
        "/sensor/right_marker", 10, std::bind(&PidController::rightMarkerCallback, this, std::placeholders::_1));
    m_subLeftTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/left_track_position", 10, std::bind(&PidController::leftTrackPosCallback, this, std::placeholders::_1));
    m_subRightTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/right_track_position", 10, std::bind(&PidController::rightTrackPosCallback, this, std::placeholders::_1));
    m_subTagId = this->create_subscription<std_msgs::msg::String>(
        "/sensor/tag_id", 10, std::bind(&PidController::tagIdCallback, this, std::placeholders::_1));
    m_subPlan = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&PidController::planCallback, this, std::placeholders::_1));
    m_subNavVel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/nav_vel", 10, std::bind(&PidController::navVelCallback, this, std::placeholders::_1));

    // ROS 2 Publishers
    m_pubCmdVel = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    m_pubSelectTrack = this->create_publisher<std_msgs::msg::String>("/sensor/select_track", 10);
    m_pubControllerState = this->create_publisher<std_msgs::msg::String>("/controller_state", 10);

    // ROS 2 Services
    m_srvAutotune = this->create_service<std_srvs::srv::Trigger>(
        "/autotune",
        std::bind(&PidController::autotuneCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Safety timeout check timer (50Hz = 20ms)
    m_safetyTimer = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&PidController::safetyCheckCallback, this)
    );

    // Register dynamic parameter callback
    m_callbackHandle = this->add_on_set_parameters_callback(
        std::bind(&PidController::onParameterChange, this, std::placeholders::_1)
    );

    // Move to run state
    p_stateMachine->transitionTo(State::IDLE, "INITIALIZATION_COMPLETE");
    publishControllerState();
    
    RCLCPP_INFO(this->get_logger(), "Line follower ROS 2 node running.");
}

/**
 * @brief  Destructor resetting wheel speeds to zero.
 */
PidController::~PidController()
{
    publishVelocity(0.0, 0.0);
}

/**
 * @brief  Computes PID steering correction from lateral error.
 * @param  error  Signed lateral distance from tape center (meters).
 * @param  dt     Time elapsed since last update (seconds).
 * @return Steering angular velocity correction (rad/s).
 */
double PidController::computeSteering(double error, double dt)
{
    // Preconditions
    assert(std::isfinite(error));
    assert(dt > 0.0);

    // Proportional Term
    double p = m_kp * error;

    // Integral Term with anti-windup clamping
    m_integralError += error * dt;
    m_integralError = std::max(-m_windupLimit, std::min(m_integralError, m_windupLimit));
    double i = m_ki * m_integralError;

    // Derivative Term
    double d = 0.0;
    if (dt > 1e-4) {
        d = m_kd * (error - m_prevError) / dt;
    }
    m_prevError = error;

    double steering = p + i + d;
    
    // Clamp output (postcondition assertion guard)
    steering = std::max(-m_maxOutput, std::min(steering, m_maxOutput));

    // Postconditions
    assert(std::isfinite(steering));
    assert(std::abs(steering) <= m_maxOutput);

    return steering;
}

/**
 * @brief  Callback processing new track position error. Performs safety checks,
 *         runs PID calculations, and publishes cmd_vel targets.
 * @param  msg  Shared pointer to Float32 error.
 */
void PidController::trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_firstMessageReceived = true;
    // Validate Input
    assert(msg != nullptr);
    assert(std::isfinite(msg->data));

    double dt = 0.01; // Decoupled fixed timestep matching the simulation step / physical MGS1600 rate
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    State currentState = p_stateMachine->getCurrentState();
    if (currentState == State::IDLE || currentState == State::INITIALIZE || currentState == State::STOP || currentState == State::ERROR) {
        m_integralError = 0.0;
        m_prevError = 0.0;
        publishVelocity(0.0, 0.0);
        
        // Throttled logging (1.0 second throttle)
        if (m_logCounter++ % 100 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "State: %s | Err: %.3f | Steer: 0.000 (PID Reset)", 
                p_stateMachine->getCurrentStateString().c_str(), msg->data);
        }
        return;
    }

    // Safety checks: Estimate RPM command values to detect saturation
    double error = static_cast<double>(msg->data);
    
    // Speed scaling: reduce velocity proportionally to lateral error magnitude.
    // We use 0.090m (physical sensor range) as the scaling basis, not m_lostThreshold,
    // so a full-scale 90mm error drops speed to 30% nominal. This provides strong
    // braking for the 60 kg robot during high-curvature turns without waiting for
    // the fault threshold (250mm) to be reached.
    // Base speed from navigation server if received, fallback to m_nominalSpeed
    double base_speed = m_navVelReceived ? m_navLinearVel : m_nominalSpeed;

    // Compute PID steering
    double angularVel = computeSteering(error, dt);

    double abs_error = std::abs(error);
    double error_scale = 1.0 - (0.7 * (std::min(abs_error, 0.090) / 0.090));
    double abs_steer = std::abs(angularVel);
    double steer_scale = 1.0 - (0.7 * (std::min(abs_steer, 1.5) / 1.5));
    double speed_scale = std::min(error_scale, steer_scale);
    double linearVel = base_speed * speed_scale;

    // Inverse kinematics to calculate wheel velocities (RPMs)
    double v_l = linearVel - (angularVel * m_wheelBase / 2.0);
    double v_r = linearVel + (angularVel * m_wheelBase / 2.0);
    double rpm_l = (v_l / m_wheelRadius) * 60.0 / (2.0 * M_PI);
    double rpm_r = (v_r / m_wheelRadius) * 60.0 / (2.0 * M_PI);

    // Update safety monitor only when actively navigating
    if (currentState != State::IDLE && currentState != State::INITIALIZE && currentState != State::STOP && currentState != State::ERROR) {
        p_faultMonitor->update(error, m_trackDetect, rpm_l, rpm_r, m_maxRpm);
        if (p_faultMonitor->hasFault()) {
            handleFault(p_faultMonitor->getFaultType());
            return;
        }
    }

    // Manage State transitions & controls
    switch (currentState) {
        case State::FOLLOW_LINE:
            // Line Follower active
            if (m_leftMarker || m_rightMarker) {
                p_stateMachine->transitionTo(State::JUNCTION_DETECTED, "MARKER_DETECTED");
                publishControllerState();
            }
            publishVelocity(linearVel, angularVel);
            break;

        case State::JUNCTION_DETECTED:
            // Slow to 60% nominal approaching junction, keep steering active
            publishVelocity(linearVel * 0.60, angularVel);
            break;

        case State::EXECUTE_TURN: {
            // During turn execution: cap at 40% nominal for sharp cornering stability.
            // The 60 kg robot cannot track a 1m-radius arc at full speed.
            publishVelocity(linearVel * 0.40, angularVel);
            
            // Transition once marker is cleared and turn duration has elapsed
            double elapsed = (this->now() - m_turnStartTime).seconds();
            if (!m_leftMarker && !m_rightMarker && elapsed >= m_turnDuration) {
                p_stateMachine->transitionTo(State::RESUME_TRACKING, "JUNCTION_CLEARED");
                publishControllerState();
            }
            break;
        }

        case State::RESUME_TRACKING:
            // Resume full nominal tracking
            publishVelocity(linearVel, angularVel);
            p_stateMachine->transitionTo(State::FOLLOW_LINE, "RESUMED");
            publishControllerState();
            break;

        case State::ERROR:
            publishVelocity(0.0, 0.0);
            break;

        default:
            break;
    }

    // Throttled logging (1.0 second throttle)
    if (m_logCounter++ % 10 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "State: %s | Err: %.3f | Steer: %.3f", 
            p_stateMachine->getCurrentStateString().c_str(), error, angularVel);
    }
}

/**
 * @brief  Callback processing track presence status.
 * @param  msg  Shared pointer to Bool.
 */
void PidController::trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    assert(msg != nullptr);
    m_trackDetect = msg->data;
}

/**
 * @brief  Callback for left markers.
 * @param  msg  Shared pointer to Bool.
 */
void PidController::leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    assert(msg != nullptr);
    m_leftMarker = msg->data;
}

/**
 * @brief  Callback for right markers.
 * @param  msg  Shared pointer to Bool.
 */
void PidController::rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    assert(msg != nullptr);
    m_rightMarker = msg->data;
}

/**
 * @brief  Callback for left branch positions during junctions.
 * @param  msg  Shared pointer to Float32.
 */
void PidController::leftTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    assert(msg != nullptr);
    m_leftTrackPos = static_cast<double>(msg->data);
}

/**
 * @brief  Callback for right branch positions during junctions.
 * @param  msg  Shared pointer to Float32.
 */
void PidController::rightTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    assert(msg != nullptr);
    m_rightTrackPos = static_cast<double>(msg->data);
}

/**
 * @brief  Callback processing RFID tag reads.
 * @param  msg  Shared pointer to String.
 */
void PidController::tagIdCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string tagId = msg->data;
    RCLCPP_INFO(this->get_logger(), "READ TAG: ID=%s", tagId.c_str());

    State currentState = p_stateMachine->getCurrentState();
    if (currentState == State::JUNCTION_DETECTED) {
        p_stateMachine->transitionTo(State::READ_TAG, "TAG_READ_RECEIVED");
        publishControllerState();
        
        // Execute Fork Selection mathematically using the plan
        std_msgs::msg::String branchMsg;
        
        // 1. Find tag coordinates
        double tag_x = 0.0, tag_y = 0.0;
        bool tag_found = false;
        if (tagId == "TAG_LEFT") {
            tag_x = -5.0; tag_y = 0.0; tag_found = true;
        } else if (tagId == "TAG_TOP") {
            tag_x = -2.0; tag_y = 2.0; tag_found = true;
        } else if (tagId == "TAG_RIGHT") {
            tag_x = 5.0; tag_y = 0.0; tag_found = true;
        } else if (tagId == "TAG_BOT") {
            tag_x = 2.0; tag_y = -2.0; tag_found = true;
        }

        if (m_currentPlan.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Plan is empty. Halting.");
            handleFault("GOAL_REACHED");
            return;
        }

        int tag_idx = -1;
        if (tag_found) {
            double min_d = 1.0; // Max search distance 1m
            for (size_t i = 0; i < m_currentPlan.poses.size(); ++i) {
                double d = std::hypot(m_currentPlan.poses[i].pose.position.x - tag_x,
                                      m_currentPlan.poses[i].pose.position.y - tag_y);
                if (d < min_d) {
                    min_d = d;
                    tag_idx = static_cast<int>(i);
                }
            }
        }

        if (tag_idx == -1) {
            RCLCPP_WARN(this->get_logger(), "Tag %s not found in plan or too far. Treating as goal reached/end of path.", tagId.c_str());
            handleFault("GOAL_REACHED");
            return;
        }

        // If this is the last pose in the plan, we reached the destination
        if (tag_idx == static_cast<int>(m_currentPlan.poses.size() - 1)) {
            RCLCPP_INFO(this->get_logger(), "Reached destination tag %s. Goal reached.", tagId.c_str());
            handleFault("GOAL_REACHED");
            return;
        }

        // Determine fork at next junction
        int junc_idx = tag_idx + 1;
        if (junc_idx >= static_cast<int>(m_currentPlan.poses.size() - 1)) {
            // No junction after this tag, or junction is the goal itself, go STRAIGHT
            branchMsg.data = "STRAIGHT";
            RCLCPP_INFO(this->get_logger(), "No decision junction ahead of tag %s, sending: STRAIGHT", tagId.c_str());
        } else {
            // Incoming vector (from tag to junction)
            double in_x = m_currentPlan.poses[junc_idx].pose.position.x - m_currentPlan.poses[tag_idx].pose.position.x;
            double in_y = m_currentPlan.poses[junc_idx].pose.position.y - m_currentPlan.poses[tag_idx].pose.position.y;
            
            // Outgoing vector (from junction to next waypoint)
            double out_x = m_currentPlan.poses[junc_idx + 1].pose.position.x - m_currentPlan.poses[junc_idx].pose.position.x;
            double out_y = m_currentPlan.poses[junc_idx + 1].pose.position.y - m_currentPlan.poses[junc_idx].pose.position.y;
            
            // Cross product
            double cross_product = in_x * out_y - in_y * out_x;
            
            if (cross_product > 0.1) {
                branchMsg.data = "LEFT";
            } else if (cross_product < -0.1) {
                branchMsg.data = "RIGHT";
            } else {
                branchMsg.data = "STRAIGHT";
            }
            RCLCPP_INFO(this->get_logger(), "Math Fork at junction (cross: %.3f): %s", cross_product, branchMsg.data.c_str());
        }

        m_pubSelectTrack->publish(branchMsg);
        
        // Immediately move to executing the turn
        m_turnStartTime = this->now();
        p_stateMachine->transitionTo(State::EXECUTE_TURN, "BRANCH_SELECTED");
        publishControllerState();
    }
}

/**
 * @brief  Callback processing Navigation Plan.
 * @param  msg  Shared pointer to Path.
 */
void PidController::planCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    assert(msg != nullptr);
    m_currentPlan = *msg;
    m_currentWaypointIndex = 0;
    
    if (m_currentPlan.poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received empty navigation plan. Stopping.");
        p_stateMachine->transitionTo(State::IDLE, "EMPTY_PLAN_RECEIVED");
        publishControllerState();
        publishVelocity(0.0, 0.0);
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received new navigation plan with %zu waypoints.", m_currentPlan.poses.size());

    State currentState = p_stateMachine->getCurrentState();
    if (currentState == State::IDLE || currentState == State::INITIALIZE || currentState == State::ERROR) {
        p_stateMachine->reset(); // Resets to IDLE
        p_faultMonitor->reset(); // Clears any fault/error timer
        p_stateMachine->transitionTo(State::FOLLOW_LINE, "NEW_PLAN_RECEIVED");
        publishControllerState();
    }
}

/**
 * @brief  Timer callback running at 50Hz to verify updates are received.
 */
void PidController::safetyCheckCallback()
{
    if (!m_firstMessageReceived) {
        return;
    }
    p_faultMonitor->checkTimeout(m_lastSensorUpdateTime);
    if (p_faultMonitor->hasFault()) {
        handleFault(p_faultMonitor->getFaultType());
    }
}

/**
 * @brief  Publishes geometry_msgs/Twist target velocity command.
 * @param  linearVel   Target linear speed (m/s).
 * @param  angularVel  Target steering angle rate (rad/s).
 */
void PidController::publishVelocity(double linearVel, double angularVel)
{
    // Limit angular velocity dynamically based on linear velocity to ensure 
    // the robot does not perform inplace rotation (keeps wheels turning forward)
    if (std::abs(linearVel) > 1e-4) {
        double max_allowed_angular_vel = 2.0 * std::abs(linearVel) / m_wheelBase;
        angularVel = std::max(-max_allowed_angular_vel, std::min(angularVel, max_allowed_angular_vel));
    } else {
        angularVel = 0.0;
    }

    geometry_msgs::msg::TwistStamped twistMsg;
    twistMsg.header.stamp = this->now();
    twistMsg.header.frame_id = "base_link";
    twistMsg.twist.linear.x = linearVel;
    twistMsg.twist.angular.z = angularVel;
    m_pubCmdVel->publish(twistMsg);
}

/**
 * @brief  Publishes the current state machine state to /controller_state.
 *         Used by the MapStudio diagnostics panel for real-time monitoring.
 */
void PidController::publishControllerState()
{
    std_msgs::msg::String stateMsg;
    stateMsg.data = p_stateMachine->getCurrentStateString();
    m_pubControllerState->publish(stateMsg);
}

/**
 * @brief  Transitions to ERROR state and stops the AGV.
 * @param  faultType  Description of the triggered safety fault.
 */
void PidController::handleFault(const std::string& faultType)
{
    State currentState = p_stateMachine->getCurrentState();
    if (currentState != State::ERROR) {
        p_stateMachine->transitionTo(State::ERROR, "FAULT_" + faultType);
        publishControllerState();
        publishVelocity(0.0, 0.0);
        
        // Log structured error state
        std::string faultLog = p_faultMonitor->getFaultLog(stateToString(currentState));
        RCLCPP_ERROR(this->get_logger(), "[SAFETY_FAULT] %s", faultLog.c_str());
    }
}

/**
 * @brief  Service callback to simulate autotuning the PID parameters.
 * @param  request   Trigger request.
 * @param  response  Trigger response.
 */
void PidController::autotuneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused
    
    // Simulate an autotuning algorithm (e.g. Ziegler-Nichols heuristic)
    // Scale proportional and derivative gains based on the target nominal speed
    m_kp = 1.5 * (1.0 + (m_nominalSpeed - 0.2) * 5.0);
    m_ki = 0.02; // Keep integral gain relatively low to prevent windup
    m_kd = 0.12 * (1.0 + (m_nominalSpeed - 0.2) * 5.0);
    
    // Reset internal errors
    m_integralError = 0.0;
    m_prevError = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
        "[AUTOTUNE] Complete! Set Kp=%.3f, Ki=%.3f, Kd=%.3f for Nominal Speed=%.2f m/s", 
        m_kp, m_ki, m_kd, m_nominalSpeed);
        
    response->success = true;
    response->message = "Autotune complete! PID parameters dynamically updated.";
}

rcl_interfaces::msg::SetParametersResult PidController::onParameterChange(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Success";

    for (const auto& param : parameters) {
        if (param.get_name() == "pid.kp") {
            m_kp = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter pid.kp updated to %.3f", m_kp);
        } else if (param.get_name() == "pid.ki") {
            m_ki = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter pid.ki updated to %.3f", m_ki);
        } else if (param.get_name() == "pid.kd") {
            m_kd = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter pid.kd updated to %.3f", m_kd);
        } else if (param.get_name() == "pid.windup_limit") {
            m_windupLimit = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter pid.windup_limit updated to %.3f", m_windupLimit);
        } else if (param.get_name() == "pid.max_output") {
            m_maxOutput = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter pid.max_output updated to %.3f", m_maxOutput);
        } else if (param.get_name() == "robot.nominal_speed") {
            m_nominalSpeed = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter robot.nominal_speed updated to %.3f", m_nominalSpeed);
        } else if (param.get_name() == "robot.max_rpm") {
            m_maxRpm = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter robot.max_rpm updated to %.3f", m_maxRpm);
        } else if (param.get_name() == "robot.wheel_base") {
            m_wheelBase = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter robot.wheel_base updated to %.3f", m_wheelBase);
        } else if (param.get_name() == "robot.wheel_radius") {
            m_wheelRadius = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter robot.wheel_radius updated to %.3f", m_wheelRadius);
        } else if (param.get_name() == "safety.lost_threshold") {
            m_lostThreshold = param.as_double();
            p_faultMonitor->setLostThreshold(m_lostThreshold);
            RCLCPP_INFO(this->get_logger(), "Parameter safety.lost_threshold updated to %.3f", m_lostThreshold);
        } else if (param.get_name() == "safety.max_frozen_steps") {
            m_maxFrozenSteps = static_cast<int>(param.as_int());
            p_faultMonitor->setMaxFrozenSteps(m_maxFrozenSteps);
            RCLCPP_INFO(this->get_logger(), "Parameter safety.max_frozen_steps updated to %d", m_maxFrozenSteps);
        } else if (param.get_name() == "safety.turn_duration") {
            m_turnDuration = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter safety.turn_duration updated to %.3f", m_turnDuration);
        }
    }
    return result;
}

void PidController::navVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    assert(msg != nullptr);
    m_navLinearVel = msg->linear.x;
    m_navAngularVel = msg->angular.z;
    m_navVelReceived = true;
}

} // namespace path_follower

/**
 * @brief  Main entry point starting the ROS 2 node.
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_follower::PidController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
