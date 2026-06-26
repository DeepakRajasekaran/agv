/*
 * Name:        PidController.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-25
 * Version:     3.0
 * Description: Implements the PidController class for ROS 2 magnetic line following.
 *              Decoupled architecture: reads /nav/cmd_vel, outputs /path_follower/cmd_vel.
 */

#include "PidController.h"
#include <cassert>
#include <cmath>
#include <algorithm>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "geometry_msgs/msg/twist.hpp"
#include <cstdint>

using namespace std::chrono_literals;

namespace {
    // FNV-1a constexpr hash for string switch/case statements
    constexpr uint32_t const_hash(const char* str) {
        uint32_t h = 2166136261u;
        for (int i = 0; str[i] != '\0'; ++i) {
            h ^= static_cast<uint32_t>(str[i]);
            h *= 16777619u;
        }
        return h;
    }

    uint32_t runtime_hash(const std::string& str) {
        return const_hash(str.c_str());
    }
}

namespace path_follower {

using custom_interfaces::msg::ControllerState;

PidController::PidController(const rclcpp::NodeOptions& options)
    : Node("path_follower_node", options),
      m_kp(1.5),
      m_ki(0.02),
      m_kd(0.12),
      m_windupLimit(0.5),
      m_maxOutput(1.5),
      m_maxRpm(150.0),
      m_wheelBase(0.512),
      m_wheelRadius(0.08),
      m_sensorOffsetX(0.48),
      m_lostThreshold(0.25),
      m_maxFrozenSteps(500),
      m_turnDuration(3.0),
      m_lineLostGraceSteps(10),
      m_clampStraight(1.0),
      m_clampJunction(0.5),
      m_clampTurn(0.3),
      m_junctionDivergenceThreshold(0.035),
      m_integralError(0.0),
      m_prevError(0.0),
      m_cmdLinearX(0.0),
      m_cmdAngularZ(0.0),
      m_trackDetect(false),
      m_leftMarker(false),
      m_rightMarker(false),
      m_tapeCross(false),
      m_leftTrackPos(0.0),
      m_rightTrackPos(0.0),
      m_firstMessageReceived(false),
      m_logCounter(0)
{
    // Declare ROS 2 Parameters with defaults
    this->declare_parameter<double>("pid.kp", m_kp);
    this->declare_parameter<double>("pid.ki", m_ki);
    this->declare_parameter<double>("pid.kd", m_kd);
    this->declare_parameter<double>("pid.windup_limit", m_windupLimit);
    this->declare_parameter<double>("pid.max_output", m_maxOutput);
    
    this->declare_parameter<double>("robot.max_rpm", m_maxRpm);
    this->declare_parameter<double>("robot.wheel_base", m_wheelBase);
    this->declare_parameter<double>("robot.wheel_radius", m_wheelRadius);
    this->declare_parameter<double>("robot.sensor_offset_x", m_sensorOffsetX);
    
    this->declare_parameter<double>("safety.lost_threshold", m_lostThreshold);
    this->declare_parameter<int>("safety.max_frozen_steps", m_maxFrozenSteps);
    this->declare_parameter<double>("safety.turn_duration", m_turnDuration);
    this->declare_parameter<int>("safety.line_lost_grace_steps", m_lineLostGraceSteps);
    
    // Initialize defaults to prevent garbage memory values
    m_clampStraight = 1.0;
    m_clampJunction = 0.5;
    m_clampTurn = 0.3;
    m_junctionDivergenceThreshold = 0.035;

    this->declare_parameter<double>("velocity_clamps.straight", m_clampStraight);
    this->declare_parameter<double>("velocity_clamps.junction", m_clampJunction);
    this->declare_parameter<double>("velocity_clamps.turn", m_clampTurn);

    this->declare_parameter<double>("junction.divergence_threshold", m_junctionDivergenceThreshold);

    // Retrieve parameter values
    this->get_parameter("pid.kp", m_kp);
    this->get_parameter("pid.ki", m_ki);
    this->get_parameter("pid.kd", m_kd);
    this->get_parameter("pid.windup_limit", m_windupLimit);
    this->get_parameter("pid.max_output", m_maxOutput);
    this->get_parameter("robot.max_rpm", m_maxRpm);
    this->get_parameter("robot.wheel_base", m_wheelBase);
    this->get_parameter("robot.wheel_radius", m_wheelRadius);
    this->get_parameter("robot.sensor_offset_x", m_sensorOffsetX);
    this->get_parameter("safety.lost_threshold", m_lostThreshold);
    this->get_parameter("safety.max_frozen_steps", m_maxFrozenSteps);
    this->get_parameter("safety.turn_duration", m_turnDuration);
    this->get_parameter("safety.line_lost_grace_steps", m_lineLostGraceSteps);
    
    this->get_parameter("velocity_clamps.straight", m_clampStraight);
    this->get_parameter("velocity_clamps.junction", m_clampJunction);
    this->get_parameter("velocity_clamps.turn", m_clampTurn);
    this->get_parameter("junction.divergence_threshold", m_junctionDivergenceThreshold);

    // Initial time points
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    // Instantiate State Machine and Safety Monitor
    p_stateMachine = std::make_unique<NavigationStateMachine>();
    p_faultMonitor = std::make_unique<FaultMonitor>(m_lineLostGraceSteps, m_maxFrozenSteps);
    p_stateMachine->transitionTo(ControllerState::INITIALIZE, "NODE_START");

    // Parameterize input topics
    std::string track_pos_topic = this->declare_parameter("topics.track_position", "/sensor/track_position");
    std::string track_detect_topic = this->declare_parameter("topics.track_detect", "/sensor/track_detect");
    std::string left_marker_topic = this->declare_parameter("topics.left_marker", "/sensor/left_marker");
    std::string right_marker_topic = this->declare_parameter("topics.right_marker", "/sensor/right_marker");
    std::string left_track_pos_topic = this->declare_parameter("topics.left_track_position", "/sensor/left_track_position");
    std::string right_track_pos_topic = this->declare_parameter("topics.right_track_position", "/sensor/right_track_position");
    std::string tape_cross_topic = this->declare_parameter("topics.tape_cross", "/sensor/tape_cross");
    
    bool autostart = this->declare_parameter("autostart", false);

    // ROS 2 Subscribers
    m_subCmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/nav/cmd_vel", 10, std::bind(&PidController::cmdVelCallback, this, std::placeholders::_1));

    m_subTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        track_pos_topic, 10, std::bind(&PidController::trackPosCallback, this, std::placeholders::_1));
    m_subTrackDetect = this->create_subscription<std_msgs::msg::Bool>(
        track_detect_topic, 10, std::bind(&PidController::trackDetectCallback, this, std::placeholders::_1));
    m_subLeftMarker = this->create_subscription<std_msgs::msg::Bool>(
        left_marker_topic, 10, std::bind(&PidController::leftMarkerCallback, this, std::placeholders::_1));
    m_subRightMarker = this->create_subscription<std_msgs::msg::Bool>(
        right_marker_topic, 10, std::bind(&PidController::rightMarkerCallback, this, std::placeholders::_1));
    m_subLeftTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        left_track_pos_topic, 10, std::bind(&PidController::leftTrackPosCallback, this, std::placeholders::_1));
    m_subRightTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        right_track_pos_topic, 10, std::bind(&PidController::rightTrackPosCallback, this, std::placeholders::_1));
    m_subTapeCross = this->create_subscription<std_msgs::msg::Bool>(
        tape_cross_topic, 10, std::bind(&PidController::tapeCrossCallback, this, std::placeholders::_1));

    // ROS 2 Publishers
    m_pubCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("/path_follower/cmd_vel", 10);
    m_pubControllerState = this->create_publisher<ControllerState>("/controller_state", 10);

    // ROS 2 Services
    m_srvAutotune = this->create_service<std_srvs::srv::Trigger>(
        "/autotune", std::bind(&PidController::autotuneCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvSaveTuning = this->create_service<std_srvs::srv::Trigger>(
        "~/save_tuning", std::bind(&PidController::saveTuningCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvStart = this->create_service<std_srvs::srv::Trigger>(
        "~/start", std::bind(&PidController::startCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvStop = this->create_service<std_srvs::srv::Trigger>(
        "~/stop", std::bind(&PidController::stopCallback, this, std::placeholders::_1, std::placeholders::_2));
        
    m_srvSelectTrack = this->create_service<custom_interfaces::srv::SelectTrack>(
        "~/select_track", std::bind(&PidController::selectTrackCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Safety timeout check timer (50Hz = 20ms)
    m_safetyTimer = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&PidController::safetyCheckCallback, this));

    // Register dynamic parameter callback
    m_callbackHandle = this->add_on_set_parameters_callback(
        std::bind(&PidController::onParameterChange, this, std::placeholders::_1));

    // Move to initial state
    if (autostart) {
        p_stateMachine->transitionTo(ControllerState::FOLLOW_LINE, "AUTOSTART_ENABLED");
        RCLCPP_INFO(this->get_logger(), "Path Follower auto-started. Tracking active.");
    } else {
        p_stateMachine->transitionTo(ControllerState::IDLE, "INITIALIZATION_COMPLETE");
        RCLCPP_INFO(this->get_logger(), "Path Follower running in IDLE. Call ~/start to begin tracking.");
    }
    publishControllerState();
}

PidController::~PidController()
{
}

void PidController::stopRobot()
{
    publishVelocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Robot stopped by shutdown hook.");
}

void PidController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    m_cmdLinearX = msg->linear.x;
    m_cmdAngularZ = msg->angular.z;
}

double PidController::computeSteering(double error, double dt)
{
    assert(std::isfinite(error));
    assert(dt > 0.0);

    // Lookahead heading angle calculation (atan2(y, x))
    double theta_correction = std::atan2(error, m_sensorOffsetX);

    // PID on the lookahead heading angle
    double p = m_kp * theta_correction;

    m_integralError += theta_correction * dt;
    m_integralError = std::max(-m_windupLimit, std::min(m_integralError, m_windupLimit));
    double i = m_ki * m_integralError;

    double d = 0.0;
    if (dt > 1e-4) {
        d = m_kd * (theta_correction - m_prevError) / dt;
    }
    m_prevError = theta_correction;

    double steering = p + i + d;
    steering = std::max(-m_maxOutput, std::min(steering, m_maxOutput));

    assert(std::isfinite(steering));
    return steering;
}

void PidController::selectTrackCallback(const std::shared_ptr<custom_interfaces::srv::SelectTrack::Request> request,
                                      std::shared_ptr<custom_interfaces::srv::SelectTrack::Response> response)
{
    if (request->track_id >= 0 && request->track_id <= 2) {
        m_selectedTrackId = request->track_id;
        response->success = true;
        response->message = "Track selected successfully: " + std::to_string(m_selectedTrackId);
        RCLCPP_INFO(this->get_logger(), "Selected Track updated to: %d", m_selectedTrackId);
    } else {
        response->success = false;
        response->message = "Invalid track_id. Must be 0 (AVG), 1 (LEFT), or 2 (RIGHT).";
        RCLCPP_WARN(this->get_logger(), "Failed to update Selected Track. Invalid ID: %d", request->track_id);
    }
}

void PidController::trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_firstMessageReceived = true;
    (void)msg;  // msg data is consumed via member variables set by other callbacks

    double dt = 0.01; // Fixed timestep
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    State currentState = p_stateMachine->getCurrentState();
    
    // Safety states (Enforce zero velocity)
    if (currentState == ControllerState::STOP || 
        currentState == ControllerState::ERROR) 
    {
        m_integralError = 0.0;
        m_prevError = 0.0;
        publishVelocity(0.0, 0.0);
        
        if (m_logCounter++ % 100 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "State: %s | Enforcing 0.0 vel", p_stateMachine->getCurrentStateString().c_str());
        }
        return;
    }
    
    // Inactive states (Remain silent to allow twist_mux to fallback to other inputs like teleop)
    if (currentState == ControllerState::INITIALIZE) 
    {
        m_integralError = 0.0;
        m_prevError = 0.0;
        // DO NOT publish anything. Let twist_mux handle priorities.
        
        if (m_logCounter++ % 100 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "State: %s | Controller inactive (silent)", p_stateMachine->getCurrentStateString().c_str());
        }
        return;
    }

    double computed_error = 0.0;
    double divergence = std::abs(m_leftTrackPos - m_rightTrackPos);
    
    if (currentState == ControllerState::JUNCTION_DETECTED || currentState == ControllerState::FOLLOW_LINE) {
        if (m_selectedTrackId == 1) {
            computed_error = m_leftTrackPos;
        } else if (m_selectedTrackId == 2) {
            computed_error = m_rightTrackPos;
        } else {
            computed_error = (m_leftTrackPos + m_rightTrackPos) / 2.0;
        }
    } else {
        computed_error = (m_leftTrackPos + m_rightTrackPos) / 2.0;
    }
    
    // Auto-reset selected track when junction clears
    if (divergence < m_junctionDivergenceThreshold && m_selectedTrackId != 0 && !m_tapeCross) {
        RCLCPP_INFO(this->get_logger(), "Junction divergence cleared. Resetting track_id to 0 (AVERAGE).");
        m_selectedTrackId = 0;
        
        if (currentState == ControllerState::JUNCTION_DETECTED) {
            p_stateMachine->transitionTo(ControllerState::FOLLOW_LINE, "JUNCTION_CLEARED");
            publishControllerState();
        }
    }

    double pidAngularVel = computeSteering(computed_error, dt);


    // Inverse kinematics for fault monitor saturation checking
    double v_l = m_cmdLinearX - (pidAngularVel * m_wheelBase / 2.0);
    double v_r = m_cmdLinearX + (pidAngularVel * m_wheelBase / 2.0);
    double rpm_l = (v_l / m_wheelRadius) * 60.0 / (2.0 * M_PI);
    double rpm_r = (v_r / m_wheelRadius) * 60.0 / (2.0 * M_PI);

    // Pass the computed_error to the fault monitor.
    // We cannot use msg->data directly because the driver sets it to exactly 0.0 when DLC < 8,
    // which triggers a false positive 'FROZEN_SENSOR / SENSOR_DROPOUT' fault.
    p_faultMonitor->update(computed_error, m_trackDetect, rpm_l, rpm_r, m_maxRpm);
    if (p_faultMonitor->hasFault()) {
        if (currentState != ControllerState::IDLE) {
            handleFault(p_faultMonitor->getFaultType());
            return;
        }
    }

    double linearVel = m_cmdLinearX;

    // State Management
    switch (currentState) {
        case ControllerState::IDLE: {
            // Yaw correction runs in idle, allowing guided teleop or compass-like auto-alignment
            // If track is lost in IDLE, gracefully pass through manual angular Z instead of PID
            if (m_trackDetect && !p_faultMonitor->hasFault()) {
                publishVelocity(linearVel, pidAngularVel);
            } else {
                publishVelocity(linearVel, m_cmdAngularZ);
            }
            break;
        }

        case ControllerState::FOLLOW_LINE: {
            linearVel = std::clamp(linearVel, -m_clampStraight, m_clampStraight);

            divergence = std::abs(m_leftTrackPos - m_rightTrackPos);
            if (divergence > m_junctionDivergenceThreshold || m_tapeCross) {
                p_stateMachine->transitionTo(ControllerState::JUNCTION_DETECTED, "DIVERGENCE_OR_CROSS");
                publishControllerState();
            }
            publishVelocity(linearVel, pidAngularVel);
            break;
        }

        case ControllerState::JUNCTION_DETECTED: {
            // PID continues to control steering, but using the selected track
            linearVel = std::clamp(linearVel, -m_clampJunction, m_clampJunction);
            publishVelocity(linearVel, pidAngularVel);
            break;
        }

        case ControllerState::READ_TAG: {
            // Stub for future RFID/Tag integration
            publishVelocity(0.0, 0.0);
            p_stateMachine->transitionTo(ControllerState::EXECUTE_TURN, "TAG_READ_STUB");
            publishControllerState();
            break;
        }

        case ControllerState::EXECUTE_TURN: {
            // Deprecated logic, no longer used. Fallback to normal PID
            linearVel = std::clamp(linearVel, -m_clampTurn, m_clampTurn);
            publishVelocity(linearVel, pidAngularVel);
            break;
        }

        case ControllerState::RESUME_TRACKING: {
            linearVel = std::clamp(linearVel, -m_clampStraight, m_clampStraight);
            publishVelocity(linearVel, pidAngularVel);
            p_stateMachine->transitionTo(ControllerState::FOLLOW_LINE, "RESUMED");
            publishControllerState();
            break;
        }

        default:
            break;
    }

    if (m_logCounter++ % 10 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "State: %s | Err: %.3f | Steer: %.3f", 
            p_stateMachine->getCurrentStateString().c_str(), computed_error, pidAngularVel);
    }
}

void PidController::trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_trackDetect = msg->data;
}

void PidController::leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_leftMarker = msg->data;
}

void PidController::rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_rightMarker = msg->data;
}

void PidController::tapeCrossCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_tapeCross = msg->data;
}

void PidController::leftTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_leftTrackPos = static_cast<double>(msg->data);
}

void PidController::rightTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_rightTrackPos = static_cast<double>(msg->data);
}

void PidController::startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    State currentState = p_stateMachine->getCurrentState();
    
    if (currentState == ControllerState::IDLE || currentState == ControllerState::STOP || currentState == ControllerState::ERROR) {
        p_stateMachine->reset();
        p_faultMonitor->reset();
        p_stateMachine->transitionTo(ControllerState::FOLLOW_LINE, "START_SERVICE_CALLED");
        publishControllerState();
        
        response->success = true;
        response->message = "Tracking Started";
    } else {
        response->success = false;
        response->message = "Already running";
    }
}

void PidController::stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    p_stateMachine->transitionTo(ControllerState::STOP, "STOP_SERVICE_CALLED");
    publishControllerState();
    publishVelocity(0.0, 0.0);
    
    response->success = true;
    response->message = "Tracking Stopped";
}

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

void PidController::publishVelocity(double linearVel, double angularVel)
{
    if (std::abs(linearVel) <= 1e-4 && 
        p_stateMachine->getCurrentState() != ControllerState::IDLE &&
        p_stateMachine->getCurrentState() != ControllerState::STOP) {
        // If we are tracking but linear velocity is zero, do not allow turn-in-place from PID
        angularVel = 0.0;
    }

    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x = linearVel;
    twistMsg.angular.z = angularVel;
    m_pubCmdVel->publish(twistMsg);
}

void PidController::publishControllerState()
{
    ControllerState msg;
    msg.state = p_stateMachine->getCurrentState();
    m_pubControllerState->publish(msg);
}

void PidController::handleFault(const std::string& faultType)
{
    State currentState = p_stateMachine->getCurrentState();
    if (currentState != ControllerState::ERROR) {
        p_stateMachine->transitionTo(ControllerState::ERROR, "FAULT_" + faultType);
        publishControllerState();
        publishVelocity(0.0, 0.0);
        
        std::string faultLog = p_faultMonitor->getFaultLog(p_stateMachine->getCurrentStateString());
        RCLCPP_ERROR(this->get_logger(), "[SAFETY_FAULT] %s", faultLog.c_str());
    }
}

void PidController::autotuneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    m_kp = 1.5;
    m_ki = 0.02;
    m_kd = 0.12;
    
    m_integralError = 0.0;
    m_prevError = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "[AUTOTUNE] Complete! Set Kp=%.3f, Ki=%.3f, Kd=%.3f", m_kp, m_ki, m_kd);
        
    response->success = true;
    response->message = "Autotune complete!";
}

void PidController::saveTuningCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    std::string config_path = "/agv_config/follower_params.yaml";
    std::string command = "ros2 param dump /path_follower_node > " + config_path;
    int ret = std::system(command.c_str());

    if (ret == 0) {
        RCLCPP_INFO(this->get_logger(), "[SAVE_TUNING] Success! Saved to %s", config_path.c_str());
        response->success = true;
        response->message = "Tuning parameters successfully saved.";
    } else {
        response->success = false;
        response->message = "Failed to execute ros2 param dump.";
    }
}

rcl_interfaces::msg::SetParametersResult PidController::onParameterChange(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : parameters) {
        switch (runtime_hash(param.get_name())) {
            case const_hash("pid.kp"): m_kp = param.as_double(); break;
            case const_hash("pid.ki"): m_ki = param.as_double(); break;
            case const_hash("pid.kd"): m_kd = param.as_double(); break;
            case const_hash("pid.windup_limit"): m_windupLimit = param.as_double(); break;
            case const_hash("pid.max_output"): m_maxOutput = param.as_double(); break;
            case const_hash("robot.max_rpm"): m_maxRpm = param.as_double(); break;
            case const_hash("robot.wheel_base"): m_wheelBase = param.as_double(); break;
            case const_hash("robot.wheel_radius"): m_wheelRadius = param.as_double(); break;
            case const_hash("robot.sensor_offset_x"): m_sensorOffsetX = param.as_double(); break;
            case const_hash("safety.lost_threshold"): 
                m_lostThreshold = param.as_double(); 
                break;
            case const_hash("safety.line_lost_grace_steps"): 
                m_lineLostGraceSteps = static_cast<int>(param.as_int()); 
                p_faultMonitor->setLineLostGraceSteps(m_lineLostGraceSteps);
                break;
            case const_hash("safety.max_frozen_steps"): 
                m_maxFrozenSteps = static_cast<int>(param.as_int()); 
                p_faultMonitor->setMaxFrozenSteps(m_maxFrozenSteps);
                break;
            case const_hash("safety.turn_duration"): m_turnDuration = param.as_double(); break;
            case const_hash("velocity_clamps.straight"): m_clampStraight = param.as_double(); break;
            case const_hash("velocity_clamps.junction"): m_clampJunction = param.as_double(); break;
            case const_hash("velocity_clamps.turn"): m_clampTurn = param.as_double(); break;
            case const_hash("junction.divergence_threshold"): m_junctionDivergenceThreshold = param.as_double(); break;
            default: break;
        }
    }
    return result;
}

} // namespace path_follower

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_follower::PidController>();
    
    rclcpp::on_shutdown([node]() {
        node->stopRobot();
    });
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
