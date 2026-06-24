/*
 * Name:        PidController.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-24
 * Version:     2.0
 * Description: Implements the PidController class for ROS 2 magnetic line following.
 */

#include "PidController.h"
#include <cassert>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "geometry_msgs/msg/twist_stamped.hpp"
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
      m_sensorOffsetX(0.48),
      m_lostThreshold(0.25),
      m_maxFrozenSteps(50),
      m_turnDuration(3.0),
      m_junctionDivergenceThreshold(0.02),
      m_turnIndex(0),
      m_loopSequence(true),
      m_integralError(0.0),
      m_prevError(0.0),
      m_turnStartTime(this->now()),
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
    
    this->declare_parameter<double>("robot.nominal_speed", m_nominalSpeed);
    this->declare_parameter<double>("robot.max_rpm", m_maxRpm);
    this->declare_parameter<double>("robot.wheel_base", m_wheelBase);
    this->declare_parameter<double>("robot.wheel_radius", m_wheelRadius);
    this->declare_parameter<double>("robot.sensor_offset_x", m_sensorOffsetX);
    
    this->declare_parameter<double>("safety.lost_threshold", m_lostThreshold);
    this->declare_parameter<int>("safety.max_frozen_steps", m_maxFrozenSteps);
    this->declare_parameter<double>("safety.turn_duration", m_turnDuration);
    
    this->declare_parameter<double>("junction.divergence_threshold", m_junctionDivergenceThreshold);
    this->declare_parameter<std::vector<std::string>>("junction.turn_sequence", {"STRAIGHT"});
    this->declare_parameter<bool>("junction.loop_sequence", m_loopSequence);

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
    this->get_parameter("robot.sensor_offset_x", m_sensorOffsetX);
    this->get_parameter("safety.lost_threshold", m_lostThreshold);
    this->get_parameter("safety.max_frozen_steps", m_maxFrozenSteps);
    this->get_parameter("safety.turn_duration", m_turnDuration);
    this->get_parameter("junction.divergence_threshold", m_junctionDivergenceThreshold);
    this->get_parameter("junction.turn_sequence", m_turnSequence);
    this->get_parameter("junction.loop_sequence", m_loopSequence);

    // Initial time points
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    // Instantiate State Machine and Safety Monitor
    p_stateMachine = std::make_unique<NavigationStateMachine>();
    p_faultMonitor = std::make_unique<FaultMonitor>(m_lostThreshold, m_maxFrozenSteps);
    p_stateMachine->transitionTo(State::INITIALIZE, "NODE_START");

    // ROS 2 Subscribers
    m_subTrackPos = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/track_position_test", 10, std::bind(&PidController::trackPosCallback, this, std::placeholders::_1));
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
    m_subTapeCross = this->create_subscription<std_msgs::msg::Bool>(
        "/sensor/tape_cross", 10, std::bind(&PidController::tapeCrossCallback, this, std::placeholders::_1));

    // ROS 2 Publishers
    m_pubCmdVel = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    m_pubControllerState = this->create_publisher<std_msgs::msg::String>("/controller_state", 10);

    // ROS 2 Services
    m_srvAutotune = this->create_service<std_srvs::srv::Trigger>(
        "/autotune", std::bind(&PidController::autotuneCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvSaveTuning = this->create_service<std_srvs::srv::Trigger>(
        "~/save_tuning", std::bind(&PidController::saveTuningCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvStart = this->create_service<std_srvs::srv::Trigger>(
        "~/start", std::bind(&PidController::startCallback, this, std::placeholders::_1, std::placeholders::_2));
    m_srvStop = this->create_service<std_srvs::srv::Trigger>(
        "~/stop", std::bind(&PidController::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

    // ROS 2 Service Clients (MGS Driver)
    m_cliFollowLeft = this->create_client<std_srvs::srv::Trigger>("/sensor/follow_left");
    m_cliFollowRight = this->create_client<std_srvs::srv::Trigger>("/sensor/follow_right");
    m_cliClearFollow = this->create_client<std_srvs::srv::Trigger>("/sensor/clear_follow");

    // Safety timeout check timer (50Hz = 20ms)
    m_safetyTimer = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&PidController::safetyCheckCallback, this));

    // Register dynamic parameter callback
    m_callbackHandle = this->add_on_set_parameters_callback(
        std::bind(&PidController::onParameterChange, this, std::placeholders::_1));

    // Move to run state
    p_stateMachine->transitionTo(State::IDLE, "INITIALIZATION_COMPLETE");
    publishControllerState();
    RCLCPP_INFO(this->get_logger(), "Path Follower running. Call ~/start to begin tracking.");
}

PidController::~PidController()
{
    publishVelocity(0.0, 0.0);
}

double PidController::computeSteering(double error, double dt)
{
    assert(std::isfinite(error));
    assert(dt > 0.0);

    // Lookahead heading angle calculation (atan2(y, x))
    // We treat the lateral error 'e_y' as the Y offset, and the sensor distance 'm_sensorOffsetX' as the X offset.
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

void PidController::trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_firstMessageReceived = true;
    assert(msg != nullptr);
    assert(std::isfinite(msg->data));

    double dt = 0.01; // Fixed timestep
    m_lastSensorUpdateTime = std::chrono::steady_clock::now();

    State currentState = p_stateMachine->getCurrentState();
    if (currentState == State::IDLE || currentState == State::INITIALIZE || currentState == State::STOP || currentState == State::ERROR) {
        m_integralError = 0.0;
        m_prevError = 0.0;
        publishVelocity(0.0, 0.0);
        
        if (m_logCounter++ % 100 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "State: %s | Err: %.3f", p_stateMachine->getCurrentStateString().c_str(), msg->data);
        }
        return;
    }

    double error = static_cast<double>(msg->data);
    double angularVel = computeSteering(error, dt);

    // Speed scaling based on error magnitude
    double abs_error = std::abs(error);
    double error_scale = 1.0 - (0.7 * (std::min(abs_error, 0.090) / 0.090));
    double abs_steer = std::abs(angularVel);
    double steer_scale = 1.0 - (0.7 * (std::min(abs_steer, 1.5) / 1.5));
    double speed_scale = std::min(error_scale, steer_scale);
    double linearVel = m_nominalSpeed * speed_scale * 0.30; // Clamped to 30% for testing

    // Inverse kinematics for fault monitor saturation checking
    double v_l = linearVel - (angularVel * m_wheelBase / 2.0);
    double v_r = linearVel + (angularVel * m_wheelBase / 2.0);
    double rpm_l = (v_l / m_wheelRadius) * 60.0 / (2.0 * M_PI);
    double rpm_r = (v_r / m_wheelRadius) * 60.0 / (2.0 * M_PI);

    p_faultMonitor->update(error, m_trackDetect, rpm_l, rpm_r, m_maxRpm);
    if (p_faultMonitor->hasFault()) {
        handleFault(p_faultMonitor->getFaultType());
        return;
    }

    // State Management
    switch (currentState) {
        case State::FOLLOW_LINE: {
            double divergence = std::abs(m_leftTrackPos - m_rightTrackPos);
            if (divergence > m_junctionDivergenceThreshold || m_tapeCross) {
                p_stateMachine->transitionTo(State::JUNCTION_DETECTED, "DIVERGENCE_OR_CROSS");
                publishControllerState();
            }
            publishVelocity(linearVel, angularVel);
            break;
        }

        case State::JUNCTION_DETECTED: {
            // Drop speed approaching junction
            publishVelocity(linearVel * 0.60, angularVel);
            
            // Execute programmatic sequence
            executeJunctionTurn();
            break;
        }

        case State::EXECUTE_TURN: {
            // Slower speed for sharp cornering
            publishVelocity(linearVel * 0.40, angularVel);
            
            // Check if tracks have merged back into one line
            double divergence = std::abs(m_leftTrackPos - m_rightTrackPos);
            double elapsed = (this->now() - m_turnStartTime).seconds();
            
            // Ensure minimum time spent in turn so we don't prematurely exit before crossing the gap
            if (divergence < m_junctionDivergenceThreshold && elapsed >= 1.0 && !m_tapeCross) {
                
                // Clear the manual track follow override
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                m_cliClearFollow->async_send_request(req);
                
                p_stateMachine->transitionTo(State::RESUME_TRACKING, "TRACKS_MERGED");
                publishControllerState();
            }
            break;
        }

        case State::RESUME_TRACKING:
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

    if (m_logCounter++ % 10 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "State: %s | Err: %.3f | Steer: %.3f", 
            p_stateMachine->getCurrentStateString().c_str(), error, angularVel);
    }
}

void PidController::executeJunctionTurn()
{
    if (m_turnSequence.empty()) {
        RCLCPP_WARN(this->get_logger(), "Turn sequence is empty! Halting.");
        handleFault("EMPTY_SEQUENCE");
        return;
    }

    if (m_turnIndex >= m_turnSequence.size()) {
        if (m_loopSequence) {
            m_turnIndex = 0; // Wrap around
            RCLCPP_INFO(this->get_logger(), "Turn sequence wrapped around to 0.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Reached end of turn sequence. Halting.");
            handleFault("SEQUENCE_COMPLETE");
            return;
        }
    }

    std::string nextTurn = m_turnSequence[m_turnIndex];
    m_turnIndex++;

    RCLCPP_INFO(this->get_logger(), "Executing Turn: %s", nextTurn.c_str());

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    if (nextTurn == "LEFT") {
        m_cliFollowLeft->async_send_request(req);
    } else if (nextTurn == "RIGHT") {
        m_cliFollowRight->async_send_request(req);
    } else if (nextTurn == "STRAIGHT") {
        // Clear follow implicitly favors dominant track or straight line
        m_cliClearFollow->async_send_request(req);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown turn command '%s', defaulting to clear.", nextTurn.c_str());
        m_cliClearFollow->async_send_request(req);
    }

    m_turnStartTime = this->now();
    p_stateMachine->transitionTo(State::EXECUTE_TURN, "TURN_COMMAND_SENT");
    publishControllerState();
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
    
    if (currentState == State::IDLE || currentState == State::STOP || currentState == State::ERROR) {
        p_stateMachine->reset();
        p_faultMonitor->reset();
        p_stateMachine->transitionTo(State::FOLLOW_LINE, "START_SERVICE_CALLED");
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
    p_stateMachine->transitionTo(State::STOP, "STOP_SERVICE_CALLED");
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

void PidController::publishControllerState()
{
    std_msgs::msg::String stateMsg;
    stateMsg.data = p_stateMachine->getCurrentStateString();
    m_pubControllerState->publish(stateMsg);
}

void PidController::handleFault(const std::string& faultType)
{
    State currentState = p_stateMachine->getCurrentState();
    if (currentState != State::ERROR) {
        p_stateMachine->transitionTo(State::ERROR, "FAULT_" + faultType);
        publishControllerState();
        publishVelocity(0.0, 0.0);
        
        std::string faultLog = p_faultMonitor->getFaultLog(stateToString(currentState));
        RCLCPP_ERROR(this->get_logger(), "[SAFETY_FAULT] %s", faultLog.c_str());
    }
}

void PidController::autotuneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    m_kp = 1.5 * (1.0 + (m_nominalSpeed - 0.2) * 5.0);
    m_ki = 0.02;
    m_kd = 0.12 * (1.0 + (m_nominalSpeed - 0.2) * 5.0);
    
    m_integralError = 0.0;
    m_prevError = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "[AUTOTUNE] Complete! Set Kp=%.3f, Ki=%.3f, Kd=%.3f for Nominal Speed=%.2f m/s", 
        m_kp, m_ki, m_kd, m_nominalSpeed);
        
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
            case const_hash("robot.nominal_speed"): m_nominalSpeed = param.as_double(); break;
            case const_hash("robot.max_rpm"): m_maxRpm = param.as_double(); break;
            case const_hash("robot.wheel_base"): m_wheelBase = param.as_double(); break;
            case const_hash("robot.wheel_radius"): m_wheelRadius = param.as_double(); break;
            case const_hash("robot.sensor_offset_x"): m_sensorOffsetX = param.as_double(); break;
            case const_hash("safety.lost_threshold"): 
                m_lostThreshold = param.as_double(); 
                p_faultMonitor->setLostThreshold(m_lostThreshold);
                break;
            case const_hash("safety.max_frozen_steps"): 
                m_maxFrozenSteps = static_cast<int>(param.as_int()); 
                p_faultMonitor->setMaxFrozenSteps(m_maxFrozenSteps);
                break;
            case const_hash("safety.turn_duration"): m_turnDuration = param.as_double(); break;
            case const_hash("junction.divergence_threshold"): m_junctionDivergenceThreshold = param.as_double(); break;
            case const_hash("junction.turn_sequence"): m_turnSequence = param.as_string_array(); break;
            case const_hash("junction.loop_sequence"): m_loopSequence = param.as_bool(); break;
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
