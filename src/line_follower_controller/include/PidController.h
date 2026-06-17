/*
 * Name:        PidController.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Declares the PidController class for ROS 2.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <string>
#include <chrono>

#include "NavigationStateMachine.h"
#include "FaultMonitor.h"

namespace line_follower {

class PidController : public rclcpp::Node {
public:
    explicit PidController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PidController();

    // Disable copy/move constructors for safety
    PidController(const PidController&) = delete;
    PidController& operator=(const PidController&) = delete;

private:
    // ROS 2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subTrackPos;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subTrackDetect;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subLeftMarker;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subRightMarker;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subLeftTrackPos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subRightTrackPos;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subTagId;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_subPlan;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subNavVel;

    // ROS 2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pubCmdVel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubSelectTrack;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubControllerState;

    // Callbacks
    void trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void rightTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void tagIdCallback(const std_msgs::msg::String::SharedPtr msg);
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void navVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS 2 Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvAutotune;
    void autotuneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Timer for safety monitor timeout check (50Hz)
    rclcpp::TimerBase::SharedPtr m_safetyTimer;
    void safetyCheckCallback();

    // PID Internal Logic
    double computeSteering(double error, double dt);

    // Helper functions
    void publishVelocity(double linearVel, double angularVel);
    void handleFault(const std::string& faultType);
    void publishControllerState();

    // Tunable Parameters
    double m_kp;
    double m_ki;
    double m_kd;
    double m_windupLimit;
    double m_maxOutput;
    
    double m_nominalSpeed;
    double m_maxRpm;
    double m_wheelBase;
    double m_wheelRadius;
    
    double m_lostThreshold;
    int m_maxFrozenSteps;
    double m_turnDuration;

    // Control Loop State
    double m_integralError;
    double m_prevError;
    std::chrono::steady_clock::time_point m_lastLoopTime;
    std::chrono::steady_clock::time_point m_lastSensorUpdateTime;
    rclcpp::Time m_turnStartTime;
    
    bool m_trackDetect;
    bool m_leftMarker;
    bool m_rightMarker;
    double m_leftTrackPos;
    double m_rightTrackPos;
    std::string m_tagId;
    nav_msgs::msg::Path m_currentPlan;
    size_t m_currentWaypointIndex;
    
    // Subclass pointer instances
    std::unique_ptr<NavigationStateMachine> p_stateMachine;
    std::unique_ptr<FaultMonitor> p_faultMonitor;
    
    bool m_firstMessageReceived;
    double m_navLinearVel;
    double m_navAngularVel;
    bool m_navVelReceived;

    // Dynamic Parameter Update callback
    OnSetParametersCallbackHandle::SharedPtr m_callbackHandle;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);

    // Logging throttle helper
    int m_logCounter;
};

} // namespace line_follower

#endif // PID_CONTROLLER_H
