/*
 * Name:        PidController.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-24
 * Version:     2.0
 * Description: Declares the PidController class for ROS 2 magnetic line following
 *              with lookahead steering and mapless junction handling.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <custom_interfaces/msg/controller_state.hpp>
#include <custom_interfaces/srv/select_track.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "NavigationStateMachine.h"
#include "FaultMonitor.h"

namespace path_follower {

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subTapeCross;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subCmdVel;

    // ROS 2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pubCmdVel;
    rclcpp::Publisher<custom_interfaces::msg::ControllerState>::SharedPtr m_pubControllerState;

    // Subscriber Callbacks
    void trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void rightTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void tapeCrossCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS 2 Services (Controller)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvAutotune;
    void autotuneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvSaveTuning;
    void saveTuningCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvStart;
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_srvStop;
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<custom_interfaces::srv::SelectTrack>::SharedPtr m_srvSelectTrack;
    void selectTrackCallback(const std::shared_ptr<custom_interfaces::srv::SelectTrack::Request> request,
                             std::shared_ptr<custom_interfaces::srv::SelectTrack::Response> response);

    // Track Selection State
    int m_selectedTrackId = 0; // 0=AVG, 1=LEFT, 2=RIGHT

    // Timer for safety monitor timeout check (50Hz)
    rclcpp::TimerBase::SharedPtr m_safetyTimer;
    void safetyCheckCallback();

    // PID Internal Logic
    double computeSteering(double error, double dt);

    // Helper functions
    void publishVelocity(double linearVel, double angularVel);
    void handleFault(const std::string& faultType);
    void publishControllerState();

    // Tunable Parameters (PID)
    double m_kp;
    double m_ki;
    double m_kd;
    double m_windupLimit;
    double m_maxOutput;

    // Robot Parameters
    double m_maxRpm;
    double m_wheelBase;
    double m_wheelRadius;
    double m_sensorOffsetX;

    // Safety Parameters
    double m_lostThreshold;
    int m_maxFrozenSteps;
    double m_turnDuration;

    // Velocity & Junction Clamps
    double m_clampStraight;
    double m_clampJunction;
    double m_clampTurn;
    double m_junctionDivergenceThreshold;

    // Control Loop State
    double m_integralError;
    double m_prevError;
    double m_cmdLinearX;
    double m_cmdAngularZ;
    std::chrono::steady_clock::time_point m_lastSensorUpdateTime;

    bool m_trackDetect;
    bool m_leftMarker;
    bool m_rightMarker;
    bool m_tapeCross;
    double m_leftTrackPos;
    double m_rightTrackPos;

    // Subclass pointer instances
    std::unique_ptr<NavigationStateMachine> p_stateMachine;
    std::unique_ptr<FaultMonitor> p_faultMonitor;

    bool m_firstMessageReceived;

    // Dynamic Parameter Update callback
    OnSetParametersCallbackHandle::SharedPtr m_callbackHandle;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);

    // Logging throttle helper
    int m_logCounter;
};

} // namespace path_follower

#endif // PID_CONTROLLER_H
