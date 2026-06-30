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
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>


#include "FaultMonitor.h"
#include "behavior_nodes.h"
#include <behaviortree_cpp/bt_factory.h>

namespace path_follower {

class PidController : public rclcpp::Node {
public:
    explicit PidController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PidController();
    void stopRobot();

    // Disable copy/move constructors for safety
    PidController(const PidController&) = delete;
    PidController& operator=(const PidController&) = delete;

private:
    // ROS 2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subTrackPos;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subTrackDetect;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subLeftTrackPos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subRightTrackPos;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subTapeCross;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subCmdVel;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subLeftMarker;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subRightMarker;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subProtectiveBreach;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subWarningBreach;

    // ROS 2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pubCmdVel;
    rclcpp::Publisher<custom_interfaces::msg::ControllerState>::SharedPtr m_pubControllerState;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubDivergence;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr m_pubLidarCmd;

    // Subscriber Callbacks
    void trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void trackDetectCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void leftTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void rightTrackPosCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void tapeCrossCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void leftMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rightMarkerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void protectiveBreachCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void warningBreachCallback(const std_msgs::msg::Bool::SharedPtr msg);

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

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_cliQuickstop;

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
    bool isTrackDetectStable() const;
    std::chrono::milliseconds getTrackDetectStableElapsed() const;

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
    int m_gracePeriodMs;
    int m_maxFrozenSteps;
    int m_trackDetectStableMs;

    // Velocity & Junction Clamps
    double m_clampStraight;
    double m_clampJunction;
    double m_clampMarkerJunction;
    double m_clampTurnJunction;
    double m_junctionDivergenceThreshold;

    // Behavior Tree Params
    double m_btErrorScalingMaxDist;
    double m_btMinScale;
    double m_btErrorThreshold;
    double m_btFallbackScale;

    // Safety and switching parameters
    double m_accelLimit;
    std::vector<double> m_fieldSwitchThresholds;
    std::vector<int64_t> m_fieldSwitchCommands;

    // Control Loop State
    double m_integralError;
    double m_prevError;
    double m_cmdLinearX;
    double m_cmdAngularZ;
    std::chrono::steady_clock::time_point m_lastSensorUpdateTime;
    std::chrono::steady_clock::time_point m_trackDetectTrueStartTime;

    bool m_trackDetect;
    bool m_trackDetectStableTimerActive;
    bool m_tapeCross;
    bool m_leftMarker;
    bool m_rightMarker;
    bool m_protectiveBreach;
    bool m_warningBreach;
    bool m_lastQuickstopRequest;
    uint16_t m_lastLidarCmdPublished;
    double m_leftTrackPos;
    double m_rightTrackPos;
    double m_lastPidAngularVel;
    double m_currentPublishedLinearVel;

    // Response time estimator
    double m_lastErrorForZc;
    std::chrono::steady_clock::time_point m_lastZeroCrossingTime;
    bool m_hasCrossedZero;

    // State machine logic
    uint8_t m_currentState;
    std::string stateToString(uint8_t state) const;
    void transitionTo(uint8_t newState, const std::string& trigger);
    std::unique_ptr<FaultMonitor> p_faultMonitor;

    // Behavior Tree
    BT::BehaviorTreeFactory m_btFactory;
    BT::Tree m_btTree;

    bool m_firstMessageReceived;

    // Dynamic Parameter Update callback
    OnSetParametersCallbackHandle::SharedPtr m_callbackHandle;
    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);

    // Logging throttle helper
    int m_logCounter;
};

} // namespace path_follower

#endif // PID_CONTROLLER_H
