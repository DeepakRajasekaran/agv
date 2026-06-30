#ifndef BEHAVIOR_ORCHESTRATOR_H
#define BEHAVIOR_ORCHESTRATOR_H

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <vector>
#include <chrono>

namespace path_follower {

struct BehaviorConfig {
    double clampStraight = 1.0;
    double clampJunction = 0.5;
    double clampMarkerJunction = 0.3;
    double clampTurnJunction = 0.4;
    double accelLimit = 0.5;
    double junctionDivergenceThreshold = 0.035;
    std::vector<double> fieldSwitchThresholds = {0.3, 0.7};
    std::vector<int64_t> fieldSwitchCommands = {1, 2, 3};
    double btErrorScalingMaxDist = 0.15;
    double btMinScale = 0.2;
    double btErrorThreshold = 0.08;
    double btFallbackScale = 0.4;
    int lineLostGraceSteps = 10;
    int maxFrozenSteps = 5;
    double exitBufferDurationS = 2.0;
};

struct SensorInputs {
    double dt = 0.0;
    double left_track_pos = 0.0;
    double right_track_pos = 0.0;
    bool track_detect = false;
    bool tape_cross = false;
    bool left_marker = false;
    bool right_marker = false;
    bool protective_breach = false;
    bool warning_breach = false;
    double nav_cmd_linear_x = 0.0;
    double left_rpm = 0.0;
    double right_rpm = 0.0;
    double max_rpm = 0.0;
};

struct BehaviorOutputs {
    double linear_velocity = 0.0;
    double target_error = 0.0;
    uint8_t current_state = 0;
    uint16_t lidar_cmd = 1;
    bool trigger_quickstop_edge = false;
    bool quickstop_state = false;
    double divergence = 0.0;
    bool has_fault = false;
    std::string fault_type = "NONE";
};

class BehaviorOrchestrator {
public:
    BehaviorOrchestrator(const BehaviorConfig& config, rclcpp::Logger logger);
    ~BehaviorOrchestrator() = default;

    void setConfig(const BehaviorConfig& config);
    BehaviorOutputs update(const SensorInputs& inputs);

    uint8_t getCurrentState() const { return m_currentState; }
    void forceState(uint8_t state, const std::string& reason);
    int getSelectedTrackId() const { return m_selectedTrackId; }
    void setSelectedTrackId(int id) { m_selectedTrackId = id; }
    
    // Safety injections
    void injectEStop(bool active);
    void checkTimeout(const std::chrono::steady_clock::time_point& lastUpdateTime);

private:
    std::string stateToString(uint8_t state) const;

    BehaviorConfig m_config;
    rclcpp::Logger m_logger;
    BT::BehaviorTreeFactory m_btFactory;
    BT::Tree m_btTree;

    uint8_t m_currentState;
    int m_selectedTrackId;
    double m_currentPublishedLinearVel;
    bool m_lastQuickstopRequest;
    uint16_t m_lastLidarCmdPublished;
    int m_logCounter;

    // Folded Fault Monitor State
    bool m_hasFault;
    std::string m_faultType;
    bool m_estopActive;
    double m_lastTrackPos;
    int m_frozenStepsCount;
    int m_lineLostCount;

    // Behavior Exit Buffer State
    bool m_wasInBehavior;
    double m_lastBehaviorVelocity;
    double m_behaviorExitTime;
};

} // namespace path_follower

#endif // BEHAVIOR_ORCHESTRATOR_H
