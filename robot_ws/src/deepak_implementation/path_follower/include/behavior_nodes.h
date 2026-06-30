#ifndef BEHAVIOR_NODES_H
#define BEHAVIOR_NODES_H

#include "behaviortree_cpp/behavior_tree.h"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace path_follower {

class IsErrorHigh : public BT::ConditionNode {
public:
    IsErrorHigh(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("error"),
            BT::InputPort<double>("threshold"),
            BT::OutputPort<double>("last_high_time")
        };
    }

    BT::NodeStatus tick() override {
        double error = 0.0;
        double threshold = 0.0;

        if (!getInput("error", error) || !getInput("threshold", threshold)) {
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(error) > threshold) {
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            double seconds = std::chrono::duration<double>(now).count();
            setOutput("last_high_time", seconds);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class IsErrorStable : public BT::ConditionNode {
public:
    IsErrorStable(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("last_high_time"),
            BT::InputPort<double>("duration")
        };
    }

    BT::NodeStatus tick() override {
        double last_high_time = 0.0;
        double duration = 0.0;
        
        if (!getInput("last_high_time", last_high_time) || !getInput("duration", duration)) {
            return BT::NodeStatus::FAILURE;
        }

        auto now = std::chrono::steady_clock::now().time_since_epoch();
        double current_time = std::chrono::duration<double>(now).count();

        if (current_time - last_high_time >= duration) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ReduceVelocity : public BT::SyncActionNode {
public:
    ReduceVelocity(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("nominal_velocity"),
            BT::InputPort<double>("current_error"),
            BT::InputPort<double>("error_scaling_max_dist"),
            BT::InputPort<double>("min_scale"),
            BT::OutputPort<double>("safe_velocity")
        };
    }

    BT::NodeStatus tick() override {
        double nominal_vel = 0.0;
        double error = 0.0;
        double max_dist = 0.15;
        double min_scale = 0.2;

        if (!getInput("nominal_velocity", nominal_vel) || !getInput("current_error", error)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("error_scaling_max_dist", max_dist);
        getInput("min_scale", min_scale);

        if (max_dist <= 0.0) max_dist = 0.15; // prevent div-by-zero

        double scale = 1.0 - (std::abs(error) / max_dist);
        scale = std::clamp(scale, min_scale, 1.0);
        
        double safe_vel = nominal_vel * scale;
        setOutput("safe_velocity", safe_vel);

        return BT::NodeStatus::SUCCESS;
    }
};

class SetSafeVelocity : public BT::SyncActionNode {
public:
    SetSafeVelocity(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("nominal_velocity"),
            BT::InputPort<double>("scale"),
            BT::OutputPort<double>("safe_velocity")
        };
    }

    BT::NodeStatus tick() override {
        double nominal_vel = 0.0;
        double scale = 0.4;
        if (!getInput("nominal_velocity", nominal_vel)) return BT::NodeStatus::FAILURE;
        getInput("scale", scale);
        
        setOutput("safe_velocity", nominal_vel * scale);
        return BT::NodeStatus::SUCCESS;
    }
};

class SetNominalVelocity : public BT::SyncActionNode {
public:
    SetNominalVelocity(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("nominal_velocity"),
            BT::OutputPort<double>("safe_velocity")
        };
    }

    BT::NodeStatus tick() override {
        double nominal_vel = 0.0;
        if (!getInput("nominal_velocity", nominal_vel)) {
            return BT::NodeStatus::FAILURE;
        }
        
        setOutput("safe_velocity", nominal_vel);
        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * @brief  Stateful Action Node for managing double-marker junctions.
 */
class JunctionManager : public BT::SyncActionNode {
public:
    JunctionManager(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config), m_state(State::NONE), m_prevMarkersActive(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("left_marker"),
            BT::InputPort<bool>("right_marker"),
            BT::InputPort<double>("nominal_velocity"),
            BT::InputPort<double>("clamp_velocity"),
            BT::OutputPort<double>("safe_velocity"),
            BT::OutputPort<bool>("in_junction")
        };
    }

    BT::NodeStatus tick() override {
        bool left = false;
        bool right = false;
        double nominal_vel = 0.0;
        double clamp_vel = 0.0;

        bool ok = getInput("left_marker", left) && 
                  getInput("right_marker", right) &&
                  getInput("nominal_velocity", nominal_vel) && 
                  getInput("clamp_velocity", clamp_vel);
        assert(ok); // precondition

        bool markers_active = (left && right);

        // State Machine logic
        switch (m_state) {
            case State::NONE:
                if (markers_active) {
                    m_state = State::ENTRY;
                }
                break;

            case State::ENTRY:
                if (!markers_active) {
                    m_state = State::TRANSITION;
                }
                break;

            case State::TRANSITION:
                if (markers_active && !m_prevMarkersActive) {
                    m_state = State::EXIT;
                }
                break;

            case State::EXIT:
                if (!markers_active) {
                    m_state = State::NONE;
                }
                break;
        }

        m_prevMarkersActive = markers_active;

        if (m_state != State::NONE) {
            setOutput("safe_velocity", clamp_vel);
            setOutput("in_junction", true);
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("in_junction", false);
        return BT::NodeStatus::FAILURE;
    }

private:
    enum class State {
        NONE,
        ENTRY,
        TRANSITION,
        EXIT
    };
    State m_state;
    bool m_prevMarkersActive;
};

/**
 * @brief  Stateful Action Node for managing single-marker turns.
 */
class TurnManager : public BT::SyncActionNode {
public:
    TurnManager(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config), m_state(State::NONE), m_prevLeftMarker(false), m_prevRightMarker(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("left_marker"),
            BT::InputPort<bool>("right_marker"),
            BT::InputPort<double>("nominal_velocity"),
            BT::InputPort<double>("clamp_velocity"),
            BT::OutputPort<double>("safe_velocity"),
            BT::OutputPort<int>("selected_track_id")
        };
    }

    BT::NodeStatus tick() override {
        bool left = false;
        bool right = false;
        double nominal_vel = 0.0;
        double clamp_vel = 0.0;

        bool ok = getInput("left_marker", left) && 
                  getInput("right_marker", right) &&
                  getInput("nominal_velocity", nominal_vel) && 
                  getInput("clamp_velocity", clamp_vel);
        assert(ok); // precondition

        // State Machine logic
        switch (m_state) {
            case State::NONE:
                if ((left || right) && !(left && right)) {
                    m_state = State::ENTRY;
                    m_selectedTrackId = left ? 1 : 2; // 1 = LEFT, 2 = RIGHT
                }
                break;

            case State::ENTRY:
                if (!left && !right) {
                    m_state = State::TRANSITION;
                }
                break;

            case State::TRANSITION:
                if ((left && !m_prevLeftMarker) || (right && !m_prevRightMarker)) {
                    m_state = State::EXIT;
                }
                break;

            case State::EXIT:
                if (!left && !right) {
                    m_state = State::NONE;
                    m_selectedTrackId = 0; // 0 = AVERAGE
                }
                break;
        }

        m_prevLeftMarker = left;
        m_prevRightMarker = right;

        if (m_state != State::NONE) {
            setOutput("safe_velocity", clamp_vel);
            setOutput("selected_track_id", m_selectedTrackId);
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("selected_track_id", 0);
        return BT::NodeStatus::FAILURE;
    }

private:
    enum class State {
        NONE,
        ENTRY,
        TRANSITION,
        EXIT
    };
    State m_state;
    int m_selectedTrackId = 0;
    bool m_prevLeftMarker;
    bool m_prevRightMarker;
};

/**
 * @brief  Sync Action Node for warning/protective safety breaches.
 */
class SafetyManager : public BT::SyncActionNode {
public:
    SafetyManager(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("protective_breach"),
            BT::InputPort<bool>("warning_breach"),
            BT::InputPort<double>("nominal_velocity"),
            BT::OutputPort<double>("safe_velocity"),
            BT::OutputPort<bool>("trigger_quickstop")
        };
    }

    BT::NodeStatus tick() override {
        bool protective = false;
        bool warning = false;
        double nominal_vel = 0.0;

        bool ok = getInput("protective_breach", protective) && 
                  getInput("warning_breach", warning) && 
                  getInput("nominal_velocity", nominal_vel);
        assert(ok); // precondition

        if (protective) {
            setOutput("safe_velocity", 0.0);
            setOutput("trigger_quickstop", true);
            return BT::NodeStatus::SUCCESS;
        }

        if (warning) {
            setOutput("safe_velocity", nominal_vel * 0.5);
            setOutput("trigger_quickstop", false);
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("trigger_quickstop", false);
        return BT::NodeStatus::FAILURE;
    }
};

} // namespace path_follower

#endif // BEHAVIOR_NODES_H
