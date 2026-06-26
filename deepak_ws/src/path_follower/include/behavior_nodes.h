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
            BT::OutputPort<double>("safe_velocity")
        };
    }

    BT::NodeStatus tick() override {
        double nominal_vel = 0.0;
        double error = 0.0;

        if (!getInput("nominal_velocity", nominal_vel) || !getInput("current_error", error)) {
            return BT::NodeStatus::FAILURE;
        }

        double scale = 1.0 - (std::abs(error) / 0.15);
        scale = std::clamp(scale, 0.2, 1.0);
        
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

} // namespace path_follower

#endif // BEHAVIOR_NODES_H
