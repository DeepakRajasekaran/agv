#ifndef BEHAVIOR_NODES_H
#define BEHAVIOR_NODES_H

#include "behaviortree_cpp/behavior_tree.h"
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
            BT::InputPort<double>("threshold")
        };
    }

    BT::NodeStatus tick() override {
        double error = 0.0;
        double threshold = 0.0;

        if (!getInput("error", error) || !getInput("threshold", threshold)) {
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(error) > threshold) {
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

        // Extremely simple dynamic scale:
        // As error increases, speed drops. We cap it at a minimum of 30% of nominal.
        // Assuming max physical error is roughly 0.15 before losing tape.
        double scale = 1.0 - (std::abs(error) / 0.15);
        scale = std::clamp(scale, 0.3, 1.0);
        
        double safe_vel = nominal_vel * scale;
        setOutput("safe_velocity", safe_vel);

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
