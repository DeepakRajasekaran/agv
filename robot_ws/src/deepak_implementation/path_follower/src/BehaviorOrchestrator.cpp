#include "BehaviorOrchestrator.h"
#include "behavior_nodes.h"
#include <custom_interfaces/msg/controller_state.hpp>
#include <algorithm>

namespace path_follower {

using custom_interfaces::msg::ControllerState;

BehaviorOrchestrator::BehaviorOrchestrator(const BehaviorConfig& config, rclcpp::Logger logger)
    : m_config(config),
      m_logger(logger),
      m_currentState(ControllerState::IDLE),
      m_selectedTrackId(0),
      m_currentPublishedLinearVel(0.0),
      m_lastQuickstopRequest(false),
      m_lastLidarCmdPublished(1),
      m_logCounter(0),
      m_hasFault(false),
      m_faultType("NONE"),
      m_estopActive(false),
      m_lastTrackPos(0.0),
      m_frozenStepsCount(0),
      m_lineLostCount(0)
{
    // Behavior Tree Initialization
    m_btFactory.registerNodeType<IsErrorHigh>("IsErrorHigh");
    m_btFactory.registerNodeType<IsErrorStable>("IsErrorStable");
    m_btFactory.registerNodeType<ReduceVelocity>("ReduceVelocity");
    m_btFactory.registerNodeType<SetSafeVelocity>("SetSafeVelocity");
    m_btFactory.registerNodeType<SetNominalVelocity>("SetNominalVelocity");
    m_btFactory.registerNodeType<JunctionManager>("JunctionManager");
    m_btFactory.registerNodeType<TurnManager>("TurnManager");
    m_btFactory.registerNodeType<SafetyManager>("SafetyManager");

    const std::string bt_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree>
    <Fallback>
      <SafetyManager protective_breach="{protective_breach}" warning_breach="{warning_breach}" nominal_velocity="{nominal_vel}" safe_velocity="{safe_vel}" trigger_quickstop="{trigger_quickstop}" />
      <JunctionManager left_marker="{left_marker}" right_marker="{right_marker}" nominal_velocity="{nominal_vel}" clamp_velocity="{clamp_junction_vel}" safe_velocity="{safe_vel}" in_junction="{in_junction}" />
      <TurnManager left_marker="{left_marker}" right_marker="{right_marker}" nominal_velocity="{nominal_vel}" clamp_velocity="{clamp_turn_vel}" safe_velocity="{safe_vel}" selected_track_id="{selected_track_id}" />
      <Sequence>
        <IsErrorHigh error="{current_error}" threshold="{error_threshold}" last_high_time="{last_high_time}" />
        <ReduceVelocity nominal_velocity="{nominal_vel}" current_error="{current_error}" error_scaling_max_dist="{error_scaling_max_dist}" min_scale="{min_scale}" safe_velocity="{safe_vel}" />
      </Sequence>
      <Sequence>
        <IsErrorStable last_high_time="{last_high_time}" duration="3.0" />
        <SetNominalVelocity nominal_velocity="{nominal_vel}" safe_velocity="{safe_vel}" />
      </Sequence>
      <SetSafeVelocity nominal_velocity="{nominal_vel}" safe_velocity="{safe_vel}" scale="{fallback_scale}" />
    </Fallback>
  </BehaviorTree>
</root>
)";
    m_btTree = m_btFactory.createTreeFromText(bt_xml);
    
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    double start_time = std::chrono::duration<double>(now).count();
    m_btTree.rootBlackboard()->set("last_high_time", start_time);
}

void BehaviorOrchestrator::setConfig(const BehaviorConfig& config) {
    m_config = config;
}

std::string BehaviorOrchestrator::stateToString(uint8_t state) const {
    switch(state) {
        case ControllerState::IDLE: return "IDLE";
        case ControllerState::INITIALIZE: return "INITIALIZE";
        case ControllerState::FOLLOW_LINE: return "FOLLOW_LINE";
        case ControllerState::JUNCTION_DETECTED: return "JUNCTION_DETECTED";
        case ControllerState::READ_TAG: return "READ_TAG";
        case ControllerState::RESUME_TRACKING: return "RESUME_TRACKING";
        case ControllerState::STOP: return "STOP";
        case ControllerState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void BehaviorOrchestrator::forceState(uint8_t state, const std::string& reason) {
    if (m_currentState != state) {
        RCLCPP_INFO(m_logger, "[STATE] %s -> %s (Trigger: %s)", 
            stateToString(m_currentState).c_str(), stateToString(state).c_str(), reason.c_str());
        m_currentState = state;
    }
}

BehaviorOutputs BehaviorOrchestrator::update(const SensorInputs& inputs) {
    BehaviorOutputs outputs;
    outputs.current_state = m_currentState;
    
    // 1. Check Protective Breach edges for Quickstop Service
    if (inputs.protective_breach != m_lastQuickstopRequest) {
        m_lastQuickstopRequest = inputs.protective_breach;
        outputs.trigger_quickstop_edge = true;
        outputs.quickstop_state = inputs.protective_breach;
    }

    m_hasFault = false;
    m_faultType = "NONE";
    m_frozenStepsCount = 0;
    m_lineLostCount = 0;
    // Don't reset estop state here as it's externally injected

    // Update Internal Safety/Fault State
    if (m_estopActive) {
        m_hasFault = true;
        m_faultType = "E_STOP";
    } else {
        // Check Sensor Dropout
        double currentTrackPos = (inputs.left_track_pos + inputs.right_track_pos) / 2.0;
        if (inputs.track_detect) {
            if (std::abs(currentTrackPos - m_lastTrackPos) < 1e-7) {
                m_frozenStepsCount++;
            } else {
                m_frozenStepsCount = 0;
            }
            m_lastTrackPos = currentTrackPos;
        } else {
            m_frozenStepsCount = 0;
        }
        
        if (m_frozenStepsCount > m_config.maxFrozenSteps) {
            m_hasFault = true;
            m_faultType = "SENSOR_DROPOUT";
        }

        // Check Line Lost
        if (!inputs.track_detect) {
            m_lineLostCount++;
        } else {
            m_lineLostCount = 0;
        }

        if (m_lineLostCount > m_config.lineLostGraceSteps) {
            m_hasFault = true;
            m_faultType = "LINE_LOST";
        }

        // Check Motor Saturation
        if (std::abs(inputs.left_rpm) > inputs.max_rpm || std::abs(inputs.right_rpm) > inputs.max_rpm) {
            m_hasFault = true;
            m_faultType = "MOTOR_SATURATION";
        }
    }

    // Assign Fault State to Outputs
    outputs.has_fault = m_hasFault;
    outputs.fault_type = m_faultType;

    // 1. Fault Overrides
    if (m_hasFault && m_currentState != ControllerState::ERROR && m_currentState != ControllerState::IDLE) {
        forceState(ControllerState::ERROR, "FAULT_DETECTED");
    }

    // 2. State transitions based on safety
    if (m_currentState == ControllerState::STOP && !inputs.protective_breach && !m_hasFault) {
        forceState(ControllerState::FOLLOW_LINE, "PROTECTIVE_FIELD_CLEARED");
    } else if (inputs.protective_breach && m_currentState != ControllerState::STOP && m_currentState != ControllerState::ERROR) {
        forceState(ControllerState::STOP, "PROTECTIVE_FIELD_BREACH");
    }
    
    outputs.current_state = m_currentState;

    if (m_currentState == ControllerState::STOP || m_currentState == ControllerState::ERROR || m_currentState == ControllerState::INITIALIZE) {
        m_currentPublishedLinearVel = 0.0;
        outputs.linear_velocity = 0.0;
        
        double divergence = std::abs(inputs.left_track_pos - inputs.right_track_pos);
        outputs.divergence = divergence;
        if (m_selectedTrackId == 1) {
            outputs.target_error = inputs.left_track_pos;
        } else if (m_selectedTrackId == 2) {
            outputs.target_error = inputs.right_track_pos;
        } else {
            outputs.target_error = (inputs.left_track_pos + inputs.right_track_pos) / 2.0;
        }

        if (m_logCounter++ % 100 == 0) {
            RCLCPP_INFO(m_logger,
                "State: %s | Enforcing 0.0 vel", stateToString(m_currentState).c_str());
        }
        return outputs;
    }

    double computed_error = 0.0;
    double divergence = std::abs(inputs.left_track_pos - inputs.right_track_pos);
    outputs.divergence = divergence;
    
    if (m_currentState == ControllerState::JUNCTION_DETECTED || m_currentState == ControllerState::FOLLOW_LINE) {
        if (m_selectedTrackId == 1) {
            computed_error = inputs.left_track_pos;
        } else if (m_selectedTrackId == 2) {
            computed_error = inputs.right_track_pos;
        } else {
            computed_error = (inputs.left_track_pos + inputs.right_track_pos) / 2.0;
        }
    } else {
        computed_error = (inputs.left_track_pos + inputs.right_track_pos) / 2.0;
    }
    outputs.target_error = computed_error;

    // Auto-reset selected track when junction clears based on divergence (for external navigation selections)
    bool bt_turn_active = false;
    (void)m_btTree.rootBlackboard()->get("turn_active", bt_turn_active);
    
    if (!bt_turn_active && divergence < m_config.junctionDivergenceThreshold && m_selectedTrackId != 0 && !inputs.tape_cross) {
        RCLCPP_INFO(m_logger, "Junction divergence cleared. Resetting track_id to 0 (AVERAGE).");
        m_selectedTrackId = 0;
        if (m_currentState == ControllerState::JUNCTION_DETECTED) {
            forceState(ControllerState::FOLLOW_LINE, "JUNCTION_CLEARED");
            outputs.current_state = m_currentState;
        }
    }

    // 4. Update Behavior Tree Variables
    m_btTree.rootBlackboard()->set("current_error", std::abs(computed_error));
    m_btTree.rootBlackboard()->set("nominal_vel", inputs.nav_cmd_linear_x);
    m_btTree.rootBlackboard()->set("error_scaling_max_dist", m_config.btErrorScalingMaxDist);
    m_btTree.rootBlackboard()->set("min_scale", m_config.btMinScale);
    m_btTree.rootBlackboard()->set("error_threshold", m_config.btErrorThreshold);
    m_btTree.rootBlackboard()->set("fallback_scale", m_config.btFallbackScale);
    m_btTree.rootBlackboard()->set("left_marker", inputs.left_marker);
    m_btTree.rootBlackboard()->set("right_marker", inputs.right_marker);
    m_btTree.rootBlackboard()->set("protective_breach", inputs.protective_breach);
    m_btTree.rootBlackboard()->set("warning_breach", inputs.warning_breach);
    m_btTree.rootBlackboard()->set("clamp_junction_vel", m_config.clampMarkerJunction);
    m_btTree.rootBlackboard()->set("clamp_turn_vel", m_config.clampTurnJunction);
    
    BT::NodeStatus bt_status = m_btTree.tickExactlyOnce();
    (void)bt_status;
    
    double safe_velocity = inputs.nav_cmd_linear_x;
    if (!m_btTree.rootBlackboard()->get("safe_vel", safe_velocity)) {
        safe_velocity = inputs.nav_cmd_linear_x;
    }

    bool in_junction = false;
    (void)m_btTree.rootBlackboard()->get("in_junction", in_junction);

    static bool prev_bt_turn_active = false;
    
    if (bt_turn_active) {
        int bt_selected_track_id = 0;
        if (m_btTree.rootBlackboard()->get("selected_track_id", bt_selected_track_id)) {
            if (bt_selected_track_id != m_selectedTrackId) {
                m_selectedTrackId = bt_selected_track_id;
                RCLCPP_INFO(m_logger, "Track selection updated by BT to: %d", m_selectedTrackId);
            }
        }
    } else if (prev_bt_turn_active) {
        m_selectedTrackId = 0;
        RCLCPP_INFO(m_logger, "Turn exited. Cleared track selection to 0.");
    }
    prev_bt_turn_active = bt_turn_active;

    // 5. Native Junction Logic (drift-based fallback) 
    // Disabled side-switching based on divergence (per ponytail-audit)
    if (m_currentState == ControllerState::FOLLOW_LINE) {
        if (inputs.tape_cross) { // Kept tape cross to trigger junction if needed
            forceState(ControllerState::JUNCTION_DETECTED, "TAPE_CROSS");
            // Do NOT assign m_selectedTrackId randomly anymore. Higher level systems must command it.
            outputs.current_state = m_currentState;
        }
    }

    // BT Junction transition overriding
    if (in_junction) {
        if (m_currentState != ControllerState::JUNCTION_DETECTED) {
            forceState(ControllerState::JUNCTION_DETECTED, "BT_JUNCTION_START");
            outputs.current_state = m_currentState;
        }
    } else if (m_currentState == ControllerState::JUNCTION_DETECTED && !in_junction && m_selectedTrackId == 0) {
        forceState(ControllerState::FOLLOW_LINE, "BT_JUNCTION_END");
        outputs.current_state = m_currentState;
    }

    // 6. Hard State-Based Clamping
    if (m_currentState == ControllerState::FOLLOW_LINE) {
        safe_velocity = std::clamp(safe_velocity, -m_config.clampStraight, m_config.clampStraight);
    } else if (m_currentState == ControllerState::JUNCTION_DETECTED) {
        // Only apply the generic junction clamp if the BT didn't already enforce a stricter marker clamp.
        // But since BT sets safe_velocity, we just make sure it doesn't exceed clampJunction.
        safe_velocity = std::clamp(safe_velocity, -m_config.clampJunction, m_config.clampJunction);
    }
    
    if (m_currentState == ControllerState::RESUME_TRACKING) {
        safe_velocity = std::clamp(safe_velocity, -m_config.clampStraight, m_config.clampStraight);
        forceState(ControllerState::FOLLOW_LINE, "RESUMED");
        outputs.current_state = m_currentState;
    } else if (m_currentState == ControllerState::READ_TAG) {
        safe_velocity = 0.0;
        forceState(ControllerState::FOLLOW_LINE, "TAG_READ_STUB");
        outputs.current_state = m_currentState;
    }

    // 7. Acceleration Limit
    double max_step = m_config.accelLimit * inputs.dt;
    if (safe_velocity > m_currentPublishedLinearVel) {
        m_currentPublishedLinearVel = std::min(m_currentPublishedLinearVel + max_step, safe_velocity);
    } else {
        if (safe_velocity == 0.0) {
            m_currentPublishedLinearVel = 0.0;
        } else {
            m_currentPublishedLinearVel = std::max(m_currentPublishedLinearVel - max_step, safe_velocity);
        }
    }
    outputs.linear_velocity = m_currentPublishedLinearVel;

    // 8. Field Switching
    uint16_t desiredLidarCmd = 1;
    if (m_config.fieldSwitchThresholds.size() + 1 == m_config.fieldSwitchCommands.size()) {
        double abs_speed = std::abs(outputs.linear_velocity);
        size_t idx = 0;
        while (idx < m_config.fieldSwitchThresholds.size() && abs_speed > m_config.fieldSwitchThresholds[idx]) {
            idx++;
        }
        desiredLidarCmd = static_cast<uint16_t>(m_config.fieldSwitchCommands[idx]);
    }
    if (desiredLidarCmd != m_lastLidarCmdPublished) {
        m_lastLidarCmdPublished = desiredLidarCmd;
        RCLCPP_INFO(m_logger, "Switched lidar safety field to command %d (Speed: %.3f)", desiredLidarCmd, outputs.linear_velocity);
    }
    outputs.lidar_cmd = m_lastLidarCmdPublished;

    m_logCounter++;

    return outputs;
}

void BehaviorOrchestrator::injectEStop(bool active) {
    m_estopActive = active;
    if (active) {
        m_hasFault = true;
        m_faultType = "E_STOP";
    }
}

void BehaviorOrchestrator::checkTimeout(const std::chrono::steady_clock::time_point& lastUpdateTime) {
    if (m_estopActive) {
        m_hasFault = true;
        m_faultType = "E_STOP";
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdateTime).count();

    if (duration > 500) {  // 500 ms timeout
        m_hasFault = true;
        m_faultType = "TIMEOUT";
    }
}

} // namespace path_follower
