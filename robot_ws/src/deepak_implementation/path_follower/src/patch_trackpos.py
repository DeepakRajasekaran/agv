import re

with open('/home/lucifer/anscer_workspace/agv/robot_ws/src/deepak_implementation/path_follower/src/PidController.cpp', 'r') as f:
    content = f.read()

new_trackpos = """void PidController::trackPosCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_firstMessageReceived = true;
    (void)msg;  // msg data is consumed via member variables set by other callbacks

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - m_lastSensorUpdateTime).count();
    m_lastSensorUpdateTime = now;
    
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.02; // Nominal 50Hz
    }

    uint8_t preState = m_behaviorOrchestrator->getCurrentState();

    SensorInputs inputs;
    inputs.dt = dt;
    inputs.left_track_pos = m_leftTrackPos;
    inputs.right_track_pos = m_rightTrackPos;
    inputs.track_detect = m_trackDetect;
    inputs.tape_cross = m_tapeCross;
    inputs.left_marker = m_leftMarker;
    inputs.right_marker = m_rightMarker;
    inputs.protective_breach = m_protectiveBreach;
    inputs.warning_breach = m_warningBreach;
    inputs.nav_cmd_linear_x = m_cmdLinearX;
    inputs.has_fault = p_faultMonitor->hasFault();

    BehaviorOutputs outputs = m_behaviorOrchestrator->update(inputs);

    if (outputs.trigger_quickstop_edge) {
        if (m_cliQuickstop->wait_for_service(std::chrono::milliseconds(10))) {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = outputs.quickstop_state;
            m_cliQuickstop->async_send_request(request,
                [this, target_state = outputs.quickstop_state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        RCLCPP_INFO(this->get_logger(), "Quickstop service call (data=%d) success: %s", target_state, response->message.c_str());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Quickstop service call failed: %s", e.what());
                    }
                });
        } else {
            RCLCPP_WARN(this->get_logger(), "Quickstop service not available!");
        }
    }

    if (outputs.current_state != preState) {
        publishControllerState();
    }

    double divergence = std::abs(m_leftTrackPos - m_rightTrackPos);
    std_msgs::msg::Float32 divergenceMsg;
    divergenceMsg.data = static_cast<float>(divergence);
    m_pubDivergence->publish(divergenceMsg);

    std_msgs::msg::UInt16 cmdMsg;
    cmdMsg.data = outputs.lidar_cmd;
    m_pubLidarCmd->publish(cmdMsg);

    if (outputs.current_state == ControllerState::STOP || 
        outputs.current_state == ControllerState::ERROR || 
        outputs.current_state == ControllerState::INITIALIZE || 
        outputs.current_state == ControllerState::READ_TAG) 
    {
        m_integralError = 0.0;
        m_prevError = 0.0;
        publishVelocity(0.0, 0.0);
        return;
    }

    double pidAngularVel = 0.0;
    static bool was_lost = false;
    
    if (m_trackDetect) {
        if (was_lost) {
            m_prevError = std::atan2(outputs.target_error, m_sensorOffsetX);
            was_lost = false;
        }
        pidAngularVel = computeSteering(outputs.target_error, dt);
        m_lastPidAngularVel = pidAngularVel;
    } else {
        pidAngularVel = m_lastPidAngularVel;
        m_integralError = 0.0;
        was_lost = true;
    }

    double v_l = m_cmdLinearX - (pidAngularVel * m_wheelBase / 2.0);
    double v_r = m_cmdLinearX + (pidAngularVel * m_wheelBase / 2.0);
    double rpm_l = (v_l / m_wheelRadius) * 60.0 / (2.0 * M_PI);
    double rpm_r = (v_r / m_wheelRadius) * 60.0 / (2.0 * M_PI);

    p_faultMonitor->update(outputs.target_error, m_trackDetect, rpm_l, rpm_r, m_maxRpm);
    if (p_faultMonitor->hasFault()) {
        if (outputs.current_state != ControllerState::IDLE) {
            handleFault(p_faultMonitor->getFaultType());
            return;
        }
    }

    if (outputs.current_state == ControllerState::IDLE) {
        if (m_trackDetect && !p_faultMonitor->hasFault()) {
            publishVelocity(outputs.linear_velocity, pidAngularVel);
        } else {
            publishVelocity(outputs.linear_velocity, m_cmdAngularZ);
        }
    } else {
        publishVelocity(outputs.linear_velocity, pidAngularVel);
    }
}"""

pattern = re.compile(r'void PidController::trackPosCallback.*?^}', re.MULTILINE | re.DOTALL)
content = pattern.sub(new_trackpos, content)

with open('/home/lucifer/anscer_workspace/agv/robot_ws/src/deepak_implementation/path_follower/src/PidController.cpp', 'w') as f:
    f.write(content)
