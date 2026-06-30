import re

with open('/home/lucifer/anscer_workspace/agv/robot_ws/src/deepak_implementation/path_follower/include/behavior_nodes.h', 'r') as f:
    content = f.read()

new_tick = """    BT::NodeStatus tick() override {
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
        State prev_state = m_state;
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

        if (m_state != prev_state) {
            std::cout << "[JunctionManager] State changed: " << (int)prev_state << " -> " << (int)m_state << " (left: " << left << ", right: " << right << ")" << std::endl;
        }

        m_prevMarkersActive = markers_active;

        if (m_state != State::NONE) {
            setOutput("safe_velocity", clamp_vel);
            setOutput("in_junction", true);
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("in_junction", false);
        return BT::NodeStatus::FAILURE;
    }"""

pattern = re.compile(r'    BT::NodeStatus tick\(\) override \{.*?return BT::NodeStatus::FAILURE;\n    \}', re.MULTILINE | re.DOTALL)
content = pattern.sub(new_tick, content, count=1)

with open('/home/lucifer/anscer_workspace/agv/robot_ws/src/deepak_implementation/path_follower/include/behavior_nodes.h', 'w') as f:
    f.write(content)
