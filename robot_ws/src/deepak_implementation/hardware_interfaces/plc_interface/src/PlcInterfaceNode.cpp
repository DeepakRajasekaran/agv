/*
 * Name:        PlcInterfaceNode.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-29
 * Version:     1.0
 * Description: Implementation of the Modbus TCP PLC interface node. Maps byte offsets to registers.
 */

#include "plc_interface/PlcInterfaceNode.h"
#include <chrono>
#include <cassert>
#include <stdexcept>

using namespace std::chrono_literals;

/**
 * @brief  Constructor for PlcInterfaceNode. Initializes parameters and ROS entities.
 */
PlcInterfaceNode::PlcInterfaceNode() 
    : Node("plc_interface"),
      p_ctx(nullptr),
      m_connected(false),
      m_lastHeartbeatVal(0),
      m_quickstopVal(0),
      m_lidarCmdVal(0)
{
    this->declare_parameter("plc_ip", "192.168.1.5");
    this->declare_parameter("plc_port", 502);

    p_pubLidarFb = this->create_publisher<std_msgs::msg::Bool>("~/lidar_fb", 10);
    p_pubEstopFb = this->create_publisher<std_msgs::msg::Bool>("~/estop_fb", 10);
    
    p_subLidarCmd = this->create_subscription<std_msgs::msg::UInt16>(
        "~/lidar_cmd", 10,
        [this](const std_msgs::msg::UInt16::SharedPtr msg) {
            assert(msg != nullptr); // Precondition
            m_lidarCmdVal = msg->data;
        });
        
    p_srvQuickstop = this->create_service<std_srvs::srv::SetBool>(
        "~/trigger_quickstop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            assert(request != nullptr); // Precondition
            assert(response != nullptr); // Precondition
            
            m_quickstopVal = request->data ? 7 : 0;
            response->success = true;
            response->message = request->data ? "Quickstop Triggered (7)" : "Quickstop Reset (0)";
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", response->message.c_str());
        });

    std::string ip = this->get_parameter("plc_ip").as_string();
    int port = static_cast<int>(this->get_parameter("plc_port").as_int());
    
    p_ctx = modbus_new_tcp(ip.c_str(), port);
    assert(p_ctx != nullptr); // Precondition: context allocation must succeed
    
    int ret = modbus_set_response_timeout(p_ctx, 0, 500000);
    if (ret != 0) {
        throw std::runtime_error("Failed to set Modbus response timeout");
    }

    m_lastHeartbeatTime = this->now();
    
    p_timer = this->create_wall_timer(20ms, std::bind(&PlcInterfaceNode::timerCallback, this));
}

/**
 * @brief  Destructor to safely close and free the Modbus context.
 */
PlcInterfaceNode::~PlcInterfaceNode() {
    if (p_ctx) {
        modbus_close(p_ctx);
        modbus_free(p_ctx);
    }
}

/**
 * @brief  Handles connection and auto-reconnection to the PLC.
 */
void PlcInterfaceNode::connectToPlc() {
    assert(p_ctx != nullptr);
    if (!m_connected) {
        int ret = modbus_connect(p_ctx);
        if (ret == -1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Failed to connect to PLC: %s", modbus_strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to PLC!");
        m_connected = true;
        m_lastHeartbeatTime = this->now();
    }
}

/**
 * @brief  Main execution loop for PLC communication triggered by timer.
 */
void PlcInterfaceNode::timerCallback() {
    assert(p_ctx != nullptr);
    connectToPlc();
    if (!m_connected) {
        return;
    }
    readFromPlc();
}

/**
 * @brief  Reads Modbus registers from the PLC and updates ROS publishers/states.
 */
void PlcInterfaceNode::readFromPlc() {
    assert(m_connected == true);
    
    uint16_t readRegs[18];
    int ret = modbus_read_registers(p_ctx, 0, 18, readRegs);
    if (ret == -1) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Modbus read failed: %s", modbus_strerror(errno));
        modbus_close(p_ctx);
        m_connected = false;
        return;
    }

    uint32_t currentHeartbeat = ((uint32_t)readRegs[0] << 16) | readRegs[1];
    if (currentHeartbeat != m_lastHeartbeatVal) {
        m_lastHeartbeatVal = currentHeartbeat;
        m_lastHeartbeatTime = this->now();
    }

    if ((this->now() - m_lastHeartbeatTime).seconds() > 0.5) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "COMM_LOST: PLC heartbeat frozen!");
    }

    std_msgs::msg::Bool lidarMsg, estopMsg;
    lidarMsg.data = (readRegs[14] & 0x01) != 0; 
    estopMsg.data = (readRegs[17] & 0x01) != 0; 
    
    p_pubLidarFb->publish(lidarMsg);
    p_pubEstopFb->publish(estopMsg);

    writeToPlc(readRegs);
}

/**
 * @brief  Writes Modbus registers back to the PLC reflecting PC state.
 * @param  readRegs Pointer to the previously read registers.
 */
void PlcInterfaceNode::writeToPlc(const uint16_t* readRegs) {
    assert(readRegs != nullptr);
    assert(m_connected == true);

    uint16_t writeHb[2] = { readRegs[0], readRegs[1] };
    int ret = modbus_write_registers(p_ctx, 100, 2, writeHb);
    if (ret == -1) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to echo heartbeat: %s", modbus_strerror(errno));
    }

    ret = modbus_write_register(p_ctx, 108, m_quickstopVal);
    if (ret == -1) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to write quickstop: %s", modbus_strerror(errno));
    }

    ret = modbus_write_register(p_ctx, 122, m_lidarCmdVal);
    if (ret == -1) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to write lidar cmd: %s", modbus_strerror(errno));
    }
}

/**
 * @brief  Main entry point for plc_interface node.
 * @param  argc Argument count.
 * @param  argv Arguments array.
 * @return Exit status.
 */
int main(int argc, char **argv) {
    assert(argc >= 1); // Precondition
    rclcpp::init(argc, argv);
    std::shared_ptr<PlcInterfaceNode> p_node = std::make_shared<PlcInterfaceNode>();
    assert(p_node != nullptr); // Postcondition
    rclcpp::spin(p_node);
    rclcpp::shutdown();
    return 0;
}
