/*
 * Name:        PlcInterfaceNode.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-29
 * Version:     1.0
 * Description: Header for the Modbus TCP PLC interface node handling hardware I/O mapping.
 */

#ifndef PLC_INTERFACE_NODE_H
#define PLC_INTERFACE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <modbus/modbus.h>

class PlcInterfaceNode : public rclcpp::Node {
public:
    PlcInterfaceNode();
    virtual ~PlcInterfaceNode();

private:
    void timerCallback();
    void connectToPlc();
    void readFromPlc();
    void writeToPlc(const uint16_t* readRegs);

    // ROS 2 Pointers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubLidarFb;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_pubEstopFb;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr p_subLidarCmd;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr p_srvQuickstop;
    rclcpp::TimerBase::SharedPtr p_timer;

    // Modbus context pointer
    modbus_t* p_ctx;

    // Member variables
    bool m_connected;
    uint32_t m_lastHeartbeatVal;
    rclcpp::Time m_lastHeartbeatTime;
    uint16_t m_quickstopVal;
    uint16_t m_lidarCmdVal;
};

#endif // PLC_INTERFACE_NODE_H
