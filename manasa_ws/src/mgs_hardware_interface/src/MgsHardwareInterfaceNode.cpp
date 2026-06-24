/*
Name: MgsHardwareInterfaceNode.cpp
Author: Antigravity
Date: 2026-06-24
Version: 1.0
Description: Main entry point for the mgs_hardware_interface package.
*/

#include "rclcpp/rclcpp.hpp"
#include "mgs_hardware_interface/MgsInterface.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  mgs_hardware_interface::MgsInterface interface;
  interface.initialize();
  
  if (!interface.start()) {
    RCLCPP_ERROR(rclcpp::get_logger("mgs_node"), "Failed to start CAN interface.");
    return 1;
  }
  
  auto node = interface.getRosNode();
  rclcpp::spin(node);
  
  interface.stop();
  rclcpp::shutdown();
  return 0;
}
