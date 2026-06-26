/*
Name: RoboteqHardwareInterfaceNode.cpp
Author: Manasa
Date: 2026-06-23
Version: 1.0
Description: Main entry point node to instantiate the RoboteqInterface and spin it.
*/

#include "rclcpp/rclcpp.hpp"
#include "roboteq_hardware_interface/RoboteqInterface.h"
#include <memory>
#include <iostream>

/**
 * @brief Main function initializing ROS 2, starting the interface, and spinning the node.
 * @param argc - Number of input arguments.
 * @param argv - Argument vectors.
 * @return int - Status exit code.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Use smart pointer to manage lifetime of mediator class
  auto pInterface = std::make_unique<roboteq_hardware_interface::RoboteqInterface>();

  pInterface->initialize();
  if (!pInterface->start())
  {
    std::cerr << "Failed to start Roboteq CANopen hardware interface!" << std::endl;
    return 1;
  }

  // Spin the ROS node
  rclcpp::spin(pInterface->getRosNode());

  pInterface->stop();
  rclcpp::shutdown();

  return 0;
}
