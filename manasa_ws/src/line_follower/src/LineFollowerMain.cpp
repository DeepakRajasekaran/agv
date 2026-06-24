/*
Name: LineFollowerMain.cpp
Author: Antigravity
Date: 2026-06-24
Version: 1.0
Description: Main entry point for the line follower node.
*/

#include "rclcpp/rclcpp.hpp"
#include "line_follower/LineFollowerNode.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<line_follower::LineFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
