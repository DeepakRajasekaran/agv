#include "rclcpp/rclcpp.hpp"
#include "line_follower/NavigationStateMachine.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_follower::NavigationStateMachine>());
  rclcpp::shutdown();
  return 0;
}
