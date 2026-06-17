cmake_minimum_required(VERSION 3.8)
project(line_follower_controller)

# Enable C++17 as standard for ROS 2
add_compile_options(-std=c++17)

# Enable maximum compiler warnings as per ANSCER/NASA guidelines
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Werror
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

add_executable(line_follower_controller_node
  src/PidController.cpp
  src/NavigationStateMachine.cpp
  src/FaultMonitor.cpp
)

ament_target_dependencies(line_follower_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  nav_msgs
)

# Install executable
install(TARGETS line_follower_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
