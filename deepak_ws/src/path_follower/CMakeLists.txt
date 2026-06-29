cmake_minimum_required(VERSION 3.8)
project(path_follower)

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
find_package(custom_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(
  include
)

add_executable(path_follower_node
  src/PidController.cpp

  src/FaultMonitor.cpp
)

ament_target_dependencies(path_follower_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  custom_interfaces
  nav_msgs
  behaviortree_cpp
)

# Install executable
install(TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS 

  scripts/nav_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
