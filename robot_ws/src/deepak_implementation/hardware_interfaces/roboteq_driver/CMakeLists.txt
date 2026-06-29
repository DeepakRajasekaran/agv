cmake_minimum_required(VERSION 3.8)
project(roboteq_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# 1. Hardware Interface Library
add_library(roboteq_hardware_interface SHARED
  src/HardwareInterface.cpp
)
target_include_directories(roboteq_hardware_interface PRIVATE
  include
)
ament_target_dependencies(roboteq_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  custom_interfaces
)

# 2. CAN Driver Standalone Node
add_executable(roboteq_driver_node src/roboteq_driver_node.cpp src/RoboteqCanDriver.cpp)
target_include_directories(roboteq_driver_node PRIVATE include)
ament_target_dependencies(roboteq_driver_node
  rclcpp
  custom_interfaces
  std_srvs
)

# Export the library so others can find it via pluginlib
pluginlib_export_plugin_description_file(hardware_interface roboteq_driver_plugins.xml)

# Install targets
install(
  TARGETS roboteq_hardware_interface roboteq_driver_node
  EXPORT export_roboteq_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
