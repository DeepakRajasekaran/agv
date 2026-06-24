cmake_minimum_required(VERSION 3.8)
project(mgs_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mgs_driver_node src/MgsDriverNode.cpp)
target_include_directories(mgs_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mgs_driver_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  mgs_driver_node
  "rclcpp"
  "std_msgs"
  "std_srvs"
)

install(TARGETS mgs_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)

ament_package()
