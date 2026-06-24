cmake_minimum_required(VERSION 3.8)
project(line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Line follower library
add_library(${PROJECT_NAME}_lib SHARED
  src/PidController.cpp
  src/LineFollowerNode.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp geometry_msgs std_msgs std_srvs)

# Line follower executable
add_executable(line_follower_node src/LineFollowerMain.cpp)
target_link_libraries(line_follower_node ${PROJECT_NAME}_lib)

# Navigation state machine library
add_library(nav_state_machine_lib SHARED
  src/NavigationStateMachine.cpp
)
target_include_directories(nav_state_machine_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nav_state_machine_lib rclcpp std_msgs std_srvs)

# Navigation state machine executable
add_executable(navigation_state_machine_node src/NavigationStateMachineMain.cpp)
target_link_libraries(navigation_state_machine_node nav_state_machine_lib)

install(TARGETS ${PROJECT_NAME}_lib nav_state_machine_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS line_follower_node navigation_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
