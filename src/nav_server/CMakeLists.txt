cmake_minimum_required(VERSION 3.8)
project(nav_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(libpqxx REQUIRED libpqxx)

add_executable(nav_server_node src/nav_server_node.cpp)
target_include_directories(nav_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${libpqxx_INCLUDE_DIRS}
)

ament_target_dependencies(nav_server_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(nav_server_node
  ${libpqxx_LIBRARIES}
)

install(TARGETS nav_server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
