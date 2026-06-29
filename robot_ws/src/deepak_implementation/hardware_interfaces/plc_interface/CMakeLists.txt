cmake_minimum_required(VERSION 3.8)
project(plc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Enforce maximum warnings as per coding guidelines
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)

# Use pkg-config to find libmodbus
pkg_check_modules(MODBUS libmodbus)

if(NOT MODBUS_FOUND)
  message(WARNING "pkg-config failed to find libmodbus. Falling back to sysroot paths for cross-compilation.")
  set(MODBUS_INCLUDE_DIRS ${CMAKE_SYSROOT}/usr/include/modbus)
  if(EXISTS "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libmodbus.so")
    set(MODBUS_LIBRARIES ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libmodbus.so)
  else()
    # Fallback to local host path if not using a cross-compile sysroot
    set(MODBUS_LIBRARIES /usr/lib/x86_64-linux-gnu/libmodbus.so)
  endif()
endif()

add_executable(plc_interface_node src/PlcInterfaceNode.cpp)
target_include_directories(plc_interface_node PUBLIC include ${MODBUS_INCLUDE_DIRS})
target_link_libraries(plc_interface_node ${MODBUS_LIBRARIES})
ament_target_dependencies(plc_interface_node rclcpp std_msgs std_srvs)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  plc_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
