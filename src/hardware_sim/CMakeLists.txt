cmake_minimum_required(VERSION 3.8)
project(hardware_sim)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/hardware_sim_node.py
  scripts/TapeMap.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model assets
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
