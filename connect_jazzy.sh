#!/bin/bash
# Shell script to connect to the running ROS 2 Jazzy Docker container
# with the ROS environment and colcon workspace setup automatically sourced.

docker exec -it ros_jazzy_magnetic_sim_container bash -c "source /opt/ros/jazzy/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; exec bash"
