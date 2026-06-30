#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/user/ros2-esp32-rover-stack/microros_ws/install/setup.bash
ros2 launch rover_bringup rover.launch.py
