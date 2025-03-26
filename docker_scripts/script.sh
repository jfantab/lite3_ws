#!/bin/bash

source /opt/ros/foxy/setup.bash;
export ROS_DOMAIN_ID=42;
cd ~/Lite3_ROS
source install/setup.bash;
ros2 launch transfer transfer_launch.py;