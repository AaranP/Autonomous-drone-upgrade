#!/bin/bash

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source ~/kw076/Autonomous-drone-upgrage/icon_drone/devel/setup.bash

# Sync system clock
#sudo systemctl restart chrony

# Reset any existing ROS nodes
#pkill -f ros & sleep 5

# Set ROS networking
export ROS_MASTER_URI=http://128.189.246.106:11311 
export ROS_HOSTNAME=206.87.208.93

rviz