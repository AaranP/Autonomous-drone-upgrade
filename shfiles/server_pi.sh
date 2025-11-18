#!/bin/bash

echo "--- Setting up ROS Network Configuration for Onboard Computer ---"

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source ~/KW076/Autonomous-drone-upgrade/icon_drone/devel/setup.bash

# Set display (for RViz if needed)
# export DISPLAY=:0

# Sync system clock
#sudo systemctl restart chrony

# Reset any existing ROS nodes
pkill -f ros & sleep 3

# Set ROS networking
export ROS_MASTER_URI=http://128.189.245.13:11311 
export ROS_HOSTNAME=128.189.245.13

roscore & sleep 3

# Launch VINS and FUEL in parallel
roslaunch vins fast_drone_250.launch & sleep 2
roslaunch exploration_manager exploration.launch rviz:=false & sleep 2
roslaunch px4ctrl run_ctrl.launch & sleep 2

# Start decompression republishers
rosrun image_transport republish compressed in:=/image1 raw out:=/vins/image1 & sleep 2
rosrun image_transport republish compressed in:=/image2 raw out:=/vins/image2 & sleep 2
