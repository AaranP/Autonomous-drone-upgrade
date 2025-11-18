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
pkill -f ros & sleep 5

# Start ROS core in background
roscore & sleep 5

# Get the primary IP address of the onboard computer
# Using 'hostname -I' for Linux, 'ipconfig getifaddr en0' for macOS (though onboard is likely Linux)
ONBOARD_IP=$(hostname -I | awk '{print $1}')

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not determine onboard computer's IP address. Please ensure network is connected."
    exit 1
fi

echo "Detected Onboard IP Address: $ONBOARD_IP"

# Define ROS environment variables
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_HOSTNAME="$ONBOARD_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
# Use 'sed -i' for Linux, 'sed -i ""' for macOS
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setting up ROS environment for Ground Station..."
########################################
########################################
# Server (Raspberry Pi) Setup Script
########################################
########################################

# Launch necessary modules in background
roslaunch fdilink_ahrs ahrs_data.launch & sleep 5
roslaunch mavros px4.launch & sleep 5
# rosrun mavros mavsys rate --all 100

# Launch RealSense with optimized config
roslaunch realsense2_camera rs_camera.launch \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  enable_pointcloud:=false \
  enable_sync:=true \
  unite_imu_method:=none \
  depth_width:=640 depth_height:=480 depth_fps:=30 \
  infra_width:=640 infra_height:=480 infra_fps:=30 \
  align_depth:=false \
  & sleep 5

# Start IR image compression
#rosrun image_transport republish raw in:=/camera/infra1/image_rect_raw compressed out:=/image1 &
#rosrun image_transport republish raw in:=/camera/infra2/image_rect_raw compressed out:=/image2 &

roslaunch px4ctrl run_ctrl.launch & sleep 5

# Start video recording node (ready to receive commands)
roslaunch video_recorder video_recorder.launch &
# Launch VINS and FUEL in parallel
roslaunch vins fast_drone_250.launch & sleep 2
roslaunch exploration_manager exploration.launch rviz:=false & sleep 2
roslaunch px4ctrl run_ctrl.launch & sleep 2