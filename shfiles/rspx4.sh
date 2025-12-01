#!/bin/bash

sudo chmod 777 /dev/ttyACM0 & sleep 2;

echo "--- Starting RealSense Camera ---"
# Lower the framerate to reduce power and bandwidth demand
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=15 infra_width:=640 infra_height:=480 infra_fps:=15 &
sleep 20; # Give camera time to start publishing


echo "--- Starting MAVROS ---"
roslaunch mavros px4.launch &
sleep 20; # Give MAVROS time to initialize and connect to FCU


echo "--- Starting VINS-Fusion ---"
mkdir -p /root/vins_output/pose_graph # Ensure the output directories exist
chmod -R 777 /root/vins_output

roslaunch vins fast_drone_250.launch &
sleep 20; # Give VINS time to start processing

echo "All processes started."
wait;