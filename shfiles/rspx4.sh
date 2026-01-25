#!/bin/bash

sudo chmod 777 /dev/ttyACM0 & sleep 2;

echo "--- Starting RealSense Camera ---"
# Lower the framerate to reduce power and bandwidth demand

roslaunch realsense2_camera rs_camera.launch & 
sleep 5; # Give camera time to start publishing
echo "--- Realsense Camera is running! ---"


echo "--- Starting FDILINK IMU ---"
roslaunch fdilink_ahrs ahrs_data.launch &
sleep 5

#roslaunch mavros px4.launch &
sleep 5; # Give MAVROS time to initialize and connect to FCU


echo "--- Starting VINS-Fusion ---"
mkdir -p /root/vins_output/pose_graph # Ensure the output directories exist
chmod -R 777 /root/vins_output

roslaunch vins fast_drone_250.launch &
sleep 10; # Give VINS time to start processing

echo "All processes started."
wait;
