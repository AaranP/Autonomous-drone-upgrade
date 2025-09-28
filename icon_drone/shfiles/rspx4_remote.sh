sudo chmod 777 /dev/ttyACM0 & sleep 2;
export ROS_MASTER_URI=http://<B IP>:11311
export ROS_IP=<B IP>
roscore && sleep 5;

roslaunch realsense2_camera rs_camera.launch & sleep 2;

# For testing use physical IMU
roslaunch fdilink_ahrs ahrs_data.launch & sleep 2;

# Launch the Flight Controller 
roslaunch mavros px4.launch & sleep 2;

# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
# rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;

# On the PC
# roslaunch vins fast_drone_250.launch & sleep 1;
# rosrun pose_utils odom2pose.py;
wait;
