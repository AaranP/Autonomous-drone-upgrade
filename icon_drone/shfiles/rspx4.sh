sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch realsense2_camera rs_camera.launch use_nodelet:=false \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true \
  infra_fps:=15 infra_width:=640 infra_height:=480 \
  use_nodelet:=false initial_reset:=true & sleep 1;
roslaunch fdilink_ahrs ahrs_data.launch & sleep 1;
roslaunch mavros px4.launch & sleep 1;

# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
# rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch vins fast_drone_250.launch & sleep 1;
rosrun pose_utils odom2pose.py;
wait;
