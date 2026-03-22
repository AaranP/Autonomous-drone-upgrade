# Launch VINS and then FUEL

echo "--- Starting VINS-Fusion ---"
mkdir -p /root/vins_output/pose_graph # Ensure the output directories exist
chmod -R 777 /root/vins_output

roslaunch vins fast_drone_250.launch &
sleep 10; # Give VINS time to start processing

echo "--- Starting FUEL ---"

# 1. Decompress from the standard RealSense compressed topic
rosrun image_transport republish compressedDepth in:=/camera/depth/image_rect_raw raw out:=/camera/depth/decompressed & 
sleep 2

# 2. Launch exploration, telling it to use the DECOMPRESSED topic
roslaunch exploration_manager exploration.launch 