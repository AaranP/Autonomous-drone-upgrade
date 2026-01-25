# Launch VINS and FUEL in parallel
echo "--- Starting FUEL ---"

# 1. Decompress from the standard RealSense compressed topic
rosrun image_transport republish compressedDepth in:=/camera/depth/image_rect_raw raw out:=/camera/depth/decompressed & 
sleep 2

# 2. Launch exploration, telling it to use the DECOMPRESSED topic
roslaunch exploration_manager exploration.launch 