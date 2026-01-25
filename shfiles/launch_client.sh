# ...existing code...
# Launch VINS and FUEL in parallel
echo "--- Starting FUEL ---"

# 1. Decompress the depth image from the drone for local processing
# This takes /camera/depth/image_rect_raw/compressed and turns it into /camera/depth/image_rect_raw locally
rosrun image_transport republish compressedDepth in:=/depth_image raw out:=/depth_image_out & sleep 2

#rosrun image_transport republish compressed in:=/image2 raw out:=/vins/image2 & sleep 2

# 2. Decompress VINS tracking image (optional, only if you want to view it with rqt_image_view or other tools)
# rosrun image_transport republish compressed in:=/vins_fusion/image_track raw out:=/vins_fusion/image_track &

#Run simulation 
roslaunch exploration_manager exploration.launch & sleep 2
# ...existing code...