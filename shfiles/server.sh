#!/bin/bash

# Create the VINS-Fusion output directory if it doesn't exist
mkdir -p /root/vins_output

echo "--- Setting up ROS Network Configuration for Drone ---"

# Get Drone ROS IP from environment variable passed by run_container.sh
if [ -z "$DRONE_ROS_IP" ]; then
    echo "ERROR: DRONE_ROS_IP environment variable is not set!"
    echo "Please ensure 'run_container.sh' on the host passes this IP. Exiting."
    exit 1
fi

echo "Detected Drone IP Address: ${DRONE_ROS_IP}"

# Ensure ROS setup is sourced first, as these might try to auto-detect ROS_IP
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# NOW, explicitly set ROS_MASTER_URI, ROS_IP, and ROS_HOSTNAME
# This ensures your chosen IP takes precedence
export ROS_MASTER_URI="http://${DRONE_ROS_IP}:11311"
export ROS_IP="${DRONE_ROS_IP}"
export ROS_HOSTNAME="${DRONE_ROS_IP}" # Optional, but good practice

echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
echo "ROS_IP set to: ${ROS_IP}"
echo "ROS environment configured."

# --- Start ROS Master automatically in the background ---
if ! pgrep -x "roscore" > /dev/null; then
    echo "Starting ROS Master (roscore) in the background..."
    roscore & # Start rocore in the background
    sleep 3 # Give roscore a moment to initialize
    echo "ROS Master started."
else
    echo "ROS Master (roscore) is already running."
fi

# Attempt to connect to ROS Master locally to confirm it's running
echo "Attempting to connect to ROS Master locally..."
timeout 10 bash -c "while ! rostopic list > /dev/null 2>&1; do echo 'Waiting for ROS Master...'; sleep 1; done"
if [ $? -eq 0 ]; then
    echo "ROS Master is running and reachable locally!"
else
    echo "WARNING: ROS Master did not respond within 10 seconds locally. Check roscore process."
fi

echo ""
# Keep the container alive with a bash session
echo "Entering interactive bash shell. You can now run additional ROS commands."
exec /bin/bash