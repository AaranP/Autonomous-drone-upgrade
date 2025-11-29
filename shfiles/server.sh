#!/bin/bash

echo "--- Setting up ROS Network Configuration for Drone ---"

# Get Drone ROS IP from environment variable passed by run_container.sh
if [ -z "$DRONE_ROS_IP" ]; then
    echo "ERROR: DRONE_ROS_IP environment variable is not set!"
    echo "Please ensure 'run_container.sh' on the host passes this IP. Exiting."
    exit 1
fi

echo "Detected Drone IP Address: ${DRONE_ROS_IP}"

export ROS_MASTER_URI="http://${DRONE_ROS_IP}:11311"
export ROS_IP="${DRONE_ROS_IP}"
export ROS_HOSTNAME="${DRONE_ROS_IP}" # Optional, but good practice

echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
echo "ROS_IP set to: ${ROS_IP}"

echo "Attempting to connect to ROS Master..."
timeout 5 bash -c "while ! rostopic list > /dev/null 2>&1; do echo 'Waiting for ROS Master...'; sleep 1; done"
if [ $? -eq 0 ]; then
    echo "ROS Master is running!"
else
    echo "WARNING: ROS Master did not respond within 5 seconds. It might start later, or there's an issue."
fi

# Keep the container alive with a bash session
exec /bin/bash