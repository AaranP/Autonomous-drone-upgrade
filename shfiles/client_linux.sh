#!/bin/bash

# Ensure all scripts are executable and have correct line endings (CRLF to LF)
# dos2unix can be run on all shell scripts on the host to prevent issues.

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# Get Ground Station Host IP from environment variable passed by run_groundstation_container.sh
if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP environment variable is not set!"
    echo "Please ensure 'run_linux_groundstation.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Detected Ground Station IP Address: ${GROUND_STATION_HOST_IP}"

# Get Raspberry Pi Target IP from environment variable passed by run_groundstation_container.sh
if [ -z "$RASPBERRY_PI_TARGET_IP" ]; then
    echo "ERROR: RASPBERRY_PI_TARGET_IP environment variable is not set!"
    echo "Please ensure 'run_linux_groundstation.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Using Raspberry Pi Target IP Address: ${RASPBERRY_PI_TARGET_IP}"

# Get ROS Connection Type
if [ -z "$ROS_CONNECTION_TYPE" ]; then
    echo "ERROR: ROS_CONNECTION_TYPE environment variable is not set! Exiting."
    exit 1
fi
echo "ROS Connection Type: ${ROS_CONNECTION_TYPE}"

# --- Source ROS setup files FIRST ---
# This is crucial so that subsequent 'export' commands override any auto-detected ROS settings
source /opt/ros/noetic/setup.bash || { echo "ERROR: Failed to source /opt/ros/noetic/setup.bash. Exiting."; exit 1; }
source /root/catkin_ws/devel/setup.bash || { echo "ERROR: Failed to source /root/catkin_ws/devel/setup.bash. Exiting."; exit 1; }


if [ "$ROS_CONNECTION_TYPE" == "direct" ]; then
    export ROS_MASTER_URI="http://${RASPBERRY_PI_TARGET_IP}:11311"
    echo "Set ROS_MASTER_URI to direct IP: ${ROS_MASTER_URI}"
elif [ "$ROS_CONNECTION_TYPE" == "tailscale" ]; then
    # Assuming the Pi's Tailscale hostname is 'ledrone' as per your requirement.
    DRONE_HOSTNAME="ledrone"
    # Attempt to resolve hostname. getent hosts works well within containers on --network=host.
    DRONE_RESOLVED_IP=$(getent hosts "${DRONE_HOSTNAME}" | awk '{print $1}' | head -n 1)

    if [ -z "$DRONE_RESOLVED_IP" ]; then
        echo "ERROR: Could not resolve IP for drone hostname '${DRONE_HOSTNAME}'. Please ensure drone is online and Tailscale/local DNS is working." \
        "Using fallback IP ${RASPBERRY_PI_TARGET_IP} as ROS_MASTER_URI. Check Tailscale setup."
        export ROS_MASTER_URI="http://${RASPBERRY_PI_TARGET_IP}:11311"
    else
        export ROS_MASTER_URI="http://${DRONE_RESOLVED_IP}:11311"
        echo "Set ROS_MASTER_URI to Tailscale resolved IP: ${ROS_MASTER_URI}"
    fi
else
    echo "ERROR: Invalid ROS_CONNECTION_TYPE '${ROS_CONNECTION_TYPE}'. Exiting."
    exit 1
fi

# --- IMPORTANT FIX: Set ROS_IP to 127.0.0.1 for local container communication ---
# This ensures local ROS nodes bind to the loopback interface, preventing "Unable to contact my own server" errors
# when the container's host IP (e.g., WSL's Tailscale IP) cannot be reliably bound for internal services.
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="localhost" # Good practice when ROS_IP is 127.0.0.1

echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
echo "ROS_IP set to: ${ROS_IP}"
echo "ROS_HOSTNAME set to: ${ROS_HOSTNAME}" # Added for clarity

echo "Attempting to connect to ROS Master on Raspberry Pi at ${ROS_MASTER_URI}..."
# Use a timeout to avoid indefinite waiting
timeout 10 bash -c "while ! rostopic list > /dev/null 2>&1; do echo 'Waiting for ROS Master...'; sleep 1; done"
if [ $? -eq 0 ]; then
    echo "ROS Master is running!"
else
    echo "ERROR: Failed to connect to ROS Master at ${ROS_MASTER_URI}."
    echo "Please check the following:"
    echo "1. Ensure the Raspberry Pi is powered on and connected to the network."
    echo "2. Verify the Docker container on the Raspberry Pi is running and 'server.sh' has been executed."
    echo "3. Confirm the Raspberry Pi's IP address (${RASPBERRY_PI_TARGET_IP} or resolved Tailscale IP) is correct and reachable from this Ground Station (e.g., using ping or if firewalls are not blocking)."
    echo "4. Double-check that the chosen IP types (Direct/Tailscale) match on both the Pi and Ground Station."
fi

echo "ROS environment configured. You can now run ROS commands."

echo "Starting ROS client..."
exec /bin/bash # Keep the container alive with a bash shell