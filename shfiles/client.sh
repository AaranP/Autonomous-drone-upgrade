#!/bin/bash

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# Get Ground Station Host IP from environment variable passed by run_groundstation_container.sh
if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP environment variable is not set!"
    echo "Please ensure 'run_groundstation_container.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Detected Ground Station Host IP Address (from host script): ${GROUND_STATION_HOST_IP}"

# Get Raspberry Pi Target IP from environment variable passed by run_groundstation_container.sh
if [ -z "$RASPBERRY_PI_TARGET_IP" ]; then
    echo "ERROR: RASPBERRY_PI_TARGET_IP environment variable is not set!"
    echo "Please ensure 'run_groundstation_container.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Using Raspberry Pi Target IP Address (ROS_MASTER_URI): ${RASPBERRY_PI_TARGET_IP}"

# --- Source ROS setup files FIRST ---
# This is crucial so that subsequent 'export' commands override any auto-detected ROS settings
source /opt/ros/noetic/setup.bash || { echo "ERROR: Failed to source /opt/ros/noetic/setup.bash. Exiting."; exit 1; }
source /root/catkin_ws/devel/setup.bash || { echo "ERROR: Failed to source /root/catkin_ws/devel/setup.bash. Exiting."; exit 1; }

# Set ROS_MASTER_URI to point to the remote Raspberry Pi
export ROS_MASTER_URI="http://${RASPBERRY_PI_TARGET_IP}:11311"

# Ensure ROS_MASTER_URI is set from the RASPBERRY_PI_TARGET_IP passed from the host script
if [ -z "$RASPBERRY_PI_TARGET_IP" ]; then
    echo "ERROR: RASPBERRY_PI_TARGET_IP environment variable not set. ROS Master connection will fail."
    # Optionally, you could exit or prompt for manual input here
else
    export ROS_MASTER_URI="http://${RASPBERRY_PI_TARGET_IP}:11311"
    echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
fi

# --- Set ROS_IP and ROS_HOSTNAME ---
# This allows external nodes (like on the Pi) to connect to this container's ROS nodes.
if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP environment variable not set. ROS IP will default to local."
    export ROS_IP="127.0.0.1"
    export ROS_HOSTNAME="localhost"
else
    export ROS_IP="${GROUND_STATION_HOST_IP}"
    # Check if we are using Tailscale. If so, use the Tailscale hostname for ROS_HOSTNAME
    # to ensure proper name resolution across the VPN. The container has an /etc/hosts
    # entry mapping this hostname to 127.0.0.1 for local loopback connections.
    if [[ "${GROUND_STATION_HOST_IP}" == 100.* ]] && [ -n "$GROUND_STATION_TAILSCALE_HOSTNAME" ]; then
        export ROS_HOSTNAME="${GROUND_STATION_TAILSCALE_HOSTNAME}"
    else
        # Fallback to the IP if not on Tailscale or hostname not provided.
        export ROS_HOSTNAME="${GROUND_STATION_HOST_IP}"
    fi
fi

echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
echo "ROS_IP set to: ${ROS_IP}"
echo "ROS_HOSTNAME set to: ${ROS_HOSTNAME}"

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
    echo "3. Confirm the Raspberry Pi's IP address (${RASPBERRY_PI_TARGET_IP}) is correct and reachable from this Ground Station (e.g., using ping or if firewalls are not blocking)."
    echo "4. Double-check that the chosen IP types (Direct/Tailscale) match on both the Pi and Ground Station."
fi

echo "ROS environment configured. You can now run ROS commands."

echo "Starting ROS client..."
exec /bin/bash # Keep the container alive with a bash shell