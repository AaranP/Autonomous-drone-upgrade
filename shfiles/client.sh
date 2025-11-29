#!/bin/bash

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# Get Ground Station Host IP from environment variable passed by run_groundstation_container.sh
if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP environment variable is not set!"
    echo "Please ensure 'run_groundstation_container.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Detected Ground Station IP Address: ${GROUND_STATION_HOST_IP}"

# Get Raspberry Pi Target IP from environment variable passed by run_groundstation_container.sh
if [ -z "$RASPBERRY_PI_TARGET_IP" ]; then
    echo "ERROR: RASPBERRY_PI_TARGET_IP environment variable is not set!"
    echo "Please ensure 'run_groundstation_container.sh' on the host passes this IP. Exiting."
    exit 1
fi
echo "Using Raspberry Pi Target IP Address: ${RASPBERRY_PI_TARGET_IP}"

export ROS_MASTER_URI="http://${RASPBERRY_PI_TARGET_IP}:11311"
export ROS_IP="${GROUND_STATION_HOST_IP}"
export ROS_HOSTNAME="${GROUND_STATION_HOST_IP}" # Optional, good practice for some ROS tools

echo "ROS_MASTER_URI set to: ${ROS_MASTER_URI}"
echo "ROS_IP set to: ${ROS_IP}"

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
    # Do not exit here, allow user to try ROS commands, but warn them.
fi

echo "ROS environment configured. You can now run ROS commands."

# --- ROS IP Configuration for the Ground Station (Client) ---
# GROUND_STATION_HOST_IP is passed from run_groundstation_container.sh script
if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP environment variable is not set. Cannot configure ROS_IP."
    exit 1
fi

export ROS_IP="${GROUND_STATION_HOST_IP}"
echo "Set ROS_IP to: ${ROS_IP}"

# --- ROS_MASTER_URI Configuration (Points to Drone) ---
# Assuming the Pi's Tailscale hostname is 'ledrone' as per your requirement.
# This relies on Tailscale DNS resolving 'ledrone' to its Tailscale IP,
# or local DNS resolving it to a direct IP if available.
# Ensure 'ledrone' is configured as the Tailscale hostname for your Pi.
DRONE_HOSTNAME="ledrone" 

# Attempt to resolve the Drone's IP using the hostname.
# This will prioritize Tailscale DNS if configured, then local DNS.
DRONE_RESOLVED_IP=$(getent hosts "${DRONE_HOSTNAME}" | awk '{print $1}' | head -n 1)

if [ -z "$DRONE_RESOLVED_IP" ]; then
    echo "ERROR: Could not resolve IP for drone hostname '${DRONE_HOSTNAME}'. Please ensure drone is online and Tailscale/local DNS is working."
    exit 1
fi

export ROS_MASTER_URI="http://${DRONE_RESOLVED_IP}:11311"
echo "Set ROS_MASTER_URI to: ${ROS_MASTER_URI}"

# You might want to add other client-specific ROS commands here
# For example, launching Rviz or PlotJuggler
echo "Starting ROS client..."
# Example: rviz
/bin/bash # Keep the container alive with a bash shell