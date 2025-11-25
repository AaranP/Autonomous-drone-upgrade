#!/bin/bash

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# ===== CONFIGURATION FOR YOUR NETWORK =====
# The hostname alias for your Raspberry Pi. This is used to automatically resolve its IP.
# Common default for Raspberry Pi is 'raspberrypi'. If using mDNS, it might be 'raspberrypi.local'.
ONBOARD_HOSTNAME_ALIAS="ledrone" # Using 'ledrone' as per your Dockerfile

# Ground Station (PC) IP address - automatically detected
# GROUND_IP=$(hostname -I | awk '{print $1}') # This gets the WSL2 internal IP

# Use the host IP passed from run_groundstation_container.sh if available
if [ -n "$GROUND_STATION_HOST_IP" ]; then
    GROUND_IP="$GROUND_STATION_HOST_IP"
    echo "Using Ground Station Host IP from environment: $GROUND_IP"
else
    GROUND_IP=$(hostname -I | awk '{print $1}')
    echo "Detected Ground Station IP Address (WSL2 internal): $GROUND_IP"
fi
# ===== END CONFIGURATION =====

# Check if Ground Station IP was successfully determined
if [ -z "$GROUND_IP" ]; then
    echo "Error: Could not automatically determine the IP address of this Ground Station."
    echo "Please ensure this machine is connected to a network."
    exit 1
fi
echo "Detected Ground Station IP Address: $GROUND_IP"

# --- Attempt to resolve Raspberry Pi IP or prompt for manual input ---
ONBOARD_IP="" # Initialize to empty
while true; do
    if [ -z "$ONBOARD_IP" ]; then # Only attempt auto-resolution if IP is not yet set (first loop iteration)
        if [ -n "$ONBOARD_HOSTNAME_ALIAS" ]; then
            echo "Attempting to resolve Raspberry Pi IP from hostname: $ONBOARD_HOSTNAME_ALIAS"
            # Use getent hosts to query DNS/hosts file/mDNS for the IP
            ONBOARD_IP=$(getent hosts "$ONBOARD_HOSTNAME_ALIAS" | awk '{print $1}' | head -n 1)
        fi
    fi

    if [ -z "$ONBOARD_IP" ]; then
        echo "--------------------------------------------------------------------------------"
        echo "WARNING: Could not automatically determine the IP address for Raspberry Pi using hostname '$ONBOARD_HOSTNAME_ALIAS'."
        echo "Please ensure the Raspberry Pi is on the network and its hostname is resolvable (e.g., via mDNS as raspberrypi.local)."
        read -p "Please manually enter the Raspberry Pi's IP address (e.g., 192.168.1.100): " MANUAL_IP
        if [ -z "$MANUAL_IP" ]; then
            echo "No IP address entered. Exiting."
            exit 1
        fi
        ONBOARD_IP="$MANUAL_IP"
    fi

    # Basic validation for the entered IP (non-empty)
    if [ -n "$ONBOARD_IP" ]; then
        echo "Using Raspberry Pi IP Address: $ONBOARD_IP"
        break # Exit loop if IP is set
    else
        echo "Invalid IP address. Please try again."
        ONBOARD_IP="" # Reset to try again
    fi
done

# Clean up all ROS environment variables from current shell
unset ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_PACKAGE_PATH
unset ROS_DISTRO
unset ROS_ROOT

# Define ROS environment variables to connect to the Pi's ROS Master
export ROS_MASTER_URI="http://$ONBOARD_IP:11311"
export ROS_IP="$GROUND_IP" # ROS_IP is used by local nodes to advertise themselves

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"

# Source ROS setup files for the current shell session within the container
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash # Assuming your ground station also needs workspace packages

echo "Attempting to connect to ROS Master on Raspberry Pi..."
# Run a ROS command to test connection. 'timeout' prevents hanging if master is unreachable.
# 'rostopic list' is a good general test.
if timeout 5 rostopic list > /dev/null 2>&1; then
    echo "Successfully connected to ROS Master on Raspberry Pi!"
    echo "Setup complete for Ground Station. You can now run ROS commands."
else
    echo "--------------------------------------------------------------------------------"
    echo "ERROR: Failed to connect to ROS Master at $ROS_MASTER_URI."
    echo "Please check the following:"
    echo "1. Ensure the Raspberry Pi is powered on and connected to the network."
    echo "2. Verify the Docker container on the Raspberry Pi is running and 'server.sh' has been executed."
    echo "3. Check for any firewalls blocking port 11311 on either the Raspberry Pi or this Ground Station PC."
    echo "4. Double-check the Raspberry Pi's IP address ($ONBOARD_IP) is correct."
    echo "--------------------------------------------------------------------------------"
    exit 1
fi