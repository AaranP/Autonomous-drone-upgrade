#!/bin/bash

echo "--- Setting up ROS Network Configuration for Linux Ground Station ---"

# ===== CONFIGURATION FOR YOUR NETWORK =====
# The Raspberry Pi's IP address will be automatically determined if its hostname is resolvable.
# If automatic detection fails, you may need to uncomment and manually set ONBOARD_IP.
# ONBOARD_IP="128.189.245.13"

# The hostname alias for your Raspberry Pi. This is used to automatically resolve its IP.
# Common default for Raspberry Pi is 'raspberrypi'. If using mDNS, it might be 'raspberrypi.local'.
ONBOARD_HOSTNAME_ALIAS="ledrone"

# Ground Station (PC) IP address - automatically detected
GROUND_IP=$(hostname -I | awk '{print $1}')
# ===== END CONFIGURATION =====

# Attempt to resolve the Raspberry Pi's IP address from its hostname alias
ONBOARD_IP="" # Initialize to empty
if [ -n "$ONBOARD_HOSTNAME_ALIAS" ]; then
    echo "Attempting to resolve Raspberry Pi IP from hostname: $ONBOARD_HOSTNAME_ALIAS"
    # Use getent hosts to query DNS/hosts file/mDNS for the IP
    # 'head -n 1' ensures we only take the first IP if multiple are returned
    ONBOARD_IP=$(getent hosts "$ONBOARD_HOSTNAME_ALIAS" | awk '{print $1}' | head -n 1)
fi

# Check if ONBOARD_IP was successfully determined
if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not automatically determine the IP address for Raspberry Pi using hostname '$ONBOARD_HOSTNAME_ALIAS'."
    echo "Please ensure the Raspberry Pi is on the network and its hostname is resolvable (e.g., via mDNS as raspberrypi.local)."
    echo "Alternatively, uncomment and manually set the 'ONBOARD_IP' variable in this script."
    exit 1
fi

# Check if Ground Station IP was successfully determined
if [ -z "$GROUND_IP" ]; then
    echo "Error: Could not automatically determine the IP address of this Ground Station."
    echo "Please ensure this machine is connected to a network."
    exit 1
fi

echo "Detected Raspberry Pi IP Address: $ONBOARD_IP"
echo "Detected Ground Station IP Address: $GROUND_IP"

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

# Removed: Modifying ~/.bashrc is not suitable for ephemeral Docker containers.
# Removed: Modifying /etc/hosts is not suitable for Docker containers and often not needed if using IP for ROS_MASTER_URI.

# Source ROS setup files for the current shell session within the container
# These should already be sourced by the Dockerfile's .bashrc, but explicit sourcing
# ensures they are available if the script is run in a non-interactive shell.
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash # Assuming your ground station also needs workspace packages

echo "Setup complete for Linux Ground Station."