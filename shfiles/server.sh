#!/bin/bash

echo "--- Setting up ROS Network Configuration for Raspberry Pi (Onboard Computer) ---"

# ===== AUTOMATICALLY DETECT IP ADDRESS =====
# Get the primary IP address of the current machine
# This command typically returns the IP address(es) of the active network interface(s).
# We take the first one found.
ONBOARD_IP=$(hostname -I | awk '{print $1}')

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not automatically determine the IP address of this machine."
    echo "Please ensure you are connected to a network and try again, or manually set ONBOARD_IP."
    exit 1
fi

ROS_HOSTNAME="$ONBOARD_IP" # Pi's hostname or IP (usually same as ONBOARD_IP)
# ===== END CONFIGURATION =====

echo "Automatically detected IP Address: $ONBOARD_IP"

# Define ROS environment variables to point to itself (as ROS Master)
ROS_MASTER_URI="http://$ONBOARD_IP:11311"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
#echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."

# Removed: Modifying ~/.bashrc is not suitable for ephemeral Docker containers.
# Removed: Modifying /etc/hosts is not suitable for Docker containers and often not needed if using IP for ROS_MASTER_URI.

# Source ROS setup files for the current shell session within the container
# These should already be sourced by the Dockerfile's .bashrc, but explicit sourcing
# ensures they are available if the script is run in a non-interactive shell.
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash # Assuming your ground station also needs workspace packages

echo "Setup complete for Raspberry pi."