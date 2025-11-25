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
export ROS_MASTER_URI="http://$ONBOARD_IP:11311" # Use export directly
export ROS_IP="$ONBOARD_IP" # It's good practice for ROS_IP to be set for the master itself
                               # if it's also running other nodes.
                               # If only roscore, ROS_HOSTNAME is more critical.

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
echo "ROS_IP set to: $ROS_IP"

# Source ROS setup files for the current shell session within the container
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash # Assuming your ground station also needs workspace packages

echo "Setup complete for Raspberry pi."

# --- Start ROS Master ---
echo "Starting ROS Master (roscore)..."
exec roscore # 'exec' replaces the current shell with roscore, keeping it in the foreground