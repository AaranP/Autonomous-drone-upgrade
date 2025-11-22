#!/bin/bash

echo "--- Setting up ROS Network Configuration for Raspberry Pi (Onboard Computer) ---"

# ===== AUTOMATICALLY DETECT IP ADDRESS =====
# Prioritize WiFi (wlan0) IP for ROS reachability from ground station
ONBOARD_IP=$(ip -4 addr show wlan0 2>/dev/null | awk '/inet / {print $2}' | cut -d/ -f1 | head -1)

# Fallback: default route outbound IP
if [ -z "$ONBOARD_IP" ]; then
  ONBOARD_IP=$(ip route get 8.8.8.8 2>/dev/null | awk '{print $7; exit}')
fi

# Final fallback: hostname -I
if [ -z "$ONBOARD_IP" ]; then
  ONBOARD_IP=$(hostname -I | awk '{print $1}')
fi

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not automatically determine WiFi IP. Ensure wlan0 is connected."
    exit 1
fi

echo "Prioritized WiFi IP: $ONBOARD_IP (wlan0)"

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
echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Raspberry Pi."

source /opt/ros/noetic/setup.bash
source ~/KW076/Autonomous-drone-upgrade/devel/setup.bash