#!/bin/bash

echo "--- Setting up ROS Network Configuration for Raspberry Pi (Onboard Computer) ---"

# Get the primary IP address of the Raspberry Pi
# Using 'hostname -I' for Linux (common on Pi)
ONBOARD_IP=$(hostname -I | awk '{print $1}')

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not determine Raspberry Pi's IP address. Please ensure network is connected."
    exit 1
fi

echo "Detected Raspberry Pi IP Address: $ONBOARD_IP"

# Define ROS environment variables to point to itself (as ROS Master)
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_HOSTNAME="$ONBOARD_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
# Use 'sed -i' for Linux (common on Pi)
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Raspberry Pi."