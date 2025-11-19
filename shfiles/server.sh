#!/bin/bash

echo "--- Setting up ROS Network Configuration for Raspberry Pi (Onboard Computer) ---"

# ===== EDIT THESE VALUES FOR YOUR NETWORK =====
ONBOARD_IP="128.189.245.13"          # Pi's IP address
ROS_HOSTNAME="128.189.245.13"        # Pi's hostname or IP (usually same as ONBOARD_IP)
# ===== END CONFIGURATION =====

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: ONBOARD_IP is not set. Please edit the script and set your Pi's IP address."
    exit 1
fi

echo "Configured Raspberry Pi IP Address: $ONBOARD_IP"

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