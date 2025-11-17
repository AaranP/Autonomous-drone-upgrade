#!/bin/bash

echo "--- Setting up ROS Network Configuration for Onboard Computer ---"

# Get the primary IP address of the onboard computer
# Using 'hostname -I' for Linux, 'ipconfig getifaddr en0' for macOS (though onboard is likely Linux)
ONBOARD_IP=$(hostname -I | awk '{print $1}')

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not determine onboard computer's IP address. Please ensure network is connected."
    exit 1
fi

echo "Detected Onboard IP Address: $ONBOARD_IP"

# Define ROS environment variables
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_HOSTNAME="$ONBOARD_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
# Use 'sed -i' for Linux, 'sed -i ""' for macOS
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Onboard Computer."