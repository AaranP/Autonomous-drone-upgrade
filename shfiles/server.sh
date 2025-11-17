#!/bin/bash

echo "--- Setting up ROS Network Configuration for Ground Station ---"

# Get the primary IP address of the remote computer (assuming macOS for the user)
REMOTE_IP=$(ipconfig getifaddr en0 || hostname -I | awk '{print $1}') # Fallback for Linux

if [ -z "$REMOTE_IP" ]; then
    echo "Error: Could not determine remote computer's IP address. Please ensure network is connected."
    exit 1
fi

echo "Detected Ground Station IP Address: $REMOTE_IP"

# --- Set Onboard Computer details directly here ---
ONBOARD_IP="128.189.246.106" # <--- REPLACE WITH YOUR ONBOARD COMPUTER'S ACTUAL IP (e.g., "192.168.1.100")
ONBOARD_HOSTNAME_ALIAS="icondrone" # <--- REPLACE WITH YOUR DESIRED HOSTNAME ALIAS (e.g., "fast-drone")
# --- End of direct setting ---

# No need for input validation if values are hardcoded, but ensure they are set
if [ -z "$ONBOARD_IP" ] || [ -z "$ONBOARD_HOSTNAME_ALIAS" ]; then
    echo "Error: ONBOARD_IP or ONBOARD_HOSTNAME_ALIAS is not set in the script. Please edit the script."
    exit 1
fi

# Define ROS environment variables
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_HOSTNAME="$REMOTE_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
# Using 'sed -i ""' for macOS compatibility
sed -i "" '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i "" '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"

# Add/Update entry in /etc/hosts for the onboard computer
echo "Adding/Updating entry for '$ONBOARD_HOSTNAME_ALIAS' in /etc/hosts..."
# Remove existing entry for the alias if it exists (using 'sed -i ""' for macOS)
sudo sed -i "" "/\s$ONBOARD_HOSTNAME_ALIAS$/d" /etc/hosts
# Add the new entry
echo "$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS" | sudo tee -a /etc/hosts > /dev/null

echo "Added '$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS' to /etc/hosts."
echo "Configuration saved to ~/.bashrc and /etc/hosts. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Ground Station."