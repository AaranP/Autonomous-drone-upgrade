#!/bin/bash

echo "--- Setting up ROS Network Configuration for Linux Ground Station ---"

# Get the primary IP address of the Linux Ground Station
REMOTE_IP=$(hostname -I | awk '{print $1}')

if [ -z "$REMOTE_IP" ]; then
    echo "Error: Could not determine Ground Station's IP address. Please ensure network is connected."
    exit 1
fi

echo "Detected Ground Station IP Address: $REMOTE_IP"

# --- Set Onboard Computer Hostname Alias directly here ---
# This is the alias you want to use for the Raspberry Pi (e.g., in SSH, ROS)
ONBOARD_HOSTNAME_ALIAS="icondrone" # <--- Ensure this matches the Raspberry Pi's actual hostname or mDNS name
# --- End of direct setting ---

# Attempt to dynamically resolve the ONBOARD_HOSTNAME_ALIAS to an IP address
echo "Attempting to resolve IP for hostname: $ONBOARD_HOSTNAME_ALIAS..."

# Try resolving with .local (mDNS/Bonjour) first
# Using '-W 1' for Linux ping timeout
ONBOARD_IP=$(ping -c 1 -W 1 "${ONBOARD_HOSTNAME_ALIAS}.local" 2>/dev/null | awk '/^PING/{print $3}' | sed 's/[()]//g')

# If .local didn't work, try resolving without it (e.g., if already in /etc/hosts or via DNS)
if [ -z "$ONBOARD_IP" ]; then
    ONBOARD_IP=$(ping -c 1 -W 1 "$ONBOARD_HOSTNAME_ALIAS" 2>/dev/null | awk '/^PING/{print $3}' | sed 's/[()]//g')
fi

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not dynamically resolve IP for hostname '$ONBOARD_HOSTNAME_ALIAS' or '${ONBOARD_HOSTNAME_ALIAS}.local'."
    echo "Please ensure the Raspberry Pi is online and its hostname is resolvable (e.g., via mDNS/Bonjour)."
    echo "You might need to manually add an entry to /etc/hosts or ensure the Raspberry Pi has a static/reserved IP."
    exit 1
fi

echo "Dynamically resolved Raspberry Pi IP Address: $ONBOARD_IP"

# Define ROS environment variables to connect to the Pi's ROS Master
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_HOSTNAME="$REMOTE_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI and ROS_HOSTNAME lines to avoid duplicates
# Use 'sed -i' for Linux
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_HOSTNAME=\"$ROS_HOSTNAME\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"

# Add/Update entry in /etc/hosts for the onboard computer
echo "Adding/Updating entry for '$ONBOARD_HOSTNAME_ALIAS' in /etc/hosts..."
# Remove existing entry for the alias if it exists (using 'sed -i' for Linux)
sudo sed -i "/\s$ONBOARD_HOSTNAME_ALIAS$/d" /etc/hosts
# Add the new entry
echo "$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS" | sudo tee -a /etc/hosts > /dev/null

echo "Added '$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS' to /etc/hosts."
echo "Configuration saved to ~/.bashrc and /etc/hosts. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Linux Ground Station."