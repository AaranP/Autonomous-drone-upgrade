#!/bin/bash

echo "--- Setting up ROS Network Configuration for Linux Ground Station ---"

# ===== EDIT THESE VALUES FOR YOUR NETWORK =====
ONBOARD_IP="128.189.245.13"          # Pi's IP address
ONBOARD_HOSTNAME_ALIAS="raspberrypi"   # Pi's hostname alias (optional, for /etc/hosts)
GROUND_IP="206.87.209.35"            # Ground Station (PC) IP address
# ===== END CONFIGURATION =====

if [ -z "$ONBOARD_IP" ] || [ -z "$GROUND_IP" ]; then
    echo "Error: ONBOARD_IP and GROUND_IP must be set. Please edit the script."
    exit 1
fi

echo "Configured Raspberry Pi IP Address: $ONBOARD_IP"
echo "Configured Ground Station IP Address: $GROUND_IP"

# Clean up all ROS environment variables from current shell
unset ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_PACKAGE_PATH
unset ROS_DISTRO
unset ROS_ROOT

# Remove ROS_HOSTNAME from ~/.bashrc if it exists
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Define ROS environment variables to connect to the Pi's ROS Master
export ROS_MASTER_URI="http://$ONBOARD_IP:11311"
export ROS_IP="$GROUND_IP"

# Add/Update ROS environment variables in .bashrc for persistence
echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Remove existing ROS_MASTER_URI, ROS_IP, ROS_HOSTNAME lines to avoid duplicates
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_IP=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc

# Add new lines
echo "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> ~/.bashrc
echo "export ROS_IP=\"$ROS_IP\"" >> ~/.bashrc

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"

# Add/Update entry in /etc/hosts for the onboard computer (optional)
if [ -n "$ONBOARD_HOSTNAME_ALIAS" ]; then
    echo "Adding/Updating entry for '$ONBOARD_HOSTNAME_ALIAS' in /etc/hosts..."
    # Remove existing entry for the alias if it exists
    sudo sed -i "/\s$ONBOARD_HOSTNAME_ALIAS$/d" /etc/hosts
    # Add the new entry
    echo "$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS" | sudo tee -a /etc/hosts > /dev/null
    echo "Added '$ONBOARD_IP $ONBOARD_HOSTNAME_ALIAS' to /etc/hosts."
fi

echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal."
echo "Setup complete for Linux Ground Station."

source /opt/ros/noetic/setup.bash
source ~/kw076/Autonomous-drone-upgrades/devel/setup.bash