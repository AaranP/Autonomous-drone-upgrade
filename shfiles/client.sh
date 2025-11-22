#!/bin/bash

echo "--- Setting up ROS Network Configuration for Linux Ground Station ---"

# ===== CONFIGURATION FOR YOUR NETWORK =====
# The Raspberry Pi's IP address will be automatically determined if its hostname is resolvable.
# If automatic detection fails, you may need to uncomment and manually set ONBOARD_IP.
# ONBOARD_IP="128.189.245.13"

# The hostname alias for your Raspberry Pi. This is used to automatically resolve its IP.
# Common default for Raspberry Pi is 'raspberrypi'. If using mDNS, it might be 'raspberrypi.local'.
ONBOARD_HOSTNAME_ALIAS="raspberrypi"

# Ground Station (PC/Mac) WiFi IP - automatically detected (en0 typical WiFi)
GROUND_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || hostname | cut -d. -f1)
# ===== END CONFIGURATION =====

# Attempt to resolve the Raspberry Pi's IP address from its hostname alias
ONBOARD_IP="" # Initialize to empty
if [ -n "$ONBOARD_HOSTNAME_ALIAS" ]; then
    echo "Attempting to resolve Raspberry Pi WiFi IP from hostname: $ONBOARD_HOSTNAME_ALIAS(.local)"
    # macOS-compatible: ping mDNS/DNS (prioritizes pingable WiFi IP), fallback nslookup
    PING_RSLT=$(ping -c1 -W2 "$ONBOARD_HOSTNAME_ALIAS.local" 2>/dev/null | awk 'NR==2 && /bytes from/ {print $4}' | cut -d: -f1 || \
                ping -c1 -W2 "$ONBOARD_HOSTNAME_ALIAS" 2>/dev/null | awk 'NR==2 && /bytes from/ {print $4}' | cut -d: -f1 || \
                echo "")
    ONBOARD_IP=$(echo "$PING_RSLT" | head -n1 | xargs)
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
source ~/kw076/Autonomous-drone-upgrade/devel/setup.bash