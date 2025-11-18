#!/usr/bin/env bash
set -euo pipefail

echo "--- Setting up ROS Network Configuration for Onboard (Pi) Computer ---"

# Hardcoded Pi (onboard) IP configuration
# Edit these values to match your Pi before running this script.
ONBOARD_IP="128.189.246.106"   # IP address of the Raspberry Pi (ROS master)

echo "Using hardcoded Onboard IP Address: $ONBOARD_IP"

# Define ROS environment variables (Pi acts as ROS master)
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_IP="$ONBOARD_IP"

echo "Adding/Updating ROS environment variables in ~/.bashrc..."

# Backup ~/.bashrc (timestamped) then remove any previous ROS entries
if [ -f "$HOME/.bashrc" ]; then
  cp -f "$HOME/.bashrc" "$HOME/.bashrc.rviz_backup_$(date +%s)" || true
fi
sed -i.bak '/^export ROS_MASTER_URI=/d' "$HOME/.bashrc" 2>/dev/null || true
sed -i.bak '/^export ROS_IP=/d' "$HOME/.bashrc" 2>/dev/null || true

# Append new values
printf "%s\n" "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> "$HOME/.bashrc"
printf "%s\n" "export ROS_IP=\"$ROS_IP\"" >> "$HOME/.bashrc"

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"
echo "Configuration saved to ~/.bashrc. Please run 'source ~/.bashrc' or open a new terminal." 
echo "Setup complete for Onboard Computer."
