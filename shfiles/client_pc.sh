#!/usr/bin/env bash
set -euo pipefail

# Hardcoded network configuration
# Edit these values to match your network before running the script.
ONBOARD_IP="128.189.246.106"    # IP of the Raspberry Pi (ROS master)
ONBOARD_ALIAS="icondrone"      # alias to add to /etc/hosts on this PC
GROUND_IP="206.87.209.35"      # IP of this ground station (PC)

echo "Configuring ground station (PC) to use ROS master at $ONBOARD_IP"
echo "Ground station IP: $GROUND_IP, Onboard alias: $ONBOARD_ALIAS"

# Prepare env vars
ROS_MASTER_URI="http://$ONBOARD_IP:11311"
ROS_IP="$GROUND_IP"

# Backup ~/.bashrc before modifying
if [ -f "$HOME/.bashrc" ]; then
    cp -f "$HOME/.bashrc" "$HOME/.bashrc.rviz_backup_$(date +%s)" || true
fi

echo "Updating ~/.bashrc (backup created) -- adding ROS_MASTER_URI and ROS_IP"
sed -i.bak '/^export ROS_MASTER_URI=/d' "$HOME/.bashrc" 2>/dev/null || true
sed -i.bak '/^export ROS_IP=/d' "$HOME/.bashrc" 2>/dev/null || true
printf "%s\n" "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" >> "$HOME/.bashrc"
printf "%s\n" "export ROS_IP=\"$ROS_IP\"" >> "$HOME/.bashrc"

echo "Wrote to ~/.bashrc:" 
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"
echo "  ROS_IP=$ROS_IP"

# Update /etc/hosts for onboard alias (safe: backup then replace)
echo "Updating /etc/hosts to map $ONBOARD_ALIAS -> $ONBOARD_IP (requires sudo)"
sudo cp /etc/hosts "/etc/hosts.rviz_backup_$(date +%s)" || true
sudo sed -i.bak "/\s$ONBOARD_ALIAS$/d" /etc/hosts 2>/dev/null || true
echo "$ONBOARD_IP $ONBOARD_ALIAS" | sudo tee -a /etc/hosts > /dev/null

echo "Added entry: $ONBOARD_IP $ONBOARD_ALIAS to /etc/hosts"
echo "Done. To apply changes now, run: source ~/.bashrc"
echo "Verify with: rostopic list"
