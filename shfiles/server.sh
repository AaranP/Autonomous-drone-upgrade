#!/bin/bash

echo "--- Setting up ROS Network Configuration for Raspberry Pi (Onboard Computer) ---"

# ===== AUTOMATICALLY DETECT IP ADDRESS =====
# Get the primary IP address of the current machine
# This command typically returns the IP address(es) of the active network interface(s).
# We take the first one found.
ONBOARD_IP=$(hostname -I | awk '{print $1}')

if [ -z "$ONBOARD_IP" ]; then
    echo "Error: Could not automatically determine the IP address of this machine."
    echo "Please ensure you are connected to a network and try again, or manually set ONBOARD_IP."
    exit 1
fi

ROS_HOSTNAME="$ONBOARD_IP" # Pi's hostname or IP (usually same as ONBOARD_IP)
# ===== END CONFIGURATION =====

echo "Automatically detected IP Address: $ONBOARD_IP"

# Define ROS environment variables to point to itself (as ROS Master)
export ROS_MASTER_URI="http://$ONBOARD_IP:11311" # Use export directly
export ROS_IP="$ONBOARD_IP" # It's good practice for ROS_IP to be set for the master itself
                               # if it's also running other nodes.
                               # If only roscore, ROS_HOSTNAME is more critical.

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to: $ROS_HOSTNAME"
echo "ROS_IP set to: $ROS_IP"

# Source ROS setup files for the current shell session within the container
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash # Assuming your ground station also needs workspace packages

echo "Setup complete for Raspberry pi."

# --- ROS IP Configuration for the Drone (Server) ---
DRONE_IP=""

# Try to get the local network IP first
DRONE_IP=$(hostname -I | awk '{print $1}' | head -n 1)

if [ -z "$DRONE_IP" ]; then
    echo "WARNING: Could not determine Drone's direct IP. Attempting to use Tailscale IP."
    # As the container uses --network=host, `tailscale ip -4` will give the host's (Pi's) Tailscale IP
    DRONE_IP=$(tailscale ip -4 2>/dev/null)
    if [ -z "$DRONE_IP" ]; then
        echo "ERROR: Could not determine Drone's IP (direct or Tailscale). Please ensure Tailscale is running and logged in on the Pi."
        exit 1
    fi
    echo "Using Drone Tailscale IP: ${DRONE_IP}"
else
    echo "Using Drone direct IP: ${DRONE_IP}"
fi

export ROS_IP="${DRONE_IP}"
echo "Set ROS_IP to: ${ROS_IP}"

# --- ROS_MASTER_URI Configuration (Points to Ground Station) ---
# Assuming the Mac's Tailscale hostname is 'mac-groundstation'.
# This relies on Tailscale DNS resolving 'mac-groundstation' to its Tailscale IP,
# or local DNS resolving it to a direct IP if available.
GROUND_STATION_MASTER_HOSTNAME="aaranmac" # Change this if your Mac's Tailscale hostname is different

# Attempt to resolve the Ground Station's IP using the hostname.
# This will pick up local LAN IP (if mDNS works) or Tailscale IP (if Tailscale DNS works).
GROUND_STATION_RESOLVED_IP=$(getent hosts "${GROUND_STATION_MASTER_HOSTNAME}" | awk '{print $1}' | head -n 1)

if [ -z "$GROUND_STATION_RESOLVED_IP" ]; then
    echo "ERROR: Could not resolve IP for ground station hostname '${GROUND_STATION_MASTER_HOSTNAME}'. Please ensure ground station is online and Tailscale/local DNS is working."
    exit 1
fi

export ROS_MASTER_URI="http://${GROUND_STATION_RESOLVED_IP}:11311"
echo "Set ROS_MASTER_URI to: ${ROS_MASTER_URI}"

# You might want to add other server-specific ROS commands here
# For example, launching your drone's ROS nodes
#echo "Starting ROS server..."
# Example: roslaunch fastdrone_bringup drone_system.launch
exec /bin/bash
# --- Start ROS Master ---
#echo "Starting ROS Master (roscore)..."
#exec roscore # 'exec' replaces the current shell with roscore, keeping it in the foreground