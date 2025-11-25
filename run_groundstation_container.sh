#on window install VcXsrv and disable access control
#On m1 macs Open a terminal within XQuartz (Applications -> Utilities -> Terminal) and run xhost +.


#!/bin/bash

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest" # Uncommented this line
CONTAINER_NAME="fastdrone_groundstation_container"

# --- IMPORTANT: Set these variables ---
# Replace with the actual IP address of your Raspberry Pi (onboard computer)
# You can find this by running 'hostname -I | awk '{print $1}'' on the Pi.
# ONBOARD_PI_IP="<YOUR_RASPBERRY_PI_IP>" # No longer used for ROS config in container

# Automatically detect the IP address of the ground station PC
# This is used for ROS_IP, so the Pi knows how to send data back to this PC.
# GROUND_STATION_IP=$(hostname -I | awk '{print $1}') # No longer used for ROS config in container

# --- X11 Forwarding Setup (for GUI applications like Rviz, PlotJuggler) ---
# This part is platform-dependent. Choose the appropriate section below.

# Default DISPLAY variable (will be overridden by platform-specific logic)
X_DISPLAY=""

# --- Linux Host (non-WSL) ---
# Check if not running in WSL
if [[ "$OSTYPE" == "linux-gnu"* && ! -f /proc/version || "$(grep -i microsoft /proc/version)" == "" ]]; then
    echo "Detected native Linux host."
    xhost +local:docker # Allow Docker to connect to your X server
    X_DISPLAY=":0"
fi

# --- macOS Host (M1 Mac or Intel Mac) ---
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Detected macOS host."
    # Ensure XQuartz is installed and running.
    # Open XQuartz, then in its terminal, run 'xhost +'
    # Get the IP address of the host for DISPLAY
    X_DISPLAY="$(ipconfig getifaddr en0):0" # Adjust 'en0' if your primary network interface is different
    # You might need to run 'xhost + <container_ip>' from XQuartz terminal
    # or simply 'xhost +' for broader access.
fi

# --- Windows Host (via WSL2 and Docker Desktop) ---
# Check for WSL2 environment
if [[ -f /proc/version && "$(grep -i microsoft /proc/version)" != "" ]]; then
    echo "Detected Windows host (running via WSL2)."
    # Ensure VcXsrv or Xming is installed and running on Windows.
    # VcXsrv: Launch with "Disable access control" checked.
    # Get the IP address of the WSL2 host (Windows machine)
    # This typically works for WSL2 to connect to VcXsrv running on Windows.
    X_DISPLAY="$(ip route show default | awk '/default via/ {print $3}'):0" # More robust way to get Windows host IP from WSL2
fi

if [ -z "$X_DISPLAY" ]; then
    echo "WARNING: Could not determine X_DISPLAY for your OS type. GUI applications may not work."
    echo "Please manually set X_DISPLAY before running this script if you need GUI."
fi

# Extract just the IP part from X_DISPLAY for X11 forwarding.
# For WSL2, this is the WSL internal gateway (e.g., 172.25.0.1).
# This IP is suitable for X11 forwarding to VcXsrv running on Windows.
WSL_INTERNAL_HOST_IP=""
if [[ -n "$X_DISPLAY" ]]; then
    WSL_INTERNAL_HOST_IP=$(echo "$X_DISPLAY" | cut -d':' -f1)
    echo "Detected WSL Internal Host IP for X11: $WSL_INTERNAL_HOST_IP"
fi

# --- IMPORTANT: Set your Physical Windows PC's IP for ROS_IP ---
# Replace <YOUR_PHYSICAL_WINDOWS_PC_IP> with the IPv4 Address you found in step 1 (e.g., 192.168.5.100).
# This is the IP that your Raspberry Pi will use to send ROS messages back to this ground station.
GROUND_STATION_HOST_IP="192.168.5.99" # <-- **SET THIS TO YOUR ACTUAL PHYSICAL WINDOWS IP**
echo "Using Physical Windows Host IP for ROS_IP: $GROUND_STATION_HOST_IP"

# --- Run the Docker Container ---
echo "Starting Docker Ground Station Container..."
# echo "Connecting to ROS Master at: http://${ONBOARD_PI_IP}:11311" # Handled by client.sh
# echo "Ground Station ROS_IP set to: ${GROUND_STATION_IP}" # Handled by client.sh

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network=host \
    -e DISPLAY="${X_DISPLAY}" \
    -e GROUND_STATION_HOST_IP="${GROUND_STATION_HOST_IP}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/shfiles:/root/shfiles" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && source /root/shfiles/client.sh && /bin/bash"