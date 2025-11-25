#on window install VcXsrv and disable access control
#On m1 macs Open a terminal within XQuartz (Applications -> Utilities -> Terminal) and run xhost +.


#!/bin/bash

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest"
CONTAINER_NAME="fastdrone_groundstation_container"

# --- IMPORTANT: Set these variables ---
# Replace with the actual IP address of your Raspberry Pi (onboard computer)
# You can find this by running 'hostname -I | awk '{print $1}'' on the Pi.
ONBOARD_PI_IP="<YOUR_RASPBERRY_PI_IP>" 

# Automatically detect the IP address of the ground station PC
# This is used for ROS_IP, so the Pi knows how to send data back to this PC.
GROUND_STATION_IP=$(hostname -I | awk '{print $1}')

# --- X11 Forwarding Setup (for GUI applications like Rviz, PlotJuggler) ---
# This part is platform-dependent. Choose the appropriate section below.

# Default DISPLAY variable (will be overridden by platform-specific logic)
X_DISPLAY=""

# --- Linux Host ---
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Detected Linux host."
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
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || "$OSTYPE" == "win32" ]]; then
    echo "Detected Windows host (running via Git Bash/WSL)."
    # Ensure VcXsrv or Xming is installed and running on Windows.
    # VcXsrv: Launch with "Disable access control" checked.
    # Get the IP address of the WSL2 host (Windows machine)
    # This typically works for WSL2 to connect to VcXsrv running on Windows.
    X_DISPLAY="$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0"
fi

if [ -z "$X_DISPLAY" ]; then
    echo "WARNING: Could not determine X_DISPLAY for your OS type. GUI applications may not work."
    echo "Please manually set X_DISPLAY before running this script if you need GUI."
fi

# --- Run the Docker Container ---
echo "Starting Docker Ground Station Container..."
echo "Connecting to ROS Master at: http://${ONBOARD_PI_IP}:11311"
echo "Ground Station ROS_IP set to: ${GROUND_STATION_IP}"

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network=host \
    -e DISPLAY="${X_DISPLAY}" \
    -e ROS_MASTER_URI="http://${ONBOARD_PI_IP}:11311" \
    -e ROS_IP="${GROUND_STATION_IP}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/shfiles:/root/shfiles" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && sh /root/shfiles/client.sh && /bin/bash"