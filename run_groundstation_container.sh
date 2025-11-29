#on window install VcXsrv and disable access control
#On m1 macs Open a terminal within XQuartz (Applications -> Utilities -> Terminal) and run xhost +.


#!/bin/bash

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
#IMAGE_TAG="latest"
CONTAINER_NAME="fastdrone_groundstation_container"

# --- X11 Forwarding Setup (for GUI applications like Rviz, PlotJuggler) ---
# This part is platform-dependent. Choose the appropriate section below.

# Default DISPLAY variable (will be overridden by platform-specific logic)
X_DISPLAY=""
# Default GROUND_STATION_HOST_IP (will be overridden by platform-specific logic)
GROUND_STATION_HOST_IP=""

# --- Detect OS and set variables ---

# macOS Host (M1 Mac or Intel Mac)
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Detected macOS host."
    # Ensure XQuartz is installed and running, and 'xhost +' has been run in an XQuartz terminal.
    # Set DISPLAY to use host.docker.internal for reliable connection to XQuartz on Docker Desktop.
    X_DISPLAY="host.docker.internal:0"
    
    # Try Tailscale IP first, then fallback to direct device IP.
    GROUND_STATION_HOST_IP=$(tailscale ip -4 2>/dev/null)
    if [ -z "$GROUND_STATION_HOST_IP" ]; then
        echo "WARNING: Could not determine Mac's Tailscale IP. Attempting to use direct device IP (en0)."
        # Fallback to direct IP
        # 'en0' is typical for primary Ethernet or Wi-Fi. Adjust if your active interface is different (e.g., en1, en2, Wi-Fi).
        GROUND_STATION_HOST_IP=$(ipconfig getifaddr en0 2>/dev/null)
        if [ -z "$GROUND_STATION_HOST_IP" ]; then
            echo "ERROR: Could not determine Mac's IP (Tailscale or direct en0). Please ensure Tailscale is running and logged in, or check network connection for 'en0'."
            exit 1
        fi
        echo "Using Mac direct IP: ${GROUND_STATION_HOST_IP}"
    else
        echo "Using Mac Tailscale IP: ${GROUND_STATION_HOST_IP}"
    fi

# Windows Host (via WSL2 and Docker Desktop)
# Check for WSL2 environment by looking for /proc/version containing 'microsoft'
elif [[ -f /proc/version && "$(grep -i microsoft /proc/version)" != "" ]]; then
    echo "Detected Windows host (running via WSL2)."
    # Ensure VcXsrv or Xming is installed and running on Windows with "Disable access control" checked.
    # Get the IP address of the WSL2 host (which is the Windows machine's IP from WSL's perspective).
    X_DISPLAY="$(ip route show default | awk '/default via/ {print $3}'):0"
    
    # Try Tailscale IP first, then fallback to direct WSL host IP.
    GROUND_STATION_HOST_IP=$(tailscale ip -4 2>/dev/null)
    if [ -z "$GROUND_STATION_HOST_IP" ]; then
        echo "WARNING: Could not determine WSL host Tailscale IP. Attempting to use direct WSL host IP."
        # Fallback to direct WSL host IP
        GROUND_STATION_HOST_IP="$(ip route show default | awk '/default via/ {print $3}')" # Use the WSL host IP for ROS
        if [ -z "$GROUND_STATION_HOST_IP" ]; then
            echo "ERROR: Could not determine WSL host IP (Tailscale or direct). Please ensure Docker Desktop and WSL2 are running correctly and Tailscale is running and logged in."
            exit 1
        fi
        echo "Using Windows (WSL) direct IP: ${GROUND_STATION_HOST_IP}"
    else
        echo "Using Windows (WSL) Tailscale IP: ${GROUND_STATION_HOST_IP}"
    fi

# Native Linux Host (non-WSL)
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Detected native Linux host."
    # This command allows Docker to connect to your X server.
    # It must be run on the host *before* the container starts (this script does it).
    xhost +local:docker
    X_DISPLAY=":0"
    
    # Try Tailscale IP first, then fallback to direct device IP.
    GROUND_STATION_HOST_IP=$(tailscale ip -4 2>/dev/null)
    if [ -z "$GROUND_STATION_HOST_IP" ]; then
        echo "WARNING: Could not determine Linux host Tailscale IP. Attempting to use direct device IP."
        # Fallback to direct IP
        GROUND_STATION_HOST_IP=$(hostname -I | awk '{print $1}' | head -n 1) # Use head -n 1 to get only the first IP
        if [ -z "$GROUND_STATION_HOST_IP" ]; then
            # Fallback if hostname -I doesn't work as expected for some reason.
            GROUND_STATION_HOST_IP=$(ip route get 1.1.1.1 | awk '{print $7; exit}')
        fi
        if [ -z "$GROUND_STATION_HOST_IP" ]; then
            echo "ERROR: Could not determine Linux host IP (Tailscale or direct). Please check network connection and ensure Tailscale is running and logged in."
            exit 1
        fi
        echo "Using Linux direct IP: ${GROUND_STATION_HOST_IP}"
    else
        echo "Using Linux Tailscale IP: ${GROUND_STATION_HOST_IP}"
    fi

else
    # Fallback for unknown OS or if detection fails
    echo "WARNING: Could not determine OS type. GUI applications and ROS communication may not work."
    echo "Please manually set X_DISPLAY and GROUND_STATION_HOST_IP before running this script if needed."
fi

# Final checks after OS detection
if [ -z "$X_DISPLAY" ]; then
    echo "WARNING: X_DISPLAY is not set. GUI applications may not work."
fi

if [ -z "$GROUND_STATION_HOST_IP" ]; then
    echo "ERROR: GROUND_STATION_HOST_IP is not set. ROS communication will likely fail."
    exit 1
fi

echo "Using X_DISPLAY: ${X_DISPLAY}"
echo "Using GROUND_STATION_HOST_IP for ROS_IP: ${GROUND_STATION_HOST_IP}"

# --- Run the Docker Container ---
echo "Starting Docker Ground Station Container..."
# The ROS_MASTER_URI and ROS_IP are now set within client.sh, using GROUND_STATION_HOST_IP passed as env var.

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network=host \
    -e DISPLAY="${X_DISPLAY}" \
    -e GROUND_STATION_HOST_IP="${GROUND_STATION_HOST_IP}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/shfiles:/root/shfiles" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && source /root/shfiles/client.sh && /bin/bash"
