#on window install VcXsrv and disable access control
#On m1 macs Open a terminal within XQuartz (Applications -> Utilities -> Terminal) and run xhost +.


#!/bin/bash

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
#IMAGE_TAG="latest" # Assuming 'latest' is the default if not specified here
CONTAINER_NAME="fastdrone_groundstation_container"

# --- X11 Forwarding Setup (for GUI applications like Rviz, PlotJuggler) ---
X_DISPLAY="" # Will be set by OS detection

# Variables to store potential IPs for the ground station host
GS_DIRECT_IP=""
GS_TAILSCALE_IP=""

# Final IPs to be used for ROS communication, will be determined by user choice
FINAL_GROUND_STATION_IP=""
FINAL_RASPBERRY_PI_IP=""

# --- Detect OS and set variables ---

# macOS Host (M1 Mac or Intel Mac)
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Detected macOS host."
    # Ensure XQuartz is installed and running, and 'xhost +' has been run in an XQuartz terminal.
    X_DISPLAY="host.docker.internal:0" # For XQuartz on Docker Desktop
    
    # Get Mac's Direct IP
    GS_DIRECT_IP=$(ipconfig getifaddr en0 2>/dev/null)
    if [ -z "$GS_DIRECT_IP" ]; then
        echo "WARNING: Could not determine Mac's direct IP from 'en0'. Check your network connection."
    fi

    # Try to get Mac's Tailscale IP
    if command -v tailscale &> /dev/null; then
        GS_TAILSCALE_IP=$(tailscale ip -4 2>/dev/null)
        if [ -z "$GS_TAILSCALE_IP" ]; then
            echo "WARNING: Tailscale is installed but not active or could not get IP. Ensure 'tailscale up' is run on your Mac."
        fi
    else
        echo "INFO: Tailscale client not found on macOS host. Tailscale option will be unavailable."
    fi


# Windows Host (via WSL2 and Docker Desktop)
elif [[ -f /proc/version && "$(grep -i microsoft /proc/version)" != "" ]]; then
    echo "Detected Windows host (running via WSL2)."
    X_DISPLAY="$(ip route show default | awk '/default via/ {print $3}'):0" # For VcXsrv
    
    # Get WSL host's Direct IP
    GS_DIRECT_IP="$(ip route show default | awk '/default via/ {print $3}')"
    if [ -z "$GS_DIRECT_IP" ]; then
        echo "WARNING: Could not determine WSL host direct IP. Ensure Docker Desktop and WSL2 are running correctly."
    fi

    # Try to get Windows/WSL's Tailscale IP
    if command -v tailscale &> /dev/null; then
        GS_TAILSCALE_IP=$(tailscale ip -4 2>/dev/null)
        if [ -z "$GS_TAILSCALE_IP" ]; then
            echo "WARNING: Tailscale is installed in WSL but not active or could not get IP. Ensure 'tailscale up' is run in WSL."
        fi
    else
        echo "INFO: Tailscale client not found in WSL. Tailscale option will be unavailable."
    fi

# Native Linux Host (non-WSL)
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Detected native Linux host."
    xhost +local:docker # Allow Docker to connect to your X server
    X_DISPLAY=":0"
    
    # Get Linux's Direct IP
    GS_DIRECT_IP=$(hostname -I | awk '{print $1}' | head -n 1) # Use head -n 1 to get only the first IP
    if [ -z "$GS_DIRECT_IP" ]; then
        GS_DIRECT_IP=$(ip route get 1.1.1.1 | awk '{print $7; exit}')
    fi
    if [ -z "$GS_DIRECT_IP" ]; then
        echo "WARNING: Could not determine Linux host direct IP. Check network connection."
    fi

    # Try to get Linux's Tailscale IP
    if command -v tailscale &> /dev/null; then
        GS_TAILSCALE_IP=$(tailscale ip -4 2>/dev/null)
        if [ -z "$GS_TAILSCALE_IP" ]; then
            echo "WARNING: Tailscale is installed but not active or could not get IP. Ensure 'tailscale up' is run on your Linux host."
        fi
    else
        echo "INFO: Tailscale client not found on Linux host. Tailscale option will be unavailable."
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

# --- IP Selection Logic for ROS Communication ---
echo ""
echo "--- ROS Network Configuration for GROUND STATION (Your Mac/PC) ---"
if [ -n "$GS_DIRECT_IP" ]; then
    echo "Your Ground Station's detected Direct IP:     ${GS_DIRECT_IP}"
fi
if [ -n "$GS_TAILSCALE_IP" ]; then
    echo "Your Ground Station's detected Tailscale IP:  ${GS_TAILSCALE_IP}"
fi
echo ""

# Ensure at least one IP is available to choose from
if [ -z "$GS_DIRECT_IP" ] && [ -z "$GS_TAILSCALE_IP" ]; then
    echo "ERROR: No usable IP addresses detected for Ground Station. Exiting."
    exit 1
fi

while true; do
    echo "Choose which IP to use for *this* Ground Station (your Mac/Linux PC):"

    options=()
    if [ -n "$GS_DIRECT_IP" ]; then
        options+=("1. Direct (Non-Tailscale) IP")
    fi
    if [ -n "$GS_TAILSCALE_IP" ]; then
        options+=("2. Tailscale VPN IP")
    fi

    for opt in "${options[@]}"; do
        echo "$opt"
    done

    default_choice="1"
    if [ -z "$GS_DIRECT_IP" ] && [ -n "$GS_TAILSCALE_IP" ]; then # If direct not available, default to tailscale
        default_choice="2"
    fi

    read -p "Enter choice ($(echo "${options[@]}" | sed 's/^[0-9]\. /' | sed 's/[0-9]\. /, /g'), default $default_choice): " choice
    choice=${choice:-$default_choice} # Default if no input

    if [ "$choice" == "1" ] && [ -n "$GS_DIRECT_IP" ]; then
        FINAL_GROUND_STATION_IP="${GS_DIRECT_IP}"
        break
    elif [ "$choice" == "2" ] && [ -n "$GS_TAILSCALE_IP" ]; then
        FINAL_GROUND_STATION_IP="${GS_TAILSCALE_IP}"
        break
    else
        echo "Invalid choice. Please try again."
    fi
done

# Prompt for Raspberry Pi's IP (user must get this from Pi's run_container.sh output)
echo ""
echo "Now, please get the Raspberry Pi's chosen ROS IP from its 'run_container.sh' output."
while [ -z "$FINAL_RASPBERRY_PI_IP" ]; do
    read -p "Enter Raspberry Pi's chosen ROS IP (e.g., 206.87.216.212 or 100.x.y.z): " input_pi_ip
    # Basic IP validation for common ranges, including your school network example and Tailscale
    if [[ "$input_pi_ip" =~ ^(192\.168\.|10\.|172\.(1[6-9]|2[0-9]|3[0-1])\.)([0-9]{1,3}\.){1}[0-9]{1,3}$ ]] || \
       [[ "$input_pi_ip" =~ ^100\.([0-9]{1,3}\.){2}[0-9]{1,3}$ ]] || \
       [[ "$input_pi_ip" =~ ^206\.87\.21[26]\.[0-9]{1,3}$ ]]; then # Adjusted for your specific school network example
        FINAL_RASPBERRY_PI_IP="$input_pi_ip"
    else
        echo "Invalid IP format. Please ensure it's a valid local, Tailscale, or school network IP."
    fi
done

if [ -z "$FINAL_GROUND_STATION_IP" ] || [ -z "$FINAL_RASPBERRY_PI_IP" ]; then
    echo "ERROR: IP addresses for ROS communication were not properly set. Exiting."
    exit 1
fi

echo "--- Final ROS Configuration for Ground Station ---"
echo "Ground Station ROS_IP: ${FINAL_GROUND_STATION_IP}"
echo "Raspberry Pi (ROS_MASTER_URI) IP: ${FINAL_RASPBERRY_PI_IP}"
echo "---"

# --- Run the Docker Container ---
echo "Starting Docker Ground Station Container..."

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network=host \
    -e DISPLAY="${X_DISPLAY}" \
    -e GROUND_STATION_HOST_IP="${FINAL_GROUND_STATION_IP}" \
    -e RASPBERRY_PI_TARGET_IP="${FINAL_RASPBERRY_PI_IP}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/shfiles:/root/shfiles" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && source /root/shfiles/client.sh && /bin/bash"
