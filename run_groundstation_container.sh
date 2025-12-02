#on window install VcXsrv and disable access control
#On m1 macs Open a terminal within XQuartz (Applications -> Utilities -> Terminal) and run xhost +.


#!/bin/bash
chmod +x "$0"  # Ensure script is executable (safe to run multiple times)

# The 'chmod +x run_groundstation_container.sh' should be run once from your terminal, not inside the script itself.
# Removed: chmod +x run_groundstation_container.sh

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest" # Assuming 'latest' is the default if not specified here
CONTAINER_NAME="fastdrone_groundstation_container"
DRONE_HOSTNAME="ledrone" # Expected Tailscale hostname of the Raspberry Pi

# --- X11 Forwarding Setup (for GUI applications like Rviz, PlotJuggler) ---
X_DISPLAY="" # Will be set by OS detection

# Variables to store potential IPs for the ground station host
GS_DIRECT_IP=""
GS_TAILSCALE_IP=""

# Final IPs to be used for ROS communication, will be determined by user choice
FINAL_GROUND_STATION_IP=""
FINAL_RASPBERRY_PI_IP=""

# --- IP Validation Function ---
# Function to validate if an input string is a valid IP address
is_valid_ip() {
    local ip=$1
    # Regex to match IPv4 addresses (simplified for common local/Tailscale IPs)
    # Allows 192.168.x.x, 10.x.x.x, 172.16-31.x.x, 100.x.x.x, and the 206.87.212/216.x range
if [[ "$ip" =~ ^(192\.168\.|10\.|172\.(1[6-9]|2[0-9]|3[0-1])\.)([0-9]{1,3}\.){1}[0-9]{1,3}$ ]] || \
       [[ "$ip" =~ ^100\.([0-9]{1,3}\.){2}[0-9]{1,3}$ ]] || \
       [[ "$ip" =~ ^206\.87\.21[26]\.[0-9]{1,3}$ ]]; then
        return 0 # Valid IP
    else
        return 1 # Invalid IP
    fi
}


# --- Detect OS and set variables ---
CURRENT_OS_TYPE=""

# macOS Host (M1 Mac or Intel Mac)
if [[ "$OSTYPE" == "darwin"* ]]; then
    CURRENT_OS_TYPE="macos"
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
    CURRENT_OS_TYPE="wsl"
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
    CURRENT_OS_TYPE="linux"
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
    CURRENT_OS_TYPE="unknown"
    echo "WARNING: Could not determine OS type. GUI applications and ROS communication may not work."
    echo "Please manually set X_DISPLAY and GROUND_STATION_HOST_IP before running this script if needed."
fi

# --- X11 Docker Volume (Unix socket only for native Linux) ---
X11_VOLUME=""
XAUTHORITY_VOLUME="" # Initialize for all OS types

if [ "$CURRENT_OS_TYPE" == "linux" ]; then
    echo "Configuring X11 forwarding for native Linux host..."
    # Ensure X server allows connection from local Docker processes
    xhost +local:docker # Allow Docker to connect to your X server
    
    X11_VOLUME="-v /tmp/.X11-unix:/tmp/.X11-unix"
    
    # --- XAUTHORITY SETUP FOR LINUX ---
    # Create a temporary Xauthority file for the container if xauth is available
    if command -v xauth &> /dev/null; then
        XAUTH_HOST_PATH="${HOME}/.docker.xauth" # Use HOME for write permissions
        touch "${XAUTH_HOST_PATH}" # Ensure it exists
        # Merge the host's current display authorization into the temporary file
        xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH_HOST_PATH}" nmerge -
        XAUTHORITY_VOLUME="-v ${XAUTH_HOST_PATH}:${XAUTH_HOST_PATH} -e XAUTHORITY=${XAUTH_HOST_PATH}"
        echo "XAUTHORITY file '${XAUTH_HOST_PATH}' created and mounted."
    else
        echo "WARNING: 'xauth' command not found on your Linux host. Xauthority forwarding cannot be used, which might lead to display issues."
        echo "Please install 'xauth' (e.g., 'sudo apt install x11-xserver-utils' on Ubuntu) if problems persist."
    fi
    # --- END XAUTHORITY SETUP ---
fi

# Final checks after OS detection
if [ -z "$X_DISPLAY" ]; then
    echo "WARNING: X_DISPLAY is not set. GUI applications may not work."
fi

# --- Configure ping command flags based on OS ---
PING_CMD_ARGS="-c 1"
if [ "$CURRENT_OS_TYPE" == "macos" ]; then
    PING_CMD_ARGS+=" -t 1" # macOS uses -t for timeout
else # For Linux/WSL or unknown, assume -W for timeout
    PING_CMD_ARGS+=" -W 1" # Linux/WSL uses -W for timeout
fi


# --- Select Ground Station's own ROS IP (your Mac/PC) ---
echo ""
echo "--- ROS Network Configuration for GROUND STATION (Your Mac/PC) ---"
if [ -n "$GS_DIRECT_IP" ]; then
    echo "Your Ground Station's detected Direct IP:     ${GS_DIRECT_IP}"
fi
if [ -n "$GS_TAILSCALE_IP" ]; then
    echo "Your Ground Station's detected Tailscale IP:  ${GS_TAILSCALE_IP}"
fi
echo ""

# Ensure at least one IP is available to choose from for Ground Station
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

    # Dynamically format the options for the prompt
    formatted_choices_nums=""
    first_option=true
    for i in "${!options[@]}"; do
        choice_num_only=$(echo "${options[$i]}" | cut -d'.' -f1)
        if [ "$first_option" = true ]; then
            formatted_choices_nums="$choice_num_only"
            first_option=false
        else
            formatted_choices_nums+=", $choice_num_only"
        fi
    done
    read -p "Enter choice (${formatted_choices_nums}, default $default_choice): " choice
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

# --- Determine Raspberry Pi's Target IP ---
FINAL_RASPBERRY_PI_IP=""
echo ""
echo "--- Determining Raspberry Pi's ROS Master IP ---"

# Attempt to automatically resolve Pi's Tailscale IP if GS is also using Tailscale
if [ -n "$GS_TAILSCALE_IP" ] && [ "$FINAL_GROUND_STATION_IP" == "$GS_TAILSCALE_IP" ]; then
    echo "Ground Station is using Tailscale. Attempting to resolve Raspberry Pi by Tailscale hostname (${DRONE_HOSTNAME}) using Tailscale DNS..."
    RESOLVED_PI_TS_IP=$(dig @100.100.100.100 +short A "$DRONE_HOSTNAME" 2>/dev/null | grep '^100\.' | head -n 1)

    if [ -n "$RESOLVED_PI_TS_IP" ]; then
        echo "Resolved ${DRONE_HOSTNAME} to ${RESOLVED_PI_TS_IP}. Attempting to ping..."
        if ping $PING_CMD_ARGS "$RESOLVED_PI_TS_IP" &> /dev/null; then
            echo "Successfully resolved and pinged ${DRONE_HOSTNAME} (${RESOLVED_PI_TS_IP}) via Tailscale DNS."
            FINAL_RASPBERRY_PI_IP="$RESOLVED_PI_TS_IP"
        else
            echo "Resolved ${DRONE_HOSTNAME} to ${RESOLVED_PI_TS_IP} but failed to ping that IP. Connectivity issue. Please check Pi's Tailscale status."
            echo "Falling back to manual Raspberry Pi IP entry."
        fi
    else
        echo "Failed to resolve ${DRONE_HOSTNAME} to a Tailscale IP using Tailscale DNS."
        echo "Falling back to manual Raspberry Pi IP entry."
    fi
else
    echo "Ground Station is not using Tailscale or Tailscale client not found. Automatic hostname resolution for Pi will not be attempted."
    echo "Falling back to manual Raspberry Pi IP entry."
fi


# If automatic resolution failed, ask for manual IP
if [ -z "$FINAL_RASPBERRY_PI_IP" ]; then
    echo ""
    echo "Automatic resolution of Raspberry Pi's IP failed or was not attempted."
    echo "Please manually enter the Raspberry Pi's IP address."

    if [ "$FINAL_GROUND_STATION_IP" == "$GS_TAILSCALE_IP" ]; then
        # GS is using Tailscale, so Pi should ideally be Tailscale IP
        while true; do
            read -p "Enter Raspberry Pi's Tailscale IP (e.g., 100.x.y.z): " input_pi_ip
            if is_valid_ip "$input_pi_ip"; then
                FINAL_RASPBERRY_PI_IP="$input_pi_ip"
                break
            else
                echo "Invalid IP address format. Please enter a valid Tailscale IP (e.g., 100.x.y.z)."
            fi
        done
    elif [ -n "$GS_DIRECT_IP" ]; then # Only offer direct if GS has a direct IP
        # GS is using Direct, so Pi should ideally be Direct IP
        while true; do
            read -p "Enter Raspberry Pi's Direct IP (e.g., 206.87.216.212): " input_pi_ip
            if is_valid_ip "$input_pi_ip"; then
                FINAL_RASPBERRY_PI_IP="$input_pi_ip"
                break
            else
                echo "Invalid IP address format. Please enter a valid Direct IP."
            fi
        done
    else
        # Fallback if no valid method to connect to Pi
        echo "ERROR: Ground Station cannot determine expected Pi IP type or no connection method available. Exiting."
        exit 1
    fi
fi

if [ -z "$FINAL_RASPBERRY_PI_IP" ]; then
    echo "ERROR: Raspberry Pi's ROS IP was not set. Exiting."
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
    ${X11_VOLUME} \
    ${XAUTHORITY_VOLUME} \  # Add this line to pass XAUTHORITY
    -v "$(pwd)/shfiles:/root/shfiles" \
    --add-host "${DRONE_HOSTNAME}:${FINAL_RASPBERRY_PI_IP}" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && source /root/shfiles/client.sh && /bin/bash"