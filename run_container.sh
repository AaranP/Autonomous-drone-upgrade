#!/bin/bash

docker run --privileged --rm tonistiigi/binfmt --install all

chmod +x shfiles/server.sh

# --- IP Selection Logic for ROS Communication on Raspberry Pi ---
PI_DIRECT_IP=""
PI_TAILSCALE_IP=""
DRONE_ROS_IP=""

echo "--- Detecting Raspberry Pi Network Interfaces ---"

# Get Pi's Direct IP
# Using 'ip a' which is more modern and robust than 'hostname -I' for multiple IPs
PI_DIRECT_IP=$(ip -4 addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1) # Assumes eth0, adjust if using wlan0 or other
if [ -z "$PI_DIRECT_IP" ]; then
    echo "WARNING: Could not determine Raspberry Pi's direct IP from eth0. Trying wlan0."
    PI_DIRECT_IP=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)
fi
if [ -z "$PI_DIRECT_IP" ]; then
    echo "WARNING: Could not determine Raspberry Pi's direct IP. Check network connection."
fi

# Get Pi's Tailscale IP
if command -v tailscale &> /dev/null; then
    PI_TAILSCALE_IP=$(tailscale ip -4)
    if [ -z "$PI_TAILSCALE_IP" ]; then
        echo "WARNING: Tailscale is installed but not active or could not get IP. Ensure 'sudo tailscale up' is run."
    fi
else
    echo "INFO: Tailscale client not found on Raspberry Pi host. Tailscale option will be unavailable."
fi

echo ""
echo "--- ROS Network Configuration for DRONE (Raspberry Pi) ---"
if [ -n "$PI_DIRECT_IP" ]; then
    echo "Raspberry Pi's detected Direct IP:     ${PI_DIRECT_IP}"
fi
if [ -n "$PI_TAILSCALE_IP" ]; then
    echo "Raspberry Pi's detected Tailscale IP:  ${PI_TAILSCALE_IP}"
fi
echo ""

# Ensure at least one IP is available to choose from
if [ -z "$PI_DIRECT_IP" ] && [ -z "$PI_TAILSCALE_IP" ]; then
    echo "ERROR: No usable IP addresses detected for Raspberry Pi. Exiting."
    exit 1
fi

while true; do
    echo "Choose connection method for the Drone's ROS Master:"
    
    options=()
    if [ -n "$PI_DIRECT_IP" ]; then
        options+=("1. Direct (Non-Tailscale) IP")
    fi
    if [ -n "$PI_TAILSCALE_IP" ]; then
        options+=("2. Tailscale VPN IP")
    fi

    for opt in "${options[@]}"; do
        echo "$opt"
    done

    default_choice="1"
    if [ -z "$PI_DIRECT_IP" ] && [ -n "$PI_TAILSCALE_IP" ]; then # If direct not available, default to tailscale
        default_choice="2"
    fi
    
    read -p "Enter choice ($(echo "${options[@]}" | sed 's/^[0-9]\. /' | sed 's/[0-9]\. /, /g'), default $default_choice): " choice
    choice=${choice:-$default_choice} # Default to 1 if no input

    if [ "$choice" == "1" ] && [ -n "$PI_DIRECT_IP" ]; then
        DRONE_ROS_IP="${PI_DIRECT_IP}"
        break
    elif [ "$choice" == "2" ] && [ -n "$PI_TAILSCALE_IP" ]; then
        DRONE_ROS_IP="${PI_TAILSCALE_IP}"
        break
    else
        echo "Invalid choice. Please try again."
    fi
done

if [ -z "$DRONE_ROS_IP" ]; then
    echo "ERROR: Drone's ROS IP was not set. Exiting."
    exit 1
fi

echo "--- Final Drone ROS Configuration ---"
echo "Drone ROS_MASTER_URI will be: http://${DRONE_ROS_IP}:11311"
echo "Drone ROS_IP will be: ${DRONE_ROS_IP}"
echo "--------------------------------------------------------------------------------"
echo "IMPORTANT: On your Ground Station, use THIS IP for ROS_MASTER_URI: ${DRONE_ROS_IP}"
echo "--------------------------------------------------------------------------------"

echo "--- Starting Docker Drone Container ---"
docker run -it --rm \
    --name fast_drone_container \
    --privileged \
    --network=host \
    -e DRONE_ROS_IP="${DRONE_ROS_IP}" \
    -e ROS_MASTER_URI="http://${DRONE_ROS_IP}:11311" \
    -e ROS_IP="${DRONE_ROS_IP}" \
    -v /dev:/dev \
    -v "$(pwd)/src/fastdrone/config:/root/catkin_ws/src/fastdrone/config" \
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/config:/root/catkin_ws/src/fastdrone/src/realflight_modules/VINS-Fusion/config" \
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/vins_estimator/launch:/root/catkin_ws/src/fastdrone/src/realflight_modules/VINS-Fusion/vins_estimator/launch" \
    -v "$(pwd)/src/planner/plan_manage/launch:/root/catkin_ws/src/planner/plan_manage/launch" \
    -v "$(pwd)/shfiles:/root/shfiles" \
    -v "$(pwd)/vins_output:/root/vins_output" \
    fastdrone_image_pi:latest-arm64 \
    /root/shfiles/server.sh # Execute the server setup script
    
#Opens another terminal in the docker session (this line will only run if the above docker run command exits)
#docker exec -it fast_drone_container /bin/bash
