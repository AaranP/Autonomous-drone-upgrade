#!/bin/bash

docker run --privileged --rm tonistiigi/binfmt --install all

chmod +x shfiles/server.sh

# --- IP Selection Logic for ROS Communication on Raspberry Pi ---
PI_DIRECT_IP=""
PI_TAILSCALE_IP=""
DRONE_ROS_IP=""

echo "--- Detecting Raspberry Pi Network Interfaces ---"

# Get all IPv4 addresses assigned to the Raspberry Pi
all_ips=$(hostname -I)

# Loop through IPs to find the direct and Tailscale IPs
for ip in $all_ips; do
    # Tailscale IPs are typically in the 100.x.x.x range
    if [[ $ip == 100.* ]]; then
        PI_TAILSCALE_IP=$ip
    # Any other IP is considered the direct connection
    else
        PI_DIRECT_IP=$ip
    fi
done

# If no direct IP was found, it might be the only one, so check all_ips again
if [ -z "$PI_DIRECT_IP" ] && [ -n "$all_ips" ]; then
    PI_DIRECT_IP=$(echo "$all_ips" | awk '{print $1}')
fi

# --- Debugging: Show the detected IPs ---
echo "--- Detected IPs ---"
echo "Direct IP found: ${PI_DIRECT_IP:-None}"
echo "Tailscale IP found: ${PI_TAILSCALE_IP:-None}"
echo "--------------------"

# Check if at least one IP was found
if [ -z "$PI_DIRECT_IP" ] && [ -z "$PI_TAILSCALE_IP" ]; then
    echo "Error: No network interfaces found. Please check your connection."
    exit 1
fi

# --- User Selection ---
while true; do
    echo "Choose connection method for the Drone's ROS Master:"

    # Store which options are valid to make logic simpler
    direct_valid="false"
    tailscale_valid="false"
    prompt_options=""

    # Option 1: Direct IP
    if [ -n "$PI_DIRECT_IP" ]; then
        echo "1. Direct (Non-Tailscale) IP: ${PI_DIRECT_IP}"
        direct_valid="true"
        prompt_options="Direct"
    fi

    # Option 2: Tailscale IP
    if [ -n "$PI_TAILSCALE_IP" ]; then
        echo "2. Tailscale VPN IP: ${PI_TAILSCALE_IP}"
        tailscale_valid="true"
        if [ -n "$prompt_options" ]; then
            prompt_options+=", Tailscale"
        else
            prompt_options="Tailscale"
        fi
    fi

    # Determine default choice
    default_choice="1"
    if [ "$direct_valid" = "false" ] && [ "$tailscale_valid" = "true" ]; then
        default_choice="2"
    fi
    
    read -p "Enter choice (${prompt_options}, default $default_choice): " choice
    choice=${choice:-$default_choice} # Default to choice if no input

    # --- CORRECTED LOGIC ---
    # Use explicit string comparison for maximum shell compatibility.
    if [ "$choice" = "1" ] && [ "$direct_valid" = "true" ]; then
        DRONE_ROS_IP="${PI_DIRECT_IP}"
        break
    elif [ "$choice" = "2" ] && [ "$tailscale_valid" = "true" ]; then
        DRONE_ROS_IP="${PI_TAILSCALE_IP}"
        break
    else
        echo "Invalid choice. Please try again."
    fi
done

if [ -z "$DRONE_ROS_IP" ]; then
    echo "Error: No ROS IP selected. Exiting."
    exit 1
fi

echo "--- Using IP ${DRONE_ROS_IP} for ROS communication ---"

# --- Run Docker Container ---
echo "--- Starting Docker Drone Container ---"
docker run -it --rm \
    --name fast_drone_container \
    --privileged \
    --network=host \
    -e DRONE_ROS_IP="${DRONE_ROS_IP}" \
    -e ROS_MASTER_URI="http://${DRONE_ROS_IP}:11311" \
    -e ROS_IP="${DRONE_ROS_IP}" \
    -v /dev:/dev \
    # --- CORRECTED VOLUME MOUNTS ---
    # The path inside the container (after the colon) should not have the 'fastdrone' subdirectory.
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/config:/root/catkin_ws/src/realflight_modules/VINS-Fusion/config" \
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/vins_estimator/launch:/root/catkin_ws/src/realflight_modules/VINS-Fusion/vins_estimator/launch" \
    -v "$(pwd)/src/planner/plan_manage/launch:/root/catkin_ws/src/planner/plan_manage/launch" \
    -v "$(pwd)/shfiles:/root/shfiles" \
    -v "$(pwd)/vins_output:/root/vins_output" \
    # --- CORRECTED IMAGE NAME AND TAG ---
    fastdrone_image_pi:latest-arm64 \
    /root/shfiles/server.sh # Execute the server setup script
    
#Opens another terminal in the docker session (this line will only run if the above docker run command exits)
#docker exec -it fast_drone_container /bin/bash
