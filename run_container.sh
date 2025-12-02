#!/bin/bash

# --- IP Selection Logic for ROS Communication on Raspberry Pi ---
PI_DIRECT_IP=""
PI_TAILSCALE_IP=""
DRONE_ROS_IP=""

echo "--- Detecting Raspberry Pi Network Interfaces ---"

# --- NEW ROBUST IP DETECTION ---
# Get all non-loopback, non-docker IPv4 addresses
all_ips=$(ip -4 addr | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -vE '^127\.|^172\.17\.')

# Loop through IPs to find the direct and Tailscale IPs
for ip in $all_ips; do
    # Tailscale IPs are typically in the 100.x.x.x range
    if [[ $ip == 100.* ]]; then
        PI_TAILSCALE_IP=$ip
    # Any other valid IPv4 is considered the direct connection
    else
        PI_DIRECT_IP=$ip
    fi
done
# --- END NEW IP DETECTION ---


# --- Debugging: Show the detected IPs ---
echo "--- Detected IPs ---"
echo "Direct IP found: ${PI_DIRECT_IP:-None}"
echo "Tailscale IP found: ${PI_TAILSCALE_IP:-None}"
echo "--------------------"

# Check if at least one IP was found
if [ -z "$PI_DIRECT_IP" ] && [ -z "$PI_TAILSCALE_IP" ]; then
    echo "Error: No usable IPv4 network interface found. Please check your connection."
    exit 1
fi

# --- User Selection ---
while true; do
    echo "Choose connection method for the Drone's ROS Master:"

    direct_valid="false"
    tailscale_valid="false"
    prompt_options=""

    if [ -n "$PI_DIRECT_IP" ]; then
        echo "1. Direct (Non-Tailscale) IP: ${PI_DIRECT_IP}"
        direct_valid="true"
        prompt_options="Direct"
    fi
    if [ -n "$PI_TAILSCALE_IP" ]; then
        echo "2. Tailscale VPN IP: ${PI_TAILSCALE_IP}"
        tailscale_valid="true"
        if [ -n "$prompt_options" ]; then prompt_options+=", "; fi
        prompt_options+="Tailscale"
    fi

    default_choice="1"
    if [ "$direct_valid" = "false" ] && [ "$tailscale_valid" = "true" ]; then
        default_choice="2"
    fi
    
    read -p "Enter choice (${prompt_options}, default $default_choice): " choice
    choice=${choice:-$default_choice}

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
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/config:/root/catkin_ws/src/realflight_modules/VINS-Fusion/config" \
    -v "$(pwd)/src/realflight_modules/VINS-Fusion/vins_estimator/launch:/root/catkin_ws/src/realflight_modules/VINS-Fusion/vins_estimator/launch" \
    -v "$(pwd)/src/planner/plan_manage/launch:/root/catkin_ws/src/planner/plan_manage/launch" \
    -v "$(pwd)/shfiles:/root/shfiles" \
    -v "$(pwd)/vins_output:/root/vins_output" \
    fastdrone_image_pi:latest-arm64