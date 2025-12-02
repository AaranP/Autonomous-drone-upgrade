#!/bin/bash

# --- Configuration ---
IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest"
CONTAINER_NAME="fastdrone_groundstation_container"
DRONE_HOSTNAME="ledrone" # Expected Tailscale hostname of the Raspberry Pi

# --- Stop and remove existing container ---
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    echo "Stopping and removing existing container: $CONTAINER_NAME"
    docker stop $CONTAINER_NAME
    docker rm $CONTAINER_NAME
fi

# --- X11 Forwarding Setup ---
# Allow local connections to X server
xhost +local:docker
echo "Enabled X11 forwarding for local docker containers."

# --- Get Server IP and connection type from user ---
ROS_CONNECTION_TYPE=""
SERVER_TARGET=""

while true; do
    read -p "How do you want to connect to the ROS Master on the Raspberry Pi? (1 for direct IP / 2 for tailscale): " CONNECTION_CHOICE
    if [[ "$CONNECTION_CHOICE" == "1" ]]; then
        ROS_CONNECTION_TYPE="direct"
        while true; do
            read -p "Please enter the DIRECT IP address of the server (Raspberry Pi): " INPUT_IP
            if [[ ! $INPUT_IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                echo "Invalid IP address format. Please try again."
            else
                SERVER_TARGET=$INPUT_IP
                echo "Direct IP set to: $SERVER_TARGET"
                break
            fi
        done
        break
    elif [[ "$CONNECTION_CHOICE" == "2" ]]; then
        ROS_CONNECTION_TYPE="tailscale"
        while true; do
            read -p "Do you want to provide the Tailscale IP (1) or use the default hostname 'ledrone' (2)? " TAILSCALE_CHOICE
            if [[ "$TAILSCALE_CHOICE" == "1" ]]; then
                while true; do
                    read -p "Please enter the Tailscale IP address of the server (Raspberry Pi): " INPUT_IP
                    if [[ ! $INPUT_IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                        echo "Invalid IP address format. Please try again."
                    else
                        SERVER_TARGET=$INPUT_IP
                        echo "Tailscale IP set to: $SERVER_TARGET"
                        break
                    fi
                done
                break
            elif [[ "$TAILSCALE_CHOICE" == "2" ]]; then
                SERVER_TARGET="ledrone"
                echo "Using default Tailscale hostname: $SERVER_TARGET"
                break
            else
                echo "Invalid choice. Please enter '1' for Tailscale IP or '2' for hostname."
            fi
        done
        break
    else
        echo "Invalid choice. Please enter '1' for direct IP or '2' for tailscale."
    fi
done

# --- Get Ground Station IP (Host IP) ---
# Using --network=host, the container shares the host's network.
# So we can use the host's IP.
GROUNDSTATION_IP=$(hostname -I | awk '{print $1}')
if [ -z "$GROUNDSTATION_IP" ]; then
    echo "Could not automatically determine the ground station IP. Please ensure you are connected to a network."
    exit 1
fi
echo "Ground Station IP (this machine) set to: $GROUNDSTATION_IP"


# --- Run Docker Container ---
echo "Starting Docker container: $CONTAINER_NAME"
echo "Passing GROUND_STATION_HOST_IP=$GROUNDSTATION_IP to the container."
echo "Passing RASPBERRY_PI_TARGET_IP=$SERVER_TARGET to the container."
echo "Passing ROS_CONNECTION_TYPE=$ROS_CONNECTION_TYPE to the container."
echo "The container will execute shfiles/client.sh to set up the ROS environment."

docker run -it --rm \
    --name $CONTAINER_NAME \
    --network="host" \
    --gpus all \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e "QT_X11_NO_MITSHM=1" \
    -e "NVIDIA_VISIBLE_DEVICES=all" \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e GROUND_STATION_HOST_IP=$GROUNDSTATION_IP \
    -e RASPBERRY_PI_TARGET_IP=$SERVER_TARGET \
    -e ROS_CONNECTION_TYPE=$ROS_CONNECTION_TYPE \
    $IMAGE_NAME:$IMAGE_TAG \
    /bin/bash /root/catkin_ws/src/fastdrone/shfiles/client.sh

# --- Cleanup ---
# This will run after the container is stopped.
echo "Container stopped. Disabling X11 forwarding for local docker containers."
xhost -local:docker
