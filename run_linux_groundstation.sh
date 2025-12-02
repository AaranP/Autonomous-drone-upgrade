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

# --- Get Server IP from user ---
read -p "Please enter the IP address of the server (Raspberry Pi): " SERVER_IP

# Validate IP format (simple check)
if [[ ! $SERVER_IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Invalid IP address format. Exiting."
  exit 1
fi

echo "Server IP set to: $SERVER_IP"

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
echo "Passing RASPBERRY_PI_TARGET_IP=$SERVER_IP to the container."
echo "The container will execute shfiles/client.sh to set up the ROS environment."

docker run -it --rm \
    --name $CONTAINER_NAME \
    --network="host" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e QT_X11_NO_MITSHM=1 \
    -e GROUND_STATION_HOST_IP=$GROUNDSTATION_IP \
    -e RASPBERRY_PI_TARGET_IP=$SERVER_IP \
    $IMAGE_NAME:$IMAGE_TAG \
    /bin/bash /root/catkin_ws/src/fastdrone/shfiles/client.sh

# --- Cleanup ---
# This will run after the container is stopped.
echo "Container stopped. Disabling X11 forwarding for local docker containers."
xhost -local:docker
