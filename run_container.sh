xhost +local:docker # Allow Docker to connect to your X server
docker run -it --rm \
    --name fast_drone_container \
    --privileged \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v $(pwd)/src/Fast-Drone-250/config:/root/catkin_ws/src/Fast-Drone-250/config \
    -v $(pwd)/shfiles:/root/shfiles \
    fast_drone_noetic \
    /bin/bash