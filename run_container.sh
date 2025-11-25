docker run --privileged --rm tonistiigi/binfmt --install all

# xhost +local:docker # This line is not needed for a headless server
docker run -it --rm \
    --name fast_drone_container \
    --privileged \
    --network=host \
    # -e DISPLAY=$DISPLAY \ # Remove this line for headless server
    # -v /tmp/.X11-unix:/tmp/.X11-unix \ # Remove this line for headless server
    -v /dev:/dev \
    -v $(pwd)/src/fastdrone/config:/root/catkin_ws/src/fastdrone/config \
    -v $(pwd)/shfiles:/root/shfiles \
    fast_drone_noetic \
    /root/shfiles/server.sh # Execute the server setup script
    
#Opens another terminal in the docker session
#docker exec -it fast_drone_container /bin/bash