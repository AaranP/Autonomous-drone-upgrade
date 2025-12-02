#For seperate terminals to run the ros 

chmod +x attach_groundstation.sh
#!/bin/bash

CONTAINER_NAME="fastdrone_groundstation_container:latest"

echo "Attaching to container: $CONTAINER_NAME and setting up ROS environment..."

# Execute bash in the container, sourcing ROS setup and client.sh, then starting an interactive shell
docker exec -it "$CONTAINER_NAME" /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && source /root/shfiles/client.sh && /bin/bash"

if [ $? -ne 0 ]; then
    echo "Error: Could not attach to container '$CONTAINER_NAME'. Is it running?"
    echo "Please ensure 'run_groundstation_container.sh' is running in another terminal."
fi