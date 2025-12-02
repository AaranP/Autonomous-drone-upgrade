# Use a ROS Noetic base image
FROM osrf/ros:noetic-desktop-full
# Set up environment variables
ENV HOME=/root
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/catkin_ws/src

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    liblapack-dev \
    libceres-dev \
    libyaml-cpp-dev \
    libomp-dev \
    ros-noetic-pcl-ros \
    terminator \
    net-tools \
    openssh-server \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-mavros \
    && rm -rf /var/lib/apt/lists/*

# Install Realsense SDK dependencies (as per official guide)
# Add Intel's public key
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# Add Intel's apt repository
RUN sh -c 'echo "deb https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" > /etc/apt/sources.list.d/realsense-apt-source.list'

# Install Realsense SDK and ROS wrappers
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# --- CORRECTED ---
# Copy only the 'src' directory from your project into the workspace 'src' folder
COPY src/ /root/catkin_ws/src/

# Build the entire catkin workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set up SSH for easier access (optional but useful)
RUN mkdir -p /var/run/sshd
RUN echo 'root:docker' | chpasswd
EXPOSE 22

# Configure environment for ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://ledrone:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=ledrone" >> ~/.bashrc

CMD ["/bin/bash"]