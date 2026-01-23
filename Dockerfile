#Builds the image for the ARM64 using ubutuntu 20.04 as base to deal with ROS noetic not having ARM version
FROM ubuntu:20.04
# Set up environment variables
ENV HOME=/root
ENV DEBIAN_FRONTEND=noninteractive

#This builds the entire ROS noetic from scratch for ARM 64 using the ARM ubuntu 20.04 base image 
# Set locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup sources.list for ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 \
    curl \
    lsb-release \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Ensure we have tools to add apt keys and secure transports
# (gnupg2 is required by apt-key; curl/lsb-release help with key retrieval and distro detection)


# Install ROS Noetic desktop-full
RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

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
    nano \
    ros-noetic-serial \
    libarmadillo-dev \
    libdw-dev \
    && rm -rf /var/lib/apt/lists/*

# Install nlopt v2.7.1
RUN cd /root && \
    git clone -b v2.7.1 https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

# Install Realsense SDK from source for ARM64, as pre-built binaries are not available.
# First, install build dependencies for librealsense.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    libssl-dev \
    pkg-config \
    libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone, build, and install librealsense from scratch since existing one doesn't support ARM natively.
# This is done in a single RUN command to keep the Docker image layers minimal.
# We build with the RSUSB backend, which doesn't require kernel patching (DKMS), making it ideal for ARM devices.
ENV REALSENSE_VERSION=2.50.0
RUN cd /root && \
    git clone --depth 1 --branch v${REALSENSE_VERSION} https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake ../ -DBUILD_EXAMPLES=false \
              -DBUILD_GRAPHICAL_EXAMPLES=false \
              -DCMAKE_BUILD_TYPE=Release \
              -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && \
    make install && \
    cd /root && \
    rm -rf /root/librealsense

# Install GeographicLib datasets for MAVROS
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# --- CORRECTED ---
# Copy all project files into the workspace 'src' folder
COPY src/ /root/catkin_ws/src/

# Remove conflicting utility packages from uav_simulator/Utils if src/utils is preferred
RUN rm -rf /root/catkin_ws/src/uav_simulator/Utils/cmake_utils \
    /root/catkin_ws/src/uav_simulator/Utils/pose_utils \
    /root/catkin_ws/src/uav_simulator/Utils/quadrotor_msgs \
    /root/catkin_ws/src/uav_simulator/Utils/rviz_plugins \
    /root/catkin_ws/src/uav_simulator/Utils/uav_utils

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