#!/bin/bash

set -e
set -x

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms \
librealsense2-utils \
librealsense2-dev \
librealsense2-dbg

sudo apt-get install ros-noetic-mavros
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh

sudo apt update
sudo apt install -y \
  ros-noetic-serial \
  libvtk7-dev \
  libdw-dev  \
  ros-noetic-realsense2-camera  \
  ros-noetic-nodelet

catkin_make
source devel/setup.bash
