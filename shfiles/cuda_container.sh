#!/bin/bash

# This script installs the NVIDIA Container Toolkit on a Debian-based system (like Ubuntu).
# Prerequisite: You must have the NVIDIA drivers for your GPU installed on your Linux host machine first.

echo "--- Setting up the NVIDIA package repository ---"
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

echo "--- Updating package list and installing the NVIDIA Container Toolkit ---"
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

echo "--- Configuring Docker to use the NVIDIA runtime ---"
sudo nvidia-ctk runtime configure --runtime=docker

echo "--- Restarting the Docker service ---"
sudo systemctl restart docker

echo "--- NVIDIA Container Toolkit installation complete ---"
echo "You should now be able to run Docker containers with GPU access using the --gpus all flag."