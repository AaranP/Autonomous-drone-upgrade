#!/bin/bash

# Define the image name and tag
IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest"

echo "--- Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG} using Dockerfile.groundstation ---"
# Build the Docker image. The '.' assumes the Dockerfile is in the current directory.
# Use --platform linux/amd64 for broad compatibility on most ground station PCs (Windows/Linux/Intel Macs)
# For M1 Macs, Docker Desktop can emulate amd64, or you can build native arm64 if all dependencies support it.
# Added --no-cache to ensure all layers are rebuilt and new dependencies are installed.
docker build -f Dockerfile.groundstation --platform linux/amd64 -t "${IMAGE_NAME}:${IMAGE_TAG}" . 

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "--- Docker image built successfully: ${IMAGE_NAME}:${IMAGE_TAG} ---"
else
    echo "Error: Docker image build failed."
    exit 1
fi