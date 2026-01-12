#!/bin/bash

IMAGE_NAME="fastdrone_groundstation"
IMAGE_TAG="latest"

echo "--- Building NATIVE ARM64 Docker image: ${IMAGE_NAME}:${IMAGE_TAG} ---"

# Use 'build' subcommand and specify the platform
# We remove 'buildx' and use 'docker build' for simplicity, or use 'docker buildx build'
docker build --platform linux/arm64 \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    -f Dockerfile.groundstation . 

if [ $? -eq 0 ]; then
    echo "--- Docker image built successfully: ${IMAGE_NAME}:${IMAGE_TAG} ---"
else
    echo "Error: Docker image build failed."
    exit 1
fi