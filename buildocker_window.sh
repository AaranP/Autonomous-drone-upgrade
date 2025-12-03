#!/bin/bash
# Fixes cross-build emulation for ARM64 (required like in builddocker.sh)
docker run --privileged --rm tonistiigi/binfmt --install all

# Define the image name and tag for the ARM64 build
IMAGE_NAME="fastdrone_image_pi"
IMAGE_TAG="latest-arm64"
TAR_FILE="${IMAGE_NAME}.tar"

echo "--- Building ARM64 Docker image for Raspberry Pi: ${IMAGE_NAME}:${IMAGE_TAG} ---"
# Use 'docker buildx build' to cross-compile for ARM64.
# The '--load' flag makes the image available to the local Docker daemon so it can be saved.
docker buildx build --platform linux/arm64 -t "${IMAGE_NAME}:${IMAGE_TAG}" . --load

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "--- Docker image built successfully. ---"
    echo "--- Saving Docker image to ${TAR_FILE} ---"
    # Save the built image to a .tar file
    docker save -o "${TAR_FILE}" "${IMAGE_NAME}:${IMAGE_TAG}"

    # Check if the save was successful
    if [ $? -eq 0 ]; then
        echo "--- Docker image saved successfully to ${TAR_FILE}. ---"
        echo ""
        echo "Next steps:"
        echo "1. Transfer the file '${TAR_FILE}' to your Raspberry Pi."
        echo "2. On the Pi, load the image using: docker load -i ${TAR_FILE}"
        echo "3. You can then run a container from the image."
    else
        echo "Error: Failed to save Docker image."
    fi
else
    echo "Error: Docker image build failed."
fi