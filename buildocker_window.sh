#!/bin/bash

# Define the image name and tag
IMAGE_NAME="fastdrone_image"
IMAGE_TAG="latest"
TAR_FILE="${IMAGE_NAME}.tar"

echo "--- Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG} ---"
# Build the Docker image. The '.' assumes the Dockerfile is in the current directory.
docker build --platform linux/amd64 -t "${IMAGE_NAME}:${IMAGE_TAG}" .

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
        echo "1. Transfer the file '${TAR_FILE}' to your target machine."
        echo "2. On the target machine, load the image using: docker load -i ${TAR_FILE}"
        echo "3. You can then run a container from the image using: docker run -it ${IMAGE_NAME}:${IMAGE_TAG} /bin/bash"
    else
        echo "Error: Failed to save Docker image."
    fi
else
    echo "Error: Docker image build failed."
fi