#!/bin/bash
IMAGE_NAME="spot_subt_ws"
TAG="latest"

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME:$TAG"
docker build -t $IMAGE_NAME:$TAG .

# Exit if the build fails
if [ $? -ne 0 ]; then
    echo "Failed to build Docker image"
    exit 1
fi

echo "Docker image built successfully!"