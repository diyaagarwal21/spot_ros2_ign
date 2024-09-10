#!/bin/bash
IMAGE_NAME="spot_subt_ws"
TAG="latest"

# Run the Docker container
echo "Running Docker container for: $IMAGE_NAME:$TAG"
docker run --rm -it \
  --network host \
  --volume $(pwd):/app \
  $IMAGE_NAME:$TAG
