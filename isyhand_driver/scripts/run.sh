#!/bin/bash
set -e

# Set ISyHand device
DEVICE="${1:-/dev/ttyUSB0}"

# Path to your ROS workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISYHAND_ROS_WS="$(cd "$SCRIPT_DIR/../../../" && pwd)"
CONTAINER_WS="/workspaces/isyhand_ros-dev"
IMAGE_NAME="isyhand_ros_image"
CONTAINER_NAME="isyhand_ros_container"

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    docker exec -it -u admin --workdir $CONTAINER_WS $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Build the image if needed
echo "🛠  Building Docker image: $IMAGE_NAME"
docker build \
  --build-arg USERNAME="admin" \
  --build-arg LOCAL_WS=$ISYHAND_ROS_WS \
  --build-arg CONTAINER_WS=$CONTAINER_WS \
  -t $IMAGE_NAME \
  -f $ISYHAND_ROS_WS/src/isyhand_driver/docker/Dockerfile.base \
  $ISYHAND_ROS_WS/src
# add these if cache issues:
  # --no-cache \
  # --progress=plain \

DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
#DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
#DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")

# Run the container
echo "🚀 Launching container: $CONTAINER_NAME"
docker run -it --rm \
  --privileged \
  --network host \
  --ipc=host \
  ${DOCKER_ARGS[@]} \
  --name "$CONTAINER_NAME" \
  --device $DEVICE:/dev/ttyUSB0 \
  -v "$ISYHAND_ROS_WS":$CONTAINER_WS \
  -v /etc/localtime:/etc/localtime:ro \
  --workdir $CONTAINER_WS \
   $IMAGE_NAME \
   /bin/bash

