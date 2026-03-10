#!/bin/bash
set -e

CONTAINER_NAME="lbr_stack_container"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

xhost +local:docker

# remove existing container if present
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

# build image using repo root as build context
docker build \
    -f "${SCRIPT_DIR}/Dockerfile" \
    -t "${CONTAINER_NAME}" \
    "${REPO_ROOT}"

# run container
docker run -it \
    --network host \
    --ipc host \
    --volume "${REPO_ROOT}:/home/ros2_ws/src" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /dev/shm:/dev/shm \
    --volume /dev:/dev --privileged \
    --env DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --name "${CONTAINER_NAME}" \
    "${CONTAINER_NAME}"
