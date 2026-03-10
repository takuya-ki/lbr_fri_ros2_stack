#!/bin/bash
set -e

CONTAINER_NAME="lbr_stack_container"

xhost +local:docker

docker start "${CONTAINER_NAME}" -i
