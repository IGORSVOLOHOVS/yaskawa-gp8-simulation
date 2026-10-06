#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_yaskawa_env"
IMAGE_NAME="ros2-humble-yaskawa:latest"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure X11 permissions for local container connections
if command -v xhost &> /dev/null; then
    xhost +local:root || true
fi

DISPLAY_VAL="${DISPLAY:-:0}"

mkdir -p "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install"

echo "Starting ROS 2 Humble Container (${CONTAINER_NAME})..."

# Run podman container with X11 forwarding and host network for MotoROS2 communication
podman run -it --rm \
    --name "${CONTAINER_NAME}" \
    --net=host \
    --ipc=host \
    --security-opt label=disable \
    -e DISPLAY="${DISPLAY_VAL}" \
    -e QT_X11_NO_MITSHM=1 \
    -e QT_QPA_PLATFORM=xcb \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${WORKSPACE_DIR}/src:/ros2_ws/src:Z" \
    -v "${WORKSPACE_DIR}/build:/ros2_ws/build:Z" \
    -v "${WORKSPACE_DIR}/install:/ros2_ws/install:Z" \
    --device /dev/dri:/dev/dri:rw \
    "${IMAGE_NAME}" "$@"
