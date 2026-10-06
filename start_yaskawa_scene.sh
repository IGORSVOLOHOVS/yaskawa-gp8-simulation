#!/usr/bin/env bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ros2-humble-yaskawa:latest"
CONTAINER_NAME="ros2_yaskawa_env"

echo "======================================================="
echo "   Yaskawa Motoman GP8 Workcell 3D Scene Launcher      "
echo "======================================================="

# Check if Podman container image exists, build if missing
if ! podman image exists "${IMAGE_NAME}"; then
    echo "[1/3] Building ROS 2 Humble + MoveIt 2 Podman container image..."
    podman build -t "${IMAGE_NAME}" -f "${WORKSPACE_DIR}/Containerfile" "${WORKSPACE_DIR}"
else
    echo "[1/3] Podman container image '${IMAGE_NAME}' already exists."
fi

# Set up X11 permissions for local container visualization
if command -v xhost &> /dev/null; then
    xhost +local:root || true
fi

DISPLAY_VAL="${DISPLAY:-:0}"

# Ensure host build and install directories exist for podman mount
mkdir -p "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install"

echo "[2/3] Building ROS 2 packages inside container..."
podman run --rm \
    --net=host \
    -v "${WORKSPACE_DIR}/src:/ros2_ws/src:Z" \
    -v "${WORKSPACE_DIR}/build:/ros2_ws/build:Z" \
    -v "${WORKSPACE_DIR}/install:/ros2_ws/install:Z" \
    "${IMAGE_NAME}" \
    bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install"

echo "[3/3] Launching Yaskawa 3D Scene & MoveIt 2 Interactive Panel..."
podman run -it --rm --replace \
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
    "${IMAGE_NAME}" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch yaskawa_moveit_config moveit_robot_only.launch.py"
