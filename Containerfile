FROM docker.io/osrf/ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install only minimal required packages with --no-install-recommends for fast build and small footprint
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-ros-visualization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-kdl-parser \
    ros-humble-urdf \
    rviz2 \
    python3-colcon-common-extensions \
    python3-pip \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
