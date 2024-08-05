FROM ros:jazzy-ros-core

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-pip \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*


# Set up the ROS2 workspace
ENV ROS_WS /ros2_ws
WORKDIR /ros2_ws


# Set up entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Source ROS2 setup bash script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
