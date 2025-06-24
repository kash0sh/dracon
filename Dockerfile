# Use ROS 2 Humble base image
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-rclcpp \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

# Create workspace structure
WORKDIR /home/swarm_ws
RUN mkdir -p src
    
# Copy packages into workspace
COPY ./dracon_msgs ./src/dracon_msgs
COPY ./agent ./src/agent

# Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Source overlay by default
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /home/swarm_ws/install/setup.bash" >> ~/.bashrc

# Default entrypoint to source and stay in shell
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/swarm_ws/install/setup.bash && exec \"$@\"", "--"]

RUN cat src/dracon_msgs/package.xml
