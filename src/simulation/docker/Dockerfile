# ROS2 Jazzy + Gazebo Harmonic simulation environment for Food Cart robot
# Builds the complete workspace with Nav2, slam_toolbox, and cart packages

FROM osrf/ros:jazzy-desktop

# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic (gz-sim8) and ros_gz bridge
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install Nav2, slam_toolbox, foxglove_bridge, and other dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-twist-mux \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-vision-opencv \
    ros-jazzy-apriltag-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for test scenarios
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    transforms3d

# Create workspace
WORKDIR /ros2_ws

# Copy the entire source tree (volume-mounted at runtime, but we prepare the structure)
RUN mkdir -p /ros2_ws/src

# Set up ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Build command - to be run inside container
# colcon build --symlink-install

# Set default environment
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
