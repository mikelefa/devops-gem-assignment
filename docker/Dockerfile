# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install essential tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add ROS package repository
RUN curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS Noetic and required packages
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set working directory for ROS workspace
WORKDIR /ros_ws

# Create the src directory
RUN mkdir -p src

# Clone project repo
WORKDIR /ros_ws/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Install xvfb for headless GUI simulation (virtual framebuffer)
RUN apt-get update && apt-get install -y \
    xvfb \
    libgl1-mesa-dev \
    libxrender1 \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# Copy in custom scripts
COPY scripts/evaluate_cross_track_error.py /ros_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/
COPY scripts/pure_pursuit_sim.py /ros_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/

# Set bash as default shell
SHELL ["/bin/bash", "-c"]

# Source ROS, install dependencies, and build
WORKDIR /ros_ws
RUN source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make

# Source workspace on container start
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc
# RUN Xvfb :1 -screen 0 1920x1080x24 & \
#     export DISPLAY=:1
RUN echo "DISPLAY is $DISPLAY" \
    which xvfb-run

ENV QT_QPA_PLATFORM=offscreen

# CMD xvfb-run -a roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:=true gui:=false use_rviz:=false

# Default command to run headless sim with evaluation
CMD xvfb-run --auto-servernum --server-args='-screen 0 1400x900x24' bash -c "\
    source /opt/ros/noetic/setup.bash && \
    source /ros_ws/devel/setup.bash && \
    roslaunch gem_gazebo gem_gazebo_rviz.launch gui:=false use_rviz:=false & \
    sleep 10 && \
    roslaunch gem_gazebo gem_sensor_info.launch & \
    sleep 5 && \
    rosrun gem_pure_pursuit_sim pure_pursuit_sim.py & \
    sleep 5"
