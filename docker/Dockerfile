ARG BUILDKIT_INLINE_CACHE=1

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
    tmux \
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
    mesa-utils \
    libxrender1 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    && rm -rf /var/lib/apt/lists/*

# Copy in custom scripts
COPY scripts/evaluate_cross_track_error.py /ros_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/
COPY scripts/pure_pursuit_sim.py /ros_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/

# Set bash as default shell
SHELL ["/bin/bash", "-c"]

# Source ROS, install dependencies, and build
RUN cd /ros_ws && \
    source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make

# Source workspace on container start
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set environment
ENV QT_QPA_PLATFORM=offscreen
ENV DISPLAY=:99

CMD ["tail", "-f", "/dev/null"]
