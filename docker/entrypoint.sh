#!/bin/bash
set -e

# Start Xvfb for headless GUI rendering
Xvfb :99 -screen 0 1280x1024x24 &
export DISPLAY=:99
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roslaunch gem_gazebo gem_gazebo_rviz.launch gui:=false use_rviz:=false