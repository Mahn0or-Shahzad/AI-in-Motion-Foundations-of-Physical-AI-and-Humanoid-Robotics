#!/bin/bash

# AI in Motion Development Environment Setup Script

echo "Setting up AI in Motion development environment..."

# Install ROS 2 Humble
echo "Installing ROS 2 Humble..."
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install openai rospy numpy transforms3d opencv-python

# Install colcon for building ROS 2 packages
echo "Installing colcon..."
sudo apt install python3-colcon-common-extensions

# Setup workspace
echo "Setting up workspace..."
mkdir -p ~/ai_in_motion_ws/src
cd ~/ai_in_motion_ws
source /opt/ros/humble/setup.bash
colcon build

echo "Environment setup complete!"
echo "To use the environment, run: source /opt/ros/humble/setup.bash && source ~/ai_in_motion_ws/install/setup.bash"