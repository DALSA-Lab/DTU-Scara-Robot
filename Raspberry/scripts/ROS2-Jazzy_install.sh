#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 --domain-id <ROS_DOMAIN_ID> --version <ros-base|ros-desktop>"
    exit 1
}

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --domain-id) ROS_DOMAIN_ID="$2"; shift ;;
        --version) ROS_VERSION="$2"; shift ;;
        *) usage ;;
    esac
    shift
done

# Check if both arguments are provided
if [ -z "$ROS_DOMAIN_ID" ] || [ -z "$ROS_VERSION" ]; then
    usage
fi

# Validate ROS version argument
if [[ "$ROS_VERSION" != "ros-base" && "$ROS_VERSION" != "ros-desktop" ]]; then
    echo "Invalid ROS version. Use 'ros-base' or 'ros-desktop'."
    exit 1
fi

# Install necessary packages
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y

# Add ROS repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update

# Install the specified ROS version
if [ "$ROS_VERSION" == "ros-base" ]; then
    echo "Installing ros-jazzy-base"
    sudo apt install ros-jazzy-ros-base -y
elif [ "$ROS_VERSION" == "ros-desktop" ]; then
    echo "Installing ros-jazzy-ros-desktop"
    sudo apt install ros-jazzy-desktop -y
fi

# Source ROS setup script
source /opt/ros/jazzy/setup.bash

# Add source command to .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc

# Install colcon extensions
sudo apt install python3-colcon-common-extensions -y

echo "ROS installation completed. Please restart your terminal or run 'source ~/.bashrc' to apply changes."
