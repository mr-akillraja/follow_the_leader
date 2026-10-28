#!/bin/bash
set -e

# Colors for output
GREEN='\03---[0;32m'
NC='\03---[0m' # No Color

echo -e "${GREEN}Starting ROS 2 Jazzy and Gazebo Harmonic Installation for Ubuntu 24.04...${NC}"

# Check for Ubuntu 24.04
if ! grep -q "24.04" /etc/os-release; then
  echo "Error: This script is intended for Ubuntu 24.04."
  exit 1
fi

echo -e "${GREEN}1. Setting up language and locale...${NC}"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo -e "${GREEN}2. Enabling Universe repository...${NC}"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo -e "${GREEN}3. Adding ROS 2 GPG key and repository...${NC}"
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "${GREEN}4. Installing ROS 2 Base and Dev tools...${NC}"
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

echo -e "${GREEN}5. Installing Gazebo Harmonic and ROS 2 integration...${NC}"
sudo apt install -y ros-jazzy-ros-gz

echo -e "${GREEN}6. Initializing rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo -e "${GREEN}Installation Complete!${NC}"
echo "Please source your ROS 2 environment in your terminal via:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "Or add it to your .bashrc:"
echo "  echo \"source /opt/ros/jazzy/setup.bash\" >> ~/.bashrc"
