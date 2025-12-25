#!/bin/bash
set -e

echo "Starting ROS 2 Jazzy Jalisco installation for Ubuntu 24.04..."

# 1. Set locale
echo "Setting locale..."
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Setup Sources
echo "Setting up sources..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
# Download and dearmor the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/index.key | sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

# Ensure the keyring is readable
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 packages
echo "Installing ROS 2 Jazzy (Desktop)..."
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y

# 4. Install additional tools
echo "Installing development tools and ROS Bridge..."
sudo apt install ros-dev-tools -y
sudo apt install ros-jazzy-rosbridge-suite -y

# 5. Environment setup
echo "Setting up environment..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 to .bashrc"
fi

echo "-------------------------------------------------------"
echo "ROS 2 Jazzy installation complete!"
echo "Please restart your terminal or run: source ~/.bashrc"
echo "-------------------------------------------------------"
