#!/bin/bash
echo "=== BAT DAU CAI DAT ROS 2 HUMBLE TU DONG ==="
echo "Qua trinh nay co the mat 15-30 phut tuy mang..."
echo "Vui long nhap mat khau Ubuntu khi duoc hoi."

# Kiem tra locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Them source list
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Cai dat ROS 2
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Cai dat them Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install ros-humble-joint-state-publisher -y

# Setup moi truong
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "=== CAI DAT HOAN TAT! ==="
echo "Ban co the bat dau chay du an."
