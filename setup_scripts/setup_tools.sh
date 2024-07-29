#!/bin/bash
set -euxo pipefail
# PX4 and dependencies
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# ROS humble 
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

# PX4 dependencies
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# gazebo classic
sudo apt remove gz-garden -y
sudo apt install aptitude -y
sudo aptitude install gazebo libgazebo11 libgazebo-dev -y
