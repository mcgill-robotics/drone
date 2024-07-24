#!/bin/bash

sudo apt update && sudo apt upgrade -y
# Install i3
/usr/lib/apt/apt-helper download-file https://debian.sur5r.net/i3/pool/main/s/sur5r-keyring/sur5r-keyring_2024.03.04_all.deb keyring.deb SHA256:f9bb4340b5ce0ded29b7e014ee9ce788006e9bbfe31e96c09b2118ab91fca734
sudo apt install ./keyring.deb
echo "deb http://debian.sur5r.net/i3/ $(grep '^DISTRIB_CODENAME=' /etc/lsb-release | cut -f2 -d=) universe" | sudo tee /etc/apt/sources.list.d/sur5r-i3.list
sudo apt update
sudo apt install -y i3

# sudo apt-get install -y ubuntu-desktop

# PX4 and dependencies
#cd
#git clone https://github.com/PX4/PX4-Autopilot.git --recursive
#bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

# gazebo classic
#sudo apt update && sudo apt install -y locales
#sudo locale-gen en_US en_US.UTF-8
#sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
#export LANG=en_US.UTF-8
#sudo apt install -y software-properties-common
#sudo add-apt-repository universe
#sudo apt install -y gazebo libgazebo11 libgazebo-dev

# ROS humble 
#sudo apt update && sudo apt install curl -y
#sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
#sudo apt update && sudo apt upgrade -y
#sudo apt install -y ros-humble-desktop
#sudo apt install -y ros-dev-tools
#source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

