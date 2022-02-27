#!/bin/bash

echo -e "\e[96mDo you have an NVIDIA GPU that you would like to use with carla? [Y/n]"


Yes="Y"
no="n"
NC='\033[0m' # No Color
printf "${NC}"

read selector
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo apt-get update  # To get the latest package lists

#install carla
sudo apt-get install carla-simulator=0.9.12

 pip3 install carla

sudo apt-get install libomp5 -y



if [[ "$selector" == "$Yes" ]]; then
    echo -e "\e[32mSetting Vulkan to use NVIDIA GPU"
    printf "${NC}"
    export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"
else
    echo -e "\e[31mNot setting vulkan to use NVIDIA GPU."
fi


echo -e "\e[96mInstalling the Carla-ROS Bridge"
printf "${NC}"
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
REQUIRED_PKG="ros-noetic-desktop-full"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  echo -e "\e[31mNo $REQUIRED_PKG. Setting up $REQUIRED_PKG."
  printf "${NC}"
  sudo apt-get --yes install $REQUIRED_PKG
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
fi
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
echo -e "Making a home folder to install the bridge package"
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge
git clone https://github.com/carla-simulator/ros-bridge.git
cd catkin_ws/src
ln -s ../../ros-bridge
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r

#build
catkin_make
echo "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo -e "\e[96mThe bridge need an alias from python to python 3. Would you like to set this? [Y/n]"


read selector
if [[ "$selector" == "$Yes" ]]; then
    echo -e "\e[32mSetting alias python to python3."
    printf "${NC}"
    alias python=python3
    alias pip=pip3
    echo "alias python=python3" >> ~/.bashrc
    echo "alias pip=pip3" >> ~/.bashrc
    source ~/.bashrc
    pip install pygame
    sudo apt-get install python-is-python3 -y
else
    echo -e "\e[31mNot Setting alias."
fi
