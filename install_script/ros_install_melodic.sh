#!/usr/bin/env bash

ros1_distro=melodic

#================= step 1. Install ROS melodic =================================
cd
# enable all Ubuntu packages:
echo $PASSWORD | sudo -S apt-add-repository universe
echo $PASSWORD | sudo -S apt-add-repository multiverse
echo $PASSWORD | sudo -S apt-add-repository restricted

# add ROS repository to apt sources
echo $PASSWORD | sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo $PASSWORD | sudo -S apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -

## delete ROS old key : sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 

# install ROS desktop-full
echo $PASSWORD | sudo -S apt-get update
echo $PASSWORD | sudo -S apt install -y ros-$ros1_distro-desktop-full

echo $PASSWORD | sudo -S apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
echo $PASSWORD | sudo -S apt install python-rosdep

echo $PASSWORD | sudo -S rosdep init
rosdep update

echo $PASSWORD | sudo -S apt-get install -y python-rosinstall \
                        python-rosinstall-generator \
                        python-wstool \
                        build-essential

##  configure variable about ROS in ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS $ros1_distro installed successfully." 
#sudo chown -R $USER: $HOME
#==============================================================================


# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
