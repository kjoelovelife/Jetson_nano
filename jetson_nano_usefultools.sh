#!/usr/bin/env bash

# Shell script scripts to install useful tools , ROS melodic on unbuntu 18.04 with Jetson-nano
# -------------------------------------------------------------------------
#Copyright © 2019 Wei-Chih Lin , kjoelovelife@gmail.com 

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
# -------------------------------------------------------------------------
# reference
# https://chtseng.wordpress.com/2019/05/01/nvida-jetson-nano-%E5%88%9D%E9%AB%94%E9%A9%97%EF%BC%9A%E5%AE%89%E8%A3%9D%E8%88%87%E6%B8%AC%E8%A9%A6/
#
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

#======== apt update and upgrade ==============================
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove

#=========================  Install ROS melodic =======================================
cd

# enable all Ubuntu packages:
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted

# add ROS repository to apt sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# install ROS desktop-full
sudo apt-get update
sudo apt install -y ros-melodic-desktop-full

sudo rosdep init
rosdep update

sudo apt-get install -y python-rosinstall \
                        python-rosinstall-generator \
                        python-wstool \
                        build-essential
#=======================================================================================

#======== configure virtualenv ================================
## if you want to exit virtualenv , just enter "deactivate"
sudo apt-get install -y virtualenv
cd
mkdir envs;cd envs
virtualenv -p python3 AI
### if you want to use virtualenv AI, please enter bellow command 
#echo "source ~/envs/AI/bin/activate" >> ~/.bashrc
#source ~/envs/AI/bin/activate
#======== configure OpenCV ( Theese command just for Jetson-nano developer kit) ========
#cd ~/envs/AI/lib/python3.6/site-packages/
#ln -s /usr/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so
#=======================================================================================

#======== install userful tools ================
cd
sudo apt-get install -y libfreetype6-dev \
                        zlib1g-dev \
                        libjpeg8-dev \
                        libhdf5-dev \
                        libssl-dev \
                        libffi-dev \
                        python3-dev \
                        libhdf5-serial-dev \
                        hdf5-tools \
                        libblas-dev \
                        liblapack-dev \
                        libatlas-base-dev \
                        build-essential \
                        cmake \
                        libgtk-3-dev \
                        libboost-all-dev \
                        nano \
                        virtualenv \
                        rsync \
			gedit \
                        libgflags-dev \
			git \
                        python3-scipy \
                        python-scipy \
                        libcanberra-gtk-module \
			libcanberra-gtk3-module
## And can install [ pkg-config , zip ]
#=======================================================================================

#================ install library for machine learning with python. ===================
## Install package with python3
#python3 -m pip install --upgrade pip setuptools wheel
pip3 install matplotlib \
                 scikit-build \
                 imutils \
                 pillow \
                 keras \
                 scikit-learn \
                 notebook \
                 Jetson.GPIO \
                 Adafruit-MotorHAT \
                 --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow-gpu==1.13.1+nv19.4 --user

#=======================================================================================

#================== About python3 ===========================================================
# if you want install in system , You need to upgrade pip3 :  [ python3 -m pip install --upgrade pip ]
# And then , you need to modified " /usr/bin/pip " , detail : https://stackoverflow.com/questions/49836676/error-after-upgrading-pip-cannot-import-name-main
# Notice : you need to exit virtualenv AI , please enter " deactivate "

#=============================================================================================

#======== let gpio can be used on your account. ========
cd
#sudo groupadd -f -r gpio     # you need to enter this line
#sudo usermod -aG gpio $USER  # you need to enter this line 
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
#=======================================================================================

#==== configure SWAP size , ideal is double RAM. Default is 4G ========================
# you can use [ df -h ] to see how space you can use on microSD now
#             [ free -h ] to see how space you can use with swap    
size=4G
cd
sudo fallocate -l $size /swapfile
sudo chmod 600 /swapfile
ls -lh /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon –show
sudo cp /etc/fstab /etc/fstab.bak
# long time to use
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
#=======================================================================================

#========== Install DLIB , a tool can use machine learning , computer vision , image recognition...etc. ==========
# if it had error about " atal error: Python.h: No such file or directory " 
# Please reset virable of python
######### use theese to modified error #####################
# sudo find / -name “Python.h"
# /usr/include/python3.6m/Python.h # note: this path maybe different.
# /usr/include/python2.7/Python.h
# export CPLUS_INCLUDE_PATH=/usr/include/python3.6m
############################################################
cd
mkdir temp; cd temp
git clone https://github.com/davisking/dlib.git
cd dlib
sudo mkdir build; cd build
sudo cmake .. -DDLIB_USE_CUDA=1 -DUSE_AVX_INSTRUCTIONS=1
sudo cmake --build .
cd ..
sudo python setup.py install
sudo ldconfig
#=========================================================================================

#==== Install Darknet , let jetson-nano can train Yolo model with darknet. ====
#echo "export CUBA_HOME=/usr/local/cuda-10.0"
#echo "export PATH=/usr/local/cuda-10.0/bin:$PATH" >> ~/.bashrc
#echo "export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH" >> ~/.bashrc
#source ~/.bashrc
#cd ~/Jetson_nano/darknet
#sudo make -j4
#==========================================================================================
# Install YOLO3-4-py , let jetson-nano can infer YOLO model with GPU.

###### YoloV3 on github : https://github.com/madhawav/YOLO3-4-py #####
export GPU=1
pip install yolo34py-gpu
#===================================================================

#==== Install Jetson stats , about resource monitoring with series of NVIDIA Jetson ====
cd ~/Jetson_nano/jetson_stats
sudo ./install_jetson_stats.sh
######### How to use ########
# you can enter [ jtop ] , to see what state on jetson-nano now . 
# you can enter [ jetson_release ] , to see what version of software on this jetson-nano
#############################
#=======================================================================================

# Configure power mode : 5W
#sudo nvpmodel -m1

# Configure power mode : 10W
sudo nvpmodel -m0

## If you want to see power mode , use 
sudo nvpmodel -q

#============= Clone Jetbot-ROS in ~/Jetson_nano/Jetbot/catkin_ws/src =========================
cd ~/Jetson_nano
mkdir -p Jet_falcon/catkin_ws/src

# set variable " workspace"
workspace="Jetson_nano/Jet_falcon"

cd ~/$workspace/catkin_ws/src
git clone https://github.com/dusty-nv/jetbot_ros 
#==============================================================================================

#========================= configure Jetson-inference ======================================
# clone Jetson-inference in ~/Jetson_nano/Jetbot/catkin_ws/src
cd ~/$workspace
git clone https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init

## build Jetson-inference from source
mkdir build && cd build
cmake ../
sudo make
sudo make install
#==========================================================================================

#========================= clone ros_deep_learning ======================================

# install dependencies
sudo apt-get install ros-melodic-vision-msgs ros-melodic-image-transport ros-melodic-image-publisher

# clone the repo
cd ~/$workspace/catkin_ws/src
git clone https://github.com/dusty-nv/ros_deep_learning

#========================================================================================

#=== configure variable about ROS in ~/.bashrc  ========================================
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
#=======================================================================================


#========================= build ROS package ( ~/Jetson_nano/Jetbot/catkin_ws )==========

cd ~/$workspace/catkin_ws && catkin_make

#========================================================================================



# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
