#!/usr/bin/env bash

# Shell script scripts to install useful tools , ROS melodic on unbuntu 18.04
# -------------------------------------------------------------------------
#Copyright © 2018 Wei-Chih Lin , kjoelovelife@gmail.com 

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

## install userful tools
cd
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y python-pip \
                        libfreetype6-dev 
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
                        rsync

## And can install [ pkg-config , zip ]
                        
## Install ROS melodic
cd
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

sudo apt install -y ros-melodic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python-rosinstall \
                        python-rosinstall-generator \
                        python-wstool \
                        build-essential

## install library for machine learning with python.
cd
sudo pip install matplotlib \
                 scikit-build \
                 imutils \
                 pillow \
                 scipy \
                 keras \
                 scikit-learn \
                 notebook \
                 Jetson.GPIO \
                 -extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow-gpu==1.13.1+nv19.4

# let gpio can be used on your account.
cd
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# configure SWAP size , ideal is double RAM. Default is 4G
# you can use [ df -h ] to see how space you can use on microSD now
#             [ free -h ] to see how space you can use with swap    
cd
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
ls -lh /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon –show
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Install DLIB , a tool can use machine learning , computer vision , image recognition...etc.
# if it had error about " atal error: Python.h: No such file or directory " 
# Please reset virable of python
#============ use theese ========================
# sudo find / -name “Python.h"
# /usr/include/python3.6m/Python.h # note: this path maybe different.
# /usr/include/python2.7/Python.h
# export CPLUS_INCLUDE_PATH=/usr/include/python3.6m
#======================================================
cd
mkdir temp; cd temp
git clone https://github.com/davisking/dlib.git
cd dlib
mkdir build; cd build
cmake .. -DDLIB_USE_CUDA=1 -DUSE_AVX_INSTRUCTIONS=1
cmake --build .
cd ..
python setup.py install
sudo ldconfig

# Install Darknet , let jetson-nano can train Yolo model with darknet.
cd ~/Jetson_nano/darknet
sudo make

# Install YOLO3-4-py , let jetson-nano can infer YOLO model with GPU.
export GPU=1
sudo pip install yolo34py-gpu

# Install Jetson stats , about resource monitoring with series of NVIDIA Jetson
cd ~/Jetson_nano/jetson_stats
sudo ./install_jetson_stats.sh –s

# Configure power mode : 5W
sudo nvpmodel -m1

## If you want to see power mode , use 
sudo nvpmodel -q


# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
