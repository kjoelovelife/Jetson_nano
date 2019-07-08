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

# Configure power mode : 5W
#sudo nvpmodel -m1

# Configure power mode : 10W
sudo nvpmodel -m0

## If you want to see power mode , use 
sudo nvpmodel -q

#==== Step1. configure SWAP size , ideal is double RAM. Default is 4G =================
## you can use [ df -h ] to see how space you can use on microSD now
#             [ free -h ] to see how space you can use with swap    

#size=8G
#cd
#sudo fallocate -l $size /swapfile
#sudo chmod 600 /swapfile
#ls -lh /swapfile
#sudo mkswap /swapfile
#sudo swapon /swapfile
#sudo swapon –show
#sudo cp /etc/fstab /etc/fstab.bak

## long time to use

#echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

## After this step , please reboot , and can use " free -h " to view information.
#=======================================================================================

#======== Step2. apt update and upgrade ==============================
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
#==============================================================

#======== Step3. install dependencies ============================================
cd
sudo apt-get install -y build-essential cmake 'pkg-config' \
                        libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev \
                        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
                        libxvidcore-dev libx264-dev \
                        libgtk2.0-dev libgtk-3-dev \
                        libatlas-base-dev gfortran \
                        python2-dev python3-dev \
                        python2-pip python3-pip                        
#=================================================================================

#======== Step4. configure virtualenv ================================
## if you want to exit virtualenv , just enter "deactivate"
sudo apt-get install -y virtualenv
cd
mkdir envs;cd envs
virtualenv -p python3 AI
## if you want to use virtualenv AI, please enter bellow command 
#echo "source ~/envs/AI/bin/activate" >> ~/.bashrc
#source ~/envs/AI/bin/activate
#======== configure OpenCV ( Theese command just for Jetson-nano developer kit) ========
#cd ~/envs/AI/lib/python3.6/site-packages/
#ln -s /usr/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so
#=======================================================================================

#========= Step5. install library with python. ===================
## Install package with python3
#python3 -m pip install --upgrade pip setuptools wheel
#  For installing scipy , need to install gfortran :   
pip3 install numpy

#======== configure openCV_contrib ========
work_space = 'Jetson-nano/OpenCV-Face-Recognition'
cd ~/$work_space
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.1.zip
unzip opencv.zip

wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.1.zip
unzip opencv_contrib.zip

cd ~/$work_space/opencv-3.3.0
mkdir build
cd build
cmake -D OPENCV_EXTRA_MODULES_PATH=~/$work_space/opencv_contrib-3.3.1/modules \
make -j5
sudo make install
sudo ldconfig
#==========================================
#==========================================

#========= step6. configure jupyter lab ================================================		 
# Install traitlets (master, to support the unlink() method)
sudo python3 -m pip install git+https://github.com/ipython/traitlets@master

# Install jupyter lab
sudo apt-get install nodejs npm
sudo pip3 install jupyter jupyterlab
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
sudo jupyter labextension install @jupyterlab/statusbar
jupyter lab --generate-config

# if jupyter notebook has the error : " bash: jupyter: command not found "
# can enter this command to solve : " pip3 install --upgrade --force-reinstall --no-cache-dir jupyter notebook "
#=======================================================================================

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
