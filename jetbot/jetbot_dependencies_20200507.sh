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

#======= ste1. Enable i2c ===============
# Enable i2c permissions
sudo usermod -aG i2c $USER
#===========================================


#======== Step2. apt update and upgrade ==============================
sudo apt-get update
sudo apt install -y python3-pip python3-pil
sudo pip3 install cython
sudo pip3 install --upgrade numpy 
#==============================================================

#========= Step3. install tensorflow ==========================
cd
sudo apt-get install -y libhdf5-serial-dev \
                        hdf5-tools \ 
                        libhdf5-dev \
                        zlib1g-dev \
                        'zip' \
                        libjpeg8-dev \
                        liblapack-dev \
                        libblas-dev \
                        gfortran
sudo pip3 install -U pip testresources setuptools
sudo pip3 install -U numpy==1.16.1 \
                     future==0.17.1 \
                     mock==3.0.5 \
                     h5py==2.9.0 \
                     keras_preprocessing==1.0.5 \
                     keras_applications==1.0.8 \
                     gast==0.2.2 \
                     futures \
                     protobuf \
                     pybind11
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow
#=============================================================

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

#======== Step5. install pytorch =============================
# Download source for pytorch 
wget https://nvidia.box.com/shared/static/veo87trfaawj5pfwuqvhl6mzc5b55fbj.whl -O torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl
sudo pip3 install torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl
# install torchvision
sudo pip3 install torchvision
#==============================================================

#======== Step6. install traitlets and jupyterlab  ============
sudo python3 -m pip install git+https://github.com/ipython/traitlets@master
sudo apt install -y npm
cd ~/Jetson_nano/driver
wget https://nodejs.org/dist/v12.13.0/node-v12.13.0-linux-arm64.tar.xz
tar -xJf node-v12.13.0-linux-arm64.tar.xz
cd node-v12.13.0-linux-arm64
sudo cp -R * /usr/local/
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
sudo jupyter labextension install @jupyterlab/statusbar
jupyter lab --generate-config
# if jupyter notebook has the error : " bash: jupyter: command not found "
# can enter this command to solve : " pip3 install --upgrade --force-reinstall jupyter notebook "
# if you have problem with "get 403 ..." , can install ipykernel with this text : sudo python3 -m pip install ipykernel --user
#=============================================================

#========= step7. install repo of jetbot and configure jetbot service ==================	 
## clone the jetbot repo with git 
#cd ~/Jetson_nano
#git clone https://github.com/NVIDIA-AI-IOT/jetbot 
cd ~/Jetson_nano/jetbot
sudo apt install python3-smbus cmake
sudo python3 setup.py install
cd jetbot/utils
python3 create_stats_service.py
sudo mv jetbot_stats.service /etc/systemd/system/jetbot_stats.service
sudo systemctl enable jetbot_stats
sudo systemctl start jetbot_stats
python3 create_jupyter_service.py
sudo mv jetbot_jupyter.service /etc/systemd/system/jetbot_jupyter.service
sudo systemctl enable jetbot_jupyter
sudo systemctl start jetbot_jupyter
#sudo chmod 777 /usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg/jetbot/* -R
#=======================================================================================

#======= step8. configure Jetson nano GPIO =================
sudo pip3 install Jetson.GPIO
sudo apt-get install git-all
cd ~/Jetson_nano
git clone https://github.com/NVIDIA/jetson-gpio.git
cd jetson-gpio
sudo python3 setup.py install
#===========================================================

#======= ste19. configure jetcam ===============
cd ~/Jetson_nano/jetcam
sudo python3 setup.py install
#===========================================

#==== Step10. configure SWAP size , ideal is double RAM. Default is 4G =================
## you can use [ df -h ] to see how space you can use on microSD now
#             [ free -h ] to see how space you can use with swap    

size=4G
cd
sudo fallocate -l $size /swapfile
sudo chmod 600 /swapfile
ls -lh /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon -show
sudo cp /etc/fstab /etc/fstab.bak
## long time to use
sudo echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
## After this step , please reboot , and can use " free -h " to view information.
#=======================================================================================

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

#===== setup wifi SSID ===============
## disconnect wifi
#sudo nmcli device disconnect wlan0

## rescan wifi 
#sudo nmcli device wifi rescan

## list wifi
#sudo nmcli device wifi list

## connect wifi
#sudo nmcli device wifi connect "SSID" password "SSID password"

#============================================
