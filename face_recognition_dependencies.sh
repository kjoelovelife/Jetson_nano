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

size=8G
cd
sudo fallocate -l $size /swapfile
sudo chmod 600 /swapfile
ls -lh /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon –show
sudo cp /etc/fstab /etc/fstab.bak

## long time to use

echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

## After this step , please reboot , and can use " free -h " to view information.
#=======================================================================================

#======== Step2. apt update and upgrade ==============================
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
#==============================================================

#======== Step3. install dependencies ============================================
cd
sudo apt-get install -y build-essential cmake 'pkg-config' \
                        libjpeg-dev libtiff5-dev libpng-dev libfreetype6-dev \
                        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
                        libxvidcore-dev libx264-dev \
                        libgtk2.0-dev libgtk-3-dev \
                        libatlas-base-dev gfortran \
                        python-dev python3-dev python3-pil python3-smbus \
                        python-pip python3-pip \
                        firefox cmake gfortran rsync python3-smbus \
                        libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev 'zip' libjpeg8-dev              
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
sudo python3 -m pip install --upgrade pip setuptools wheel
#  For installing scipy , need to install gfortran :   
sudo pip3 install -U numpy nbresuse matplotlib keras Cython Jetson.GPIO Adafruit-MotorHAT h5py \
                scipy imutils \
                grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor protobuf keras-applications keras-preprocessing wrapt google-pasta

## Install tensorflow
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow-gpu
## can find on this page : https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html

# Download source for pytorch 
wget https://nvidia.box.com/shared/static/veo87trfaawj5pfwuqvhl6mzc5b55fbj.whl -O torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl
sudo pip3 install torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl --user
# install torchvision
sudo pip3 install torchvision

## configure openCV_contrib
work_path="Jetson_nano/OpenCV-Face-Recognition"
cd ~/$work_path
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.1.zip
unzip opencv.zip

wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.1.zip
unzip opencv_contrib.zip

cd ~/$work_path/opencv-3.3.1
mkdir build
cd build
cmake -D OPENCV_EXTRA_MODULES_PATH=~/$work_path/opencv_contrib-3.3.1/modules ..
make -j5
sudo make install
sudo ldconfig
#==================================================================

#========= step6. configure jupyter lab ==============================================		 
#$ Install traitlets (master, to support the unlink() method)
sudo python3 -m pip install git+https://github.com/ipython/traitlets@master

#$ Install jupyter lab
sudo apt-get install nodejs npm
sudo -H pip3 install jupyter jupyterlab
sudo jupyter serverextension enable --py nbresuse
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
cd ~/Jetson_nano
git clone https://github.com/jupyterlab/jupyterlab-statusbar
cd jupyterlab-statusbar 
npm install
npm run build
sudo jupyter lab build
#sudo jupyter labextension install @jupyterlab/statusbar
jupyter lab --generate-config
jupyter notebook password

# if jupyter notebook has the error : " bash: jupyter: command not found "
# can enter this command to solve : " pip3 install --upgrade --force-reinstall --no-cache-dir jupyter notebook "
#=======================================================================================

#========= step7. configure jetbot service  =========================================	 
## clone the jetbot repo with git 
cd ~/Jetson_nano
git clone https://github.com/NVIDIA-AI-IOT/jetbot
/bin/'cp' ~/Jetson_nano/OpenCV-Face-Recognition/USB_camera/camera.py ~/Jetson_nano/jetbot/jetbot/camera.py  
cd ~/Jetson_nano/jetbot
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
sudo chmod 777 /usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg/jetbot/* -R
#=======================================================================================

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
