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
#sudo swapon -show
#sudo cp /etc/fstab /etc/fstab.bak

## long time to use

#echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

## After this step , please reboot , and can use " free -h " to view information.
#=======================================================================================

#======== Step2. apt update and upgrade ==============================
#sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
#sudo apt-get update
#sudo apt-get upgrade
#sudo apt autoremove
#==============================================================

#======== Step3. install dependencies ============================================
#cd
#sudo apt-get install -y build-essential cmake 'pkg-config' \
#                        libjpeg-dev libtiff5-dev libpng-dev libfreetype6-dev \
#                        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
#                        libxvidcore-dev libx264-dev \
#                        libgtk2.0-dev libgtk-3-dev \
#                        libatlas-base-dev gfortran \
#                        python-dev python3-dev python3-pil python3-smbus \
#                        python-pip python3-pip \
#                        firefox cmake gfortran rsync python3-smbus \
#                        libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev 'zip' libjpeg8-dev              
#=================================================================================

#======== Step4. configure virtualenv ================================
## if you want to exit virtualenv , just enter "deactivate"
#sudo apt-get install -y virtualenv
#cd
#mkdir envs;cd envs
#virtualenv -p python3 AI
## if you want to use virtualenv AI, please enter bellow command 
#echo "source ~/envs/AI/bin/activate" >> ~/.bashrc
#source ~/envs/AI/bin/activate
#======== configure OpenCV ( Theese command just for Jetson-nano developer kit) ========
#cd ~/envs/AI/lib/python3.6/site-packages/
#ln -s /usr/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so
#=======================================================================================

#========= Step5. install library with python. ===================
## Install package with python3
#sudo python3 -m pip install --upgrade pip setuptools wheel
#sudo python3 -m pip install git+https://github.com/ipython/traitlets@master
#  For installing scipy , need to install gfortran :   
#sudo pip3 install -U numpy nbresuse matplotlib keras Cython Jetson.GPIO Adafruit-MotorHAT h5py \
#                scipy imutils testresources \
#                grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor protobuf keras-applications keras-preprocessing wrapt google-pasta


# if you want to install tensorflow must enter this command : sudo pip3 install --upgrade pip
# And , then modified file  「/usr/bin/pip3」 , 「 from  pip import __main__」 , 「 __main__._main() 」 
## Install tensorflow
#sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v43 'tensorflow<2'
## can find on this page : https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html

# Download source for pytorch 
#wget https://nvidia.box.com/shared/static/veo87trfaawj5pfwuqvhl6mzc5b55fbj.whl -O torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl
#sudo pip3 install torch-1.1.0a0+b457266-cp36-cp36m-linux_aarch64.whl
# install torchvision
#sudo pip3 install torchvision

## configure openCV_contrib
work_path="Jetson_nano/driver"
cd ~/$work_path
curl -L https://github.com/opencv/opencv/archive/4.1.1.zip -o opencv-4.1.1.zip
curl -L https://github.com/opencv/opencv_contrib/archive/4.1.1.zip -o opencv_contrib-4.1.1.zip
unzip opencv-4.1.1.zip
unzip opencv_contrib-4.1.1.zip
# remember to edit ../opencv-4.1.1/modules/core/include/opencv2/core/private.hpp , "eigen3/Eigen/Core" on line 66

cd ~/$work_path/opencv-4.1.1
mkdir build
cd build
cmake -D WITH_CUDA=ON \
    	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.1.1/modules \
    	-D WITH_GSTREAMER=ON \
    	-D WITH_LIBV4L=ON \
    	-D BUILD_opencv_python2=ON \
    	-D BUILD_opencv_python3=ON \
    	-D BUILD_TESTS=OFF \
    	-D BUILD_PERF_TESTS=OFF \
    	-D BUILD_EXAMPLES=OFF \
    	-D CMAKE_BUILD_TYPE=RELEASE \
    	-D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make
sudo make install
sudo ldconfig
#==================================================================

#========= step6. configure jupyter lab ==============================================		 
#$ Install traitlets (master, to support the unlink() method)
#sudo python3 -m pip install git+https://github.com/ipython/traitlets@master

## Install jupyter lab
#sudo apt-get install npm
## need modified file「 /usr/bin/pip3 」  , 「 from  pip import main」 , 「 main() 」 
#sudo pip3 install jupyter jupyterlab nbresuse
#sudo jupyter serverextension enable --py nbresuse
#cd ~/Jetson_nano/driver
#wget https://nodejs.org/dist/v12.13.0/node-v12.13.0-linux-arm64.tar.xz
#tar -xJf node-v12.13.0-linux-arm64.tar.xz
#cd node-v12.13.0-linux-arm64
#sudo cp -R * /usr/local/
#sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
#sudo jupyter labextension install @jupyterlab/statusbar
#jupyter lab --generate-config
#jupyter notebook password
# if jupyter notebook has the error : " bash: jupyter: command not found "
# can enter this command to solve : " pip3 install --upgrade --force-reinstall jupyter notebook "
# if you have problem with "get 403 ..." , can install ipykernel with this text : sudo python3 -m pip install ipykernel --user
#=======================================================================================

#========= step7. configure jetbot service  =========================================	 
## clone the jetbot repo with git 
#cd ~/Jetson_nano
#git clone https://github.com/NVIDIA-AI-IOT/jetbot 
#cd ~/Jetson_nano/jetbot
#sudo python3 setup.py install
#cd jetbot/utils
#python3 create_stats_service.py
#sudo mv jetbot_stats.service /etc/systemd/system/jetbot_stats.service
#sudo systemctl enable jetbot_stats
#sudo systemctl start jetbot_stats
#python3 create_jupyter_service.py
#sudo mv jetbot_jupyter.service /etc/systemd/system/jetbot_jupyter.service
#sudo systemctl enable jetbot_jupyter
#sudo systemctl start jetbot_jupyter
#sudo chmod 777 /usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg/jetbot/* -R
#=======================================================================================

#======= step8. configure Jetson nano GPIO =================
#sudo cp ~/Jetson_nano/driver/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d
#sudo udevadm control --reload-rules && sudo udevadm trigger
#sudo groupadd -f -r gpio
#sudo usermod  -a -G gpio $USER

#======= ste9. configure i2c ===============
# Enable i2c permissions
sudo usermod -aG i2c $USER
#===========================================

#======= ste10. configure jetcam ===============
cd ~/Jetson_nano/jetcam
sudo python3 setup.py install
#===========================================


# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

#===== step9. setup wifi SSID ===============
## disconnect wifi
#sudo nmcli device disconnect wlan0

## rescan wifi 
#sudo nmcli device wifi rescan

## list wifi
#sudo nmcli device wifi list

## connect wifi
#sudo nmcli device wifi connect "SSID" password "SSID password"

#============================================
