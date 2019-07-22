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
#==============================================================

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
			libcanberra-gtk3-module \
			gfortran \
			python3-pip \
			python3-pil \
			python3-smbus \
                        firefox \
                        'pkg-config'
			

## And can install [ pkg-config , zip ]
#=======================================================================================

#======== configure virtualenv ================================
## if you want to exit virtualenv , just enter "deactivate"
sudo apt-get install -y virtualenv
cd
mkdir envs;cd envs
virtualenv -p python3 cv
echo "source ~/envs/AI/bin/activate" >> ~/.bashrc
source ~/.bashrc

### if you want to use virtualenv AI, please enter bellow command 
#echo "source ~/envs/AI/bin/activate" >> ~/.bashrc
#source ~/envs/AI/bin/activate
#======== configure OpenCV ( Theese command just for Jetson-nano developer kit) ========
#cd ~/envs/AI/lib/python3.6/site-packages/
#ln -s /usr/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so
#=======================================================================================

#================ install library for machine learning with python. ===================
## Install package with python3
#python3 -m pip install --upgrade pip setuptools wheel
#  For installing scipy , need to install gfortran :   
pip3 install matplotlib scikit-build imutils keras Cython scikit-learn Jetson.GPIO Adafruit-MotorHAT numpy scipy grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor


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

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/$work_path/opencv_contrib-3.3.1/modules \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D BUILD_EXAMPLES=ON ..

make -j4
sudo make install
sudo ldconfig
cd ~/$work_path
rm -rf opencv-3.3.1 opencv_contrib-3.3.1
ls -l /usr/local/lib/python3.6/dist-packages/
cd ~/envs/cv/lib/python3.6/site-packages/
ln -s /usr/local/lib/python3.6/dist-packages/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
cd

#================== About python3 ===========================================================
# if you want install in system , You need to upgrade pip3 :  [ python3 -m pip install --upgrade pip ]
# And then , you need to modified " /usr/bin/pip " , detail : https://stackoverflow.com/questions/49836676/error-after-upgrading-pip-cannot-import-name-main
# Notice : you need to exit virtualenv AI , please enter " deactivate "

#===========================================================================================
#=======================================================================================



