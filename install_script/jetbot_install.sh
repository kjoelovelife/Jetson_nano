#!/usr/bin/env bash

# Shell script scripts to install useful tools, ROS melodic on unbuntu 18.04 with Jetson-nano
# -------------------------------------------------------------------------
#Copyright © 2019 Wei-Chih Lin, kjoelovelife@gmail.com 

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


main_path="Jetson_nano"

# Step 1: Enable i2c permissions
sudo usermod -aG i2c $USER

# Step 2: apt update and install base Python packages
sudo apt-get update
sudo apt install -y python3-pip python3-pil
sudo pip3 install cython

# Step 3: Install TensorFlow dependencies and TensorFlow for JetPack 4.4
sudo apt-get install -y libhdf5-serial-dev \
                        hdf5-tools \
                        libhdf5-dev \
                        zlib1g-dev \
                        zip \
                        libjpeg8-dev \
                        liblapack-dev \
                        libblas-dev \
                        gfortran \
                        libcanberra-gtk-module \
                        libcanberra-gtk3-module
sudo pip3 install -U pip testresources setuptools
sudo pip3 install -U numpy==1.18.5 \
                     future==0.17.1 \
                     mock==3.0.5 \
                     h5py==2.9.0 \
                     keras_preprocessing==1.0.5 \
                     keras_applications==1.0.8 \
                     gast==0.2.2 \
                     futures \
                     protobuf \
                     pybind11
sudo -H pip3 install jetson-stats
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 'tensorflow<2'

# Step 4: Create a Python 3 virtualenv named 'AI' (optional, not activated by default)
sudo apt-get install -y virtualenv
cd
mkdir -p envs && cd envs
virtualenv -p python3 AI

# Step 5: Install traitlets and JupyterLab
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt -y install nodejs node-gyp gcc g++ make
sudo pip3 install jupyter jupyterlab
sudo python3 -m pip install git+https://github.com/ipython/traitlets@4.x
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
sudo jupyter labextension install @jupyterlab/statusbar

# Step 6: Install JetBot Python package and register systemd services
cd ~/$main_path/jetbot
sudo apt install -y python3-smbus cmake
sudo python3 setup.py install
cd ~/$main_path/jetbot/jetbot/utils
python3 create_stats_service.py
sudo mv jetbot_stats.service /etc/systemd/system/jetbot_stats.service
sudo systemctl enable jetbot_stats
sudo systemctl start jetbot_stats
python3 create_jupyter_service.py
sudo mv jetbot_jupyter.service /etc/systemd/system/jetbot_jupyter.service
sudo systemctl enable jetbot_jupyter
sudo systemctl start jetbot_jupyter

# Step 7: Install Jetson.GPIO
sudo pip3 install Jetson.GPIO
sudo apt-get -y install git-all
cd ~/$main_path/driver
git clone https://github.com/NVIDIA/jetson-gpio.git
cd jetson-gpio
sudo python3 setup.py install
