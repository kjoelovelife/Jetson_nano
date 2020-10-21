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

#==== Step1. install Jupyter Clickable Image Widget =================
# reference : https://computingforgeeks.com/how-to-install-nodejs-on-ubuntu-debian-linux-mint/
#             https://github.com/jaybdub/jupyter_clickable_image_widget/tree/no_typescript
sudo apt update
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt -y install nodejs
sudo apt -y  install gcc g++ make
cd ~/Jetson_nano/jetson-dlinano
git clone https://github.com/jaybdub/jupyter_clickable_image_widget
cd jupyter_clickable_image_widget
sudo python3 setup.py build
sudo npm run build
sudo pip3 install .
sudo jupyter labextension install .
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
#====================================================================

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
