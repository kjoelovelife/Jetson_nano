#!/usr/bin/env bash

# Shell script scripts to install useful tools , such as opencv , pytorch...
# -------------------------------------------------------------------------
#Copyright © 2020 Wei-Chih Lin , kjoelovelife@gmail.com 
#update : 2020.10.26

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

## set parameter

### file path
main_path="Jetson_nano"

### install source ###
install_source="install_script"


### ros information ###
ros1_distro="melodic"

### hardware ###
platform="tegra"
kernel=$(uname -a)
shell=`echo $SHELL | awk -F '/' '{print $NF}'`

### install package ###
ros1=false
ssh_setup=false
jetson_inference=false
jetbot=false
jetcam=false
opencv_430=false
swap=false

### swap size ###
swap_size=4G


## setup msg
msg="You install :"
msg_ros=""
msg_ssh=""
msg_jetson_inference=""
msg_jetbot=""
msg_jetcam=""
msg_430=""
msg_swap=""

## Configure power mode
if [[ $kernel =~ $platform ]] ; then
    #echo $PASSWORD | sudo -S nvpmodel -m1 # 5W
    sudo -S nvpmodel -m0 # 10W
    sudo -S nvpmodel -q
fi

## install ros
echo -n "Do you want to install ROS automatically? (y/N): "
read ros_install
if [ "$ros_install" '==' "y" ] || [ "$ros_install" '==' "Y" ];
then
    # Install ROS 1 melodic
    ros1=true
else
    echo "Skip installing ROS"
fi

## setup ssh
echo -n "Do you want to setup ssh? (y/N): "
read ssh_install
if [ "$ssh_install" '==' "y" ] || [ "$ssh_install" '==' "Y" ];
then
    # setup ssh
    ssh_setup=true
else
    echo "Skip setup ssh."
fi

## install jetson-inference
echo -n "Do you want to install jetson-inference? (y/N): "
read jetson_inference_install
if [ "$jetson_inference_install" '==' "y" ] || [ "$jetson_inference_install" '==' "Y" ];
then
    # setup jetson-inference
    jetson_inference=true
else
    echo "Skip setup jetson-inference."
fi

## install jetbot
echo -n "Do you want to install jetbot? (y/N): "
read jetbot_install
if [ "$jetbot_install" '==' "y" ] || [ "$jetbot_install" '==' "Y" ];
then
    # setup jetbot
    jetbot=true
else
    echo "Skip setup jetbot."
fi

## install jetbcam
echo -n "Do you want to install jetcam? (y/N): "
read jetcam_install
if [ "$jetcam_install" '==' "y" ] || [ "$jetcam_install" '==' "Y" ];
then
    # setup jetcam
    jetcam=true
else
    echo "Skip setup jetcam."
fi

## install opencv 4.3.0
echo -n "Do you want to install opencv-4.3.0 and opencv-4.3.0-contrib? (y/N): "
read opencv430_install
if [ "$opencv430_install" '==' "y" ] || [ "$opencv430_install" '==' "Y" ];
then
    # setup opencv430
    opencv_430=true
else
    echo "Skip setup opencv-4.3.0 ."
fi

## configure SWAP size
echo -n "Do you want to configure SWAP size(4G)? (y/N): "
read swap_
if [ "$swap_" '==' "y" ] || [ "$swap_" '==' "Y" ];
then
    # read swap_size
    swap=true
else
    echo "Skip swap"
fi

## apt upgrade
#sudo apt update
#sudo apt-get install firefox gedit

## which package install

if [ $swap == true ] ; then
    cd ~/$main_path/$install_source
    ./swap.sh
    msg_swap="swap $size "
fi

if [ $ssh_setup == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | ./ssh_setup.sh
    msg_ssh="SSH."
fi

if [ $opencv_430 == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | sudo ./opencv430_install.sh
    msg_430="opencv-4.3.0 and opencv-4.3.0-contrib"
fi

if [ $jetson_inference == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | ./jetson_inference_install.sh
    msg_jetson_inference="jetson_inference."
fi

if [ $jetbot == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | sudo ./jetbot_install.sh
    msg_jetbot="jetbot."
fi

if [ $jetcam == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | sudo ./jetcam_install.sh
    msg_jetcam="jetcam."
fi

if [ $ros1 == true ] ; then
    cd ~/$main_path/$install_source
    echo $PASSWORD | ./ros_install_melodic.sh
    msg_ros="ROS , version : $ros1_distro ."
fi

## install done
echo $msg
echo $msg_ros
echo $msg_ssh
echo $msg_jetson_inference
echo $msg_jetbot
echo $msg_jetcam
echo $msg_430

