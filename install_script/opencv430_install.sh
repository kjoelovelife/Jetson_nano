#!/bin/bash
#
# Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

main_path="Jetson_nano"

echo "** Remove other OpenCV first"
#sudo sudo apt-get purge *libopencv*


echo "** Install requirement"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y python2.7-dev python3.6-dev python-dev python-numpy python3-numpy
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt-get install -y curl
sudo apt-get update


echo "** Download opencv-4.3.0"
cd ~/$main_path/driver
curl -L https://github.com/opencv/opencv/archive/4.3.0.zip -o opencv-4.3.0.zip
curl -L https://github.com/opencv/opencv_contrib/archive/4.3.0.zip -o opencv_contrib-4.3.0.zip
unzip opencv-4.3.0.zip
unzip opencv_contrib-4.3.0.zip
cd opencv-4.3.0/


echo "** Apply patch"
sed -i 's/include <Eigen\/Core>/include <eigen3\/Eigen\/Core>/g' modules/core/include/opencv2/core/private.hpp
sed -i 's/{CUDNN_INCLUDE_DIR}\/cudnn.h/{CUDNN_INCLUDE_DIR}\/cudnn_version.h/g' cmake/FindCUDNN.cmake


echo "** Building..."
mkdir release
cd release/
cmake -D WITH_CUDA=ON -D WITH_CUDNN=OFF -D CUDA_ARCH_BIN="5.3,6.2,7.2" -D CUDA_ARCH_PTX="" -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.3.0/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python2=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
#sudo make install
#echo 'export PYTHONPATH=$PYTHONPATH:'$PWD'/python_loader/' >> ~/.bashrc
#source ~/.bashrc


echo "** Install opencv-4.3.0 successfully"
echo "** Bye :)"
