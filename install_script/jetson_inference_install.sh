#!/usr/bin/env bash

# Shell script scripts to install useful tools , such as opencv , pytorch...
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

## file path
main_path="Jetson_nano"

# Detect Jetson platform by checking the kernel release string
kernel=$(uname -r)
platform="tegra"

# git and cmake should be installed
sudo apt-get install -y git cmake

if [[ $kernel =~ $platform ]] ; then
    # clone the repo and submodules
    git clone https://github.com/dusty-nv/jetson-inference ~/$main_path/driver/jetson-inference
    cd ~/$main_path/driver/jetson-inference
    git submodule update --init

    echo "Do not leave your seat! Some packages require manual selection."
    echo "We suggest you install all software and network modules."
    sleep 5s

    # build from source
    mkdir build
    cd build
    cmake ../
    make

    # install libraries
    sudo make install
else
    echo "Not running on a Jetson platform (kernel: $kernel). Skipping jetson-inference build."
fi
