#!/usr/bin/env bash

# Shell script scripts to install gedit and ssh , then create ssh key in ~/.ssh/username@hostname 
# -------------------------------------------------------------------------
#Copyright © 2018 Wei-Chih Lin , kjoelovelife@gmail.com 

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
# This script is part of project 'duckietown' , MIT. 
# Visit https://github.com/duckietown
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

echo -n "Please enter your password: "
read -s PASSWORD
echo ""

echo $PASSWORD | sudo -S apt install python-pip python-pil
#sudo pip3 install --upgrade numpy
download='https://doc-0k-24-docs.googleusercontent.com/docs/securesc/ubv6ihq9jk1tbbsiisbf29lqq8h7qa74/jam82obup0g6dhmbs052aagq66tg8ad9/1608186600000/01933900165054958065/05089611368947298974Z/1M7BeIHXcaMJKqyhVwlpeasn_3OpEDjkA?e=download&nonce=p8hfno5imjnvs&user=05089611368947298974Z&hash=c8v65ftpjrucbr5mhoc5nn92ei2khqlf'

wget -O torch-1.4.0-cp27-cp27mu-linux_aarch64.whl https://nvidia.box.com/shared/static/yhlmaie35hu8jv2xzvtxsh0rrpcu97yj.whl
echo $PASSWORD | sudo -S pip2 install torch-1.4.0-cp27-cp27mu-linux_aarch64.whl
echo $PASSWORD | sudo -S pip2 install torchvision

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

#https://drive.google.com/file/d/1M7BeIHXcaMJKqyhVwlpeasn_3OpEDjkA/view?usp=sharing
