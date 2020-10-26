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

sudo apt-get install -y vim
sudo apt-get install -y gedit
sudo apt-get install -y 'ssh'

mkdir ~/.ssh
sudo chmod 0700 ~/.ssh 
echo "HostKeyAlgorithms ssh-rsa" >> ~/.ssh/config

ssh-keygen -P "" -h -f $HOME/.ssh/$USER@$HOSTNAME

echo "IdentityFile ~/.ssh/$USER@$HOSTNAME" >> ~/.ssh/config

sudo service ssh restart

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
