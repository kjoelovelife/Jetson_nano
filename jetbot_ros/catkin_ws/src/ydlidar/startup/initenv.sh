#!/bin/bash

sudo cp ydlidar.rules /etc/udev/rules.d 
sudo cp ydlidar-V2.rules /etc/udev/rules.d 
sudo cp ydlidar-2303.rules /etc/udev/rules.d 


service udev reload
sleep 2
service udev restart

