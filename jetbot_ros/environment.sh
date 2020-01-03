#!/bin/bash

echo "Activating ROS..."
source /opt/ros/melodic/setup.bash
echo "...done."

echo "Setting up PYTHONPATH."
export PYTHONPATH=~/Jetson_nano/jetbot_ros/catkin_ws/src:$PYTHONPATH

echo "Setup ROS_HOSTNAME."
export ROS_HOSTNAME=$HOSTNAME.local
export workspace=$HOME/Jetson_nano/jetbot_ros

source $workspace/catkin_ws/devel/setup.bash

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
