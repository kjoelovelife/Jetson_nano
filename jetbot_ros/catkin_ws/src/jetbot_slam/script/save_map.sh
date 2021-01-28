#!/usr/bin/env bash
echo "Use map_server to save map..."
path="/home/$USER/Jetson_nano/jetbot_ros/catkin_ws/src/jetbot_slam/map"
map_name="map_"$(date "+%Y%m%d_%H%M%S")
if [ $# -gt 0 ]; then
    # provided a hostname, use it as map name
    map_name=$1
    echo "your map name is $map_name."
else
    echo "No map name provided. Using $map_name."
fi
rosrun map_server map_saver -f "$path/$map_name"

if test -e $path/$map_name".yaml" && test -e $path/$map_name".pgm"; then
    echo -n ""
    #echo "Done. Below is the map information."
    #echo "map name: $map_name"
    #echo "folder  : $path"
else
    echo "Failed! Please check your folder path is correct or there has a map can save on ROS."
fi
