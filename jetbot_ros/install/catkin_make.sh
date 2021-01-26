#!/bin/bash

main_path="Jetson_nano/jetbot_ros"
work_space="catkin_ws"
cvbridge_path="cvbridge_build_ws"

cd ~/$main_path/$cvbridge_path
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin config --install
python3-config --includes
catkin build
source ~/$main_path/$cvbridge_path/"install"/setup.bash --extend

cd ~/$main_path/$work_space
catkin_make #--cmake-args \
            #-DPYTHON_EXECUTABLE=/usr/bin/python3 \
            #-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            #-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so

echo "source ~/$main_path/setup/environment.sh" >> ~/.bashrc
source ~/.bashrc
