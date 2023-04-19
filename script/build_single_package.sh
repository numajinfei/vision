#!/bin/bash
###
 # @Descripttion: 
 # @version: 
 # @Author: hw
 # @Date: 2022-10-12 13:51:45
 # @LastEditors: hw
 # @LastEditTime: 2022-10-12 13:55:04
### 
node=$1
if [ ! -n "$node" ]; then
    echo "Please input a package name!"
fi

#source install/setup.sh
#source /opt/ros/galactic/setup.bash
source install/setup.sh
colcon build --packages-select $node --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.sh

