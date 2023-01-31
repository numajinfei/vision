#!/bin/bash
###
 # @Descripttion: 
 # @version: 
 # @Author: hw
 # @Date: 2022-10-12 13:51:45
 # @LastEditors: hw
 # @LastEditTime: 2023-01-05 13:16:58
### 
node=$1
type=$2
if [ ! -n "$node" ]; then
    echo "Please input a package name!"
    exit 0
fi

source install/setup.sh
colcon build --packages-select $node --cmake-args -DCMAKE_BUILD_TYPE=Release

