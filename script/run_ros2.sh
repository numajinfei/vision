#!/bin/bash

echo "pwd1 is $PWD" > dir.txt
echo "1- current dir is: $PWD"
echo "step 1..." >> dir.txt

cd /ws

echo "pwd2 is $PWD" >> dir.txt
echo "2- current dir is: $PWD"
echo "step 2..." >> dir.txt
#. /ws/install/setup.bash
#. /ros_entrypoint.sh
#ros2 launch pipeline pipeline.launch.py&
#ros2 launch gpio_raspberry gpio_raspberry.launch.py&
#ros2 launch inclinometer inclinometer.launch.py&
#ros2 launch motor_encoder motor_encoder.launch.py&
#ros2 launch script_json script_json.launch.py&
#ros2 launch mqtt_ros mqtt_ros.launch.py&
echo "step 3..." >> dir.txt
echo "-------------------------------"
echo "3- current dir is: $PWD"
echo "-------------------------------"

#domain
export ROS_DOMAIN_ID=2

# Get current time
current_time=$(date +%Y-%m-%d)_$(date +%H-%M-%S)

mkdir -p ./ros2_log
echo "--> log dir is: $PWD/ros2_log"
ros2 run gpio_raspberry gpio_raspberry_node --ros-args --params-file install/gpio_raspberry/share/gpio_raspberry/config/params.yaml > ./ros2_log/gpio_$current_time.log 2>&1 &
sleep 1
ros2 run inclinometer inclinometer_node --ros-args --params-file install/inclinometer/share/inclinometer/config/params.yaml > ./ros2_log/inclinomete_$current_time.log 2>&1 &
sleep 1
ros2 run motor_encoder motor_encoder_node --ros-args --params-file install/motor_encoder/share/motor_encoder/config/params.yaml > ./ros2_log/motor_$current_time.log 2>&1 &
sleep 1
ros2 run mqtt_ros mqtt_ros_node --ros-args --params-file install/mqtt_ros/share/mqtt_ros/config/params.yaml -r \~/request:=/mqtt_json_ros/request -r \~/response:=/mqtt_json_ros/response > ./ros2_log/mqtt_$current_time.log 2>&1 &
sleep 2
ros2 launch pipeline pipeline.galaxy.launch.py > ./ros2_log/pipeline_$current_time.log 2>&1 &
sleep 3
ros2 run script_json script_json_node --ros-args --params-file install/script_json/share/script_json/config/params.yaml -r \~/request:=/mqtt_json_ros/request -r \~/response:=/mqtt_json_ros/response -r \~/result:=/common/result > ./ros2_log/scriptjson_$current_time.log 2>&1 &

cd -


echo "-------------------------------"
echo "4- current dir is: $PWD"
echo "-------------------------------"
