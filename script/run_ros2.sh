#!/bin/bash
echo "pwd1 is $PWD" > dir.txt
echo "step 1..." >> dir.txt
cd /ws
echo "pwd2 is $PWD" >> dir.txt
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
echo $PWD
echo "-------------------------------"
ros2 run gpio_raspberry gpio_raspberry_node --ros-args --params-file install/gpio_raspberry/share/gpio_raspberry/config/params.yaml &
sleep 1
ros2 run inclinometer inclinometer_node --ros-args --params-file install/inclinometer/share/inclinometer/config/params.yaml &
sleep 1
ros2 run motor_encoder motor_encoder_node --ros-args --params-file install/motor_encoder/share/motor_encoder/config/params.yaml &
sleep 1
ros2 run mqtt_ros mqtt_ros_node --ros-args --params-file install/mqtt_ros/share/mqtt_ros/config/params.yaml -r \~/request:=/mqtt_json_ros/request -r \~/response:=/mqtt_json_ros/response &
sleep 2
ros2 launch pipeline pipeline.launch.py &
sleep 3
ros2 run script_json script_json_node --ros-args --params-file install/script_json/share/script_json/config/params.yaml -r \~/request:=/mqtt_json_ros/request -r \~/response:=/mqtt_json_ros/response -r \~/result:=/common/result &
cd ~
echo "-------------------------------"
echo $PWD
echo "-------------------------------"