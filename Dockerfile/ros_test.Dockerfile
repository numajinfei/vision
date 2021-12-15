#FROM jadehu/ros2_build
FROM docker_ros2_build:latest
LABEL maintainer="numajinfei@163.com"

COPY ./src ./ws/src
COPY ./script ./script
WORKDIR ./ws


ENV PYLON_ROOT=/opt/pylon
ENV LD_LIBRARY_PATH="/usr/local/lib/:/opt/pylon/lib:/opt/opencv/lib:/opt/pcl/lib:$LD_LIBRARY_PATH"
RUN /ros_entrypoint.sh \
    #/script/ros2_run.sh \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
  #/script/ros2_run.sh
  #colcon build --packages-select mqtt_ros camera_basler gpio_raspberry --cmake-args -DCMAKE_BUILD_TYPE=Release 
  #colcon build --packages-select mqtt_ros camera_basler gpio_raspberry --cmake-args -DCMAKE_BUILD_TYPE=Release 
  #. install/setup.bash
