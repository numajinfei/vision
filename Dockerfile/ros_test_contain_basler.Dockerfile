FROM jadehu/ros2_build AS build
LABEL maintainer="numajinfei@163.com"

COPY ./src ./ws/src
COPY ./script ./script/

COPY --from=build /etc/ld.so.conf.d/Pylon.conf /etc/ld.so.conf.d/Pylon.conf
COPY --from=build /etc/ld.so.conf.d/OpenCV.conf /etc/ld.so.conf.d/OpenCV.conf
COPY --from=build /etc/ld.so.conf.d/Pcl.conf /etc/ld.so.conf.d/Pcl.conf
COPY --from=build /etc/ld.so.conf.d/mqtt.conf /etc/ld.so.conf.d/mqtt.conf

WORKDIR ./ws


#ENV PYLON_ROOT=/opt/pylon
#ENV LD_LIBRARY_PATH="/usr/local/lib/:/opt/pylon/lib:/opt/opencv/lib:/opt/pcl/lib:$LD_LIBRARY_PATH"

RUN /ros_entrypoint.sh \
  colcon build --packages-ignore camera_galaxy --cmake-args  -DCMAKE_BUILD_TYPE=Release 
  #colcon build --packages-select mqtt_ros camera_basler gpio_raspberry --cmake-args -DCMAKE_BUILD_TYPE=Release 
