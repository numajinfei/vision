FROM jadehu/ros2:feature_a_test AS test
LABEL maintainer=numajinfei@163.com

FROM ros:galactic

COPY --from=test /usr/local/lib/libpaho* /opt/mqtt/lib/
COPY --from=test /opt/pylon/lib /opt/pylon/lib/
COPY --from=test /opt/opencv/lib /opt/opencv/lib/
COPY --from=test /ws/install /ws/install

COPY --from=test /etc/ld.so.conf.d/Pylon.conf /etc/ld.so.conf.d/Pylon.conf
COPY --from=test /etc/ld.so.conf.d/OpenCV.conf /etc/ld.so.conf.d/OpenCV.conf
COPY --from=test /etc/ld.so.conf.d/Pcl.conf /etc/ld.so.conf.d/Pcl.conf
COPY --from=test /etc/ld.so.conf.d/mqtt.conf /etc/ld.so.conf.d/mqtt.conf


COPY ./script ./
# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  vim \
  libgpiod-dev \
  libpcl-segmentation1.10 \
  && rm -rf /var/lib/apt/lists/*

RUN sed -i 's?opt/ros/$ROS_DISTRO?ws/install?g' ros_entrypoint.sh \
    && sed -i '6i sleep 3 \n . /run_ros2.sh' ros_entrypoint.sh
    #&& sed -i '6i echo $PWD > dir.txt' ros_entrypoint.sh

#ENV LD_LIBRARY_PATH="/opt/mqtt/lib/:/opt/pylon/lib:/opt/opencv/lib:/opt/pcl/lib:$LD_LIBRARY_PATH"
ENV LD_LIBRARY_PATH="/opt/mqtt/lib/:/opt/pylon/lib:/opt/opencv/lib:$LD_LIBRARY_PATH"



#ENTRYPOINT ["sh", "-c", "/ros_entrypoint.sh"]