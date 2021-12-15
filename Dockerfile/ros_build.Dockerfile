#FROM jadehu/ros2_opencv AS opencv
#FROM localhost:5000/vision_test_ros2_opencv AS opencv
FROM docker_ros2_opencv:latest
FROM docker_ros2_pcl:latest
#FROM jadehu/ros2_pcl
LABEL maintainer="numajinfei@163.com"


# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  libgpiod-dev \
  vim \
  && rm -rf /var/lib/apt/lists/*

# Install nlohmann json
RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
  && mkdir -p /usr/local/include/nlohmann \
  && mv json.hpp /usr/local/include/nlohmann/json.hpp

# Build paho mqtt c
RUN wget -O paho.mqtt.c.tar.gz https://github.com/eclipse/paho.mqtt.c/archive/refs/tags/v1.3.9.tar.gz \
  && tar -xzf paho.mqtt.c.tar.gz \
  && rm paho.mqtt.c.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \    
    -D PAHO_ENABLE_TESTING=OFF \
    -D PAHO_WITH_SSL=OFF \
    -D PAHO_HIGH_PERFORMANCE=ON \
    -S paho.mqtt.c-1.3.9/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r paho.mqtt.c-1.3.9 build

# Build paho mqtt cpp
RUN wget -O paho.mqtt.cpp.tar.gz https://github.com/eclipse/paho.mqtt.cpp/archive/refs/tags/v1.2.0.tar.gz \
  && tar -xzf paho.mqtt.cpp.tar.gz \
  && rm paho.mqtt.cpp.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \
    #-D PAHO_BUILD_STATIC=TRUE \
    -D PAHO_WITH_SSL=OFF \
    -S paho.mqtt.cpp-1.2.0/ \
    -B build/ \
  && cmake --build build/ --target install \
  && rm -r paho.mqtt.cpp-1.2.0 build

# Install opencv
COPY --from=docker_ros2_opencv:latest /opt/opencv /opt/opencv

# Install galaxy
#COPY --from=basler /opt/pylon /opt/pylon
COPY --from=docker_ros2_opencv:latest /opt/pylon /opt/pylon
#COPY --from=basler /dev/bus/usb /dev/bus/usb
#RUN mv /opt/galaxy/lib/*/* /opt/galaxy/lib/

#ENV PYLON_ROOT=/opt/pylon
