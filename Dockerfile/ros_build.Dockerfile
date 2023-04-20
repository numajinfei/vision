# FROM jadehu/ros2:opencv AS opencv
FROM jadehu/ros2:basler AS basler
FROM jadehu/ros2:galaxy AS galaxy
# FROM jadehu/ros2:pcl AS pcl
LABEL maintainer="numajinfei@163.com"

FROM ros:galactic


# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  vim \
  unzip \
  net-tools \
  iputils-ping \
  libgpiod-dev \
  libyaml-cpp-dev \

# nav and pcl:
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-bondcpp \
  ros-${ROS_DISTRO}-test-msgs \
  ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
  ros-${ROS_DISTRO}-rviz-common \
  ros-${ROS_DISTRO}-rviz-default-plugins \
  ros-${ROS_DISTRO}-angles \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-ompl \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-libg2o \

  graphicsmagick* \
  libsuitesparse-dev \
  libsdl-image1.2-dev \
  libsdl-dev \

# opencv  
  libopencv-dev \

# vision rependence packages:
  ros-${ROS_DISTRO}-cv-bridge \
  

# AI rependence packages:
  python3-pip \
  && pip3 install opencv-python \

# Phoxicontrol dependencies (gui...)
  && apt-get install -y -q software-properties-common && apt-add-repository universe \
  && apt-get update -y \
  #---ubuntu18.04:
  # && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.0.0 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb
  #---ubuntu20.04:
  && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.1 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb libxcb-xinerama0 libpcre2-16-0 \


# rm 
  && rm -rf /var/lib/apt/lists/* \
  && echo -e "[show2]--> Current path is: $PWD, \n ls:\n`ls` \nrelease file path:`ls ./downloads`" \
  && echo "[show2]--> OS information1: $(lsb_release -a)" \
  && echo "[show2]--> OS information2: $(uname -a)" \
  && echo "[show2]--> OS user: $USER"


# Install AI Reference Packages
RUN wget https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/torch-1.13.0+cpu-cp38-cp38-linux_x86_64.whl \
  && pip3 install torch-1.13.0+cpu-cp38-cp38-linux_x86_64.whl \
# Download phoxi.run
  # && wget -O /tmp/phoxi.run http://121.4.181.196:9000/build/phoxi.run \
  && wget -O /tmp/phoxi.run https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/phoxi.run \
  && chmod +x /tmp/phoxi.run

# Install nlohmann json
# RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
RUN wget https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/json.hpp \
  && mkdir -p /usr/local/include/nlohmann \
  && mv json.hpp /usr/local/include/nlohmann/json.hpp

# Build paho mqtt c
# RUN wget -O paho.mqtt.c.tar.gz https://github.com/eclipse/paho.mqtt.c/archive/refs/tags/v1.3.9.tar.gz \
RUN wget -O paho.mqtt.c.tar.gz https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/paho.mqtt.c-1.3.9.tar.gz \
  && tar -xzf paho.mqtt.c.tar.gz \
  && rm paho.mqtt.c.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \    
    -D PAHO_ENABLE_TESTING=OFF \
    -D PAHO_WITH_SSL=OFF \
    -D PAHO_HIGH_PERFORMANCE=ON \
    -S paho.mqtt.c-1.3.9/ \
    -Bbuild/ \
  && cmake --build build/ --target install \
  && rm -r paho.mqtt.c-1.3.9 build

# Build paho mqtt cpp
# RUN wget -O paho.mqtt.cpp.tar.gz https://github.com/eclipse/paho.mqtt.cpp/archive/refs/tags/v1.2.0.tar.gz \
RUN wget -O paho.mqtt.cpp.tar.gz https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/paho.mqtt.cpp-1.2.0.tar.gz \
  && tar -xzf paho.mqtt.cpp.tar.gz \
  && rm paho.mqtt.cpp.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \
    #-D PAHO_BUILD_STATIC=TRUE \
    -D PAHO_WITH_SSL=OFF \
    -S paho.mqtt.cpp-1.2.0/ \
    -Bbuild/ \
  && cmake --build build/ --target install \
  && echo "/opt/mqtt/lib" >> /etc/ld.so.conf.d/mqtt.conf \
  && mkdir -p /opt/mqtt/lib \
  && cp /usr/local/lib/libpaho* /opt/mqtt/lib \
  && rm -r paho.mqtt.cpp-1.2.0 build

# Build libredwg
RUN wget -O libredwg.tar.gz https://ftp.gnu.org/gnu/libredwg/libredwg-0.12.4.tar.gz \
  && tar -xzf libredwg.tar.gz \
  && rm libredwg.tar.gz \
  && cd libredwg-0.12.4 \
  #&& ./configure --prefix=/opt/redwg --disable-bindings --enable-release \
  && ./configure --disable-bindings --enable-release \
  && make -j `nproc` \  
  && make install \
  # 直接生成在 /usr/local下，改用/opt/redwg下会出现编译通不过问题
  # && echo "/opt/redwg/lib" >> /etc/ld.so.conf.d/redwg.conf \
  # && mkdir -p /opt/redwg/lib \
  # && cp /usr/local/lib/libredwg* /opt/redwg/lib \
  && cd - \
  && rm -r libredwg-0.12.4

# Install opencv
# COPY --from=opencv /opt/opencv /opt/opencv
# COPY --from=opencv /etc/ld.so.conf.d/OpenCV.conf /etc/ld.so.conf.d/OpenCV.conf

# Install basler
COPY --from=basler /opt/pylon /opt/pylon
COPY --from=basler /etc/ld.so.conf.d/Pylon.conf /etc/ld.so.conf.d/Pylon.conf

# Install galaxy
COPY --from=galaxy /opt/Galaxy_camera /opt/GALAXY
COPY --from=galaxy /etc/ld.so.conf.d/GALAXY.conf /etc/ld.so.conf.d/GALAXY.conf

# Copy pcl ld config file
# COPY --from=pcl /etc/ld.so.conf.d/Pcl.conf /etc/ld.so.conf.d/Pcl.conf

# Copy mqtt 
#COPY /usr/local/lib/libpaho* /opt/mqtt/lib/

# Copy PhoXiControl
COPY ./Dockerfile/PhoXiControl /usr/local/bin/PhoXiControl

# Config PhoXiControl
# add necessery ENV configuration，and Disable display
ENV PHOXI_CONTROL_PATH="/opt/Photoneo/PhoXiControl" DOCKER=1 QT_X11_NO_MITSHM=1 PHOXI_WITHOUT_DISPLAY=1
RUN set -eux \
    && mkdir /fonts && chmod a+x /fonts \
    && cd /tmp \
    && chmod a+x phoxi.run \
    && chmod a+x /usr/local/bin/PhoXiControl \
    && ./phoxi.run --accept ${PHOXI_CONTROL_PATH} \
    && rm -rf phoxi.run \
    && mkdir /root/.PhotoneoPhoXiControl

CMD ["/usr/local/bin/PhoXiControl"]