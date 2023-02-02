# FROM ubuntu:18.04
# FROM ubuntu:20.04
FROM jadehu/ros2:build AS build
LABEL maintainer="numajinfei@163.com"

USER root

RUN set -eux \
    # change apt sources file
    # && sed -i "s/security.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list \
    # && sed -i "s/archive.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list \
    && apt-get update -y \
    && apt-get install -y -q software-properties-common && apt-add-repository universe \
    && apt-get update -y \
    # phoxicontrol dependencies (gui...)
    # ubuntu18.04:
    # && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.0.0 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb
    # ubuntu20.04:
    # && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.0.0 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb
    && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.1 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb libxcb-xinerama0 libpcre2-16-0

# Build yaml-cpp 
RUN wget -O yaml-cpp.tar.gz https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.tar.gz \
  && tar -xzf yaml-cpp.tar.gz \
  && rm yaml-cpp.tar.gz \
  && cmake \
    -D CMAKE_BUILD_TYPE=Release \    
    -D YAML_BUILD_SHARED_LIBS=ON \
    -S yaml-cpp-yaml-cpp-0.7.0/ \
    -Bbuild/ \
  && cmake --build build/ --target install \
  && rm -r yaml-cpp-yaml-cpp-0.7.0/ build \
# download phoxi.run (other repo)  
  # && wget -O /tmp/phoxi.run http://121.4.181.196:9000/build/phoxi.run \
  && wget -O https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/phoxi.run \
  && chmod +x /tmp/phoxi.run \
# # mkdir ./font floder
#   && pwd && echo "mkdir ./font floder" \
#   && mkdir ./fonts && chmod +x ./fonts -R

# download phoxi.run(from repo) to /tmp/phoxi.run
# RUN wget -O /tmp/phoxi.run http://121.4.181.196:9000/build/phoxi.run \
#   && chmod +x /tmp/phoxi.run

# RUN wget http://121.4.181.196:9000/build/phoxi.run \
#   && chmod +x phoxi.run \
#   && cp phoxi.run /tmp/phoxi.run \
#   && rm ./phoxi.run


# COPY installer/phoxi.run /tmp/phoxi.run
# COPY system_files/PhoXiControl /usr/local/bin/PhoXiControl
COPY ./Dockerfile/PhoXiControl /usr/local/bin/PhoXiControl

# copy 3d scanner source code and scripts to docker image.
# COPY /architecture_measurement ./ws/src
# COPY /script ./ws/scripts
COPY ./src ./ws/src
COPY ./script ./script/

# add necessery ENV configuration，and Disable display
ENV PHOXI_CONTROL_PATH="/opt/Photoneo/PhoXiControl" DOCKER=1 QT_X11_NO_MITSHM=1 PHOXI_WITHOUT_DISPLAY=1


RUN set -eux \
    && mkdir /fonts && chmod a+x /fonts \
    && cd /tmp \
    && chmod a+x phoxi.run \
    && chmod a+x /usr/local/bin/PhoXiControl \
    && ./phoxi.run --accept ${PHOXI_CONTROL_PATH} \
    && rm -rf phoxi.run

# TODO: this is a bug SCAN-3548
RUN set -eux \
    && mkdir /root/.PhotoneoPhoXiControl

CMD ["/usr/local/bin/PhoXiControl"]
