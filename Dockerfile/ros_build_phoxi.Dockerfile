FROM jadehu/ros2:build AS build
LABEL maintainer="numajinfei@163.com"

# [@NOTE: Just create a photoneo images base on ros2:build image.]

# [Install dependencies]
RUN apt-get update && apt-get install -y --no-install-recommends \
# Phoxicontrol dependencies (gui...)
  && apt-get install -y -q software-properties-common && apt-add-repository universe \
  && apt-get update -y \
#---ubuntu18.04:
  # && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.0.0 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb
#---ubuntu20.04:
  && apt install -y avahi-utils libqt5core5a libqt5dbus5 libqt5gui5 libgtk2.0-0 libssl1.1 libgomp1 libpcre16-3 libflann-dev libssh2-1-dev libpng16-16 libglfw3-dev xcb libxcb-xinerama0 libpcre2-16-0 \
  && rm -rf /var/lib/apt/lists/* \

# [Download phoxi.run]
# RUN wget -O /tmp/phoxi.run https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/phoxi.run \
  && wget -O /tmp/phoxi.run https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/phoxi.run \
  && chmod +x /tmp/phoxi.run

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