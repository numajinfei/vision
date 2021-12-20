FROM ros:galactic
LABEL maintainer="numajinfei@163.com"

RUN apt-get update && apt-get install -y --no-install-recommends \
  # libpcl-dev \ #运行ros2测头代码会出现 #include "pcl_conversions/pcl_conversions.h" 无法找到
  ros-galactic-pcl-conversions \
  # 运行ros2双目测头代码会出现 #include "opencv2/highgui.hpp" 无法找到，ros2_opencv安装文件中没有此hpp文件
  # libopencv-dev \
  && rm -rf /var/lib/apt/lists/*
  
