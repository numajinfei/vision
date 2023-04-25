<!--
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2021-12-20 13:23:58
 * @LastEditors: hw
 * @LastEditTime: 2023-04-17 20:01:38
-->
## 1. Description
| item   | content                       | description                                                  |
| ------ | ----------------------------- | ------------------------------------------------------------ |
| branch | base                          | 工程base分支为ubuntu20.04下ros2(galactic)基本环境搭建分支，包括basler,galaxy,build等基础镜像生成dockerfile文件 |
| os     | ros2---galactic (ubuntu20.04) |                                                              |
|        |                               |                                                              |



## 2. 模块

### 1. 硬件驱动

#### 1. 相机驱动

- galaxy
- basler

#### 2. 测头驱动

- photoneo （外购测头）

------



### 2. 三方库

#### 1. 直接生成docker 镜像的三方库

- opencv：
- pcl：直接生成docker镜像

#### 2. 安装到build镜像中的三方库

- nlohman json: 只下载使用了json.hpp
- paho mqtt: 分paho-mqtt.c和paho-mqtt.cpp, 两个库都安装

------



### 3. 系统软件包

#### 1.ros2 安装包

- ros-galactic-pcl-conversions

#### 2. ros2 导航依赖包安装

- ros-galactic-bondcpp
- ros-galactic-test-msgs
- ros-galactic-behaviortree-cpp-v3
- ros-galactic-rviz-common
- ros-galactic-rviz-default-plugins
- ros-galactic-angles
- ros-galactic-cv-bridge
- ros-galactic-ompl
- ros-galactic-image-transport
- ros-galactic-gazebo-ros-pkgs
- ros-galactic-libg2o
- 

- libopencv-dev 

- graphicsmagick*

- libsdl-image1.2-dev ，libsdl-dev

- libsuitesparse-dev

  

#### 3. ros2 测头代码依赖包安装

- ros-galactic-cv-bridge

  

#### 4. AI 相关包安装

- python3-pip
- pip3 install opencv-python
- pip3 install torch torchvision torchaudio 
- pip3 install torchsummary tqdm

> 1. 如外网下载失败或缓慢，则可从国内源下载：
>
>    pip3 install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
>
>    pip3 install torch torchvision torchaudio -i https://pypi.tuna.tsinghua.edu.cn/simple
>
> 2. 直接安装包下载（可以减小生成镜像大小）
>
>    wget https://github.com/numajinfei/vision/releases/download/v0.0.1-3rdparty/torch-1.13.0+cpu-cp38-cp38-linux_x86_64.whl
>
>    pip3 install torch-1.13.0+cpu-cp38-cp38-linux_x86_64.whl
>
>    pip3 install torchsummary tqdm

#### 5. phoxi sdk

​	为外购测头photoneo的驱动sdk docker环境配置所需要

- [phoxi.run](https://www.photoneo.com/downloads/phoxi-control#)
- [PhoXiControl](https://github.com/photoneo/phoxi-docker)

# 3. 代码编译相关

ros2节点编译脚本：

```shell
node=$1
if [ ! -n "$node" ]; then
    echo "Please input a package name!"
  	exit 1
fi

source install/setup.sh
echo "build node----->: $node"
if [ "$node" == "all" ];then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

else
    #colcon build --packages-select $node --cmake-args -DCMAKE_BUILD_TYPE=Release
# 设置编译时的资源，类似cmake -j4等
    colcon build --packages-select $node --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2

fi
```



# 4. 容器创建指令

#### 1. build环境容器：

```shell
# ros2 + phoxi:
docker run -itd --privileged --env DOCKER=1 --env QT_X11_NO_MITSHM=1 --env PHOXI_WITHOUT_DISPLAY=1 -v /dev/dri/dev/dri -v /tmp/.X11-unix/:/tmp/.X11-unix -v /var/run/dbus:/var/run/dbus -v /dev/shm:/dev/shm -v /dev/rplidar:/dev/rplidar -v /dev/rplidar2:/dev/rplidar2 -v /dev/slave_com:/dev/slave_com -v /etc/localtime:/etc/localtime:ro -v /home/ubuntu/work/ws2:/ws2 --log-opt max-size=5g --log-opt max-file=50 --network host --name ros2_build --restart  unless-stopped jadehu/ros2:build
```



# 3. docker run command
## 1. docker container start:
- ros2_test:
  ```shell
  # contain basler:
  docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ~/vision/log:/ws/ros2_log/ --restart unless-stopped --name test jadehu/ros2_feature_a_test
  ```

- ros2_deploy:
  ```shell
  # contain basler:
  docker run -itd --device /dev/InclinometerPort --device /dev/gpiochip0 --device /dev/bus/usb --device /dev/motor --network host -v /etc/localtime:/etc/localtime:ro -v ~/vision/log:/ws/ros2_log --restart unless-stopped --name deploy jadehu/ros2_feature_a_deploy
  ```
  
- ros2+phoxi

  ```shell
  docker run -itd --privileged --env DOCKER=1 --env QT_X11_NO_MITSHM=1 --env PHOXI_WITHOUT_DISPLAY=1 -v /dev/dri/dev/dri -v /tmp/.X11-unix/:/tmp/.X11-unix -v /var/run/dbus:/var/run/dbus -v /dev/shm:/dev/shm -v /dev/rplidar:/dev/rplidar -v /dev/rplidar2:/dev/rplidar2 -v /dev/slave_com:/dev/slave_com -v /etc/localtime:/etc/localtime:ro -v /home/ubuntu/work/ws2:/ws2 --log-opt max-size=5g --log-opt max-file=50 --network host --name ros2_zhushi --restart  unless-stopped jadehu/ros2:feature_a_test_phoxi
  ```

  ```shell
  ## test deploy on Zhushi Server(192.168.1.8)
  docker run -itd --privileged --env DOCKER=1 --env QT_X11_NO_MITSHM=1 --env PHOXI_WITHOUT_DISPLAY=1 -v /dev/dri/dev/dri -v /tmp/.X11-unix/:/tmp/.X11-unix -v /var/run/dbus:/var/run/dbus -v /dev/shm:/dev/shm -v /dev/rplidar:/dev/rplidar -v /dev/rplidar2:/dev/rplidar2 -v /dev/slave_com:/dev/slave_com -v /etc/localtime:/etc/localtime:ro -v /home/zhushi/work:/work --log-opt max-size=5g --log-opt max-file=50 --network host --name phoxi2 chs_ros2_ros_build2
  ```

  





## 2. docker enter the running container
```shell
docker exec -it <container_name/alias_name> /bin/bash
```
