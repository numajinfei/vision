FROM ros:galactic-ros1-bridge
LABEL maintainer="numajinfei@163.com"

ENV ROS_HOSTNAME host
ENV ROS_MASTER_URI http://ros1:11311
# ENTRYPOINT ["soure /opt/ros/galactic/setup.bash", "ros2 run ros1_bridge dynamic_bridge"]
CMD ["source /opt/ros/galactic/setup.bash", "ros2 run ros1_bridge dynamic_bridge"]