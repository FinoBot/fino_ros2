FROM ros:humble-ros-core

ENV ROS_DOMAIN_ID=158

RUN apt update && apt install -y git python3-pip python3-colcon-common-extensions
RUN pip3 install pyserial

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src/fino_ros2
COPY . .