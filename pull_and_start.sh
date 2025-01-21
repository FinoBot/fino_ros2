#!/bin/bash

docker stop finobot_ros2
docker rm finobot_ros2

git pull
cd ../fino_ros2_msgs
git pull
cd ..
docker build -f fino_ros2/Dockerfile -t finoros2 .
docker run --name finobot_ros2 --restart unless-stopped --privileged --rm --network host -v /dev/ttyS0:/dev/ttyS0 -v /dev/bus/usb:/dev/bus/usb finoros2 /bin/bash -c "source install/setup.bash && ros2 launch fino_ros2 finobot_launch.py"
