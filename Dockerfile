FROM ros:humble-ros-core

ENV ROS_DOMAIN_ID=158

RUN apt update && apt install -y git python3-pip python3-colcon-common-extensions ninja-build ros-humble-joy ros-humble-ament-cmake portaudio19-dev ros-humble-depthai-ros build-essential swig libpulse-dev libasound2-dev
RUN python3 -m pip install wheel==0.37.1
RUN pip3 install --force-reinstall cython==0.29.33
RUN pip3 install --no-build-isolation git+https://github.com/lucasb-eyer/pydensecrf.git
RUN pip3 install pyserial rosdep sounddevice cython SpeechRecognition pocketsphinx

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

COPY fino_ros2 src/fino_ros2
COPY fino_ros2_msgs src/fino_ros2_msgs

RUN rosdep init && rosdep fix-permissions && rosdep update && rosdep install --from-paths src --ignore-src -r -y 
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash"   