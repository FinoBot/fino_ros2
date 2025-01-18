# FinoROS2

## Commands copy files into raspberry

```bash
scp -r fino_ros2/ finobot@IP:~/
```

## command on the raspberry

```bash
docker build -f fino_ros2/Dockerfile -t finoros2 .
```

```bash
docker run -it --privileged --rm --network host -v /dev/ttyS0:/dev/ttyS0 finoros2
```

start bittle driver + human recognition :

```bash
docker run -it --privileged --rm --network host -v /dev/ttyS0:/dev/ttyS0 finoros2 /bin/bash -c "source install/setup.bash && ros2 launch fino_ros2 finobot_launch.py"
```

Start all nodes including the camera spatial bb launch file:

```bash
docker build -f fino_ros2/Dockerfile-merge -t finoros2_and_camera .
docker run -it --privileged --rm --network host -v /dev/ttyS0:/dev/ttyS0 -v /dev/bus/usb:/dev/bus/usb finoros2_and_camera /bin/bash -c "source install/setup.bash && ros2 launch fino_ros2 finobot_and_camera_launch.py"
```
