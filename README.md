# FinoROS2

ROS2 package for Bittle Robot. The aim of this package is to make the Petoi bittle robot autonomous thanks to a 3D camera.

## prerequisites

- Hardware:
    - Petoi Bittle robot
    - Raspberry pi (Tested on pi4)
    - Caméra 3D Oak-d-s2
    - Microphone (Tested with Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H)
    - Ideally, use a more powerful battery on the bittle than the original one

- Software:
    - Python 3.5 or higher
    - Git
    - Docker (On raspberry pi)

## Set up raspberry pi

### Configure your raspberry pi by following the official documentation: https://www.raspberrypi.com/documentation/computers/getting-started.html

### On first power up

### Updage /boot/firmware/config.txt

Uncomment this line to activate I2s
```bash
dtparam=i2s=on
```

Add below this line

```bash
dtoverlay=googlevoicehat-soundcard
```

Reboot de the raspberry

### Installations

```bash
sudo apt install docker git
```

### Clone repositories

Clone the ROS2 packages 

```bash
git clone https://github.com/FinoBot/fino_ros2.git
git clone https://github.com/FinoBot/fino_ros2_msgs.git
```

## Launch the robot

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
docker build -f fino_ros2/Dockerfile -t finoros2 .
docker run --privileged --rm --network host -v /dev/ttyS0:/dev/ttyS0 -v /dev/bus/usb:/dev/bus/usb finoros2 /bin/bash -c "source install/setup.bash && ros2 launch fino_ros2 finobot_launch.py"
```
