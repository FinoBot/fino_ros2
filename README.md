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