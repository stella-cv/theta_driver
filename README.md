# theta_driver

## Getting started

```bash
docker build -t theta_driver .
docker run --rm -it --net=host --privileged theta_driver
roscore&
rosrun theta_driver theta_driver_node
```

## ROS2-Foxy

[17/05/2022] Under development