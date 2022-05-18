# theta_driver

## Getting started

### ROS1

```bash
docker build -t theta_driver .
docker run --rm -it --net=host --privileged theta_driver
roscore&
rosrun theta_driver theta_driver_node
```

### ROS2-Foxy

You need to install the libuvc and it's dependencies before using this package.

1. https://github.com/ricohapi/libuvc-theta

You will also need the sample for the Theta-Z1

2. https://github.com/ricohapi/libuvc-theta-sample

To install the package:
```
source /opt/ros/foxy/setup.bash
mkdir -p theta_driver_ws/src
cd theta_driver_ws
git clone -b ros2_foxy https://github.com/JLBicho/theta_driver.git src/theta_driver
git clone https://github.com/stella-cv/libuvc-theta-sample.git src/theta_driver/3rd/libuvc-theta-sample
colcon build
```

Try it with:
```
source install/setup.bash
ros2 run theta_driver theta_driver_node 
```

And then you can use image_view package, rqt or rviz2 to see the published image.





## Status
[18/05/2022] Works in Ubuntu20.04-ROS2-Foxy with local installation.Docker under development

[17/05/2022] Under development